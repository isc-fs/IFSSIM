"""
sim_supervisor_node — the DV pipeline's stand-in for the IFS-08 uDV.

On the real car the uDV is a microROS endpoint (USB CDC). It owns:

  * the physical GO button → emits the GO signal into the autonomy
  * the physical RES button + EBS plumbing
  * the DVPC↔uDV action endpoint that the autonomy talks to
  * forwarding control commands to the powertrain

In sim there is no microcontroller, no buttons, no plumbing. This
node fakes all of that so the autonomy stack sees an identical ROS 2
surface in sim and on the real car. **It is sim-only** and is not
launched on the real-car compose stack.

Two-phase action protocol implemented here (see docs/autonomy_pipeline.md
§"Runtime action protocol"):

  Phase 1 — StartMission (this node is the *client*, mission_control
            is the *server*). Triggered by mission_control_backend
            (web) calling our own StartMission server. Carries the
            chosen mission; mission_control fans out lifecycle
            transitions through mode_manager and reports ready/failed.

  Phase 2 — RuntimeControl (this node is the *client*). Opened once
            Phase 1 reports ready. Carries throttle/steering feedback
            from control_node, plus emergency / finished from slam.
            Terminates the action when finished or emergency arrives.

This file is currently a **lifecycle skeleton** — all transitions
return SUCCESS, the StartMission server accepts goals and immediately
reports ready=true, RuntimeControl is not opened yet. Real wiring
lands in step 5 of the DV-pipeline-alignment series (see todo).
"""

from __future__ import annotations

import time

import numpy as np

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty as EmptyMsg, Float32
from tf2_ros import TransformBroadcaster

from fs_msgs.msg import ControlCommand
from dv_msgs.action import StartMission, RuntimeControl
from dv_msgs.srv import ActivateMode  # noqa: F401 — kept for future use

from sim_supervisor.odometry import OdometryFilter


# /odom publication rate. 100 Hz target — gives the 40 Hz controller
# fresh data every tick with margin, doesn't burn the CPU. Decoupled
# from the IMU subscription rate (which is the BMI088's native ~400 Hz).
ODOM_PUBLISH_HZ: float = 100.0

# Take every Nth IMU sample, discard the rest before pushing into the
# OdometryFilter. The BMI088 publishes at ~400 Hz; the filter
# integrates per push_imu call so consuming all 400 burns sim_supervisor
# CPU. Quick CPU audit on 2026-05-11 showed sim_supervisor at 93 % CPU
# pre-decimation (full 400 Hz consumption). With IMU_DECIMATION=4 we
# integrate at 100 Hz, matching the /odom publish rate and the
# controller's 40 Hz tick rate with plenty of margin.
#
# This is the engineering call deferred by #385 (the "quantify
# whether 100 Hz loses meaningful filter quality vs 400 Hz" question).
# In practice: bias estimation during the 3 s stationary window still
# averages over ~300 samples at 100 Hz — well above the noise floor.
# Steady-state predict step doesn't benefit from sub-10 ms IMU samples
# (controller dt is 25 ms). Bumping back up to 1 (full rate) is the
# A/B test; #385 stays open for that quantification work.
IMU_DECIMATION: int = 4


# Total time we'll wait for mission_control_node.start_mission_orchestration
# to come back. Mission_control's own timeout on activate_mode is 240 s;
# add 30 s headroom for the supervisor → mission_control → mode_manager
# round-trip overhead so the inner timeout fires first and produces a
# useful diagnostic.
_ORCHESTRATION_TIMEOUT_S = 270.0
_HEARTBEAT_PERIOD_S = 0.5


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class SimSupervisorNode(LifecycleNode):
    """Sim-only DVPC stand-in. See module docstring."""

    NODE_NAME = "sim_supervisor_node"

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        # Configured in on_configure, torn down in on_cleanup.
        self._start_mission_server: ActionServer | None = None
        self._runtime_control_client: ActionClient | None = None
        self._mc_start_mission_client: ActionClient | None = None
        self._control_pub = None
        self._ebs_pub = None
        self._ebs_reset_pub = None
        self._current_mission: str | None = None

        # Active RuntimeControl goal handle (Phase 2 of #384). Opened
        # against mission_control_node after Phase 1 reports ready;
        # tracked here so a tear-down (StartMission with mission="")
        # or a mission switch can cancel the in-flight action cleanly.
        # `_runtime_ebs_latched` tracks whether we've already published
        # the latched /signal/ebs Empty for this run — we want to fire
        # it once on the rising edge, not every feedback frame.
        self._runtime_goal_handle = None
        self._runtime_ebs_latched: bool = False

        # IMU sample counter, modulo IMU_DECIMATION. Pre-decimation
        # the supervisor was integrating ~400 IMU samples/s on a
        # single CPU; counting + dropping in the callback is the
        # cheapest possible throttle.
        self._imu_sample_idx: int = 0

        # Reentrant group so the StartMission action handler can wait
        # on the inner ActionClient future (against mission_control)
        # without deadlocking on the same mutually-exclusive group.
        self._cb_group = ReentrantCallbackGroup()

        # Odometry filter (Phase 1 of the /odom split — see
        # docs/autonomy_pipeline.md §"Open questions" Q1). Subscribes
        # to /imu and /motor_rpm, publishes /odom at ODOM_PUBLISH_HZ.
        # The supervisor is the natural owner because on the real car
        # the uDV (which this node simulates) publishes /odom from the
        # same input set.
        self._odom_filter: OdometryFilter | None = None
        self._odom_pub = None
        self._odom_pub_timer = None
        # Phase 3 (#383) — steering + brake_pressure inputs from the
        # bridge + diagnostic publishers for the OdometryFilter
        # cross-check residuals. Subscriptions are bound to
        # on_activate (they only need to flow when /odom is being
        # published); diagnostic publishers are lifecycle-aware.
        self._sub_steering = None
        self._sub_brake = None
        self._yaw_residual_pub = None
        self._slip_flag_pub = None
        self._effective_alpha_pub = None
        # Phase 2 (#382): supervisor owns the odom→base_link TF
        # broadcast (slam_node stopped doing it in #382 and now
        # publishes map→odom instead, computed from slam_pose ⊖
        # latest /odom). Created in on_configure, used inside
        # _publish_odom.
        self._odom_tf_broadcaster: TransformBroadcaster | None = None
        self._sub_imu = None
        self._sub_rpm = None
        self._odom_first_publish_logged: bool = False

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure: creating I/O")

        # Phase 1 server — mission_control_backend (web) is the client.
        self._start_mission_server = ActionServer(
            self,
            StartMission,
            "start_mission",
            execute_callback=self._execute_start_mission,
            callback_group=self._cb_group,
        )

        # Inner Phase 1 client — supervisor relays the goal to
        # mission_control_node, which drives mode_manager.
        self._mc_start_mission_client = ActionClient(
            self,
            StartMission,
            "start_mission_orchestration",
            callback_group=self._cb_group,
        )

        # Phase 2 client — opens once Phase 1 reports ready.
        self._runtime_control_client = ActionClient(
            self,
            RuntimeControl,
            "runtime_control",
            callback_group=self._cb_group,
        )

        # Output: /fsds/control_command — the bridge subscribes here.
        self._control_pub = self.create_lifecycle_publisher(
            ControlCommand, "/fsds/control_command", 10,
        )

        # Latched EBS + EBS reset onto the bridge.
        self._ebs_pub = self.create_lifecycle_publisher(
            EmptyMsg, "/signal/ebs", LATCHED_QOS,
        )
        self._ebs_reset_pub = self.create_lifecycle_publisher(
            EmptyMsg, "/signal/ebs_reset", LATCHED_QOS,
        )

        # /odom infrastructure — created here, subscriptions and
        # timer come up in on_activate.
        #
        # Phase 2 (#382): supervisor owns the odom→base_link TF.
        # slam_node simultaneously publishes map→odom (drift
        # correction) computed from its absolute pose ⊖ our
        # supervisor /odom — together the chain
        # map → odom → base_link gives SLAM's absolute pose at the
        # leaf, with map→odom absorbing accumulated supervisor
        # dead-reckoning drift between SLAM ticks. tf2 has no
        # lifecycle TransformBroadcaster; the regular one is silent
        # until on_activate's filter-calibration window completes
        # and the timer fires.
        self._odom_filter = OdometryFilter()
        self._odom_pub = self.create_lifecycle_publisher(
            Odometry, "/odom", 50,
        )
        self._odom_tf_broadcaster = TransformBroadcaster(self)

        # Phase 3 (#383) diagnostic publishers — emit cross-check
        # residuals next to /odom so tuning consumers (Lichtblick
        # plot panels, offline replay) can see filter state without
        # re-deriving from raw sensors.
        self._yaw_residual_pub = self.create_lifecycle_publisher(
            Float32, "/odom_diag/yaw_residual_rad_s", 10,
        )
        self._slip_flag_pub = self.create_lifecycle_publisher(
            Bool, "/odom_diag/slip_flag", 10,
        )
        self._effective_alpha_pub = self.create_lifecycle_publisher(
            Float32, "/odom_diag/effective_alpha_vx", 10,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            "on_activate: starting /odom subscriptions + publish timer "
            f"({ODOM_PUBLISH_HZ:.0f} Hz)")

        # Reset the filter so a deactivate→activate cycle starts a
        # fresh stationary calibration. The car may have been moved
        # in sim during the inactive window; assuming continuity
        # would corrupt the bias estimates.
        if self._odom_filter is not None:
            self._odom_filter.reset()
        self._odom_first_publish_logged = False

        # Latched /signal/ebs_reset (post-#384). On the real car the
        # uDV firmware clears the EBS gate at power-up; in sim the
        # supervisor does the same the moment its lifecycle goes
        # active. Without this, the bridge's `ebs_triggered_` gate
        # stays latched from any previous run and every relayed
        # control command would be silently dropped. Pre-#384
        # control_node owned this publish on its own on_activate;
        # the topic is now exclusively supervisor-owned per the
        # diagram contract in docs/autonomy_pipeline.md.
        if self._ebs_reset_pub is not None:
            self._ebs_reset_pub.publish(EmptyMsg())

        # IMU subscription — BEST_EFFORT to match what the bridge
        # publishes, deep queue (2000) so the predict step doesn't
        # lose samples while RPM messages are being processed on the
        # same executor. Same QoS choice slam_node makes for the same
        # reason.
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2000,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub_imu = self.create_subscription(
            Imu, "/imu", self._on_imu, imu_qos,
            callback_group=self._cb_group,
        )

        # Motor RPM — 80 Hz from the bridge. BEST_EFFORT, shallow queue.
        rpm_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub_rpm = self.create_subscription(
            Float32, "/motor_rpm", self._on_rpm, rpm_qos,
            callback_group=self._cb_group,
        )

        # Phase 3 (#383) — steering + brake_pressure cross-check
        # inputs. Same BEST_EFFORT QoS as RPM since they share the
        # 100 Hz bridge cadence and consumers tolerate dropped
        # samples (the filter just uses the latest cached value).
        self._sub_steering = self.create_subscription(
            Float32, "/steering_angle", self._on_steering, rpm_qos,
            callback_group=self._cb_group,
        )
        self._sub_brake = self.create_subscription(
            Float32, "/brake_pressure", self._on_brake, rpm_qos,
            callback_group=self._cb_group,
        )

        # Publish /odom on a fixed-rate timer rather than per-IMU-tick:
        # decouples publish rate from input rate, gives downstream
        # consumers a predictable cadence regardless of IMU jitter.
        self._odom_pub_timer = self.create_timer(
            1.0 / ODOM_PUBLISH_HZ,
            self._publish_odom,
            callback_group=self._cb_group,
        )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate: stopping /odom + subs")
        if self._odom_pub_timer is not None:
            self.destroy_timer(self._odom_pub_timer)
            self._odom_pub_timer = None
        for sub in (self._sub_imu, self._sub_rpm,
                    self._sub_steering, self._sub_brake):
            if sub is not None:
                self.destroy_subscription(sub)
        self._sub_imu = None
        self._sub_rpm = None
        self._sub_steering = None
        self._sub_brake = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup: tearing down I/O")
        # Cancel any in-flight RuntimeControl before destroying the
        # client, otherwise the action's terminal callback fires
        # against a dead handle.
        self._cancel_runtime_control()
        if self._start_mission_server is not None:
            self._start_mission_server.destroy()
            self._start_mission_server = None
        self._mc_start_mission_client = None
        self._runtime_control_client = None
        self._control_pub = None
        self._ebs_pub = None
        self._ebs_reset_pub = None
        # /odom infra — subs/timer already gone via on_deactivate, but
        # we still own the publisher + filter + broadcaster.
        if self._odom_pub_timer is not None:
            self.destroy_timer(self._odom_pub_timer)
            self._odom_pub_timer = None
        self._odom_pub = None
        self._odom_tf_broadcaster = None
        self._yaw_residual_pub = None
        self._slip_flag_pub = None
        self._effective_alpha_pub = None
        self._odom_filter = None
        self._current_mission = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # /odom — IMU + RPM → dead-reckoning Odometry
    # ------------------------------------------------------------------
    def _on_imu(self, msg: Imu) -> None:
        """Drive the filter's predict step.

        Decimates the 400 Hz IMU stream by IMU_DECIMATION before
        pushing into the filter — see the module-level constant for
        the CPU rationale (#385). Bridge subscription stays at full
        depth so the unused samples flow through DDS at zero cost
        to us; we just skip the np.array construction + push_imu
        call for the dropped 3 of 4.
        """
        if self._odom_filter is None:
            return
        self._imu_sample_idx += 1
        if self._imu_sample_idx % IMU_DECIMATION:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self._odom_filter.push_imu(t, accel, gyro)

    def _on_rpm(self, msg: Float32) -> None:
        """Drive the filter's correction step."""
        if self._odom_filter is None:
            return
        # Wall-clock timestamp — the bridge publishes Float32 with no
        # header.stamp on /motor_rpm, so we mark received-time here.
        # Used for staleness inside the filter.
        self._odom_filter.push_rpm(time.monotonic(), float(msg.data))

    def _on_steering(self, msg: Float32) -> None:
        """Cache the latest front-wheel angle (rad). Used inside the
        filter's push_imu step for the kinematic-bicycle yaw cross-
        check (#383)."""
        if self._odom_filter is None:
            return
        self._odom_filter.push_steering(time.monotonic(), float(msg.data))

    def _on_brake(self, msg: Float32) -> None:
        """Cache the latest brake authority [0, 1]. Used inside
        push_rpm to scale α_vx during brake events (#383)."""
        if self._odom_filter is None:
            return
        self._odom_filter.push_brake(time.monotonic(), float(msg.data))

    def _publish_odom(self) -> None:
        """Timer-driven /odom topic + odom→base_link TF emission.
        Skips while the filter is still in stationary calibration
        (first ~3 s after activate)."""
        if (self._odom_filter is None
                or self._odom_pub is None
                or self._odom_tf_broadcaster is None):
            return
        if not self._odom_filter.is_calibrated():
            return

        s = self._odom_filter.state
        now = self.get_clock().now().to_msg()

        if not self._odom_first_publish_logged:
            self.get_logger().info(
                "/odom first publish — IMU+RPM filter calibrated")
            self._odom_first_publish_logged = True

        # 2D yaw → unit quaternion (axis-z) used by both the Odometry
        # message and the TF broadcast.
        half = 0.5 * s.yaw
        qw = float(np.cos(half))
        qz = float(np.sin(half))

        # nav_msgs/Odometry: pose in header.frame_id (odom),
        # twist in child_frame_id (base_link). REP-103 axes.
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = s.x
        msg.pose.pose.position.y = s.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = qw
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.twist.twist.linear.x = s.vx
        msg.twist.twist.linear.y = s.vy
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = s.yaw_rate
        self._odom_pub.publish(msg)

        # odom → base_link TF (Phase 2 — #382). Same pose, broadcast
        # at the 100 Hz publish rate so downstream TF lookups see a
        # high-rate dead-reckoning leaf.
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = s.x
        tf.transform.translation.y = s.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = qw
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = qz
        self._odom_tf_broadcaster.sendTransform(tf)

        # Phase 3 (#383) diagnostics — emit the filter's cross-check
        # residuals alongside /odom. Cheap (three Float32-ish topics
        # at 100 Hz); off by default in subscribers, only the tuning
        # plot panels open them.
        diag = self._odom_filter.diagnostics
        if self._yaw_residual_pub is not None:
            self._yaw_residual_pub.publish(Float32(data=float(diag.yaw_residual_rad_s)))
        if self._slip_flag_pub is not None:
            self._slip_flag_pub.publish(Bool(data=bool(diag.slip_flag)))
        if self._effective_alpha_pub is not None:
            self._effective_alpha_pub.publish(Float32(data=float(diag.effective_alpha_vx)))

    # ------------------------------------------------------------------
    # Action handlers (skeletons)
    # ------------------------------------------------------------------
    def _execute_start_mission(self, goal_handle):
        """
        Phase 1 — accept the mission, relay to mission_control_node.

        Acts as an action proxy:
          • Forwards the mission name onto mission_control's
            start_mission_orchestration goal.
          • Forwards each inner-feedback frame back to the outer caller
            (mission_control_backend on the web side; the physical uDV
            on the real car).
          • Returns the inner result verbatim.

        The supervisor exists at this layer so the autonomy stack sees
        the same action surface regardless of who's driving — sim web
        backend or real-car uDV. Both call StartMission against this
        same node-name; only the *transport* differs (DDS over the
        Docker network in sim, microROS over USB CDC on the real car —
        and on the real car, sim_supervisor_node is replaced by the
        physical uDV firmware which speaks the same action).
        """
        mission = goal_handle.request.mission
        self.get_logger().info(f"StartMission received: mission={mission!r}")
        self._current_mission = mission

        # Tear-down path. mission="" is the web backend's signal that
        # Stop Session was pressed (or the operator switched missions
        # mid-run). Cancel the in-flight RuntimeControl *before*
        # relaying to mission_control, otherwise the action server
        # gets torn down (lifecycle deactivate) while the goal is
        # still open, which logs noisy warnings on both sides.
        if mission == "":
            self._cancel_runtime_control()

        result = StartMission.Result()

        # Wait for mission_control_node's action server to come up.
        # mission_control auto-activates at launch, so this should be
        # ~immediate; cap at 5 s to fail fast if it isn't there.
        if not self._mc_start_mission_client.wait_for_server(timeout_sec=5.0):
            result.ready = False
            result.message = (
                "start_mission_orchestration server unavailable; "
                "is mission_control_node active?"
            )
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        # Send the goal. We capture inner feedback in a closure that
        # republishes onto the outer goal handle so the web client
        # sees the same heartbeat stages mission_control emits.
        inner_goal = StartMission.Goal()
        inner_goal.mission = mission

        def _on_inner_feedback(fb_msg) -> None:
            inner_fb = fb_msg.feedback
            outer_fb = StartMission.Feedback()
            outer_fb.stage = inner_fb.stage
            outer_fb.stamp = inner_fb.stamp
            try:
                goal_handle.publish_feedback(outer_fb)
            except Exception as ex:
                # Outer goal cancelled out from under us; fine.
                self.get_logger().debug(f"feedback relay skipped: {ex}")

        send_goal_future = self._mc_start_mission_client.send_goal_async(
            inner_goal, feedback_callback=_on_inner_feedback,
        )

        # Wait for the inner goal to be accepted/rejected.
        deadline = time.monotonic() + _ORCHESTRATION_TIMEOUT_S
        while not send_goal_future.done():
            if time.monotonic() >= deadline:
                result.ready = False
                result.message = "inner StartMission goal acceptance timed out"
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result
            time.sleep(0.05)

        inner_goal_handle = send_goal_future.result()
        if inner_goal_handle is None or not inner_goal_handle.accepted:
            result.ready = False
            result.message = "mission_control rejected the StartMission goal"
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        # Wait for the inner result.
        get_result_future = inner_goal_handle.get_result_async()
        while not get_result_future.done():
            if time.monotonic() >= deadline:
                result.ready = False
                result.message = (
                    "inner StartMission did not return within "
                    f"{_ORCHESTRATION_TIMEOUT_S:.0f} s"
                )
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result
            time.sleep(0.05)

        wrapper = get_result_future.result()
        # `wrapper.result` is the StartMission.Result we sent through;
        # `wrapper.status` is the action server's terminal status.
        inner_result: StartMission.Result = wrapper.result

        result.ready = inner_result.ready
        result.message = inner_result.message

        if inner_result.ready:
            self.get_logger().info(
                f"StartMission relay: mission {mission!r} ready")
            goal_handle.succeed()
            # Phase 2 — once mission_control reports the autonomy
            # lifecycle is active, open RuntimeControl against it so
            # control_node + slam_node start streaming actuator
            # commands back to us as Feedback. Idempotent: cancels
            # any previously-active goal first (mission switch path).
            #
            # Skip on the tear-down path (mission==""): in that case
            # mode_manager just reported "ready" because there was
            # nothing to do (all autonomy nodes already at target
            # state, see "already at/past target ... skipping"), and
            # we have NO mission to drive. Opening RuntimeControl
            # here against a torn-down lifecycle would race the
            # cancel we already sent at the top of this method.
            if mission:
                self._open_runtime_control()
        else:
            self.get_logger().error(
                f"StartMission relay: mission {mission!r} failed: "
                f"{inner_result.message}")
            goal_handle.abort()

        return result

    # ------------------------------------------------------------------
    # Phase 2 — RuntimeControl client + tear-down
    # ------------------------------------------------------------------
    def _open_runtime_control(self) -> None:
        """Open the RuntimeControl action against mission_control_node.

        Called from _execute_start_mission once Phase 1 reports ready.
        Cancels any previously-active goal first so a mission switch
        (e.g. operator runs trackdrive after autocross) doesn't leak
        two clients on the same server.

        The action is fire-and-forget from the supervisor's
        perspective — feedback frames trigger _on_runtime_feedback,
        which republishes them onto the bridge topics. Termination
        is handled in _on_runtime_result.
        """
        if self._runtime_control_client is None:
            self.get_logger().error(
                "_open_runtime_control: action client not configured")
            return

        # Cancel any prior goal first. This is the mission-switch path
        # (Phase 1 ran twice in one session); fresh tear-down goes
        # through _cancel_runtime_control directly.
        if self._runtime_goal_handle is not None:
            self.get_logger().info(
                "_open_runtime_control: cancelling prior goal before "
                "opening new one")
            self._cancel_runtime_control()

        # Reset the rising-edge latch so a new run can publish
        # /signal/ebs again if its own emergency arrives.
        self._runtime_ebs_latched = False

        if not self._runtime_control_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "_open_runtime_control: mission_control runtime_control "
                "server not available; actuator chain will be silent")
            return

        send_future = self._runtime_control_client.send_goal_async(
            RuntimeControl.Goal(),
            feedback_callback=self._on_runtime_feedback,
        )
        send_future.add_done_callback(self._on_runtime_goal_accepted)

    def _on_runtime_goal_accepted(self, send_future) -> None:
        """send_goal_async done-callback — stash the goal handle."""
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(
                "RuntimeControl goal rejected by mission_control_node")
            self._runtime_goal_handle = None
            return
        self._runtime_goal_handle = gh
        self.get_logger().info("RuntimeControl goal accepted; streaming")
        # Wire the terminal callback to capture result + clear state.
        gh.get_result_async().add_done_callback(self._on_runtime_result)

    def _on_runtime_feedback(self, fb_msg) -> None:
        """Republish each RuntimeControl Feedback onto the bridge.

        - throttle/steering → /fsds/control_command (ControlCommand).
        - emergency rising edge → latched /signal/ebs (Empty).
        - finished is informational here; the terminal action result
          will close out the run via _on_runtime_result.
        """
        fb = fb_msg.feedback
        if self._control_pub is not None:
            cmd = ControlCommand()
            cmd.header.stamp = fb.stamp
            cmd.throttle = float(fb.throttle)
            cmd.steering = float(fb.steering)
            # ControlCommand.brake isn't carried on RuntimeControl
            # (negative throttle = regen on the real car); leave 0.
            cmd.brake = 0.0
            self._control_pub.publish(cmd)

        if fb.emergency and not self._runtime_ebs_latched:
            self.get_logger().warn(
                "RuntimeControl feedback emergency=true — "
                "publishing latched /signal/ebs")
            if self._ebs_pub is not None:
                self._ebs_pub.publish(EmptyMsg())
            self._runtime_ebs_latched = True

    def _on_runtime_result(self, result_future) -> None:
        """Terminal callback when the action closes (finished /
        emergency / cancelled / error). Clears state so the next
        StartMission can open a fresh run."""
        try:
            wrapper = result_future.result()
            outcome = wrapper.result.outcome if wrapper else "(none)"
            msg = wrapper.result.message if wrapper else ""
        except Exception as ex:  # noqa: BLE001
            outcome = "error"
            msg = repr(ex)
        self.get_logger().info(
            f"RuntimeControl terminated: outcome={outcome!r} msg={msg!r}")
        self._runtime_goal_handle = None

    def _cancel_runtime_control(self) -> None:
        """Cancel the in-flight RuntimeControl goal (if any).

        Called when the operator hits Stop Session (StartMission goal
        arrives with mission="") or before opening a new run. Fire-
        and-forget: we don't block on the cancel_async future because
        mission_control's cancel_callback accepts unconditionally and
        the action's terminal callback will fire on its own.
        """
        gh = self._runtime_goal_handle
        if gh is None:
            return
        try:
            gh.cancel_goal_async()
            self.get_logger().info(
                "_cancel_runtime_control: sent cancel request")
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(
                f"_cancel_runtime_control: cancel_goal_async failed: {ex!r}")
        self._runtime_goal_handle = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSupervisorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
