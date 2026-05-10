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

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty as EmptyMsg, Float32

from fs_msgs.msg import ControlCommand
from dv_msgs.action import StartMission, RuntimeControl
from dv_msgs.srv import ActivateMode

from sim_supervisor.odometry import OdometryFilter


# /odom publication rate. 100 Hz target — gives the 40 Hz controller
# fresh data every tick with margin, doesn't burn the CPU. Decoupled
# from the IMU subscription rate (which is the BMI088's native ~400 Hz).
ODOM_PUBLISH_HZ: float = 100.0


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
        # Phase 1 scope: topic only. We do NOT broadcast the
        # odom→base_link TF here because slam_node still owns that
        # broadcaster — two publishers writing to the same TF parent→
        # child causes last-writer-wins flicker between dead-reckoning
        # (us) and SLAM-corrected (slam_node). Phase 2 (separate
        # branch) hands the TF to us and slam_node starts publishing
        # map→odom drift correction instead. Until then, /odom.pose
        # and the TF tree's odom→base_link describe slightly different
        # things; consumers that care about absolute pose should keep
        # using TF lookups (which see slam's estimate), and consumers
        # that care about high-rate velocity should switch to
        # /odom.twist (which we own).
        self._odom_filter = OdometryFilter()
        self._odom_pub = self.create_lifecycle_publisher(
            Odometry, "/odom", 50,
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
        if self._sub_imu is not None:
            self.destroy_subscription(self._sub_imu)
            self._sub_imu = None
        if self._sub_rpm is not None:
            self.destroy_subscription(self._sub_rpm)
            self._sub_rpm = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup: tearing down I/O")
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
        """Drive the filter's predict step."""
        if self._odom_filter is None:
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

    def _publish_odom(self) -> None:
        """Timer-driven /odom emission. Skips while the filter is
        still in stationary calibration (first ~3 s after activate)."""
        if self._odom_filter is None or self._odom_pub is None:
            return
        if not self._odom_filter.is_calibrated():
            return

        s = self._odom_filter.state
        now = self.get_clock().now().to_msg()

        if not self._odom_first_publish_logged:
            self.get_logger().info(
                "/odom first publish — IMU+RPM filter calibrated")
            self._odom_first_publish_logged = True

        # nav_msgs/Odometry: pose in header.frame_id (odom),
        # twist in child_frame_id (base_link). REP-103 axes.
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = s.x
        msg.pose.pose.position.y = s.y
        msg.pose.pose.position.z = 0.0
        # 2D yaw → quaternion
        half = 0.5 * s.yaw
        msg.pose.pose.orientation.w = float(np.cos(half))
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(np.sin(half))
        msg.twist.twist.linear.x = s.vx
        msg.twist.twist.linear.y = s.vy
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = s.yaw_rate
        self._odom_pub.publish(msg)

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
        else:
            self.get_logger().error(
                f"StartMission relay: mission {mission!r} failed: "
                f"{inner_result.message}")
            goal_handle.abort()

        return result


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
