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

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Empty as EmptyMsg
from fs_msgs.msg import ControlCommand
from dv_msgs.action import StartMission, RuntimeControl
from dv_msgs.srv import ActivateMode


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

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate")
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
        self._current_mission = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        return TransitionCallbackReturn.SUCCESS

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
