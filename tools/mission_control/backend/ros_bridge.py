"""
ROS 2 bridge for the Mission Control FastAPI backend.

Hosts a daemon-thread rclpy executor with a single Node that owns the
StartMission action client targeting sim_supervisor_node. Exposes a
sync API the FastAPI handlers can call without thinking about rclpy:

    bridge = RosBridge.get()
    result = bridge.start_mission("trackdrive", timeout_s=240.0)
    if result.ready:
        ...

Why a separate module:

  • FastAPI/uvicorn run a single asyncio event loop. rclpy needs its
    own executor running on a different thread; mixing the two via
    `asyncio.run` deadlocks because both want to drive the main loop.
  • A daemon thread + MultiThreadedExecutor isolates rclpy entirely.
    The sync API methods submit work via ActionClient.send_goal_async
    and block on rclpy.task.Future objects without touching asyncio.
  • Singleton because rclpy.init() can only be called once per process,
    and there's no value in multiple Node instances for one backend.

Lifecycle:

  • RosBridge.start()    — call once, in FastAPI's startup handler.
                            Spawns the daemon thread, blocks until rclpy
                            is initialised and the Node is ready.
  • RosBridge.shutdown() — call once, in FastAPI's shutdown handler.
                            Cleanly tears down the executor + Node and
                            joins the thread.
  • RosBridge.get()      — return the started singleton. Raises if
                            start() hasn't been called.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class StartMissionOutcome:
    """What the FastAPI handler gets back from bridge.start_mission()."""

    ready: bool
    message: str


class RosBridge:
    """Singleton rclpy host for the FastAPI backend. See module docstring."""

    _instance: Optional["RosBridge"] = None
    _lock = threading.Lock()

    # ------------------------------------------------------------------
    # Singleton management
    # ------------------------------------------------------------------
    @classmethod
    def start(cls) -> "RosBridge":
        """Initialise the singleton if needed and return it. Idempotent."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
                cls._instance._spin_up()
            return cls._instance

    @classmethod
    def get(cls) -> "RosBridge":
        with cls._lock:
            if cls._instance is None:
                raise RuntimeError(
                    "RosBridge.start() has not been called yet. "
                    "FastAPI startup handler must call it once."
                )
            return cls._instance

    @classmethod
    def shutdown(cls) -> None:
        with cls._lock:
            if cls._instance is None:
                return
            cls._instance._spin_down()
            cls._instance = None

    # ------------------------------------------------------------------
    # Internal — these run on the rclpy daemon thread
    # ------------------------------------------------------------------
    def __init__(self) -> None:
        # Imports deferred until start() actually runs, so unit tests
        # that don't touch ROS aren't forced to have rclpy installed.
        self._rclpy = None
        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._start_mission_client = None
        self._ready_event = threading.Event()

    def _spin_up(self) -> None:
        # Lazy import — keeps module load cheap when ROS isn't present
        # (e.g. CI environments running unit tests).
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from dv_msgs.action import StartMission

        self._rclpy = rclpy
        self._StartMission = StartMission

        rclpy.init()
        self._node = Node("mission_control_backend_ros_bridge")
        self._start_mission_client = ActionClient(
            self._node,
            StartMission,
            "/start_mission",
        )

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        def _spin_loop() -> None:
            try:
                self._ready_event.set()
                self._executor.spin()
            except Exception:
                logger.exception("RosBridge spin loop crashed")
            finally:
                logger.info("RosBridge spin loop exited")

        self._spin_thread = threading.Thread(
            target=_spin_loop, name="RosBridge-spin", daemon=True,
        )
        self._spin_thread.start()
        # Block briefly until the executor is actually running so the
        # first .start_mission() call doesn't race startup.
        self._ready_event.wait(timeout=5.0)
        logger.info("RosBridge ready (Node spinning)")

    def _spin_down(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._rclpy is not None:
            try:
                self._rclpy.try_shutdown()
            except Exception:
                # rclpy may already be shut down by an outer handler;
                # not worth raising in a teardown path.
                pass
        self._executor = None
        self._spin_thread = None
        self._node = None
        self._start_mission_client = None
        self._rclpy = None

    # ------------------------------------------------------------------
    # Sync API for FastAPI handlers
    # ------------------------------------------------------------------
    def is_action_server_available(self, timeout_s: float = 0.0) -> bool:
        """Probe sim_supervisor_node's start_mission action server.

        timeout_s = 0 means "ask once and return immediately"; positive
        values block up to that long waiting for the server to come up.
        """
        if self._start_mission_client is None:
            return False
        return self._start_mission_client.wait_for_server(timeout_sec=timeout_s)

    def start_mission(
        self,
        mission: str,
        timeout_s: float = 270.0,
    ) -> StartMissionOutcome:
        """Send a StartMission goal to sim_supervisor_node, block until
        the action returns a result, and translate it to an HTTP-friendly
        outcome.

        On the rclpy thread:
          1. wait_for_server (5 s cap) — fail fast if the supervisor
             isn't there.
          2. send_goal_async + spin to acceptance.
          3. get_result_async + spin to terminal status.

        Total wall-clock is dominated by the autonomy stack's
        configure+activate (Numba JIT for cone_detection_node ≈ 10–20 s
        on Apple Silicon Docker), so callers should expect this to
        block for tens of seconds.
        """
        if self._start_mission_client is None:
            return StartMissionOutcome(
                ready=False,
                message="ros_bridge not started; FastAPI startup did not run",
            )

        if not self._start_mission_client.wait_for_server(timeout_sec=5.0):
            return StartMissionOutcome(
                ready=False,
                message=(
                    "/start_mission action server unavailable; "
                    "is sim_supervisor_node active?"
                ),
            )

        goal = self._StartMission.Goal()
        goal.mission = mission

        send_future = self._start_mission_client.send_goal_async(goal)

        deadline = time.monotonic() + timeout_s
        while not send_future.done():
            if time.monotonic() >= deadline:
                return StartMissionOutcome(
                    ready=False,
                    message="goal acceptance timed out",
                )
            time.sleep(0.05)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return StartMissionOutcome(
                ready=False,
                message="sim_supervisor rejected the StartMission goal",
            )

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if time.monotonic() >= deadline:
                return StartMissionOutcome(
                    ready=False,
                    message=(
                        f"StartMission did not return within "
                        f"{timeout_s:.0f} s"
                    ),
                )
            time.sleep(0.05)

        wrapper = result_future.result()
        if wrapper is None or wrapper.result is None:
            return StartMissionOutcome(
                ready=False,
                message="StartMission completed without a result payload",
            )
        result = wrapper.result
        return StartMissionOutcome(
            ready=bool(result.ready),
            message=str(result.message),
        )
