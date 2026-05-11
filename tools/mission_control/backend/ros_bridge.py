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
        self._control_get_state_client = None
        self._ready_event = threading.Event()
        # Cache for the last-known control_node lifecycle state, with
        # a short TTL so the telemetry tick (1 Hz default) doesn't
        # storm the get_state service. Stored as
        # (state_id_or_None, monotonic_timestamp). state_id == 3 is
        # PRIMARY_STATE_ACTIVE per lifecycle_msgs.
        self._control_state_cache: tuple[Optional[int], float] = (None, 0.0)
        self._control_state_cache_ttl_s: float = 0.5

    def _spin_up(self) -> None:
        # Lazy import — keeps module load cheap when ROS isn't present
        # (e.g. CI environments running unit tests).
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from dv_msgs.action import StartMission
        from lifecycle_msgs.srv import GetState

        self._rclpy = rclpy
        self._StartMission = StartMission
        self._GetState = GetState

        rclpy.init()
        self._node = Node("mission_control_backend_ros_bridge")
        self._start_mission_client = ActionClient(
            self._node,
            StartMission,
            "/start_mission",
        )
        # Lifecycle state probe — used by is_pipeline_active() so the
        # frontend's PIPELINE RUNNING indicator reflects whether the
        # autonomy is actually configured+active, not just whether the
        # supervisor (which is always up) can be reached.
        self._control_get_state_client = self._node.create_client(
            GetState,
            "/control_node/get_state",
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
        self._control_get_state_client = None
        self._control_state_cache = (None, 0.0)
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

    def is_pipeline_active(self) -> bool:
        """Return True iff control_node is in lifecycle state `active`.

        The right signal for the UI's "PIPELINE RUNNING" indicator:
        the management trio (mode_manager / mission_control /
        sim_supervisor) is always active once the container is up,
        so `is_action_server_available()` is a poor proxy — it stays
        true even after a `start_mission("")` tear-down. control_node
        is the leaf consumer of the bring-up chain, so checking its
        state is the accurate "autonomy is doing work" signal.

        Cached for `_control_state_cache_ttl_s` (500 ms by default)
        so the 1 Hz telemetry tick doesn't storm the service.
        """
        if self._control_get_state_client is None:
            return False

        now = time.monotonic()
        cached_state, cached_ts = self._control_state_cache
        if cached_state is not None and (now - cached_ts) < self._control_state_cache_ttl_s:
            return cached_state == 3  # PRIMARY_STATE_ACTIVE

        state_id = self._get_control_state_blocking()
        # Cache successful reads only — on a failed read we keep the
        # previous cache value, which means a transient get_state hiccup
        # doesn't flicker the UI. Only refresh the timestamp on success.
        if state_id is not None:
            self._control_state_cache = (state_id, now)
            return state_id == 3

        # No successful read AND no cached value — assume inactive.
        return cached_state == 3 if cached_state is not None else False

    def _get_control_state_blocking(self) -> Optional[int]:
        """Synchronously query /control_node/get_state. Returns the
        state ID (an int from lifecycle_msgs.msg.State.PRIMARY_STATE_*)
        or None if the service is unavailable / call times out."""
        if self._control_get_state_client is None:
            return None
        if not self._control_get_state_client.service_is_ready():
            return None
        future = self._control_get_state_client.call_async(self._GetState.Request())
        deadline = time.monotonic() + 1.0
        while not future.done():
            if time.monotonic() >= deadline:
                self._control_get_state_client.remove_pending_request(future)
                return None
            time.sleep(0.02)
        result = future.result()
        if result is None:
            return None
        return int(result.current_state.id)

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
