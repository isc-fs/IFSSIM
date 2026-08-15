"""
ROS 2 bridge for the Mission Control FastAPI backend.

Post-action-decomposition the backend is the sim "operator panel" — the
AMI board + RES buttons stand-in. It does NOT talk to mission_control
directly anymore; it drives the sim uDV emulator (sim_supervisor) over
the sim panel topics, exactly the way the physical AMI/RES drive the real
uDV, and watches the pipeline's /dv/status handshake. mission_control
only ever sees the stock uDV surface.

Hosts a daemon-thread rclpy executor with a single Node that owns:

  * /sim/mission, /sim/intent, /sim/estop publishers (the panel)
  * /dv/status subscriber (the prepare/run handshake the panel waits on)
  * control_node/get_state client (pipeline-active probe)

Sync API for FastAPI handlers (public method names unchanged):

    bridge = RosBridge.get()
    prep = bridge.set_mission("autocross")   # arm + wait for DV_READY
    if prep.success:
        bridge.start_runtime()               # GO + wait for DV_RUNNING
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)

import mission_catalog as _mission_catalog

# Stock-typed interface contract (installed pipeline package; importable
# at runtime in the overlay, same as the rest of the ROS deps below).
from mission_control.interface_contract import (
    DV_FAILED,
    DV_READY,
    DV_RUNNING,
    SIM_INTENT_GO,
    SIM_INTENT_OFF,
    SIM_INTENT_READY,
    TOPIC_DV_STATUS,
    TOPIC_SIM_ESTOP,
    TOPIC_SIM_INTENT,
    TOPIC_SIM_MISSION,
    mission_id_to_ami_index,
)


@dataclass
class SetMissionOutcome:
    """Result of bridge.set_mission() (prepare phase)."""

    success: bool
    message: str

    @property
    def ready(self) -> bool:
        """Alias for older call sites that checked `.ready`."""
        return self.success


@dataclass
class RuntimeControlOutcome:
    """Result of bridge.start_runtime() (go phase)."""

    success: bool
    message: str


# Backward-compatible alias
StartMissionOutcome = SetMissionOutcome


class RosBridge:
    """Singleton rclpy host for the FastAPI backend (the sim panel)."""

    _instance: Optional["RosBridge"] = None
    _lock = threading.Lock()

    @classmethod
    def start(cls) -> "RosBridge":
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

    def __init__(self) -> None:
        self._rclpy = None
        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._mission_pub = None
        self._intent_pub = None
        self._estop_pub = None
        self._control_get_state_client = None
        self._ready_event = threading.Event()
        self._control_state_cache: tuple[Optional[int], float] = (None, 0.0)
        self._control_state_cache_ttl_s: float = 0.5
        # Latest /dv/status byte (set on the executor thread; int reads are
        # atomic under the GIL so no lock needed).
        self._dv_status: Optional[int] = None
        self._bridge_lock = threading.Lock()

    def _spin_up(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from std_msgs.msg import Bool, Int32, UInt8
        from lifecycle_msgs.srv import GetState

        self._rclpy = rclpy
        self._Bool = Bool
        self._Int32 = Int32
        self._UInt8 = UInt8
        self._GetState = GetState

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        rclpy.init()
        self._node = Node("mission_control_backend_ros_bridge")
        self._mission_pub = self._node.create_publisher(
            Int32, TOPIC_SIM_MISSION, latched)
        self._intent_pub = self._node.create_publisher(
            UInt8, TOPIC_SIM_INTENT, latched)
        self._estop_pub = self._node.create_publisher(
            Bool, TOPIC_SIM_ESTOP, latched)
        self._node.create_subscription(
            UInt8, TOPIC_DV_STATUS, self._on_dv_status, latched)
        self._control_get_state_client = self._node.create_client(
            GetState, "/control_node/get_state")

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
                pass
        self._executor = None
        self._spin_thread = None
        self._node = None
        self._mission_pub = None
        self._intent_pub = None
        self._estop_pub = None
        self._control_get_state_client = None
        self._control_state_cache = (None, 0.0)
        self._dv_status = None
        self._rclpy = None

    # ------------------------------------------------------------------
    def _on_dv_status(self, msg) -> None:
        self._dv_status = int(msg.data)

    def _publish_intent(self, intent: int) -> None:
        if self._intent_pub is not None:
            self._intent_pub.publish(self._UInt8(data=int(intent)))

    def _wait_for_dv_status(
        self, targets: set[int], timeout_s: float,
        fail_on: Optional[set[int]] = None,
    ) -> bool:
        """Poll the cached /dv/status (updated on the spin thread)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self._dv_status
            if status in targets:
                return True
            if fail_on and status in fail_on:
                return False
            time.sleep(0.05)
        return False

    def is_action_server_available(self, timeout_s: float = 0.0) -> bool:
        """Back-compat probe — now "is the sim panel bridge up?"."""
        return self._node is not None

    def is_pipeline_active(self) -> bool:
        """Return True iff control_node is in lifecycle state `active`."""
        if self._control_get_state_client is None:
            return False
        now = time.monotonic()
        cached_state, cached_ts = self._control_state_cache
        if cached_state is not None and \
                (now - cached_ts) < self._control_state_cache_ttl_s:
            return cached_state == 3
        state_id = self._get_control_state_blocking()
        if state_id is not None:
            self._control_state_cache = (state_id, now)
            return state_id == 3
        return cached_state == 3 if cached_state is not None else False

    def _get_control_state_blocking(self) -> Optional[int]:
        if self._control_get_state_client is None:
            return None
        if not self._control_get_state_client.service_is_ready():
            return None
        future = self._control_get_state_client.call_async(
            self._GetState.Request())
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

    def set_mission(
        self, mission: str, timeout_s: float = 270.0,
    ) -> SetMissionOutcome:
        """Phase 1 — select the mission + arm (READY), wait for DV_READY."""
        if self._mission_pub is None:
            return SetMissionOutcome(
                success=False,
                message="ros_bridge not started; FastAPI startup did not run",
            )

        if mission == "":
            # Tear down — disarm.
            self._publish_intent(SIM_INTENT_OFF)
            return SetMissionOutcome(success=True, message="torn down")

        mission_id = _mission_catalog.mission_name_to_id().get(mission)
        if mission_id is None:
            return SetMissionOutcome(
                success=False,
                message=(
                    f"unknown pipeline mission {mission!r}; expected one of "
                    f"{sorted(_mission_catalog.mission_name_to_id().keys())}"
                ),
            )

        ami = mission_id_to_ami_index(mission_id)
        self._mission_pub.publish(self._Int32(data=int(ami)))
        self._publish_intent(SIM_INTENT_READY)

        if self._wait_for_dv_status(
                {DV_READY, DV_RUNNING}, timeout_s, fail_on={DV_FAILED}):
            return SetMissionOutcome(
                success=True, message=f"{mission} prepared (DV_READY)")
        return SetMissionOutcome(
            success=False,
            message=f"{mission} did not reach DV_READY within {timeout_s:.0f}s",
        )

    def start_runtime(self, timeout_s: float = 60.0) -> RuntimeControlOutcome:
        """Phase 2 — GO. Drives the emulator's RES go; waits for DV_RUNNING.

        Control commands then flow from mission_control on /ctrl/cmd; the
        emulator relays them to /fsds/control_command for the UE5 bridge.
        """
        if self._intent_pub is None:
            return RuntimeControlOutcome(
                success=False, message="ros_bridge not started")
        self._publish_intent(SIM_INTENT_GO)
        if self._wait_for_dv_status(
                {DV_RUNNING}, timeout_s, fail_on={DV_FAILED}):
            return RuntimeControlOutcome(
                success=True, message="mission running (DV_RUNNING)")
        return RuntimeControlOutcome(
            success=False,
            message=f"did not reach DV_RUNNING within {timeout_s:.0f}s",
        )

    def cancel_runtime(self, timeout_s: float = 5.0) -> None:
        """Drop out of the run (disarm). The reconciler tears autonomy down."""
        with self._bridge_lock:
            self._publish_intent(SIM_INTENT_OFF)

    def stop_mission(self, timeout_s: float = 270.0) -> SetMissionOutcome:
        """Tear down: disarm the panel (intent OFF)."""
        self.cancel_runtime()
        return self.set_mission("", timeout_s=timeout_s)

    def run_mission(
        self, mission: str,
        prepare_timeout_s: float = 270.0,
        runtime_timeout_s: float = 60.0,
    ) -> tuple[SetMissionOutcome, Optional[RuntimeControlOutcome]]:
        """Prepare then immediately go (no EBS handling)."""
        prep = self.set_mission(mission, timeout_s=prepare_timeout_s)
        if not prep.success:
            return prep, None
        runtime = self.start_runtime(timeout_s=runtime_timeout_s)
        return prep, runtime

    def start_mission(
        self, mission: str, timeout_s: float = 270.0,
    ) -> SetMissionOutcome:
        """Prepare only, or tear down when mission is ''."""
        if mission == "":
            return self.stop_mission(timeout_s=timeout_s)
        return self.set_mission(mission, timeout_s=timeout_s)
