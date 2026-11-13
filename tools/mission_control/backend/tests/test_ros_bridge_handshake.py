"""The panel's prepare/GO/stop handshake with the pipeline (ros_bridge).

No ROS here: the pipeline's interface_contract is stubbed and a fake pipeline
answers the bridge's intents with /dv/status, the way mission_control does.
The case behind these tests: a run whose prepare and GO both passed at once on
the previous run's cached RUNNING, so READY and GO went out a millisecond
apart and the car was left armed without GO.
"""

import os
import sys
import threading
import time
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir))

DV_IDLE, DV_PREPARING, DV_READY, DV_RUNNING, DV_FAILED = 0, 1, 2, 3, 6
OFF, READY, GO = 0, 1, 2


def _stub_contract():
    pkg = types.ModuleType("mission_control")
    pkg.__path__ = []
    ic = types.ModuleType("mission_control.interface_contract")
    ic.DV_IDLE, ic.DV_READY, ic.DV_RUNNING, ic.DV_FAILED = (
        DV_IDLE, DV_READY, DV_RUNNING, DV_FAILED)
    ic.SIM_INTENT_OFF, ic.SIM_INTENT_READY, ic.SIM_INTENT_GO = OFF, READY, GO
    ic.TOPIC_DV_STATUS = "/dv/status"
    ic.TOPIC_SIM_ESTOP = "/sim/estop"
    ic.TOPIC_SIM_INTENT = "/sim/intent"
    ic.TOPIC_SIM_MISSION = "/sim/mission"
    ic.mission_id_to_ami_index = lambda mission_id: int(mission_id)
    sys.modules.setdefault("mission_control", pkg)
    sys.modules.setdefault("mission_control.interface_contract", ic)


_stub_contract()
import ros_bridge  # noqa: E402


class Msg:
    def __init__(self, data):
        self.data = data


class Pub:
    """A publisher that hands each message to a callback (the fake pipeline)."""

    def __init__(self, on_publish=None):
        self.sent = []
        self.on_publish = on_publish

    def publish(self, msg):
        self.sent.append(int(msg.data))
        if self.on_publish:
            self.on_publish(int(msg.data))


class FakePipeline:
    """Answers intents with /dv/status after a short delay, on its own thread
    (as the real one does, on its reconcile tick)."""

    def __init__(self, bridge, answers, delay_s=0.05, drop_first=()):
        self.bridge = bridge
        self.answers = answers  # intent -> status it settles at
        self.delay_s = delay_s
        self.drop = set(drop_first)  # intents whose first copy is lost

    def __call__(self, intent):
        if intent in self.drop:
            self.drop.discard(intent)
            return
        status = self.answers.get(intent)
        if status is None:
            return
        threading.Timer(
            self.delay_s, self.bridge._on_dv_status, args=(Msg(status),)
        ).start()


@pytest.fixture
def bridge(monkeypatch):
    monkeypatch.setattr(ros_bridge, "INTENT_RESEND_S", 0.1)
    monkeypatch.setattr(ros_bridge._mission_catalog, "mission_name_to_id",
                        lambda: {"autocross": 3})
    b = ros_bridge.RosBridge()
    b._UInt8 = b._Int32 = Msg
    b._mission_pub = Pub()
    b._intent_pub = Pub()
    return b


def previous_run_left(bridge, status):
    """The cache as the last run left it: a live heartbeat at `status`."""
    bridge._on_dv_status(Msg(status))


def test_prepare_does_not_pass_on_the_last_runs_running(bridge):
    previous_run_left(bridge, DV_RUNNING)
    # The pipeline never answers READY: prepare must time out, not succeed.
    bridge._intent_pub.on_publish = FakePipeline(bridge, {})
    keep_alive = threading.Timer(0.3, bridge._on_dv_status,
                                 args=(Msg(DV_RUNNING),))
    keep_alive.start()
    out = bridge.set_mission("autocross", timeout_s=0.6)
    keep_alive.join()
    assert not out.success


def test_prepare_waits_for_a_fresh_ready(bridge):
    previous_run_left(bridge, DV_READY)  # stale READY must not count either
    bridge._intent_pub.on_publish = FakePipeline(bridge, {READY: DV_READY},
                                                 delay_s=0.2)
    t0 = time.monotonic()
    out = bridge.set_mission("autocross", timeout_s=2.0)
    assert out.success
    assert time.monotonic() - t0 >= 0.2


def test_go_waits_for_a_fresh_running(bridge):
    previous_run_left(bridge, DV_RUNNING)
    bridge._intent_pub.on_publish = FakePipeline(bridge, {GO: DV_RUNNING},
                                                 delay_s=0.2)
    t0 = time.monotonic()
    out = bridge.start_runtime(timeout_s=2.0)
    assert out.success
    assert time.monotonic() - t0 >= 0.2


def test_go_is_sent_again_when_the_first_is_lost(bridge):
    previous_run_left(bridge, DV_READY)
    bridge._intent_pub.on_publish = FakePipeline(
        bridge, {GO: DV_RUNNING}, drop_first={GO})
    out = bridge.start_runtime(timeout_s=2.0)
    assert out.success
    assert bridge._intent_pub.sent.count(GO) >= 2


def test_prepare_fails_fast_on_a_fresh_failure(bridge):
    previous_run_left(bridge, DV_IDLE)
    bridge._intent_pub.on_publish = FakePipeline(bridge, {READY: DV_FAILED})
    t0 = time.monotonic()
    out = bridge.set_mission("autocross", timeout_s=5.0)
    assert not out.success
    assert time.monotonic() - t0 < 1.0


def test_stop_waits_until_the_pipeline_is_disarmed(bridge):
    previous_run_left(bridge, DV_RUNNING)
    bridge._intent_pub.on_publish = FakePipeline(bridge, {OFF: DV_IDLE},
                                                 delay_s=0.2)
    t0 = time.monotonic()
    out = bridge.set_mission("", timeout_s=5.0)
    assert out.success
    assert time.monotonic() - t0 >= 0.2
    assert bridge._intent_pub.sent[0] == OFF


def test_stop_does_not_wait_on_a_silent_pipeline(bridge):
    # Nothing ever received: no heartbeat means nothing is running.
    bridge._intent_pub.on_publish = FakePipeline(bridge, {})
    t0 = time.monotonic()
    out = bridge.set_mission("", timeout_s=5.0)
    assert out.success
    assert time.monotonic() - t0 < 0.5


def test_full_cycle_sends_off_ready_go_in_order(bridge):
    previous_run_left(bridge, DV_RUNNING)
    bridge._intent_pub.on_publish = FakePipeline(
        bridge, {OFF: DV_IDLE, READY: DV_READY, GO: DV_RUNNING})
    assert bridge.stop_mission(timeout_s=2.0).success
    prep, run = bridge.run_mission("autocross", prepare_timeout_s=2.0,
                                   runtime_timeout_s=2.0)
    assert prep.success and run is not None and run.success
    sent = bridge._intent_pub.sent
    # Every READY comes after the last OFF; every GO after the last READY.
    assert max(i for i, x in enumerate(sent) if x == OFF) < \
        min(i for i, x in enumerate(sent) if x == READY)
    assert max(i for i, x in enumerate(sent) if x == READY) < \
        min(i for i, x in enumerate(sent) if x == GO)


def test_as_snapshot_reports_fresh_heartbeats_and_drops_stale_ones():
    b = ros_bridge.RosBridge()
    assert b.as_snapshot() == {"dv_status": None, "as_state": None}
    b._on_dv_status(Msg(5))
    b._on_as_state(Msg(1))
    assert b.as_snapshot() == {"dv_status": 5, "as_state": 1}
    b._as_state_stamp -= ros_bridge.DV_STATUS_STALE_S + 1.0
    assert b.as_snapshot() == {"dv_status": 5, "as_state": None}
