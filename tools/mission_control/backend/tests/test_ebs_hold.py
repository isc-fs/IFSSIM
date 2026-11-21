"""The gate before the EKF calibrates: EBS latched and the car still.

The case behind these tests: a second autonomous run started right after the
first. The track load teleports the car onto the start gate, it bounces for
~0.5 s under EBS, the warm pipeline activates within a second, and the EKF
calibrated on a moving car (session log: max speed 0.102 m/s).
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir))

import ebs_hold  # noqa: E402
from sim_client import SimConnection  # noqa: E402


class FakeCar:
    """A sim clock and a speed profile; `sleep` advances the clock."""

    def __init__(self, speeds_at, latched_at=lambda t: True):
        self.t = 0.0
        self.speeds_at = speeds_at
        self.latched_at = latched_at
        self.relatches = []

    def clock(self):
        return self.t

    def sleep(self, dt):
        self.t += dt

    def speed(self):
        v = self.speeds_at(self.t)
        if isinstance(v, Exception):
            raise v
        return v

    def latched(self):
        return self.latched_at(self.t)

    def relatch(self):
        self.relatches.append(self.t)

    def wait(self, **kw):
        return ebs_hold.wait_until_held(
            self.speed, self.latched, self.relatch,
            clock=self.clock, sleep=self.sleep, **kw)


def _settle_bounce(t):
    # The profile seen after loadTrack on the bench (10 Hz trace): up to
    # 0.65 m/s, tailing off, still from ~0.6 s.
    return [0.653, 0.154, 0.173, 0.017, 0.007, 0.001][int(t * 10)] if t < 0.6 else 0.0


def test_waits_out_the_post_teleport_bounce():
    car = FakeCar(_settle_bounce)
    hold = car.wait()
    assert hold.ok
    # Not before the car stopped (~0.5 s) plus the settle time.
    assert hold.waited_s >= 0.5 + ebs_hold.SETTLE_S - 1e-9
    assert max(hold.speeds) > 0.6
    assert hold.last_speed < ebs_hold.STILL_SPEED_MPS


def test_a_parked_car_passes_after_the_settle_time():
    car = FakeCar(lambda t: 0.0)
    hold = car.wait()
    assert hold.ok
    assert abs(hold.waited_s - ebs_hold.SETTLE_S) < 1e-6
    assert car.relatches == []


def test_motion_restarts_the_settle():
    # Still, a nudge at 0.3 s, still again: the settle counts from after it.
    car = FakeCar(lambda t: 0.2 if 0.3 <= t < 0.4 else 0.0)
    hold = car.wait()
    assert hold.ok
    assert hold.waited_s >= 0.4 + ebs_hold.SETTLE_S - 1e-9


def test_a_rolling_car_fails_after_the_timeout():
    car = FakeCar(lambda t: 0.102)
    hold = car.wait(timeout_s=2.0)
    assert not hold.ok
    assert hold.waited_s >= 2.0
    assert hold.latched is True
    assert abs(hold.last_speed - 0.102) < 1e-9
    assert "held=False" in hold.describe()


def test_an_unlatched_ebs_is_latched_again_before_it_counts():
    # The latch takes a tick to land (activateEbs runs on the game thread).
    car = FakeCar(lambda t: 0.0, latched_at=lambda t: t >= 0.25)
    hold = car.wait()
    assert hold.ok
    assert car.relatches and car.relatches[0] == 0.0
    assert hold.relatched == len(car.relatches)
    assert hold.waited_s >= 0.25 + ebs_hold.SETTLE_S - 1e-9


def test_an_ebs_that_never_latches_fails():
    car = FakeCar(lambda t: 0.0, latched_at=lambda t: False)
    hold = car.wait(timeout_s=1.0)
    assert not hold.ok
    assert hold.latched is False


def test_unreadable_samples_never_count_as_still():
    car = FakeCar(lambda t: RuntimeError("rpc blip"))
    hold = car.wait(timeout_s=1.0)
    assert not hold.ok
    assert hold.last_speed is None


def test_nan_speed_is_unreadable():
    car = FakeCar(lambda t: float("nan"))
    assert not car.wait(timeout_s=1.0).ok


def _client_replying(reply):
    c = SimConnection.__new__(SimConnection)
    c._cmd = lambda cmd, timeout=None: reply
    return c


def test_latch_reader():
    assert _client_replying("true").is_ebs_latched() is True
    assert _client_replying("false").is_ebs_latched() is False
    # A lost reply ("" from _cmd) is not "released".
    for reply in ("", "unknown method"):
        try:
            _client_replying(reply).is_ebs_latched()
        except RuntimeError:
            continue
        raise AssertionError(f"{reply!r} should raise")
