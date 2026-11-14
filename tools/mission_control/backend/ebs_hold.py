"""Is the car held still under EBS? The gate before the EKF calibrates.

odometry_filter calibrates its IMU bias over a stationary window that starts
when the autonomy is activated, so the car must already be still when
/api/event/start activates it. EBS latched is not enough: loadTrack teleports
the car onto the start gate, and it bounces and settles on its suspension for
about half a second (up to ~0.6 m/s) with the EBS on. A first run hides that
behind the pipeline's 10-20 s configure; a second run right after the first
configures in well under a second and used to start calibrating mid-bounce.

Pure Python, no sim or FastAPI: the caller passes the readers.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

# |velocity| below this counts as still. The sim reads 0.000 on a parked car;
# the post-teleport settle tails off through ~0.01.
STILL_SPEED_MPS = 0.01
# How long the car must stay still (and latched) before calibration starts.
SETTLE_S = 0.5
# Give up after this long: the car is rolling or the EBS won't latch.
SETTLE_TIMEOUT_S = 10.0
POLL_S = 0.1


@dataclass
class HoldCheck:
    ok: bool
    waited_s: float
    latched: Optional[bool]          # the last latch reading (None: unreadable)
    last_speed: Optional[float]      # the last speed reading (None: unreadable)
    speeds: List[float] = field(default_factory=list)
    relatched: int = 0

    def describe(self) -> str:
        spd = "?" if self.last_speed is None else f"{self.last_speed:.3f}"
        return (f"held={self.ok} after {self.waited_s:.1f}s | EBS_latched={self.latched} "
                f"| speed={spd} m/s | max={max(self.speeds, default=0.0):.3f} m/s "
                f"| relatched={self.relatched}")


def wait_until_held(
    read_speed: Callable[[], float],
    read_latched: Callable[[], bool],
    relatch: Callable[[], None],
    *,
    still_mps: float = STILL_SPEED_MPS,
    settle_s: float = SETTLE_S,
    timeout_s: float = SETTLE_TIMEOUT_S,
    poll_s: float = POLL_S,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> HoldCheck:
    """Wait until the EBS is latched and the car has been still for `settle_s`.

    An unlatched EBS is latched again (`relatch`) and the settle restarts; an
    unreadable sample also restarts it. Returns ok=False after `timeout_s`.
    """
    start = clock()
    still_since: Optional[float] = None
    speeds: List[float] = []
    latched: Optional[bool] = None
    speed: Optional[float] = None
    relatched = 0
    while True:
        now = clock()
        try:
            latched = bool(read_latched())
        except Exception:
            latched = None
        try:
            speed = float(read_speed())
            if math.isnan(speed):
                speed = None
        except Exception:
            speed = None
        if speed is not None:
            speeds.append(speed)

        if latched is False:
            relatch()
            relatched += 1
        if latched and speed is not None and speed < still_mps:
            if still_since is None:
                still_since = now
            if now - still_since >= settle_s:
                return HoldCheck(True, now - start, latched, speed, speeds, relatched)
        else:
            still_since = None

        if now - start >= timeout_s:
            return HoldCheck(False, now - start, latched, speed, speeds, relatched)
        sleep(poll_s)
