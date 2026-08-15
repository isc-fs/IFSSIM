#!/usr/bin/env python3
"""Pre-warm the numba JIT cache for the pipeline's lazily-compiled hot paths.

Why this exists
---------------
cone_detection (RANSAC / ground removal) and path_planning (fsd_path_planning /
FaSTTUBe) JIT-compile their hot paths the first time they run — which, for the
lifecycle nodes, is the first sensor callback AFTER a mission is activated. The
path_planning cold compile alone is ~37 s (measured; the warm-cache call is
<1 s). On a cold /numba_cache that long compile overruns the node's activation /
liveness window and the process gets SIGKILL'd (-9) mid-compile. Because the
kill lands mid-compile, the cache is never fully written, so the next activation
is still cold and dies the same way — a crash-loop that never escapes, which
surfaces to the operator as "did not reach DV_READY within 270s" (a stale
DV_FAILED) or a car that reaches DRIVING then stalls with no /Path.

The fix is to do the compile HERE, at container startup, untimed and before any
mission can activate. NUMBA_CACHE_DIR points at the persistent /numba_cache
volume, so a cold cache pays this cost once (~40-75 s) and every boot after —
and every mission activation — is a sub-second cache hit.

Best-effort: any failure is logged and swallowed. Worst case the pipeline pays
the original cold-JIT cost on first activation; it never makes things worse.
"""
import time

_T0 = time.time()


def _elapsed() -> str:
    return f"{time.time() - _T0:.1f}s"


# --- cone_detection: RANSAC + ground removal njit kernels ------------------
try:
    from cone_detection.cone_detection import warmup_numba_functions
    warmup_numba_functions(also_warm_two_param=True)
    print(f"cone_detection numba warmed ({_elapsed()})", flush=True)
except Exception as exc:  # noqa: BLE001
    print(f"cone_detection warmup skipped: {exc!r}", flush=True)


# --- path_planning: fsd_path_planning / FaSTTUBe (the ~37 s cold compile) ---
# Drive the planner with a trivial synthetic corridor so every njit kernel on
# the calculate_path_in_global_frame path compiles. The core numba code is
# largely mission-agnostic, but instantiate the missions the team actually
# runs so any per-mission specialisation is covered too. Degenerate synthetic
# geometry can raise inside the planner AFTER the JIT has run — that's fine, we
# only care about triggering compilation, so per-mission errors are swallowed.
try:
    import numpy as np
    from fsd_path_planning import MissionTypes, PathPlanner

    synth_cones = [np.zeros((0, 2)) for _ in range(5)]
    synth_cones[1] = np.array([[i * 2.0, -2.0] for i in range(12)])  # right/yellow
    synth_cones[2] = np.array([[i * 2.0, 2.0] for i in range(12)])   # left/blue
    car_pos = np.array([0.0, 0.0])
    car_dir = np.array([1.0, 0.0])

    for mission in (MissionTypes.trackdrive, MissionTypes.autocross):
        try:
            planner = PathPlanner(mission)
            planner.calculate_path_in_global_frame(synth_cones, car_pos, car_dir)
        except Exception:  # noqa: BLE001
            pass  # JIT already ran; synthetic geometry may be degenerate
    print(f"path_planning numba warmed ({_elapsed()})", flush=True)
except Exception as exc:  # noqa: BLE001
    print(f"path_planning warmup skipped: {exc!r}", flush=True)


print(f"numba pre-warm complete in {_elapsed()}", flush=True)
