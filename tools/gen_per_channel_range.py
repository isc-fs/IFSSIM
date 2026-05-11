#!/usr/bin/env python3
"""tools/gen_per_channel_range.py — generate PerChannelMaxRangeM arrays
for settings.json from the Hesai ATX_S01 datasheet.

Real LiDARs have per-beam laser-power variance (datasheet App. A.1.1):
outer beams reach less far than central beams. The ATX_S01 ranges from
198 m (centre, V≈0°) down to 60 m (bottom edges) and 90 m (top edges).

The simulator caps at MaxRange=30 m (settings.json) for performance, so
this script *rescales* the datasheet shape to fit the sim budget. The
default linear remap [60, 198] m → [25, 30] m preserves the relative
ordering (centre rings reach further than edge rings) while keeping the
floor well above the FS cone-detection working range.

Usage:
  python3 tools/gen_per_channel_range.py            # datasheet, default remap
  python3 tools/gen_per_channel_range.py --pretty   # 8-per-line output
  python3 tools/gen_per_channel_range.py --param    # parametric quadratic
                                                    #   (legacy fallback)
"""
from __future__ import annotations

import argparse
import json
import sys

# Hesai ATX_S01 datasheet, Appendix A.1.2 "Sorted by Ring IDs", 2025-07-10
# revision A07. Each tuple is (V_deg, max_range_m). Pixel-enhancement
# channels (rings whose Max column is "-") are omitted: they share the
# same firing envelope as the nearest normal channel and would just be
# duplicates if listed. The interpolation in datasheet_array() handles
# the gaps by linear-interpolating across V°.
ATX_S01_RING_DATA: list[tuple[float, float]] = [
    (5.92, 90),  (5.52, 90),  (5.12, 90),
    (4.72, 120), (4.52, 120), (4.32, 120), (4.12, 120),
    (3.91, 150), (3.71, 150),
    (3.51, 198), (3.31, 198), (3.11, 198),
    (2.90, 198), (2.71, 198), (2.50, 198), (2.30, 198),
    (2.10, 198), (1.90, 198), (1.70, 198), (1.50, 198),
    (1.30, 198), (1.10, 198), (0.90, 198), (0.70, 198),
    (0.50, 198), (0.30, 198), (0.10, 198),
    (-0.10, 198),
    (-0.30, 150), (-0.50, 150), (-0.70, 150),
    (-0.90, 120), (-1.10, 120),
    (-1.30, 90), (-1.50, 90), (-1.70, 90),
    (-1.90, 60), (-2.11, 60), (-2.31, 60), (-2.51, 60),
    (-2.71, 60), (-2.91, 60), (-3.11, 60), (-3.32, 60),
    (-3.52, 60), (-3.72, 60), (-3.92, 60), (-4.13, 60),
    (-4.53, 60), (-4.93, 60), (-5.33, 60), (-5.73, 60),
    (-6.12, 60), (-6.52, 60), (-6.92, 60), (-7.32, 60),
    (-7.70, 60), (-8.10, 60), (-8.51, 60), (-8.90, 60),
    (-9.46, 60), (-10.27, 60), (-11.23, 60), (-12.41, 60),
]


def lerp_datasheet(v_deg: float) -> float:
    """Linearly interpolate the datasheet max_range_m at V°. The ring
    table is in descending-V° order; we walk it once."""
    rings = ATX_S01_RING_DATA  # descending V
    if v_deg >= rings[0][0]:
        return rings[0][1]
    if v_deg <= rings[-1][0]:
        return rings[-1][1]
    for i in range(len(rings) - 1):
        v_hi, r_hi = rings[i]
        v_lo, r_lo = rings[i + 1]
        if v_lo <= v_deg <= v_hi:
            t = (v_deg - v_lo) / max(1e-9, v_hi - v_lo)
            return r_lo + t * (r_hi - r_lo)
    return rings[-1][1]  # unreachable


def datasheet_array(num_channels: int, v_lower_deg: float, v_upper_deg: float,
                    sim_floor_m: float, sim_max_m: float) -> list[float]:
    """Sample the datasheet's V → max_range curve at the simulator's
    uniform V grid, then linearly remap [datasheet_floor, datasheet_peak]
    → [sim_floor, sim_max] so the values fit under the sim's global
    MaxRange while preserving the relative falloff."""
    ds_min = min(r for _, r in ATX_S01_RING_DATA)
    ds_max = max(r for _, r in ATX_S01_RING_DATA)
    out: list[float] = []
    for i in range(num_channels):
        v_i = v_lower_deg + i * (v_upper_deg - v_lower_deg) / max(1, num_channels - 1)
        ds = lerp_datasheet(v_i)
        t = (ds - ds_min) / max(1e-9, ds_max - ds_min)  # 0 at edges, 1 at peak
        out.append(round(sim_floor_m + t * (sim_max_m - sim_floor_m), 2))
    return out


def parametric(num_channels: int, v_lower_deg: float, v_upper_deg: float,
               max_range_m: float, edge_falloff_pct: float) -> list[float]:
    """Quadratic falloff from centre. Legacy fallback for non-ATX_S01
    LiDARs or for sanity-checking the datasheet-shaped array."""
    out = []
    v_centre = (v_lower_deg + v_upper_deg) / 2.0
    half_span = max(abs(v_upper_deg - v_centre), abs(v_lower_deg - v_centre))
    falloff = edge_falloff_pct / 100.0
    for i in range(num_channels):
        v_i = v_lower_deg + i * (v_upper_deg - v_lower_deg) / max(1, num_channels - 1)
        dist = abs(v_i - v_centre)
        norm_dist = dist / max(1e-9, half_span)
        scale = 1.0 - falloff * (norm_dist ** 2)
        out.append(round(max_range_m * scale, 2))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--num-channels", type=int, default=116,
                    help="Hesai ATX_S01 = 116")
    ap.add_argument("--v-lower-deg", type=float, default=-12.4)
    ap.add_argument("--v-upper-deg", type=float, default=5.9)
    ap.add_argument("--sim-max-m", type=float, default=30.0,
                    help="Sim MaxRange — datasheet peak (198 m) maps here")
    ap.add_argument("--sim-floor-m", type=float, default=25.0,
                    help="Sim minimum per-channel range — datasheet floor "
                         "(60 m) maps here. Leaves ~17%% headroom for the "
                         "edge channels relative to centre, which roughly "
                         "matches what dual-path validation tolerated.")
    ap.add_argument("--param", action="store_true",
                    help="Generate parametric quadratic instead of using the datasheet")
    ap.add_argument("--edge-falloff-pct", type=float, default=15.0,
                    help="(--param only) Most-edge channel reaches this %% "
                         "shorter than centre")
    ap.add_argument("--max-range-m", type=float, default=30.0,
                    help="(--param only) Centre channel range")
    ap.add_argument("--pretty", action="store_true",
                    help="Pretty-print JSON (multi-line, easier diff)")
    args = ap.parse_args()

    if args.param:
        arr = parametric(args.num_channels, args.v_lower_deg, args.v_upper_deg,
                         args.max_range_m, args.edge_falloff_pct)
    else:
        arr = datasheet_array(args.num_channels, args.v_lower_deg, args.v_upper_deg,
                              args.sim_floor_m, args.sim_max_m)

    if args.pretty:
        chunks = [arr[i:i + 8] for i in range(0, len(arr), 8)]
        print('"PerChannelMaxRangeM": [')
        for k, chunk in enumerate(chunks):
            line = ", ".join(f"{v:.2f}" for v in chunk)
            sep = "," if k < len(chunks) - 1 else ""
            print(f"  {line}{sep}")
        print("]")
    else:
        print(json.dumps(arr))
    return 0


if __name__ == "__main__":
    sys.exit(main())
