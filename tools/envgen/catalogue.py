"""Prop classes and generator profiles.

A class here is what the generator needs to place a prop: its footprint and
how far from the track it stands. What it looks like is the plugin's business
(Private/Environment/FSDSEnvironmentBuilder.cpp builds every class from
primitives); the two must agree on the names and footprints, which
tests/test_environment.py checks.

The classes are the roadmap's worst perception stressors (P2a):
- narrow props at cone scale (bollards, bins), which a LiDAR detector reads as
  cones, big orange ones included;
- timekeeping light-barrier tripods beside the timing lines;
- low tyre walls just behind the cone line;
- stacks of spare cones, about as tall as a big orange cone.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class PropClass:
    name: str
    # Footprint, centred on the prop: a circle (radius_m) or, with length_m,
    # a rectangle length_m along the prop's yaw by width_m across it.
    radius_m: float = 0.0
    length_m: float = 0.0
    width_m: float = 0.0
    height_m: float = 0.0
    # Gap between the footprint and the track area (the cone corridor), m.
    band_m: Tuple[float, float] = (1.0, 5.0)
    # Faces along the nearest track boundary (a wall), or a random yaw.
    along_boundary: bool = False


CLASSES: Dict[str, PropClass] = {c.name: c for c in (
    # 0.20 m wide and 0.75 m tall: a cone-sized post. Big orange is 0.285 x 0.505.
    PropClass("bollard", radius_m=0.10, height_m=0.75, band_m=(0.8, 6.0)),
    PropClass("bin", radius_m=0.25, height_m=0.65, band_m=(1.0, 8.0)),
    # Light barrier on a tripod: three 1.2 m legs splayed to a 0.5 m radius.
    PropClass("tripod", radius_m=0.50, height_m=1.15, band_m=(1.0, 2.5)),
    # Four stacks of three tyres, 0.6 m across: 2.4 m long, 0.6 m tall.
    PropClass("tyre_wall", length_m=2.4, width_m=0.6, height_m=0.6, band_m=(0.5, 3.0),
              along_boundary=True),
    # Four small cones nested: 0.23 m wide, about 0.50 m tall.
    PropClass("cone_stack", radius_m=0.12, height_m=0.50, band_m=(2.0, 12.0)),

    # Venues (docs/venues.md): what stands around a parking lot. They are placed
    # by a venue file, not by a profile, so their bands are only defaults.
    # A tree's footprint is its 3 m canopy, the widest part (5.3 m tall).
    PropClass("tree", radius_m=1.5, height_m=5.3, band_m=(3.0, 15.0)),
    # A 6 m street lamp; the footprint covers its 1.3 m arm.
    PropClass("lamp_post", radius_m=1.35, height_m=6.0, band_m=(1.5, 6.0)),
    PropClass("parked_car", length_m=4.4, width_m=1.8, height_m=1.5, band_m=(2.0, 10.0),
              along_boundary=True),
    PropClass("bench", length_m=1.8, width_m=0.5, height_m=0.9, band_m=(2.0, 10.0),
              along_boundary=True),
    # A 2 x 2 m slab of sidewalk, 0.15 m tall: LiDAR-only, a phantom to the car
    # (roadmap rule 2: anything under 0.30 m a wheel could reach).
    PropClass("sidewalk", length_m=2.0, width_m=2.0, height_m=0.15, band_m=(0.5, 3.0),
              along_boundary=True),
    # An earth bank 10 m long, about 2.9 m tall and 5.9 m wide at the ground:
    # the hill until terrain (roadmap P3) exists.
    PropClass("berm", length_m=10.0, width_m=5.9, height_m=2.9, band_m=(5.0, 20.0),
              along_boundary=True),
)}


@dataclass(frozen=True)
class Profile:
    name: str
    # Props per 100 m of track boundary (both cone lines together).
    per_100m: Dict[str, float]
    # Tripods per timing line (a cluster of big orange cones), at most two lines.
    tripods_per_line: int = 0


PROFILES: Dict[str, Profile] = {p.name: p for p in (
    # No props: the flat baseline, with the environment switched on.
    Profile("flat_baseline", per_100m={}),
    Profile("stress", per_100m={"bollard": 3.0, "bin": 1.5, "tyre_wall": 1.5, "cone_stack": 1.0},
            tripods_per_line=2),
)}
