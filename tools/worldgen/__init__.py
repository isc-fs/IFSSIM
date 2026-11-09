"""worldgen: real places, measured from open data, for the sim to build around a track.

A world (docs/worlds.md) is a directory under Content/tracks/worlds/ with a
world.json and a terrain grid. A track uses one through the "world" key of its
environment sidecar.

    cd tools
    python3 -m worldgen check ../Content/tracks/worlds/montmelo_straight
    python3 -m worldgen build montmelo_straight --cache ~/Library/Caches/ifssim-geo/icgc

`worldgen.world` (the format) is standard library only, so CI can check worlds.
Building one needs numpy and laspy[lazrs].
"""

from .world import FORMAT, WorldError, height_at, load, validate

__all__ = ["FORMAT", "WorldError", "height_at", "load", "validate"]
