"""Seeded random streams, named the way the plugin names its own.

The plugin derives every random stream from the scenario seed and a stream
name, hashing the name with 32-bit FNV-1a (FSDSRandom.cpp, HashStreamName).
envgen uses the same hash, so a stream is named the same way on both sides.
The environment seed is separate from the scenario seed (roadmap rule 3), so
a noise sweep over scenario seeds runs on one identical world.

The generator is SplitMix64: pure Python, a few lines, and the same sequence on
every platform and Python version. ``random.Random`` would also be stable, but
its algorithm is an implementation detail of CPython.
"""

from __future__ import annotations

_MASK64 = (1 << 64) - 1


def fnv1a32(name: str) -> int:
    """32-bit FNV-1a over the name's characters, as FSDSRandom's HashStreamName."""
    h = 2166136261
    for ch in name:
        h ^= ord(ch)
        h = (h * 16777619) & 0xFFFFFFFF
    return h


class Stream:
    """SplitMix64, seeded from the environment seed and a stream name."""

    def __init__(self, seed: int, name: str):
        if seed < 0:
            raise ValueError("seed must be non-negative")
        self._state = ((seed & _MASK64) << 32 ^ fnv1a32(name)) & _MASK64

    def next_u64(self) -> int:
        self._state = (self._state + 0x9E3779B97F4A7C15) & _MASK64
        z = self._state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & _MASK64
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & _MASK64
        return z ^ (z >> 31)

    def uniform(self, lo: float = 0.0, hi: float = 1.0) -> float:
        """A float in [lo, hi), from the top 53 bits."""
        return lo + (hi - lo) * ((self.next_u64() >> 11) / float(1 << 53))

    def randint(self, n: int) -> int:
        """An int in [0, n)."""
        if n <= 0:
            raise ValueError("n must be positive")
        return self.next_u64() % n
