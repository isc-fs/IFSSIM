"""A random track's cones become the editor's control points."""

import math

import pytest

from random_spec import centreline, control_points


def _oval(r, n, cx=0.0, cy=25.0, clockwise=False):
    """A stadium-ish ring through the origin's neighbourhood, in order."""
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n * (-1 if clockwise else 1) - math.pi / 2
        pts.append((cx + 2.0 * r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _left_of_travel(pts, cones):
    """How many path segments have the nearest of `cones` on their left."""
    n = 0
    for i, a in enumerate(pts):
        b = pts[(i + 1) % len(pts)]
        c = min(cones, key=lambda q: (q[0] - a[0]) ** 2 + (q[1] - a[1]) ** 2)
        n += 1 if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0 else -1
    return n


@pytest.mark.parametrize("clockwise", [False, True])
def test_blue_is_on_the_left_and_it_starts_at_the_origin(clockwise):
    # Blue outside, yellow inside, either way round: the path drives with blue on its left.
    blue = _oval(26.75, 120, clockwise=clockwise)
    yellow = _oval(23.25, 110, clockwise=clockwise)
    pts = control_points(blue, yellow)
    assert _left_of_travel(pts, blue) == len(pts)
    first = math.hypot(*pts[0])
    assert first == min(math.hypot(*p) for p in pts)


def test_it_thins_the_centreline_but_keeps_the_shape():
    blue = _oval(26.75, 200)
    yellow = _oval(23.25, 200)
    mid = centreline(blue, yellow)
    pts = control_points(blue, yellow)
    assert 8 <= len(pts) < len(mid) / 2
    # Every control point is on the centreline (radius 25 oval) within the tolerance.
    for x, y in pts:
        assert abs(math.hypot(x / 2.0, y - 25.0) - 25.0) < 1.0


def test_too_few_cones_is_an_error():
    with pytest.raises(ValueError):
        control_points([(0, 0), (1, 0)], [(0, 1), (1, 1)])
