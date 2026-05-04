"""Tests for LandmarkDb.update_color (#269 option b).

Pins the planner-feedback colour-update path. The contract:
- update_color(lid, new_color) flips an existing landmark's colour
  and returns True.
- Non-existent landmark id → returns False, no exception.
- Already-matching colour → returns False (no-change).
"""
from __future__ import annotations

import numpy as np

from cone_slam.color_classifier import ConeColor
from cone_slam.landmark_db import LandmarkDb


def test_update_color_changes_existing_landmark() -> None:
    db = LandmarkDb()
    lm = db.create(ConeColor.BLUE, np.array([1.0, 2.0, 0.0]), step=0)
    assert lm.color == ConeColor.BLUE

    changed = db.update_color(lm.id, ConeColor.YELLOW)

    assert changed is True
    assert db.get(lm.id).color == ConeColor.YELLOW


def test_update_color_returns_false_for_unknown_id() -> None:
    db = LandmarkDb()
    db.create(ConeColor.BLUE, np.array([1.0, 2.0, 0.0]), step=0)

    changed = db.update_color(999, ConeColor.YELLOW)

    assert changed is False


def test_update_color_returns_false_when_already_set() -> None:
    db = LandmarkDb()
    lm = db.create(ConeColor.BLUE, np.array([1.0, 2.0, 0.0]), step=0)

    changed = db.update_color(lm.id, ConeColor.BLUE)

    assert changed is False
    assert db.get(lm.id).color == ConeColor.BLUE


def test_update_color_persists_through_subsequent_observations() -> None:
    """A planner-driven colour update must not be reverted by later
    `mark_observed` calls or position updates."""
    db = LandmarkDb()
    lm = db.create(ConeColor.BLUE, np.array([1.0, 2.0, 0.0]), step=0)
    db.update_color(lm.id, ConeColor.YELLOW)

    db.mark_observed(lm.id, step=5)
    db.update_from_estimate(lambda lid: np.array([1.5, 2.5, 0.0]))

    assert db.get(lm.id).color == ConeColor.YELLOW
