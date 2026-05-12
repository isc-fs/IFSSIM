# 2026-05-06 — SLAM Phase 2 cascade-recovery freeze record

> **Status**: archived. Source branch `wip/306-slam-phase2-frozen` (commit `6e1f816`) was deleted on 2026-05-12 after its content was copied here. Tracking issue [#306](https://github.com/isc-fs/IFSSIM/issues/306) was closed on 2026-05-06.

This file preserves the handoff commit message verbatim. The branch's
*code* was obsolete relative to dev — 14k lines of deletions vs the
current tree, every assumption pre-dated the DV-pipeline rebuild in
PRs #374 / #378 / #393 / #395 — so the code itself isn't useful to
re-apply. The *findings* below, however, directly informed:

- **PR #441** — proximity veto on new-landmark spawn (built on the
  Phase 2 v1 finding that phantom landmarks corrupt iSAM2's bias
  estimate).
- **PR #453** — count-based cascade-spike detector (the assoc==0
  narrowing solves the "all-new in one scan" CLIFF signature
  described below).
- **PR #451 (Phase 2 EKF, draft)** + **PR #470 (LWS sensor model)** —
  the structural ceiling identified here ("cone-only DA cannot survive
  >1 m predicted-pose drift") is what motivated moving from a purely-
  geometric cascade response to a better-quality estimator front-end.

The conclusion the original author reached six days ago is the same
conclusion we re-derived on 2026-05-12 through live re-testing. Reading
this *before* re-running the same experiments would have saved a day
of sim time.

---

## Original commit message (verbatim)

```
WIP-FROZEN: Phase 2 SLAM cascade recovery — cone-only DA ceiling — see #306

This branch is INTENTIONALLY NOT MERGED. It exists as a record of the
Phase 2 attack on the second-hairpin SLAM cascade described in #306,
so the next team to pick this up doesn't have to re-discover what was
tried. None of these changes lap test_submodule.

What this branch contains (over Phase 1 / PR #308):

  1. factor_graph.py — stage_cone_observation accepts sigma_scale
     parameter that uniformly multiplies bearing+range σ. Mechanism for
     the "weak anchor" tier below.

  2. cone_graph_slam_node.py — three-tier cascade response replaces
     the binary skip-or-commit gate from #273:
       assoc < CASCADE_MIN_ANCHORS (=3)  → IMU-only
       3 ≤ assoc < 40 % × total          → degraded: σ×5, NO phantoms
       assoc ≥ 40 % × total              → normal commit
     suppress_new=True in the degraded tier prevents phantom landmarks
     at the diverging predicted pose, which was the failure mode of
     the first Phase 2 attempt.

  3. cone_graph_slam_node.py — gate-widen ladder (Lever 1 from #301)
     before tier dispatch: if 1 m gate yields <3 anchors and obs ≥ 5
     and step > 30, retry associate() with 3 m gate, then 5 m gate.
     Best result feeds tier dispatch.

  4. data_association.py — associate() accepts gate_override_m to
     support the ladder.

Live failure traces, ordered by attempt:

  Phase 1 alone (vanilla SLAM, smoothed controller)
    → 235 m SLAM-vs-GT divergence, IMU runaway after cascade.

  Phase 2 v1 (degraded tier WITH phantom creation)
    → 235 m divergence; 11 phantom landmarks per scan accumulated at
      the diverging predicted pose, then iSAM2 yanked pose to fit them
      and bias estimate corrupted, IMU-only update accelerated to
      48 m/s ghost speed.

  Phase 2 v2 (degraded tier with suppress_new=True, no Lever 1)
    → 51 m divergence. The cascade is a CLIFF, not a slope: assoc
      jumped 13/14 → 1/14 → 0/14 in three scans, never spending time
      in the [3, 40 %) "degraded" window. The degraded tier never
      engaged because the geometric 1 m gate fails categorically as
      soon as predicted pose drifts past 1 m — every visible cone
      misses the gate at once.

  Phase 2 v2 + Lever 1 (gate widen 1m → 3m → 5m)
    → 34 m divergence. The 3 m gate did recover anchors mechanically
      (8/2, 11/2, 13/1 ratios in the logs), but those anchors were
      WRONG matches: in cone-only environments two cones at 3 m apart
      are indistinguishable to a Euclidean gate, so the optimizer
      satisfied a false-correspondence-set and snapped pose 51° in one
      tick (yaw 124° → 175° step 350; flipped to −125° step 360).

The structural ceiling we kept hitting:

  Cone-only DA cannot survive >1 m predicted-pose drift in cone-only
  scenes. The 1 m gate fails geometrically (cliff). Wider gates trade
  one failure mode (no-anchor) for another (wrong-anchor) because
  there is no per-cone discriminator — the colour signal that other
  FS teams use is absent here (UE5 sets bReturnPhysicalMaterial=false
  at the LiDAR raycast, so PR #281 deleted the body_y-sign+height
  classifier as cargo-cult heuristic). Every purely-geometric fix
  attempted has moved the failure mode without crossing the lap
  threshold.

What still works (verified live):
  - Phase 1 controller smoothing (PR #308) — visible in /control_command.
  - GT-as-SLAM diagnostic — car laps test_submodule with perfect pose
    (cone hits + brief off-track recovery, but lap completes).
  - Cone_Detection (~14–22 cones/scan with healthy L/R split).
  - Plan_Path (FaSTTUBe, 24-pt 11.6 m latched path, no plan_empty).
  - Replay harness (#287) for offline reproduction.

Forward paths documented in the freeze/handoff issue. None require
this branch. If picked up, the most likely useful piece is the
sigma_scale plumbing in factor_graph.py — it's a clean primitive that
any future cascade-handling policy will probably want.
```
