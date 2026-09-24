# Historical design / planning docs

Documents in this folder describe work that has either landed (and the planning notes outlived their purpose) or was abandoned (and only the post-mortem matters). They are kept so the *why* behind decisions is recoverable, but they are **not** current. For current state, see [`../REFERENCE.md`](../REFERENCE.md), [`../AUTONOMY.md`](../AUTONOMY.md), and [`../SETUP.md`](../SETUP.md).

| File | What it describes | Status |
|------|-------------------|--------|
| `DOCKER_PLAN.md` | Plan for the original docker containerization work | Shipped — see `SETUP.md` and `OPERATING.md` |
| `dv_pipeline_rebuild.md` | April 2026 SLAM/planner/control rebuild plan; mentions FAST-LIO and `pipeline/odometria` that never landed | Partially shipped (cone-graph SLAM, dv_pipeline_stack rename); pivots away from FAST-LIO superseded by cone-graph SLAM |
| `glim_integration.md` | Plan to integrate GLIM as the LiDAR-IMU SLAM | **Abandoned** — replaced by cone-graph SLAM. The §15 post-mortem at the end is the load-bearing content |
| `cone_graph_slam_design.md` | Design doc for the cone-graph SLAM that shipped | Shipped — landed via PR #138 (`feat/28-cone-graph-slam`) |
| `cone_graph_slam_progress.md` | Engineering log captured while building cone-graph SLAM | Shipped — kept for the per-iteration tuning matrix and lessons learned at the end |
| `2026-09-23_benchmark-tracking-design.md` | Design proposal for experiment tracking of `tools/sim_benchmark` runs (W&B / MLflow / ClearML) | Evaluation build in `tools/sim_benchmark/tracking/` (`feat/517-bench-tracking-viewer`) |
| `2026-09-23_tracker-evaluation.md` | Results of pushing the same benchmark data into all three trackers, and the follow-up viewer on MLflow (`bench-view`) | Evaluation — tracker choice still open |
