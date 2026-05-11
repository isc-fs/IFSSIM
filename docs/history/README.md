# Historical design / planning docs

Documents in this folder describe work that has either landed (and the planning notes outlived their purpose) or was abandoned (and only the post-mortem matters). They are kept so the *why* behind decisions is recoverable, but they are **not** current. For current state, see [`../FUNCTIONALITIES.md`](../FUNCTIONALITIES.md), [`../autonomy_pipeline.md`](../autonomy_pipeline.md), and [`../GETTING_STARTED_DOCKER.md`](../GETTING_STARTED_DOCKER.md).

| File | What it describes | Status |
|------|-------------------|--------|
| `DOCKER_PLAN.md` | Plan for the original docker containerization work | Shipped — see `GETTING_STARTED_DOCKER.md` |
| `dv_pipeline_rebuild.md` | April 2026 SLAM/planner/control rebuild plan; mentions FAST-LIO and `pipeline/odometria` that never landed | Partially shipped (cone-graph SLAM, dv_pipeline_stack rename); pivots away from FAST-LIO superseded by cone-graph SLAM |
| `glim_integration.md` | Plan to integrate GLIM as the LiDAR-IMU SLAM | **Abandoned** — replaced by cone-graph SLAM. The §15 post-mortem at the end is the load-bearing content |
| `cone_graph_slam_design.md` | Design doc for the cone-graph SLAM that shipped | Shipped — landed via PR #138 (`feat/28-cone-graph-slam`) |
| `cone_graph_slam_progress.md` | Engineering log captured while building cone-graph SLAM | Shipped — kept for the per-iteration tuning matrix and lessons learned at the end |
