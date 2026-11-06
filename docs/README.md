# Documentation index

| Document | Role |
|---|---|
| [`../README.md`](../README.md) | **Start here** — build, launch, operator controls |
| [`history/`](history/) | Slice-era contracts and bring-up notes (A2–A9) |
| [`evidence/`](evidence/) | Retained smoke/CSV artifacts from pre-consolidation runs |

Authoritative experiment parameters live in repository `config/` and are
installed by `ekf_monoslam_bringup`. After editing `config/` or
`control_plan.txt`, rebuild/reinstall that package so the share tree updates.

Historical notes in `history/` may still mention `install_a1` overlays and
the removed OpenCV `plot` package; prefer the root README for current
commands. GT / EstimatorReference contract updates are recorded in
`history/A9_integrated_launch_plot2.md` (superseding the old first-odometry
/ empty-Path epoch description).
