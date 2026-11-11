# Documentation index

| Document | Role |
|---|---|
| [`../README.md`](../README.md) | **Start here** — build, launch, operator controls |
| [`history/`](history/) | Slice-era contracts and bring-up notes (A2–A9, G1) |
| [`history/G1_globalslam_interface_contract.md`](history/G1_globalslam_interface_contract.md) | Global SLAM audit + G2–G5 close-out (identity, viz, correction path) |
| [`evidence/`](evidence/) | Retained smoke/CSV artifacts from pre-consolidation runs |

Authoritative experiment parameters live in repository `config/` and are
installed by `ekf_monoslam_bringup`. After editing `config/` or
`control_plan.txt`, rebuild/reinstall that package so the share tree updates.

Supported quadrotor LocalSLAM profiles: `config/quad_params.yaml`
(`./launch2.sh`, velocity-aided) and `config/quad_params_no_velocity.yaml`
(`./launch3.sh`, T2-optimized velocity-free). Evidence:
[`evidence/lslam_nov_vel_t2/`](evidence/lslam_nov_vel_t2/).

Historical notes in `history/` may still mention `install_a1` overlays and
the removed OpenCV `plot` package; prefer the root README for current
commands. GT / EstimatorReference contract updates are recorded in
`history/A9_integrated_launch_plot2.md` (superseding the old first-odometry
/ empty-Path epoch description).
