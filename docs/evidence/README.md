# Retained validation evidence

Non-reproducible experiment artifacts kept after consolidation (not part of
the normal build/install tree).

| Path | Contents |
|---|---|
| `lslam_range_a1/` | Read-only audit of range-assisted landmark initialization ([REPORT.md](lslam_range_a1/REPORT.md)) |
| `lslam_range_b1/` | Design audit for selectable TFA300-L long-range laser ([REPORT.md](lslam_range_b1/REPORT.md)) |
| `lslam_range_b2/` | Configurable range acceptance gates ([CLOSEOUT.md](lslam_range_b2/CLOSEOUT.md)) |
| `lslam_range_b3/` | Selectable TFA300-L Webots laser + launch remap/overlay ([REPORT.md](lslam_range_b3/REPORT.md)) |
| `lslam_range_b4/` | launch4 laser velocity-free profile, fallback flag, confidence YAML ([REPORT.md](lslam_range_b4/REPORT.md)) |
| `lslam_env_a1/` | Earth-like Webots world (500 m terrain, synthetic aerial texture) ([REPORT.md](lslam_env_a1/REPORT.md)) |
| `lslam_env_a2/` | Flat home + Mars marker + USGS NAIP orthophoto ([REPORT.md](lslam_env_a2/REPORT.md)) |
| `lslam_ctrl_r4d/` | Mission sequencer ↔ acknowledged unified traj ([REPORT.md](lslam_ctrl_r4d/REPORT.md)) |
| `lslam_ctrl_r4e/` | Full-size unified mission validation ([REPORT.md](lslam_ctrl_r4e/REPORT.md)) |
| `lslam_ctrl_r5/` | Speed/jerk ablation STOPPED — B/C infeasible vs V_MAX ([REPORT.md](lslam_ctrl_r5/REPORT.md)) |
| `lslam_ctrl_r5a/` | Independent XY/Z traj limits; 3 m/s horiz feasibility ([REPORT.md](lslam_ctrl_r5a/REPORT.md)) |
| `gslam_visual_loop_a1/` | Read-only audit of ORB/global-map visual loop closure ([REPORT.md](gslam_visual_loop_a1/REPORT.md)) |
| `gslam_visual_loop_a2/` | Deterministic ORB/PnP geometry validation; NO-GO for Webots visual LC ([REPORT.md](gslam_visual_loop_a2/REPORT.md)) |
| `gslam_visual_loop_a2b/` | Corrected visual PnP seed/convert/gate/inliers; GO for feedback-off Webots visual LC ([REPORT.md](gslam_visual_loop_a2b/REPORT.md)) |
| `gslam_visual_loop_a2c/` | Mid-route revisit capability audit; activation/candidates/selection block autonomous map-wide LC ([REPORT.md](gslam_visual_loop_a2c/REPORT.md)) |
| `gslam_arch_a1/` | Dual Global BA ↔ Pose_SLAM ownership/sync audit; A2d front-end-only GO ([REPORT.md](gslam_arch_a1/REPORT.md)) |
| `gslam_visual_loop_a2d/` | Autonomous mid-route visual front end; GO for feedback-off Webots A3 ([REPORT.md](gslam_visual_loop_a2d/REPORT.md)) |
| `gslam_kf_z_a1/` | Vertical KF init / altitude audit ([REPORT.md](gslam_kf_z_a1/REPORT.md)) |
| `gslam_kf_z_a2/` | Hybrid XY-rel / Z-abs implementation ([REPORT.md](gslam_kf_z_a2/REPORT.md)) |
| `gslam_kf_z_a3/` | Controlled R/H vertical-drift comparison ([REPORT.md](gslam_kf_z_a3/REPORT.md)) |
| `gslam_kf_z_a5/` | Hybrid promoted to production default ([REPORT.md](gslam_kf_z_a5/REPORT.md)) |
| `logs/` | Operator/control CSV captures from integrated runs (`CONFB_*.csv`) |
| `smoke/` | Selected smoke/bridge log snippets from pre-consolidation overlays |

These support historical slice reports under `docs/history/`. They are **not**
accuracy or paper-parity claims. Regenerating them requires a live Webots
session and is outside the conventional CI path.

Do not commit large new recordings by default (see root `.gitignore`).
