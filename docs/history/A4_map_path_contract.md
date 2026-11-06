# A4 — Landmarks PointCloud2 and Path

## `~/landmarks` (`sensor_msgs/PointCloud2`)

| Item | Value |
|---|---|
| Topic | `~/landmarks` |
| Frame | Same parent as A2: `Odometry_frame_id` (default `odom`) |
| Fields | XYZ `float32` only (no IDs — EKF state indices are not stable) |
| Content | Current Euclidean features **and** anchors from one `get_lslam_data()` snapshot |
| Conversion | \(p_{\mathrm{pub}} = D\,p_{\mathrm{int}}\), \(D=\mathrm{diag}(1,-1,-1)\) |
| Invalid | Non-finite internal coordinates skipped |
| Empty map | Publishes `width=0` cloud so RViz/consumers clear a previous local map |
| Semantics | **Changing local sparse EKF map**, not a persistent global reconstruction |
| Custom | `lslam_data_topic` remains published unchanged |

Published on frame steps (alongside custom Lslam). Reset clears the internal map on re-init and publishes an empty cloud immediately when the re-init flag is consumed.

Covariance of the full augmented state is **not** copied for visualization.

## `~/path` (`nav_msgs/Path`)

| Item | Value |
|---|---|
| Topic | `~/path` |
| Frame | `Odometry_frame_id` |
| Poses | Same accepted public pose as each successful `~/odometry` publish |
| Stamp | Publication time (same meaning as A2 odometry — not a measurement epoch) |
| Bound | Parameter `Path_max_poses` (default 2000); oldest dropped |
| Disable | No new odometry snapshots → path does not grow (pause) |
| Reset | Clears history and publishes empty path |

No duplicate snapshots: path appends only inside successful `pub_odometry`.

## Optional RViz

Installed config: `ekf_monoslam_bringup` share `config/a4_localslam.rviz`  
Launch: `ros2 launch ekf_monoslam_bringup a4_rviz.launch.py` (requires `rviz2` on the system; not a hard build dependency of the estimator).

Fixed Frame = odometry parent (`odom`). Displays: Odometry, PointCloud2 landmarks, Path. No TF broadcaster required for these same-frame topics.

## Related A3/A5 note

Odometry feedback still cannot supply `radius_uncertainty` (`r_u`); see `docs/A5_cl_radius_contract.md`.

## Incidental fix (A4) / A5 verification

`get_lslam_data()` clears `map_anchors` on the **output** `LOCALSLAM_DATA` only, then re-fills from live `AnchorsDATA`. Live estimator anchors are not deleted.

On re-init, pending `NewRobotState_available` / keyframe flags are cleared with `initialized=false`, empty path/landmarks are published on the worker before any post-reset snapshot, and `system_init` clears map state — preventing republish of pre-reset cloud/path from a stale snapshot.

EKF worker thread starts only after publishers and control flags are initialized.
