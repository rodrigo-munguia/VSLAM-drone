# Viewer modes (`./launch2.sh --viewer …`)

| Mode | Process | Notes |
|---|---|---|
| `plot2` (default) | Plot2 in the app container | Full experiment HUD; operator zoom/pan/clear/screenshot |
| `rviz2` | Separate `rviz2` process | Standard Odometry / Path / PointCloud2 only |
| `none` | Neither | Estimator + Webots + operator (no visualization) |

Compatibility: `--no-plot2` is accepted as an alias for `--viewer none`.

## Plot2 vs RViz2

| Feature | Plot2 | RViz2 |
|---|---|---|
| Estimated pose / path / landmarks | Yes | Yes (`/odometry`, `/EKFslam/path`, `/EKFslam/landmarks`) |
| Global SLAM keyframes / landmarks | Yes when `--globalslam true` (cyan body-origin Path + magenta cloud on `/gslam/global_*`) | Yes (`Global Keyframes` / `Global Landmarks`; pose axes off) |
| Fixed frame | Public `odom` | Public `odom` |
| Camera image + 2D feature overlays | Yes | Not configured in the shipped RViz layout |
| Ground-truth trajectory | Yes (`GT_topic` PointStamped) | Yes (`/ground_truth/path`, distinct color; pose axes off) |
| Screenshot / pan-zoom keys | Operator + Plot2 window | Use RViz’s own UI |
| Radius-uncertainty / custom stats | Yes | No |
| GLFW / ImGui required | Yes | No (needs `rviz2` package) |

Global Path/cloud are sparse map geometry (not continuous global odometry).
Empty messages clear the global overlay after GS reset/reference change.
Plot2 does not also render legacy custom `Gmap` when standard outputs are used.

With `--global-corrections true`, expect discontinuous jumps on the local
trajectory/path when a BA or home-tag correction applies (same public `odom`
frame as GT and global overlays). No extra viewer axis conversion. Headless
`--viewer none` does not establish graphical acceptance.

`GT_topic` and `/ground_truth/path` share one bridge conversion keyed by
`/EKFslam/estimator_reference`. Path orientation is identity (unspecified);
RViz shows the trajectory line only.

Missing `rviz2` when `--viewer rviz2` is selected is a hard error (no silent
fallback to Plot2).
