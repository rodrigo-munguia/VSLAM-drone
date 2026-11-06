# A9 — Integrated `./launch2.sh` + Plot2 (Jazzy)

Restores the original operator workflow on ROS 2 Jazzy without the UAV
accelerator. FluidPressure altitude mode is **deferred** relative to this
application restore (do not rely on pressure in this workflow).

## One-command build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/DEV/projects/EKF-MonoSLAM-Range-To-Base

# Plot2 system dependency (exact; not installed by this project):
sudo apt-get install libglfw3-dev

COLCON_LOG_PATH=log_a1 colcon build \
  --packages-select interfaces localslam webotbridge control \
    ekf_monoslam_bringup webot_package plot2 keyboard \
  --build-base build_a1 --install-base install_a1
```

Overlay selection: default **`install_a1`**. Override with
`EKF_INSTALL_BASE=install_a1` (or another prefix you built). Do not assume
the newest `install_aN` is correct.

## One-command launch

```bash
./launch2.sh
# Stage A (GT control):
./launch2.sh --slam-feedback false
# Stage B (odometry feedback):
./launch2.sh --slam-feedback true --slam-feedback-interface odometry
```

`launch_a1.sh` is a thin wrapper that forces `EKF_INSTALL_BASE=install_a1`
and execs `launch2.sh`.

What opens:

1. Webots (separate terminal) — `webot_package robot_launch.py`
2. Operator keyboard terminal — `keyboard_node`
3. Current terminal — component container status
   (`ekf_monoslam_bringup quad_experiment.launch.py`, Plot2 on by default)

Press **`a`** in the operator terminal to start/stop the control plan.

Standalone Local SLAM (no Webots) remains:
`ros2 launch localslam localslam.launch.py ...`

## Configuration

- Source of truth for the experiment: `config/quad_params.yaml`
- Installed copy: `share/ekf_monoslam_bringup/config/quad_params.yaml`
  (reinstall `ekf_monoslam_bringup` after editing the source file)
- Control plan: `control_plan.txt` → installed as
  `share/ekf_monoslam_bringup/control_plan.txt`
  Override: `./launch2.sh --control-plan /abs/path/to/plan.txt`

Gains, noise, calibration source, and default feedback mode are **not**
silently changed by this slice.

## Keyboard menu (working)

| Key | Action | Target |
|---|---|---|
| `q` | Quit operator | local |
| `s` | Start/Stop SLAM | `/EKFslam/enable` + `ekf_run_service` (`r`/`u` or `s`) |
| `w` | Coordinated reset | `/EKFslam/reset`, `ekf_run_service`/`control_run_service` `i`, `plot_service` `c` |
| `a` | Start/Stop control plan | `control_run_service` `r` |
| `t` | Takeoff / land | `webot_service` `t`/`l` |
| `0` | Stats log request | `ekf_run_service` `0` |
| `-`/`+` | Zoom out/in | `plot_service` |
| `1`/`2`/`3` | Select XY / XZ / YZ view | `plot_service` |
| `8`/`5`/`4`/`6` | Pan up/down/left/right | `plot_service` |
| `7` | Clear plot | `plot_service` |
| `9` | Screenshot | `plot_service` (also `P` in Plot2 window) |
| `r/f/d/g/i/k/j/l` / space | Manual flight / stop | `/cmd_vel` |

Global SLAM is **not** in this launch; `globalslam_run_service` on reset is
best-effort and may be unavailable (menu states this).

## Plot2 data-source mapping

| Display | Source | Notes |
|---|---|---|
| Estimated pose / velocity | `nav_msgs/Odometry` `/odometry` | Public odom parent (+up) |
| Trajectory | `nav_msgs/Path` `/EKFslam/path` | Same snapshots as odometry; not dual-accumulated |
| Landmarks | `sensor_msgs/PointCloud2` `/EKFslam/landmarks` | Feats+anchors merged (no separate colors) |
| Image + 2D feature overlays | `interfaces/Frame` + `interfaces/Lslam` | Custom retained |
| Radius uncertainty text | `interfaces/Lslam.radius_uncertainty` | Custom retained |
| Ground truth | `GT_topic` + `/ground_truth/path` | Same bridge conversion keyed by `/EKFslam/estimator_reference`. Plot2 uses `GT_topic` as-is (no second origin, no \(D\)). RViz2 uses Path. |

Common frame: **odom parent (public +up)**. Estimated pose, path,
landmarks, and GT share that frame. GT is **not** rotated by live
roll/pitch/yaw. Position error in Plot2 is **approximate** (latest
samples; A2 odometry stamps are publication time, not matched epochs).

### GT reference (bridge) — current contract

Epoch owner: Local SLAM `interfaces/EstimatorReference` (`TRANSIENT_LOCAL`
on `/EKFslam/estimator_reference`). Heading uses the **noisy** attitude
sample accepted at `system_init` (`yaw_enu_equivalent = -yaw_at_home_internal`),
not a noise-free simulator yaw. Altitude home is instance-owned and cleared
on reset (replaces the former process-static latch).

| Event | Behavior |
|---|---|
| Estimator init (`valid=true`, `heading_valid`) | Bridge matches GPS history to `heading_stamp` |
| Altitude home latch (when `Altitude_update`) | Republish ref; bridge matches `altitude_stamp` for \(z\) origin |
| GPS while resolved | \(p_{\mathrm{pub}}=R_z(-\mathrm{yaw_{enu}})(p_w-p_{\mathrm{ref}})+D\cdot\mathrm{Robot\_init_{int}}\) → `GT_topic` and Path |
| Attitude change at fixed GPS | GT position **unchanged** |
| `valid=false` (reset) | Invalidate; publish empty Path; withhold until new epoch resolves |
| Disable / re-enable | Does **not** create a new epoch; GT may continue |
| Unresolved history / clock jump | Withhold GT; log reason (no silent latest-sample fallback) |
| Plot clear (`7`) | Clears Plot2 trails only (bridge epoch unchanged) |
| Control GT GPS path | **Unchanged** (raw `/webot_robot/gps`; not `GT_topic`) |

Path orientation: identity quaternion (unspecified). `GT_path_max_poses`
&lt; 1 clamps to 1 (not unlimited).

Vertical views (X–Z, Z–Y) project with `screen_y -= z_pub` so public +Z
is upward on screen (ImGui Y increases downward).

### Top XY display convention (from above)

| Item | Value |
|---|---|
| Plot2 top XY | `screen_x = centre_x + x_pub·s`, `screen_y = centre_y − y_pub·s` |
| Screen-right | **+X_public** |
| Screen-up | **+Y_public** |
| Viewing direction | **−Z** (from +Z toward −Z) |

Pixel Y increases downward. Pan/zoom share this projector for pose, GT,
path, landmarks, and axis arrows. Stored ROS coordinates are unchanged.

A previous defective mapping (`+Y` right, `+X` up) looked along **+Z**
(from below). That was a Plot2 projection defect, not solely a Webots
orbit/yaw difference.

**Comparing to Webots:** orbit the Webots viewport so world **+X** is
screen-right and **+Y** is screen-up; then horizontal motion matches Plot2
arrows. An arbitrary Webots camera can still disagree on screen-left/right
even when both views are geometrically correct — that is camera choice.

## Resolved mismatch

Previously the keyboard advertised Plot2 zoom/view/clear/screenshot via
`plot_service`, but Plot2 did not implement the service. Plot2 now provides
`plot_service` and applies those commands to shared view state.
