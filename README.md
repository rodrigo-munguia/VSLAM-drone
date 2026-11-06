# EKF-MonoSLAM Aided by Range-to-Base Measurements

C++ ROS 2 implementation of the SLAM approach described in:

**“UAV Navigation Using EKF-MonoSLAM Aided by Range-to-Base Measurements”**  
https://doi.org/10.3390/drones9080570

![](figures/Graphical_abstract.png)

This repository is a **standalone research / simulation stack**. It is
**independent of the UAV accelerator** project.

## Tested environment

| Item | Version |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Webots | R2025a |
| Viewer (default) | Plot2 (ImGui / GLFW) |
| Viewer (optional) | RViz2 |

Older README notes mentioning Galactic / Webots R2023a describe the upstream
paper baseline, not the current Jazzy bring-up.

## Components

**Composable nodes (app container):** Local SLAM (`EKFslam`), Webots bridge
(`WEBOTb`), Control (`ControlQuad`), optional Plot2.

**Separate processes:** Webots (`webot_package`), operator keyboard, optional
RViz2.

**Also in-tree (not required for the default experiment):** Global SLAM,
dataset playback. The legacy OpenCV **`plot`** package has been **removed**.

## Dependencies

**Required for the integrated experiment**

- ROS 2 Jazzy + `webots_ros2` (`sudo apt-get install ros-jazzy-webots-ros2`)
- Webots R2025a (`WEBOTS_HOME`)
- Armadillo + BLAS/LAPACK
- OpenCV (contrib modules as needed by Local/Global SLAM; **VIZ/VTK not required** after `plot` removal)
- GTSAM (Global SLAM package; needed if that package is built)

**Optional**

- `libglfw3-dev` — required only for Plot2
- `ros-jazzy-rviz2` — required only for `--viewer rviz2`
- ImGui sources are vendored under `src/plot2`

**Deferred:** FluidPressure altitude input remains deferred (derived GPS-height
altitude is the default).

## Build (conventional trees)

```bash
cd ~/DEV/projects/EKF-MonoSLAM-Range-To-Base
source /opt/ros/jazzy/setup.bash
# Plot2 needs GLFW headers at build time:
sudo apt-get install libglfw3-dev

colcon build \
  --packages-select interfaces localslam webotbridge control \
    ekf_monoslam_bringup webot_package plot2 keyboard \
  --build-base build --install-base install
```

Standalone Local SLAM only:

```bash
colcon build --packages-select interfaces localslam \
  --build-base build --install-base install
```

Override the install prefix with `EKF_INSTALL_BASE` if needed (default for
`./launch2.sh` is `install/`).

## One-command integrated launch

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./launch2.sh
# equivalents:
./launch2.sh --viewer plot2
./launch2.sh --viewer rviz2
./launch2.sh --viewer none
./launch2.sh --slam-feedback false   # Stage A: GT GPS control feedback
```

Starts **one** Webots, **one** application container, **one** chosen viewer
(or none), plus an operator terminal. Status/logs stay in the launching
terminal. Ctrl-C cleans up launcher-owned processes.

Viewer differences: see [`docs/viewers.md`](docs/viewers.md).

## Operator controls

In the **EKF-Operator** terminal:

| Key | Action |
|---|---|
| `a` | **Start / stop the control plan** (`control_plan.txt`) |
| `s` | Enable / disable Local SLAM (`/EKFslam/enable`; visual on start) |
| `w` | Coordinated reset (`/EKFslam/reset` + control; clear Plot2 if present) |
| `t` | Takeoff / land |
| `q` | Quit operator |
| flight keys | `r/f/d/g` translate, `i/k` up/down, `j/l` yaw, space stop |

Plot2-only keys (`-/+`, `1`–`3`, pan `4/5/6/8`, `7` clear, `9` screenshot) are
active only when `--viewer plot2`. Under RViz2, use RViz’s own navigation UI.

## Configuration

| Source | Role |
|---|---|
| `config/quad_params.yaml` | **Authoritative** experiment parameters |
| `control_plan.txt` | Default mission plan (installed with bringup) |
| `config/mobile_params.yaml`, `omni_params.yaml` | Alternate platforms |

Installed copies live under
`install/ekf_monoslam_bringup/share/ekf_monoslam_bringup/`. After editing
`config/` or `control_plan.txt`, **rebuild/reinstall `ekf_monoslam_bringup`**
so the share tree updates (do not maintain a second conflicting copy by hand).

## Standalone Local SLAM

```bash
source install/setup.bash
ros2 launch localslam localslam.launch.py
```

Remap standard inputs (`~/image`, `~/imu`, `~/odometry`, …) as documented in
`docs/history/A7_localslam_standalone.md`.

## Interfaces (summary)

**Standard (preferred):** `nav_msgs/Odometry` `/odometry`, `nav_msgs/Path`
`/EKFslam/path`, `sensor_msgs/PointCloud2` `/EKFslam/landmarks`, plus remapped
`sensor_msgs` / `mavros_msgs` sensor inputs.

**Custom retained:** `interfaces/Frame`, `interfaces/Lslam`, range-to-base and
related experiment messages; `interfaces/EstimatorReference` (latched Local
SLAM reference epoch for GT adapters).

**Ground truth:** `nav_msgs/Path` `/ground_truth/path` (RViz2) and
`geometry_msgs/PointStamped` `GT_topic` (Plot2) from the same synchronized
Webots→public-odom conversion. Path pose orientation is identity
(unspecified).

## Repository layout

```
config/          experiment YAML (authoritative)
control_plan.txt default plan
launch/          repo-level launch helpers
launch2.sh       canonical integrated entry point
src/             ROS packages + shared common/
docs/            index, viewers, history/, evidence/
tools/           smoke/, audit/, analysis/
figures/         paper graphics
build/ install/ log/   conventional colcon trees (gitignored)
```

## Ground-truth / estimator reference (summary)

Local SLAM publishes latched `~/estimator_reference`
(`interfaces/EstimatorReference`) on init and when altitude home latches;
`valid=false` on reset. The Webots bridge matches GPS history to
`heading_stamp` / `altitude_stamp` (no first-`/odometry` or empty-Path
epoch). Parameters under `/webotbridge` in `config/quad_params.yaml`:
`GT_history_duration_sec`, `GT_match_max_dt_sec`, `GT_path_max_poses`
(values &lt; 1 clamp to 1; **not** unlimited), `GT_path_frame_id`.

Inspect: `ros2 topic echo /ground_truth/path --once` after
`./launch2.sh --viewer rviz2`.

## Known limitations

- Operational smoke checks are **not** accuracy or paper-parity claims.
- Plot2 latest-estimate vs latest-GT error remains **approximate** (no full
  temporal matching of estimator stamps).
- GT Path orientation is unspecified (identity quaternion).
- FluidPressure altitude support is deferred.
- RViz2 does not show Plot2 image overlays or custom statistics.
- Deprecated OpenCV `plot` is removed; do not expect its topic semantics.

More detail: [`docs/README.md`](docs/README.md),
[`docs/history/A9_integrated_launch_plot2.md`](docs/history/A9_integrated_launch_plot2.md).
