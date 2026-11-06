# A7 — Standalone Local SLAM

Local SLAM (`localslam`) builds and launches without Webots, webotbridge, control,
keyboard, plot2, Global SLAM, or `ekf_monoslam_bringup`.

## Dependency isolation

| Required to build/run Local SLAM | Not required |
|---|---|
| `interfaces` (in-repo msgs/srvs) | `globalslam` package |
| ROS 2 Jazzy: rclcpp, rclcpp_components, std_srvs, sensor_msgs, geometry_msgs, nav_msgs, mavros_msgs, tf2, cv_bridge | `webot_package`, `webotbridge` |
| OpenCV, Armadillo (via BLAS/LAPACK) | `control`, `keyboard`, `plot2` |
| `src/common/` (Transforms, Vision, anms, **keyframe_types.hpp**) | `ekf_monoslam_bringup` |

**KEYFRAME:** moved to `src/common/keyframe_types.hpp`. Global SLAM still consumes
the same type via `globalslam_types.hpp` (includes the shared header). Localslam
no longer includes any `globalslam` source path.

## Isolated build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/DEV/projects/EKF-MonoSLAM-Range-To-Base
COLCON_LOG_PATH=log_a7 colcon build \
  --packages-up-to localslam \
  --packages-select interfaces localslam \
  --build-base build_a7 --install-base install_a7
```

(`--packages-up-to localslam` with only those packages selected builds `interfaces`
then `localslam`.)

## Standalone launch (from any cwd after sourcing install)

```bash
source /opt/ros/jazzy/setup.bash
source ~/DEV/projects/EKF-MonoSLAM-Range-To-Base/install_a7/setup.bash

ros2 launch localslam localslam.launch.py \
  use_sim_time:=false \
  sensor_input_mode:=standard \
  calibration_source:=yaml \
  image_topic:=/EKFslam/image \
  camera_info_topic:=/EKFslam/camera_info \
  imu_topic:=/EKFslam/imu \
  body_velocity_topic:=/EKFslam/body_velocity \
  downward_range_topic:=/EKFslam/downward_range \
  altitude_topic:=/EKFslam/altitude \
  odometry_topic:=/EKFslam/odometry \
  landmarks_topic:=/EKFslam/landmarks \
  path_topic:=/EKFslam/path \
  rangetobase_topic:=/Rangetobase_topic \
  robotstate_topic:=/Robotstate_topic \
  odometry_frame_id:=odom \
  odometry_child_frame_id:=base_link
```

Noise overrides (optional): `sigma_att_update:=…`, `sigma_spd_update:=…`,
`sigma_alt_update:=…`, `sigma_cam_uv:=…`, `sigma_w:=…`, `sigma_a:=…`.

Calibration (A8): `calibration_source:=yaml|camera_info` (default yaml).
See `docs/A8_camera_info_calibration.md` for CameraInfo contracts and remaps.

### Enable / reset

EKF steps start **disabled**. After sensors are publishing:

```bash
ros2 service call /EKFslam/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /EKFslam/reset std_srvs/srv/Trigger "{}"
```

Legacy: `/ekf_run_service` (`interfaces/srv/SimpleServ`) cmds `'r'`/`'s'`/`'i'`.

Global-map pose correction service `/local_slam_pos_update` remains available but
is unused unless a Global SLAM (or test) client calls it — standalone launch does
not invoke it.

## Remaining custom interfaces (explicit)

### Necessary transitional (still active in standard mode)

| Interface | Role |
|---|---|
| `interfaces/msg/Rangetobase` on `Rangetobase_topic` | Range-to-base update input when `Ground_station_range_update=true` (default). **Not redesigned in A7.** |
| `interfaces/msg/Robotstate` on `Robotstate_topic` | Approved radius-compatibility output (`radius_uncertainty` + legacy state vector) for control A5b/A6. |
| `interfaces/msg/Lslam` on `lslam_data_topic` | Legacy local-map / feature debug stream (still published). |
| `interfaces/msg/Kf` on `Kf_topic` / `Kf_cl_topic` | Keyframe pubs for optional Global SLAM consumers (published when KF available; no GS required to run). |
| `interfaces/srv/SimpleServ` `ekf_run_service` | Legacy enable/disable/reinit. |
| `interfaces/srv/LSposUpdate` `local_slam_pos_update` | Optional pose delta from Global SLAM / tests. |

### Unused when `Sensor_input_mode=standard` (legacy historical)

`Attitude_topic`, `Altitude_topic`, `Speed_topic`, `Range_topic`, `Frame_topic`,
`Gps_topic`, `OdometryD_topic`, `OdometryV_topic` — not subscribed in standard mode.

### Standard (migrated) I/O

**In:** `~/image`, `~/imu`, `~/body_velocity`, `~/downward_range`, `~/altitude`  
**Out:** `~/odometry`, `~/landmarks`, `~/path`  
**Services:** `~/enable`, `~/reset`

## A6 experiment compatibility

`ekf_monoslam_bringup` / `quad_experiment.launch.py` unchanged in behavior:
still loads Local SLAM as a composable node with experiment remaps. A6 Webots
bringup remains the supported full-stack path.

## Known limits (carried from A6)

- Coordinated control reset / radius: Robotstate unstamped; post-re-arm stale
  radius possible; odometry stamped after reset but before EKF re-init can re-arm.
- Standalone smoke does not claim Webots accuracy or paper parity.
- Complete standardization of range-to-base is **out of scope** for A7.
