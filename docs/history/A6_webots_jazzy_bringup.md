# A6 — Webots Jazzy bringup

## Environment (validated)

| Item | Status |
|---|---|
| Webots | **R2025a** (`WEBOTS_HOME=/usr/local/webots`) |
| `ros-jazzy-webots-ros2` | **2025.0.0** |
| `ros-jazzy-ros2run` | Optional; **not required** (launch uses driver path via `webots-controller`) |
| `rviz2` | Present |
| `rosbag2` | **Not installed** on this machine |
| `DISPLAY` | `:1` |
| Ubuntu / ROS | 24.04 Noble / Jazzy |
| Build | `build_a1` / `install_a1` |

## Exact installation commands

```bash
wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt install ./webots_2025a_amd64.deb
export WEBOTS_HOME=/usr/local/webots

sudo apt-get update
sudo apt-get install ros-jazzy-webots-ros2
# optional recording / CLI
sudo apt-get install ros-jazzy-rviz2 ros-jazzy-rosbag2 \
  ros-jazzy-rosbag2-storage-default-plugins ros-jazzy-ros2run
```

## Isolated build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/DEV/projects/EKF-MonoSLAM-Range-To-Base
COLCON_LOG_PATH=log_a1 colcon build \
  --packages-select interfaces localslam webotbridge control \
    ekf_monoslam_bringup webot_package plot2 keyboard \
  --build-base build_a1 --install-base install_a1
source install_a1/setup.bash
```

`plot2` is optional (`use_plot2:=false`).

## Single unambiguous launch sequence

`quad_experiment.launch.py` does **not** include `robot_launch.py`. Start **one** simulator, then **one** app container:

```bash
source /opt/ros/jazzy/setup.bash
source ~/DEV/projects/EKF-MonoSLAM-Range-To-Base/install_a1/setup.bash
export WEBOTS_HOME=/usr/local/webots

# (1) Simulator — exactly one Webots + one webot_robot driver + Ros2Supervisor
ros2 launch webot_package robot_launch.py world:=mars.wbt

# (2) Application — exactly one component_container (EKFslam, WEBOTb, ControlQuad)
# Stage A:
ros2 launch ekf_monoslam_bringup quad_experiment.launch.py \
  slam_feedback:=false use_plot2:=false
# Stage B:
# ros2 launch ekf_monoslam_bringup quad_experiment.launch.py \
#   slam_feedback:=true slam_feedback_interface:=odometry use_plot2:=false

# (3) Operator
ros2 service call /control_run_service interfaces/srv/SimpleServ "{cmd: 114}"  # 'r' run plan
# Coordinated reset:
ros2 service call /control_run_service interfaces/srv/SimpleServ "{cmd: 105}"  # 'i'
# Land:
ros2 service call /webot_service interfaces/srv/SimpleServ "{cmd: 108}"        # 'l'
```

### Recording (if rosbag2 installed)

```bash
ros2 bag record -o /tmp/ekf_a6_stageX \
  /odometry /Robotstate_topic /image /imu_out \
  /body_velocity /downward_range /altitude \
  /EKFslam/landmarks /EKFslam/path /GT_topic /Rangetobase_topic /clock /cmd_vel
```

Frame/time: app nodes use `use_sim_time:=true` against Webots `/clock`. Odometry/path/landmarks use `odom` / `base_link`. Without rosbag2, use logs under `log_a1/a6_evidence/`.

## Compatibility fixes applied (A6)

1. **`robot_launch.py`**: stock `WebotsController` expands to `ros2 run` (fails without `ros-jazzy-ros2run`). Launch invokes `webots-controller` with the absolute `webots_ros2_driver` driver binary; sets `WEBOTS_HOME` to the driver package prefix (upstream pattern).
2. **Driver CSV logs**: `logfile_*.csv` written under `~/.ros/webot_package_logs` (webots-controller cwd may be `/`).
3. **Camera remap**: Webots publishes `/camera/image_color` → webotbridge `camera`.
4. **`use_sim_time`**: set on app composable nodes.
5. **Coordinated reset**: Odometry with `header.stamp` older than the invalidate time is ignored so queued pre-reset Odometry cannot re-arm radius. Re-arm logs: `Radius feedback re-armed after accepted post-reset Odometry`.

**EXTERNPROTO:** `mars.wbt` still references R2023a PROTO URLs; R2025a loaded them without requiring a blanket retarget.

## Coordinated reset / radius (A6)

Path: `control_run_service` cmd `'i'`:

1. Disarms radius (`radius_feedback_armed_=false`), records `radius_reset_stamp_`.
2. Clears cached `r_c.r_u` / `r_s.r_u` to NaN.
3. Requests Local SLAM re-init (`ekf_run_service` `'i'`).
4. Reloads control plan / yaw latch flags; stops the running plan.
5. Re-arms radius on the next accepted **post-reset** Odometry (`stamp >= radius_reset_stamp_`) when `Slam_feedback && actual_z_flag`.

**Remaining limitation (not complete reset isolation):** `Robotstate` has no stamp — a radius published before reset but delivered after re-arm can still apply once. Odometry stamped after reset but emitted before EKF re-init finishes can also re-arm (estimator timing unchanged). External `~/reset` without control `'i'` is not observed by control.

## Evidence layout

```
log_a1/a6_evidence/ENV.txt
log_a1/a6_evidence/stageA/RESULT.txt   # GT control
log_a1/a6_evidence/stageB/RESULT.txt   # odometry feedback
```

Operational success ≠ accuracy or legacy/paper parity.
