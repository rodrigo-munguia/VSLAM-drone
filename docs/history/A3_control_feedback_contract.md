# A3 / A5b — Control feedback from standard Odometry

## Parameters

| Parameter | Meaning |
|---|---|
| `Slam_feedback` | `false`: existing Webots GT IMU/GPS path. `true`: SLAM feedback. **No auto-fallback.** |
| `Slam_feedback_interface` | When SLAM feedback is used: `odometry` (default) or `legacy`. Pose source is exclusive. |
| `Odometry_topic` | Remappable; launch maps to `/odometry`. |
| `Odometry_expect_frame_id` / `Odometry_expect_child_frame_id` | Default `odom` / `base_link`. Unexpected frames rejected. |
| `Ekf_enable_service` | Default `/EKFslam/enable` (`std_srvs/SetBool`) for StartSLAM/StopSLAM. |

## Primary pose interface

**Odometry is the primary SLAM pose-feedback interface** when `Slam_feedback_interface=odometry`.

`go_point` / exploration use `r_c.{x,y,z,yaw}` (and roll/pitch in `compute_ys`). Body velocities and angular rates from Robotstate/`Odometry.twist` are **not** consumed by the controller today.

## Custom radius exception (A5b)

`radius_uncertainty` (`r_c.r_u`) is **not** on Odometry and **cannot** be reconstructed from pose covariance (see `docs/A5_cl_radius_contract.md`).

**Intentional exception:** with `Slam_feedback_interface=odometry`, control also subscribes to `Robotstate_topic` **only** for `radius_uncertainty`. Pose, orientation, velocity, and other Robotstate fields are ignored in that mode.

| Mode | Pose / attitude | `r_u` |
|---|---|---|
| `odometry` | `nav_msgs/Odometry` only | `Robotstate.radius_uncertainty` only |
| `legacy` | full `Robotstate` (unchanged) | same message |
| `Slam_feedback=false` | Webots GT (unchanged) | not from SLAM |

`go_home` explore sizing: \(x_a=y_a=r_u\) when \(r_u\) is finite (including 0). Non-finite \(r_u\) → skip explore with explicit log.

### Association / timestamps

`interfaces/msg/Robotstate` has **no header/stamp**. Local SLAM publishes Robotstate and `~/odometry` from the same accepted snapshot on one worker, but control cannot pair them by timestamp. Radius is treated as the **latest independent sample**. This limitation is accepted for the ad hoc interface.

Radius availability is **independent** of pose-feedback readiness (`actual_z_flag` / yaw latch). Odometry updates must not erase a valid radius. Control plan re-init clears \(r_u\) to NaN to avoid stale pre-reset use.

## Mapping (verified vs legacy)

A2 orientation `R_odom←body` matches ROS body→parent. With \(D=\mathrm{diag}(1,-1,-1)\):

| Control `r_c` | From Odometry |
|---|---|
| x | \(x_{\mathrm{pub}}\) |
| y | \(-y_{\mathrm{pub}}\) |
| z | \(z_{\mathrm{pub}} + \mathrm{init\_z}\) |
| roll / pitch / yaw | public RPY; yaw relative to first public yaw sample |

Equivalent to legacy Robotstate path after its internal sign flips.

## Pose availability

Same gate as legacy SLAM pose feedback: `Slam_feedback && actual_z_flag` (GT GPS must latch `init_z`). First odometry/legacy sample only latches yaw; subsequent samples update pose fields of `r_c`.

## Enable / reset

`StartSLAM` / `StopSLAM` → `SetBool` on `Ekf_enable_service`. Visual-update / uncertainty still use legacy `ekf_run_service` cmds `v` / `u`.

**Coordinated reset (A6):** control `control_run_service` cmd `'i'` disarms radius acceptance, clears \(r_u\) to NaN, requests EKF re-init (`ekf_run_service` `'i'`), and reloads the plan. Radius callbacks are ignored until the next accepted Odometry pose update re-arms acceptance (drops queued pre-reset Robotstate deliveries on the default exclusive callback group). Arbitrary external EKF `~/reset` without this path is not observed by control.

## Gaps

- Webots E2E requires Webots R2025a + `ros-jazzy-webots-ros2` (see `docs/A6_webots_jazzy_bringup.md`).
- No stamp-based association between Odometry and Robotstate radius samples.
