# A5 — Loop-closure radius (`radius_uncertainty` / `r_u`)

## Original definition (estimator)

**Source:** `EKF::prediction_compute_CL_cov` in `src/localslam/src/ekf_prediction.cpp`  
**Published as:** `interfaces/msg/Robotstate.radius_uncertainty` (also copied into `Lslam` / CL keyframes)

### Separate state (not EKF \(P\))

A dedicated \(2\times 2\) matrix \(P_c\) (`EKF::Pc`) accumulates loop-closure / “home explore” uncertainty. It is **independent** of the robot pose block of the EKF covariance \(P\).

Initialization / reset of \(P_c\):

- Zeroed on system init (`init_euler_system_statet` / quat init).
- Zeroed after a close-loop position update (`Update_pos_with_delta`).
- Accumulation enabled when `Compute_Pc_flag` is true (`CL_Compute_Cp_from_start` at init, or toggled via legacy `ekf_run_service` cmd `'u'`).

### Accumulation (when enabled)

Let

- \(v_x,v_y = x(10),x(11)\) — body linear velocity (m/s),
- \(\Delta t\) — prediction step (s),
- \(v_{\min} =\) `CL_min_vel_cl_uncertainty_update`,
- \(\sigma_x,\sigma_y =\) `CL_Sigma_x`, `CL_Sigma_y`.

If \(\max(|v_x|,|v_y|) > v_{\min}\):

\[
Q = \begin{bmatrix}\sigma_x^2 & 0\\ 0 & \sigma_y^2\end{bmatrix},\qquad
P_c \leftarrow P_c + Q\,\Delta t.
\]

Otherwise \(P_c\) is unchanged for that step.

### Radius formula

\[
r_u = \sqrt{\max\bigl(P_c(0,0),\,P_c(1,1)\bigr)}.
\]

| Property | Value |
|---|---|
| Confidence multiplier | **None** (not \(k\sigma\); raw 1-axis std of the larger \(P_c\) diagonal) |
| Axes | Horizontal only (\(P_c\) is 2-D; no vertical component) |
| Units of \(r_u\) | Metres (used directly as explore half-extents) |
| Frame of \(P_c\) | Axis-aligned 2-D accumulator; **not** rotated by heading; **not** the public odom congruence \(D P D\) |
| Relation to EKF \(P\) | **None** — pose / twist covariance of \(P\) is a different quantity |

### Active control consumer

| Consumer | Use |
|---|---|
| `ControlQuad::go_home` | If no CL while returning home: `explore_area(x_a,y_a)` with \(x_a=y_a=r_u\) |
| `recognize_home` / plan `explore_area` | Uses FOV/`compute_ys`, **not** \(r_u\) |

Legacy path: `Robotstate.radius_uncertainty` → `Slam_callback` → `r_c.r_u`.

## Is A2 `nav_msgs/Odometry` sufficient?

**No.** Published pose covariance is a transform of the EKF robot block \(P\) (A2). It does **not** carry \(P_c\), \(\sigma_x/\sigma_y\) accumulation time, or `Compute_Pc_flag` history.

Therefore:

- Reconstructing \(r_u\) from pose-covariance trace, max eigenvalue, or diagonal terms would **not** match the legacy definition.
- Heading / \(D\) transforms of pose covariance are irrelevant to the true \(r_u\).

### A5 finding

Odometry pose covariance alone is **insufficient** to reconstruct \(r_u\).

### A5b resolution (implemented)

Keep the original ad hoc `Robotstate.radius_uncertainty` field.

- **Primary pose interface:** `nav_msgs/Odometry` (`Slam_feedback_interface=odometry`).
- **Custom exception:** also subscribe to `Robotstate_topic` for **`radius_uncertainty` only**; ignore Robotstate pose/vels.
- Reuse the exact \(P_c\)-based value published by Local SLAM (no covariance substitute).
- Odometry updates do **not** clear \(r_u\); radius availability is independent of pose latch/`actual_z_flag`.
- Control plan re-init clears \(r_u\) to NaN (stale pre-reset guard).
- `go_home`: explore when \(r_u\) is finite (including 0); skip only when non-finite.

`Robotstate` has no stamp — radius is the latest independent sample (see A3 contract).

Legacy mode and `Slam_feedback=false` are unchanged. No new message types or dependencies.

## Tests

- `test_cl_radius` (localslam): formula / accumulation / pose-cov is not a substitute.
- `test_go_home_ru` / `test_odometry_radius_feedback` (control): explore branch + pose/radius isolation / both arrival orders / reset clear.
