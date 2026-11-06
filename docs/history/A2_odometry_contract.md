# A2 — `nav_msgs/Odometry` contract (Local SLAM)

## Topic and frames

| Item | Value |
|---|---|
| Topic | `~/odometry` (node-relative; launch may remap to `/odometry`) |
| Type | `nav_msgs/msg/Odometry` |
| Parent frame | parameter `Odometry_frame_id` (default `odom`) → `header.frame_id` |
| Child frame | parameter `Odometry_child_frame_id` (default `base_link`) → `child_frame_id` |

Pose is expressed in the parent frame. Linear and angular twist are expressed in the child (body) frame.

Custom `Robotstate_topic` remains unchanged and is still published.

## Internal state (Euler model)

Index meaning (unchanged estimator):

| Index | Quantity |
|---|---|
| 0 | unused |
| 1:3 | Euler angles φ, θ, ψ |
| 4:6 | Body angular rate ω (not Euler-angle derivatives) |
| 7:9 | Position of the **robot body origin** in the internal navigation frame |
| 10:12 | Body linear velocity |

`Euler_to_Ra2b(φ,θ,ψ)` yields \(R_{n\leftarrow r}\) (nav → robot). Prediction uses \(\dot e = R_{b\leftarrow e}\,\omega\) and \(\dot p = R_{r\leftarrow n}\,v\).

## Public axes

Same axis map validated in A1:

\[
D = \mathrm{diag}(1,-1,-1)
\]

Coordinates in the public frames relate to internal nav/body by \(v_{\mathrm{pub}} = D\,v_{\mathrm{int}}\).

**Do not label this as geographic ENU.** It is a local, heading-aligned navigation frame fixed at estimator initialization (`Robot_init_*`, yaw measurements relative to `yaw_at_home`). Relation to east/north is unknown without an external geodetic datum (Webots world axes are z-up; public axes flip y and z relative to the internal estimator bases).

## Pose / twist mapping

Let \(R_{n\leftarrow r} = \mathrm{Euler\_to\_Ra2b}(\phi,\theta,\psi)\), \(R_{r\leftarrow n}=R_{n\leftarrow r}^{\mathsf T}\).

| Field | Formula |
|---|---|
| Position | \(p_{\mathrm{pub}} = D\,p_{\mathrm{int}}\) |
| Orientation | \(R_{\mathrm{odom}\leftarrow\mathrm{body}} = D\,R_{r\leftarrow n}\,D\); quaternion of that DCM, unit, \(w\ge 0\) |
| Linear twist | \(v_{\mathrm{pub}} = D\,v_{\mathrm{int}}\) (body) |
| Angular twist | \(\omega_{\mathrm{pub}} = D\,\omega_{\mathrm{int}}\) (body rates) |

Physical point: robot body origin used by the EKF (camera lever arm is separate state geometry, not this pose).

## Covariance

From the consistent \(13\times 13\) robot block of \(P\):

- Position / body-velocity / body-rate blocks: congruence \(D\,(\cdot)\,D\), retaining cross-terms inside pose and inside twist.
- Rotational pose uncertainty is **not** a raw Euler copy. With
  \(G_n = R_{r\leftarrow n}\,R_{e\leftarrow b}\) mapping Euler-angle rates to internal-nav angular velocity
  (\(R_{e\leftarrow b}=\mathrm{inv}(R_{b\leftarrow e})\) from prediction),
  \[
  J = D\,G_n,\qquad
  \Sigma_{\mathrm{rot}} = J\,P_{e}\,J^{\mathsf T},\qquad
  \Sigma_{p,\mathrm{rot}} = D\,P_{p,e}\,J^{\mathsf T}.
  \]
- ROS pose covariance stores 3-axis rotational uncertainty about the parent frame (rotation-vector sense), while the message orientation is a quaternion.

Invalid cases (skip publish, no fabricated uncertainty): non-finite state/P, pitch singularity (\(|\cos\theta|\) below threshold), mapped covariance failing PSD within numerical tolerance.

## Timestamp and lifecycle

- Stamp: `Node::get_clock()->now()` at publish (configured ROS clock). **Publication time**, not a measurement epoch. Prediction still uses wall-clock \(\Delta t\) (A1).
- Published only when the estimator marks a fresh initialized robot-state snapshot (`NewRobotState_available` after a step).
- Disable: EKF step loop stops → no new odometry.
- Reset: `initialized=false` until re-init → no odometry until a new valid snapshot exists (no stale republish).

## Validation gaps

- Webots end-to-end still blocked.
- Smoke may run with `Visual_update=false`; full visual path unvalidated.
- Control / plot / mission consumers not cut over to `~/odometry` in A2.
