# A8 — Standard CameraInfo calibration input

Standalone Local SLAM can take camera intrinsics from either YAML parameters
(`Mono_cam_*`) or a remappable `sensor_msgs/msg/CameraInfo` topic. Sources are
**mutually exclusive** (`Calibration_source`); there is no merge or silent
fallback.

## Estimator camera model (unchanged)

| Item | Contract |
|---|---|
| Intrinsics | `fc_u`, `fc_v`, `cc_u`, `cc_v`, `alpha_c` → `KK = [[fc1, α·fc2, cc1],[0,fc2,cc2],[0,0,1]]` |
| Distortion | Bouguet radial only: `distortions[0]=k1`, `[1]=k2`. Tangential `p1,p2` and `k3` are **not** applied by projection / Jacobians |
| Images | Treated as **raw / distorted**. Undistort runs inside `Inverse_projection_model` / `Undistort_a_point` |
| Resolution | `Mono_cam_img_cols × Mono_cam_img_rows` (feature border checks) |

Coefficient names in CameraInfo do **not** imply full OpenCV / `image_geometry`
compatibility — only the subset above is accepted.

## Selection parameter

| Parameter | Values | Default |
|---|---|---|
| `Calibration_source` | `yaml` \| `camera_info` | `yaml` |
| `Camera_info_image_path` | `raw` \| `rectified` | `raw` |
| `Camera_info_waiting_image_bound` | int ≥ 1 | `2` |
| `Camera_info_expected_frame_id` | string (empty = do not enforce) | `""` |

### `yaml` mode

Uses `Mono_cam_*` at startup. `~/camera_info` is **not** subscribed.

### `camera_info` mode

- Subscribes to remappable `~/camera_info` (QoS: keep-last 5, **volatile**
  durability — not transient-local; reliability via
  `Camera_info_qos_reliability` = `reliable` (default) or `best_effort`).
- Waits for the first **valid** CameraInfo before enqueueing visual frames.
- Nonvisual sensors (IMU, body velocity, altitude, range, rangetobase) proceed.
- While waiting, images are held in a bounded deque (oldest dropped).
- After apply, Image `frame_id` / size must match calibration (when frame is set).

## CameraInfo mapping

Verified against Jazzy `sensor_msgs/msg/CameraInfo`: `K`/`R`/`P` are fixed-size
arrays (9 / 9 / 12); `D` is a vector; `distortion_model` is a string.

### `Camera_info_image_path=raw`

| Field | Use |
|---|---|
| `K` | `fx=K[0]`, `fy=K[4]`, `cx=K[2]`, `cy=K[5]`, `alpha_c=K[1]/fy` |
| `D` | plumb_bob `[k1,k2,p1,p2,k3]`; only `k1,k2` used; `|p1|,|p2|,|k3|` must be ≈0 |
| `P` | ignored for intrinsics |

### `Camera_info_image_path=rectified`

| Field | Use |
|---|---|
| `P` | `fx=P[0]`, `fy=P[5]`, `cx=P[2]`, `cy=P[6]`, `alpha_c=P[1]/fy` |
| `D` | must be empty or all ≈0 (rectify **upstream**) |
| `K` | ignored for intrinsics |

**Accepted rectified-image contract:** publish already-undistorted (and, if
applicable, stereo-rectified) images whose geometry matches `P`, with `D≈0`.
Local SLAM does not run `image_proc` / stereo rectification.

### Rejected (clear error / log)

- `distortion_model` other than empty / `plumb_bob` / `plumb_bob_k3` (with k3≈0)
- Nonzero unsupported `D` coeffs (`p1`,`p2`,`k3`, or extras)
- `binning_x/y` not in `{0,1}`
- ROI not full-frame
- Zero / non-finite `fx,fy` (or incomplete `K`/`P` for the selected path)
- Width/height `< 1`

## Lifecycle

| Event | Behavior |
|---|---|
| First valid CameraInfo | Apply to node params + EKF; flush waiting images |
| Repeated identical intrinsics | Ignored (no re-apply) |
| Changed CameraInfo while active | **Rejected** until `~/reset` (or legacy re-init `'i'`) |
| Estimator reset | Allows a subsequent different CameraInfo; clears visual-map latch |

Prefer explicit reinitialization over silently changing an active map.

## Standalone remapping examples

### YAML (default; A6-compatible)

```bash
ros2 launch localslam localslam.launch.py \
  calibration_source:=yaml \
  image_topic:=/camera/image_raw
```

### CameraInfo + raw images (real camera / image_pipeline)

```bash
# Upstream: camera driver (or image_proc) publishing matching image + camera_info.
ros2 launch localslam localslam.launch.py \
  calibration_source:=camera_info \
  camera_info_image_path:=raw \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  camera_info_expected_frame_id:=camera_optical_frame
```

### Rectified images (`image_proc` / stereo)

```bash
# Rectify upstream; Local SLAM expects D≈0 and uses P.
ros2 launch localslam localslam.launch.py \
  calibration_source:=camera_info \
  camera_info_image_path:=rectified \
  image_topic:=/camera/image_rect \
  camera_info_topic:=/camera/camera_info
```

### A6 Webots experiment

Default remains `Calibration_source:=yaml`. Optional CameraInfo path (bridge
publishes `/camera_info` matching mars Mono_cam_*):

```bash
ros2 launch ekf_monoslam_bringup quad_experiment.launch.py \
  calibration_source:=camera_info
```

## Limitations

- No fisheye / equidistant / rational_polynomial / Kannala–Brandt models.
- No online calibration; no silent YAML↔CameraInfo merging.
- Extrinsics (`Mono_cam_2_robot_*`) remain YAML-only in this slice.
- Webots bridge synthesizes CameraInfo from parameters (not the driver’s
  possibly different K); keep those params aligned with image size.
