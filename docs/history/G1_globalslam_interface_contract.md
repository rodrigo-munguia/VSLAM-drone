# G1 — Global SLAM audit and ROS 2 Jazzy interface contract

**Status:** audit complete; **G2 + G3 + G4 implemented** (loadable GS; session/epoch
KF + correction identity; standard Path/cloud viz; optional `--globalslam`;
corrections gated off). Correction enablement remains a later slice.  
**Scope:** design how Global SLAM reconnects to the migrated Local SLAM stack while preserving algorithm intent and `./launch2.sh` local-only default.  
**Non-goals:** estimator-math changes, UAV accelerator work, GT redesign, reactivating obsolete `plot`.

---

## 1. What Global SLAM currently does

Global SLAM (`globalslam::Gslam` component) runs **two worker threads** after construction:

| Thread | Class | Role |
|---|---|---|
| `GMAP_LOOP` | `GMAP` | Incremental global map: ingest keyframes, ORB re-detect/match, triangulate anchors, visibility graph, local GTSAM bundle adjustment; push a **translation delta** into Local SLAM (`type=bundle_adjustment`). |
| `CLOOP_LOOP` | `CLOOP` | Loop / home-return path: on intermediate keyframes, attempt a loop; on success run pose-graph update, fuse map, BA refine; push a **translation delta** into Local SLAM (`type=close_loop`) and notify Control (`SimpleServ 'c'`). |

**Active close-loop path (code):** `CLOOP::Step2` calls `Get_pos_of_current_kf_from_Home_tag` (ArUco `DICT_6X6_250`, marker size 1.0 m) after travel distance > 10 m, gated by `activate_CL_flag` (`globalslam_run_service` cmd `'c'`). Classic visual PnP `Step` / `Get_pos_of_current_kf` still exist but are **not** invoked from `CLOOP_LOOP` today (`cloop.cpp` ~155–224).

**Correction model (arrangement B):** there is **no** `map→odom` TF. Global SLAM **directly mutates** Local SLAM’s internal robot position, features, and anchors via `LSposUpdate` → `EKF::Update_pos_with_delta` (`ekf.cpp` ~427–529). Public `/odometry` and `/EKFslam/path` therefore **jump** when a correction is applied (subject to `GS_*` / `CL_*` gates in `/lslam` YAML).

**Visualization:** G4 publishes standard `~/global_keyframes` / `~/global_landmarks`
(public `odom`). Legacy `interfaces/msg/Gmap` on `gmap_data_topic` remains during
transition; Plot2/RViz2 consume the standard outputs (not custom Gmap).

---

## 2. Baseline inventory

| Item | Location / note |
|---|---|
| Package | `src/globalslam` — shared lib component `globalslam::Gslam` |
| Params | `config/quad_params.yaml` → `/gslam` (+ LS gates `GS_*` / `CL_*` under `/lslam`) |
| Legacy launch (includes GS) | `launch/quad_slam.launch.py`, `mobile_slam.launch.py`, `omni_slam.launch.py` (cwd YAML; not `launch2`) |
| Canonical experiment | `./launch2.sh` → `ekf_monoslam_bringup` **does not** load Global SLAM |
| Shared type | `src/common/keyframe_types.hpp` (`KEYFRAME`); included by `globalslam_types.hpp` |
| Tests | none under `globalslam/` (only ament_lint stubs) |
| Runtime resources | **No** BoW vocabulary / ORBvoc files. ORB is computed in-process from keyframe images. ArUco uses OpenCV predefined dictionary. Camera intrinsics duplicated in `/gslam` YAML (not CameraInfo today). |
| External libs | **GTSAM** (hard `find_package`), Eigen3, Boost, OpenCV(+aruco), Armadillo/BLAS/LAPACK, cv_bridge |

### Isolated Jazzy build (canonical `build/` / `install/`)

```text
colcon build --packages-select interfaces globalslam
→ FAILED: CMake cannot find GTSAMConfig.cmake / gtsam-config.cmake
```

Additional packaging defects (will fail after GTSAM is present):

- `package.xml` declares almost **no** `<depend>` entries despite CMake requiring rclcpp, interfaces, OpenCV, GTSAM, etc.
- `Kf_callback` is `const` but mutates `Kf_buffer` via non-`mutable` mutex (`globalslam_component.hpp` ~53, `globalslam_component.cpp` ~412–469) — ill-formed once compiled.
- Destructor joins `gmap_loop_` only; **`cloop_loop_` is never joined** (`globalslam_component.cpp` ~75–79).
- OpenCV ArUco API uses legacy `cv::Ptr` / `DetectorParameters::create()` (works on OpenCV 4.6 present here; may need guards on newer OpenCV).

**Do not** invent `build_g1/` / `install_g1/` trees. Fix under the existing overlay once dependencies are available.

---

## 3. End-to-end dataflow (working paths)

### 3.1 Keyframe production (Local SLAM)

Selection in `EKF::` visual match path (`ekf_visual_match_feats.cpp` ~277–327):

- **Map KF** (`NewKF_available` → `Kf_topic`): spaced by frame count, match count, and travel/depth ratio.
- **CL intermediate KF** (`New_KF_cl` → `Kf_cl_topic`): denser samples when matches exist but map-KF criteria fail; carries `radius_uncertainty`.

Published by `EKFslam::pub_kf` / `pub_kf_cl` (`localslam_component.cpp` ~802–887) as `interfaces/msg/Kf`:

| Field | Content | Frame meaning |
|---|---|---|
| `img` | MONO8 image (header empty / no stamp today) | Camera optical image |
| `r2c` | `t_c2r` + quat(`Rr2c`) | Camera←robot extrinsics (internal) |
| `n2r` | `r_N` + quat(`Rn2r`) | Robot pose in **internal** nav (heading-aligned, **down-positive**) |
| `radius_uncertainty` | CL only | Horizontal uncertainty radius (metres, internal XY) |

**Not transmitted:** ORB keypoints/descriptors, landmark IDs, covariances, keyframe index, epoch id, timestamps. Global SLAM **re-detects ORB** from `img` (`gmap_visual_match.cpp` ~17–22).

Internal `KEYFRAME` also stores `Rn2c`, `t_c2n`, match index lists — reconstructed on receive:

`Rn2c = Rr2c * Rn2r`, `t_c2n = r_N + Rn2rᵀ * t_c2r` (`globalslam_component.cpp` ~388–389, ~455–456).

### 3.2 Active ROS boundaries

| Producer | Topic / service | Type | Consumer | Meaning |
|---|---|---|---|---|
| Local SLAM | `Kf_topic` | `interfaces/Kf` | Global SLAM | Map keyframes |
| Local SLAM | `Kf_cl_topic` | `interfaces/Kf` | Global SLAM | Loop/home intermediate KFs |
| Local SLAM | `local_slam_pos_update` | `interfaces/LSposUpdate` | Local SLAM (server) | Apply Δxyz (+ type string) |
| Global SLAM | client → `local_slam_pos_update` | same | Local SLAM | BA / close-loop corrections |
| Global SLAM | `gmap_data_topic` | `interfaces/Gmap` | *(was plot; unused by Plot2)* | Viz of KF poses + anchors |
| Global SLAM | `globalslam_run_service` | `interfaces/SimpleServ` | Keyboard / ops | `'i'` reset, `'0'` log, `'u'` home KF count, `'c'` activate CL |
| Global SLAM | client → `control_run_service` | `SimpleServ` `'c'` | Control | Pause/react to loop event |
| Keyboard `w` | `/EKFslam/reset` + `globalslam_run_service` `'i'` | Trigger + SimpleServ | LS + GS | Coordinated reset (GS best-effort if absent) |

File anchors: component wiring `globalslam_component.cpp` ~44–54, 278–290, 475–518; LS apply `localslam_component.cpp` ~891–908, 422–426; keyboard `keyboard_node.cpp` ~145–152.

### 3.3 Correction direction (executable math)

- BA: `Delta_kf_n = t_opt_last − t_c2n_last` then LS `x[7:9] += Δ` (`gmap_bundle_adjustment.cpp` ~275–279; `ekf.cpp` ~437–445). Same Δ applied to features/anchors (rigid translation of map + robot).
- Close loop: last pose-graph Δ similarly (`cloop_update_gmap.cpp` ~45; `ekf.cpp` ~475–520). Close-loop also zeros `Pc` (CL uncertainty).
- Gates: `/lslam` `GS_xy_update` / `GS_z_update` / max deltas vs `CL_*` (YAML ~209–217). Default **Z updates off** for both.

All of this is in the **internal** nav frame. Public outputs apply existing `D = diag(1,-1,-1)` only at Local SLAM’s odometry/path boundary — **do not apply D in Global SLAM or twice**.

---

## 4. Frame, time, and epoch contracts (as-is → required)

| Quantity | As-is | Required for migration |
|---|---|---|
| Pose parent | Internal nav (heading @ init, +Z down) | Keep for algorithm; document; convert only at public viz |
| Camera | Optical via `Rr2c`/`t_c2r` | Keep; prefer shared CameraInfo/calibration source later |
| Home-tag pose | ArUco `tvecs` remapped `x=ty, y=-tx, z=tz` then used as nav position (`cloop_get_pos.cpp` ~249–264) | Treat as **internal** position hypothesis; validate sign vs public `odom` separately |
| Timestamp | Missing on `Kf.img.header` | **Must add** source stamp (image/observation time) + clock domain (`use_sim_time`) |
| KF identity | Implicit order in buffer | **Must add** monotonic `keyframe_id` + `estimator_epoch_id` (and preferably process generation) |
| Covariance | Not on Δ service | Out of scope for first slices; LS applies open-loop Δ without P update (except CL clears `Pc`) |
| Reset survival | GS `'i'` clears local Gmap copies + reinit flags; LS reset does not auto-clear GS unless keyboard `'w'` | Explicit coordinated reset + stale KF rejection |

### EstimatorReference

`/EKFslam/estimator_reference` provides `epoch_id`, heading/altitude stamps, `valid`. **Useful for associating KF/corrections with an LS epoch**, but:

- `epoch_id` alone does **not** uniquely identify a **process restart** that reused IDs after overlay reload.
- Prefer: subscribe to EstimatorReference; stamp KF with `epoch_id`; reject updates when `valid=false` or epoch mismatch; optionally add `session_id` (UUID) later if multi-process restarts collide.

**GT (`EstimatorReference` / `/ground_truth/path`) must remain independent of Global SLAM.** Never feed GT into GS.

---

## 5. Proposed standardized interfaces

### Keep custom (algorithmic exchange)

| Interface | Why standard msgs are insufficient |
|---|---|
| `interfaces/msg/Kf` (revise, don’t replace) | Needs image + dual transforms + radius + **IDs/epoch/stamps** atomically. `Odometry`/`Path` lose image/descriptors context; splitting Image+Pose breaks atomicity. |
| `interfaces/srv/LSposUpdate` (revise lightly) | Semantic is “apply translational map/robot correction of type BA|CL”, not a pose estimate. Optional: add `epoch_id`, reject stale. |
| `interfaces/msg/Gmap` | Dense KF list + anchors for viz/debug; can **also** mirror to standard viz topics without deleting Gmap mid-migration. |

**Minimal `Kf` revision (recommended):** add `std_msgs/Header` (or stamp fields), `uint64 keyframe_id`, `uint64 estimator_epoch_id`, `bool is_cl_candidate`. Do **not** invent a second parallel keyframe message.

### Add standard (visualization / ops)

| Interface | Role |
|---|---|
| `nav_msgs/Path` e.g. `~/global_keyframes` | Public-frame KF trajectory for RViz2 (apply **one** `D` at publish) |
| `sensor_msgs/PointCloud2` e.g. `~/global_landmarks` | Public-frame anchors for RViz2 |
| `std_srvs/SetBool` `~/enable` | Pause GMAP/CLOOP processing without tearing node |
| `std_srvs/Trigger` `~/reset` | Same semantics as SimpleServ `'i'` |
| Retain `SimpleServ` temporarily | `'0'/'u'/'c'` until mapped to clearer services |

### Do **not** replace with Odometry alone

Global feedback is a **delta applied inside LS**, not a continuous independent global pose stream. Publishing `nav_msgs/Odometry` as “global pose” without defining ownership would collide with `/odometry` semantics.

### Frame ownership decision

| Option | Fit |
|---|---|
| A. Continuous local + separate global correction TF | **Not** current algorithm |
| **B. Direct LS state correction** | **Current — preserve** for algorithm fidelity |
| C. Other | N/A |

**Migration implication:** public `/odometry` may discontinuity on BA/CL. Consequences: control must tolerate jumps (already has CL pause via `'c'`); Plot2/RViz Path will show jumps; GT comparison remains approximate at jump epochs. A continuous-odometry / `map→odom` redesign is a **separate architecture slice**, not G1–G3.

---

## 6. Reset / enable contract (target)

| Scenario | Behavior |
|---|---|
| GS absent | Local-only: KF pubs ignored; `LSposUpdate` unused; keyboard GS calls best-effort no-op (today) |
| GS disable | Stop ingest/BA/CL; hold maps; no Δ |
| GS enable | Resume ingest on current LS epoch only |
| Coordinated reset (`w`) | LS `~/reset` → EstimatorReference `valid=false` → new epoch; GS `~/reset` clears GMAP/CLOOP, KF queues, viz Path/cloud; Control reinit |
| LS reset while GS running | GS must observe invalid/new epoch and **drop** queued KF + pending Δ |
| GS restart while LS running | Late subscribe: wait for latched EstimatorReference; ignore KF with foreign/missing epoch |
| Viz clear | Empty Path + empty cloud on GS reset (and Plot2 clear remains separate) |

---

## 7. Integrated application compatibility

| Surface | Plan |
|---|---|
| `./launch2.sh` | Add explicit `--globalslam true|false` (default **false**); wire optional composable node + remaps in `quad_experiment.launch.py` |
| Viewers | Plot2: optional Gmap/Path/cloud consumer later; RViz2: Path+PointCloud2 displays; `none` unchanged |
| Keyboard | Keep coordinated reset; document GS enable; map `'c'`/`'u'` when GS selected |
| Standalone LS | Unchanged (`localslam.launch.py`) |
| Lost features vs old `plot` | Global anchors / KF poses — restore via **standard** Path/Cloud (not obsolete plot) |

Preserve GT Path / EstimatorReference work and its documented timing limits.

---

## 8. Defects needing separate treatment (not silent fixes inside “just wire topics”)

1. **Jazzy build:** missing GTSAM; empty `package.xml`; const KF callback; cloop thread join.
2. **Odometry jumps** under arrangement B — document; control already reacts to CL; Path/GT UX must expect discontinuities.
3. **No epoch/stamps on KF** — stale corrections across reset are possible today.
4. **Dual calibration** — `/gslam` Mono_cam_* can drift from LS CameraInfo/YAML.
5. **Step vs Step2** — visual-match loop path is effectively dead; home-tag path is active; don’t “fix” by switching without a dedicated validation slice.
6. **Home-tag axis remap** vs public `odom` — validate numerically before trusting RViz overlays.
7. **Destructor / lifecycle** — incomplete shutdown.

---

## 9. Validation strategy (eventual; not G1)

Evidence must show **processing**, not mere node load:

1. KF accepted (log/count; epoch match).
2. BA Δ produced with known synthetic/motion case; LS pose moves by expected internal Δ; public = `D*internal`.
3. At least one real CL/home-tag operation exercised; Control receives `'c'`.
4. Stale-epoch KF and Δ rejected after reset.
5. Local-only `launch2` regression (GS off).
6. Integrated Plot2 and/or RViz2 with global Path/cloud.

Publishing an empty Path is **not** success.

---

## 10. Ordered implementation slices

### G2 — Buildable, independently launchable Global SLAM (DONE)

- **Outcome:** packaging + worker lifecycle; real `globalslam::Gslam` library;
  `Apply_global_corrections:=false`; `~/enable` / `~/reset`;
  `ros2 launch globalslam globalslam.launch.py` (cwd-independent).
- **GTSAM:** **required** for a normal build (actionable `FATAL_ERROR` if missing).
  Optional `-DGLOBALSLAM_POLICY_TESTS_ONLY=ON` builds policy gtests only
  (no component, no launch/config install).
- **Not in default `launch2`.** Corrections remain unsupported until G3+.
- See close-out notes below.

### G3 — Keyframe contract hardening (IDs, stamps, session/epoch) (DONE)

- **Outcome:** `EstimatorReference.session_id` + revised `Kf` / `LSposUpdate`; GS
  admits only matching valid `(session_id, epoch_id)`; stale corrections rejected
  at LS service and again at EKF apply.
- **Limits:** no BA/CL math changes; corrections remain off; not in `launch2`.
- See G3 close-out notes below.

### G4 — Public visualization outputs (DONE)

- **Outcome:** `~/global_keyframes` Path + `~/global_landmarks` PointCloud2 in public `odom` via single `D`; RViz + Plot2; `./launch2.sh --globalslam true` opt-in; empty on reset.
- **Limits:** viz only; keep `Gmap` during transition; corrections remain off.
- **Tests:** `test_g4_viz_conversions`; `tools/smoke/g4_smoke_globalslam_viz.py`.
- **Accept:** see **G4 close-out notes** below.

### G5 — Offline correction-path validation (DONE)

- **Outcome:** Synchronized `LSposUpdate` application results; frame-direction
  evidence `Δ_public = D * Δ_applied_internal`; duplicate/reset/gate races;
  Control `'c'` deferred until `applied=true` and suppressed for G5.
- **Limits:** no map→odom; shipped YAML + integrated launch keep
  `Apply_global_corrections=false`; no vehicle-connected correction test.
- **Tests:** `test_ls_pos_apply_policy`; `tools/smoke/g5_smoke_correction_path.py`.
- **Accept:** see **G5 close-out notes** below.

### G6a — Correction/control coordination (DONE)

- **Outcome:** `CorrectionCoordinate` pause→apply→barrier→release; `apply_timeout`
  cancels pending mutation; `CorrectionApplied` feedback barrier; legacy `'c'`
  compatibility path.
- **Limits:** isolated nodes / fake endpoints only; no Webots; corrections still
  forced off in integrated launch.
- **Tests:** `test_g6_coord_policy`, `test_g6_coord_dispatch`,
  `tools/smoke/g6a_smoke_coordination.py`.
- **Accept:** see **G6a close-out notes** below.

### G6b — Close-loop / home-tag end-to-end (vehicle/Webots)

- **Outcome:** exercise Step2 home-tag; coordinated correction with live control;
  integrated viewer.
- **Limits:** do not expand into place-recognition revival unless scoped.
- **Tests:** tagged world or recorded bag; stale-epoch; local-only regression.
- **Accept:** one logged successful CL with before/after poses and epoch IDs.

---

## 11. Recommended first surgical slice

**Start with G2:** make Global SLAM **build and load** under the existing Jazzy overlay with corrected packaging/lifecycle, **without** enabling corrections in the default experiment. That unblocks contract work (G3+) without risking Local-only GT/control regressions.

---

## G2 close-out notes (implemented)

### GTSAM supply (validated)
- **Release:** borglab/gtsam **tag 4.2.0** (`4f66a491ffc83cf092d0d818b11dc35135521612`),
  API-compatible with `ros-jazzy-gtsam` 4.2.0 (Cal3_S2, GenericProjectionFactor,
  BetweenFactor, LM, Symbol, `addPrior`, 9-arg `Rot3`).
- **Install:** user-owned prefix **outside** this repo (example:
  `$HOME/DEV/deps/gtsam-4.2.0`). Not vendored; system packages not modified.
- **Build options used:** `Release`, `GTSAM_BUILD_TESTS=OFF`,
  `GTSAM_BUILD_EXAMPLES_ALWAYS=OFF`, `GTSAM_BUILD_UNSTABLE=OFF`,
  `GTSAM_USE_SYSTEM_EIGEN=ON`, `GTSAM_WITH_TBB=ON`,
  `GTSAM_BUILD_WITH_MARCH_NATIVE=OFF`; parallelism `-j4`.
- **Expose at build/run time** (no absolute path in project source):
  ```bash
  # copy tools/gtsam_prefix.env.example → tools/gtsam_prefix.env and edit
  set -a && source tools/gtsam_prefix.env && set +a
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select interfaces globalslam \
    --build-base build --install-base install
  source install/setup.bash
  # keep LD_LIBRARY_PATH from the env file so libgtsam resolves
  ```
- **Normal build without GTSAM:** CMake `FATAL_ERROR` with the export hints above.
- **Policy-only (opt-in):**
  `colcon build --packages-select globalslam --cmake-args -DGLOBALSLAM_POLICY_TESTS_ONLY=ON`
  — builds `test_g2_run_control` only; **does not** build/register the component
  or install launch/config.

### Packaging
- Shared lib `libglobalslam_component.so` + `rclcpp_components` registration
  `globalslam::Gslam`.
- Installed `share/globalslam/config` + `launch/globalslam.launch.py` only on
  normal (GTSAM) builds.
- `package.xml` declares ROS deps; GTSAM remains an external CMake dependency.

### Worker / enable / reset concurrency
- Workers start only after ctor state/`setParameters` init; both joined in
  `stop_workers()` from the destructor.
- Lock order: `run_ctrl` → `rx_kf` → `rx_kf_cl` → `get_gm` (fixed gm↔kf deadlock).
- Commit of shared Gmap / loop-closed flags and the decision to emit
  corrections/control happen under one critical section (generation +
  `processing_enabled` + correction gate). No check-then-unlocked-commit.
- Disable/`reset` clear pending KF under the same lock hierarchy; old
  in-flight steps cannot repopulate cleared state after a generation bump.
- Outbound `LSposUpdate` / Control `'c'`: non-blocking `service_is_ready` +
  `async_send_request` **outside** locks (never wait on unavailable services).
- **Dispatch boundary:** once `async_send_request` returns, the request
  **cannot be recalled**. Disable/reset responses state this honestly.

### Correction gate
- `Apply_global_corrections` default **false** (kept false for all G2 validation).
- Suppresses BA Δ and Control `'c'` with no deferred queue.

### Standalone + smoke
```bash
source install/setup.bash   # plus GTSAM LD_LIBRARY_PATH
cd /tmp && ros2 launch globalslam globalslam.launch.py
ros2 service call /gslam/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /gslam/reset std_srvs/srv/Trigger "{}"
# Real component lifecycle (minimal KF ingest; corrections off):
python3 tools/smoke/g2_smoke_globalslam.py
```
Smoke covered: load/spin, enable/disable/re-enable, `~/reset` + legacy `'i'`,
repeated reset, unload/reload, shutdown with both workers, fake downstream
(zero outbound LS/Control `'c'`), and absent downstream (no hang).
Workers exercised with a **minimal** KF fixture (not a full BA/CL dataset).

### Calibration
Package YAML `Mono_cam_*` must match incoming keyframe images. CameraInfo not
integrated in G2.

### Remaining after G2 (superseded by G3 close-out)
See **G3 close-out notes** below.

---

## G3 close-out notes (implemented)

### Identity scopes
| Entity | Scope |
|---|---|
| `session_id` (UUID v4) | One per Local SLAM **component instance**; new process → new session |
| `epoch_id` | Increments on each successful `system_init` after re-init; disable/re-enable preserves |
| `keyframe_id` | Monotonic per `(session, epoch)`, shared across map + close-loop streams (mutually exclusive per frame) |
| `correction_id` | Monotonic per Global SLAM emission; LS rejects duplicates |

Identity is `(session_id, epoch_id)`. Same-epoch altitude/metadata EstimatorReference updates do **not** reset Global SLAM.

### Message / service fields
- `EstimatorReference`: + `session_id` (unique_identifier_msgs/UUID); initial invalid published at ctor.
- `Kf`: + `header`, `session_id`, `epoch_id`, `keyframe_id`, `source_stamp_valid`, `stream`; poses remain **internal** nav/cam (not public odom/base_link). Identity/stamp captured at KF **selection**, not publish.
- `LSposUpdate`: + `session_id`, `epoch_id`, `keyframe_id`, `correction_id`;
  response `accepted` + `reason` + G5 `applied` / `applied_delta_*` / `outcome`.
  Identity-less requests rejected.

### Policies
- **Missing source stamp:** GS drops unless `Accept_missing_source_stamp:=true` (default false). Never fabricate `now()`.
- **KF before reference:** drop with diagnostics (no identity adoption from KF).
- **Ordering:** reference and KF are independent topics; generation gates prevent stale commits.
- **Delayed old session:** retired after a newer session is adopted; cannot switch back.
- **Duplicate KF:** `(keyframe_id, stream)` per active epoch.
- **Calibration:** GS requires matching `Mono_cam` resolution vs KF image; YAML↔YAML matching is the supported config. LS `camera_info` without matching GS YAML is unsupported (size mismatch rejected).
- **GS restart:** does **not** replay past KFs; prior global map/home is not reconstructed from future frames alone.
- **Dispatch:** async `LSposUpdate` cannot be recalled once sent. Control `'c'`
  is deferred until LS reports `applied=true` (G5); still gated by
  `Apply_global_corrections`.

### Reset / apply races
- LS invalidates EstimatorReference and clears queued corrections under the apply mutex.
- LS re-validates identity at EKF mutation time.
- GS clears queues/Gmap and bumps generation on reference invalid/session/epoch change.
- Correction provenance is stamped from the originating KF, not the live reference at send.

### Validation evidence
- Unit: `test_ls_pos_update_policy`, `test_g3_admission`, plus existing localslam/webotbridge/globalslam suites.
- Smoke: `tools/smoke/g3_smoke_identity.py` (corrections off; isolated `ROS_DOMAIN_ID`); `tools/smoke/g2_smoke_globalslam.py` still passes.
- Rebuild **all** packages that use `interfaces` after this msg/srv change; restart communicating nodes.

### Remaining before correction enablement (post-G5)
- Open `Apply_global_corrections` only with an explicit experiment flag after
  reviewing G5 live-enable blockers (pause-before-dispatch for close-loop).
- Optional full calibration transport on `Kf` (beyond YAML↔YAML gate + resolution check).

---

## G4 close-out notes (implemented)

### Snapshot ownership
- Authoritative viz map = committed shared `Gmap` after GMAP/CLOOP workers
  copy results under `mutex_get_gm` + generation checks.
- Standard Path/cloud are built from one locked snapshot (keyframes + landmarks
  together), then serialized/published **outside** worker locks; generation is
  re-checked before publish so old work cannot latch after reset.
- Disable retains last committed viz (no empty clear); reset / reference
  invalidation / session-epoch change publishes empty Path+cloud.
- Same-epoch altitude metadata does **not** clear the global map (G3 policy).

### Public contracts
| Topic | Type | Frame | Reference point | Header stamp | QoS |
|---|---|---|---|---|---|
| `/gslam/global_keyframes` (`~/global_keyframes`) | `nav_msgs/Path` | `Public_frame_id` (default `odom`) | **Robot body** origin reconstructed from committed camera pose + extrinsics (`Rn2r=Rr2cᵀ Rn2c`, `r_N=t_c2n−Rn2rᵀ t_c2r`) | Path header = publication/snapshot time; each pose keeps KF source stamp when valid | Reliable, Transient Local, KeepLast(1) |
| `/gslam/global_landmarks` (`~/global_landmarks`) | `sensor_msgs/PointCloud2` | same | Sparse XYZ anchors | same stamp as Path | same |

- Conversion: `p_pub = D * p_int`, `R_odom←body = D * Rn2rᵀ * D` (not D on quat components).
- Not a `map` frame, not dense reconstruction, not continuous global odometry.
- Path/cloud do **not** carry session/epoch; stale publish prevented by internal generation.
- Legacy `gmap_data_topic` retained (internal camera poses, no D); RViz2/Plot2 use standard outputs only.

### Calibration (supported / unsupported)
- **Supported:** Local `Calibration_source=yaml` with `/lslam` and `/gslam`
  `Mono_cam_*` (rows/cols, fc, cc, alpha_c, distortions) matching in
  `config/quad_params.yaml`. Integrated launch **rejects** mismatches and
  rejects `camera_info`.
- **Unsupported in G4:** CameraInfo-driven Local SLAM with a different GS YAML;
  full runtime calibration transport on `Kf` deferred.

### Launch / operator
```bash
# Local-only (default)
./launch2.sh

# Opt-in Global SLAM (corrections forced false; processing starts enabled)
set -a && source tools/gtsam_prefix.env && set +a   # LD_LIBRARY_PATH for libgtsam
./launch2.sh --globalslam true
# Disable GS processing without tearing down:
ros2 service call /gslam/enable std_srvs/srv/SetBool "{data: false}"
```

### Parameters
| Param | Location | Semantics |
|---|---|---|
| `Public_frame_id` | `/gslam` YAML | Public parent frame (default `odom`) |
| `Path_max_poses` / `Landmark_cloud_max_points` | `/gslam` | `0` = full snapshot; limits truncate **viz copies only** |
| `Apply_global_corrections` | `/gslam` | Forced `false` in integrated launch |
| `Processing_start_enabled` | `/gslam` | Standalone default `false`; integrated forces `true` |
| `Subscribe_global_outputs` | `/plot2` | Set true by integrated launch when GS on |

### Remaining before offline correction validation
- (superseded by G5 close-out)

---

## G5 close-out notes (implemented)

### Transaction (BA / close_loop)
1. GS worker commits map under generation+identity gates → computes
   `Delta_kf_n` (internal nav) → assigns `correction_id =
   (boot_nonce<<32)|seq` → async `LSposUpdate` (cannot recall).
2. LS service: reject if disabled / not initialized / busy / admit policy
   fail; else queue **one** pending request and **wait** on the EKF worker
   (separate thread; ≤2 s) for the real apply outcome.
3. EKF re-checks identity under the apply mutex; runs
   `Update_pos_with_delta` (per-axis GS_*/CL_* gates); publishes corrected
   odom/Robotstate in the same cycle; returns response.
4. GS response callback is generation-gated; Control `'c'` only if
   `notify_control_on_apply && resp.applied` (close_loop only). BA never
   sends `'c'`.

### Result semantics (`LSposUpdate` response)
| Field | Meaning |
|---|---|
| `accepted` | Admitted and queued (not “looks valid only”) |
| `applied` | At least one translation axis mutated EKF state |
| `applied_delta_*` | Actual internal nav Δ after gates (partial axes OK) |
| `outcome` | `rejected` \| `applied` \| `gated_noop` \| `busy` \| `localslam_disabled` \| `not_initialized` \| `reset_before_apply` \| `apply_timeout` |

- Do **not** treat `accepted=true` alone as success.
- `gated_noop`: admitted, gates blocked all axes; `correction_id` consumed
  (retry same id → `duplicate_correction_id`). Treated as not-applied.
- Zero requested delta → rejected (`zero_delta`) before admission.
- Per-axis: XY is all-or-nothing (`hypot`); Z independent. Default YAML has
  `GS_z_update`/`CL_z_update` **false**.
- Magnitude uses **absolute** Z (historical signed `dz < max` accepted large
  negative Z — fixed in G5 apply policy).
- Covariance **P** unchanged; close_loop **Pc** reset only when
  `any_applied`. Path history is **not** cleared on correction (jumps OK).

### Duplicate / identity / bookkeeping
- `correction_id` scope: Global SLAM **process instance** (boot nonce in high
  32 bits) so LS session continuity after GS restart cannot collide.
- Consumed at **admission** for the active `(session, epoch)`; cleared on
  epoch bump / reset. No mid-epoch eviction; reject
  `correction_bookkeeping_full` at 4096 rather than silently re-allow.
- At most one pending; second concurrent request → `busy` (id not consumed).
- Stale responses after GS reset: ignored via `dispatch_generation`.

### Frame evidence
- Internal apply → public odom: `Δ_pub = D * Δ_int` with `D=diag(1,-1,-1)`.
- Smoke injects asymmetric `(0.05,-0.07,0.04)` with Z gated off and checks
  public jump against applied XY only.
- Unchanged by design: orientation, velocity, relative observations,
  covariance P (existing estimator behavior; not redesigned).

### Control `'c'` policy (G5)
- Suppressed whenever `Apply_global_corrections=false`.
- When corrections are opted in (test YAML only): emit **after** LS reports
  `applied=true`, not at dispatch. No `'c'` for reject / gated_noop /
  duplicate / stale generation.
- **Live-enable blocker:** original control may need pause-before-dispatch
  for close-loop; deferring `'c'` until after apply does **not** provide an
  early pause. Documented — do not rewrite control to conceal this. Home-tag
  E2E remains G6.

### Validation evidence
- Unit: `test_ls_pos_update_policy`, `test_ls_pos_apply_policy`.
- Smoke: `tools/smoke/g5_smoke_correction_path.py` (isolated domain;
  fake `control_run_service`; dedicated Apply=true YAML for BA attempt only).
- Receiver-path (injected Δ) vs genuine BA-generated Δ are reported
  **separately**. G5 smoke observed a nonzero public odom jump during an
  Apply=true synthetic KF feed (BA evidence); frame-direction proof for
  `D` remains the controlled injected asymmetric Δ.

### Still gated
- `Apply_global_corrections` default **false**; integrated
  `./launch2.sh --globalslam true` still forces false.
- Local-only remains the default launch.
- No vehicle-connected correction test.

---

## G6a close-out notes (implemented)

### Audit (legacy Control `'c'`)
- `'c'` sets `cl_flag` so `go_point`/`explore`/`go_home` interrupt and
  re-navigate to home `(0,0)` after a map jump — **not** a plan pause.
- Clearing `run_control_plan` stops progression but mid-`go_point` could
  still publish non-zero `cmd_vel` until the loop observes the flag (and
  previously exited **without** a guaranteed zero twist).
- BA and close_loop both jump public odometry; both need the same protocol
  when coordination is required.
- Hold targets are **navigation goals** in the feedback frame (e.g. home
  `(0,0)`); do **not** add the applied Δ to waypoints.

### Interfaces
| Name | Role |
|---|---|
| `interfaces/srv/CorrectionCoordinate` | prepare_pause / release / abort / query with session, epoch, transaction_id, owner_generation |
| `interfaces/msg/CorrectionApplied` | Latched LS notice after mutation + odom publish (feedback barrier) |

Why custom: `SetBool` cannot carry ownership, transaction identity, or
structured outcomes.

### Transaction
`prepare_pause` (ack when cmd source quiescent) → dispatch `LSposUpdate` →
resolve outcome → wait `CorrectionApplied` (if applied) → `release` or
`abort`. At most one owned pause; conflicts rejected; ops idempotent for
the same `(owner, transaction, session, epoch)`.

Pause ack guarantees: plan progression held; `coord_suppress_cmd` armed;
zero twist published; `in_plan_command` cleared. **Not** vehicle velocity
zero / physical hover. Operator stop/reset supersedes resume.

### apply_timeout (G5 fix)
Service timeout under the apply mutex: if still pending → **cancel**
(`outcome=cancelled`, no later mutation). If worker already finished while
reacquiring the lock → return the real result. Unreachable in-flight case
reports `apply_pending` (unresolved — do **not** auto-resume).

### GS `Coordinate_mode` (dormant while Apply=false)
`required` | `absent_ok` | `offline_bypass`. Default `required`. Both BA and
CL require pause when coordination applies. Coordinated CL uses
`release(signal_close_loop=true)` instead of also sending SimpleServ `'c'`.

### Remaining live-enable blockers (pre-G6b)
- Acknowledgement ≠ hover; no proof of near-zero body velocity.
- No Webots/vehicle-connected correction test in G6a.
- Integrated launch still forces `Apply_global_corrections=false` unless
  `--global-corrections true`.

---

## G6b close-out notes (implemented)

### Feedback barrier (what CorrectionApplied proves)
`CorrectionApplied` proves Local SLAM mutated and published a post-correction
`nav_msgs/Odometry` **plus** matching `OdometryMeta` with unique
`snapshot_seq`. It does **not** prove Control consumed that pose (separate
topics; no cross-topic ordering). G6b adds:

| Name | Role |
|---|---|
| `interfaces/msg/OdometryMeta` | Companion to each `~/odometry`; `snapshot_seq` is the association key |
| `CorrectionApplied.snapshot_seq` | Identifies the post-correction snapshot |
| `interfaces/msg/CorrectionFeedbackAccepted` | Control accepted required snapshot (or GT path) |

**Odometry feedback:** release/resume only after Control accepts meta with
matching session/epoch, `last_correction_id`, and `snapshot_seq >= required`
(or a newer compatible accepted snapshot). Stale identity rejected; reset
clears the barrier. Arrival order (event↔odom↔meta) handled with bounded
buffers. Stamp collision: prefer highest `snapshot_seq` for a shared stamp.

**GT feedback (`Slam_feedback=false`):** corrected odometry is **not** the
plan pose source (GPS/Webots). Barrier clears on matching `CorrectionApplied`
while coordinating (`feedback_mode=gt`). GT control coordinates and target
semantics unchanged. Pause still suppresses plan `cmd_vel`.

### Webots zero-Twist hold (audit)
`Control` → `cmd_vel` Twist zero → `webot_robot_driver.__cmd_vel_callback`
stores `__target_twist`. While airborne, the driver tracks **target linear
velocity → 0** (velocity PI → attitude refs) and holds `__vertical_ref`
(altitude hold via thrust). It is **not** zero actuator commands and **not**
a certified physical hover. Integrators keep winding on velocity error; land
service (`webot_service` `'l'`) and operator stop retain priority via
existing paths. Plan suppression reaches the driver when no other `cmd_vel`
publisher defeats it (operator flight keys can). Do not retune gains here.

### Opt-in enablement (G6d)
```bash
# Independent: BA Local feedback off + tag Local feedback on
./launch2.sh --globalslam true \
  --apply-ba-corrections false --apply-tag-loop-corrections true \
  --slam-feedback false
# Compatibility master (Apply_* for enabled sources only; never enables visual)
./launch2.sh --globalslam true --global-corrections true --slam-feedback false
./launch2.sh --globalslam true --global-corrections true \
  --slam-feedback true --slam-feedback-interface odometry \
  --control-plan "$PWD/control_plan_g6b.txt"
```
Parameters: `Enable_global_ba` / `Enable_tag_loop_closure` /
`Enable_visual_loop_closure` vs `Apply_ba_corrections` /
`Apply_tag_loop_corrections` / `Apply_visual_loop_corrections` (all Apply
default **false**). Tag+visual both enabled is rejected. Any Local feedback
forces `Coordinate_mode=required`; reject `absent_ok`/`offline_bypass`.
Informational `/gslam/global_loop_committed` is independent of Apply_*.
See `docs/evidence/g6b/G6d_CLOSEOUT.md`.

### Home-tag close-loop (active default path)
- Dictionary: OpenCV `DICT_6X6_250`; marker texture `marker23.png` (ID 23).
- Physical size in `mars.wbt`: 1 m × 1 m plane / box — matches
  `estimatePoseSingleMarkers(..., 1.0, ...)`.
- Activation: `globalslam_run_service` `'c'` (plan `GoHome` or operator);
  travel distance &gt; 10 m since last CL; tag within `CL_tag_max_xy_dis` /
  `CL_tag_max_xy_dis_px`.
- Pose: production `t_c2n = −Rn2cᵀ·tvec` + epoch `p_H_N` (G6b.2/G6b.3).
- Visual ORB/PnP `CLOOP::Step` is exclusive when `Enable_visual_loop_closure`
  (default off); not mission-claimed without candidate evidence.
- GT must never feed correction computation (evaluation only).

### Evidence
Prefer rosbag2 when available; else `[G6b-TXN]` structured logs.
Convention: `docs/evidence/g6b/` (notes) and gitignored `log/` run captures.
Plot2 latest/latest GT error remains approximate.

### Acceptance status
See repository close-out report after staged Webots runs. Unit/node barrier
tests are mandatory; Webots Stage A/B/C may be partial if a stage blocks.
No real-hardware claim.
