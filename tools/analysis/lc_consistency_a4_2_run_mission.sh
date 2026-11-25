#!/usr/bin/env bash
# GSLAM-LC-CONSISTENCY-A4.2.2: one extended-revisit mission with discrete sync ON.
# Reuses A3.2 geometry / feedback / diagnostics; writes under lc_consistency_a4.
# Usage: lc_consistency_a4_run_mission.sh [run_id] [params_yaml] [exclusion_px]
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ID="${1:-run_a4_2_exclusion10_sync}"
PARAMS="${2:-$ROOT/config/experiments/quad_params_a3_2a_exclusion10.yaml}"
EXCL_PX="${3:-10}"
OUT="$ROOT/docs/evidence/gslam_lc_consistency_a4_2/raw/${RUN_ID}"
AUDIT="$OUT/map_mgmt_audit"
mkdir -p "$OUT" "$OUT/ba_d5" "$AUDIT" "$OUT/landmark_audit" "$OUT/ba_audit"
: > "$OUT/launch_mission.log"
rm -f /tmp/a42_recorder_stop

if [[ ! -f "$PARAMS" ]]; then
  echo "error: params file not found: $PARAMS" >&2
  exit 1
fi
if ! rg -q "Init_min_distance_to_previous_mapped_features:[[:space:]]*${EXCL_PX}([[:space:]]|$)" "$PARAMS"; then
  echo "error: params file does not set Init_min_distance_to_previous_mapped_features: ${EXCL_PX}" >&2
  exit 1
fi
if ! rg -q "Apply_visual_loop_corrections:[[:space:]]*true" "$PARAMS"; then
  echo "error: A4 requires Apply_visual_loop_corrections: true" >&2
  exit 1
fi
if ! rg -q "Enable_visual_loop_closure:[[:space:]]*true" "$PARAMS"; then
  echo "error: A4 requires Enable_visual_loop_closure: true" >&2
  exit 1
fi

PLAN="$ROOT/control_plan_a3f_rect380_revisit40.txt"
WORLD="earth_aerial_a3d0_380_extended.wbt"

unset GSLAM_INIT_HYBRID_KF_Z_ABS || true
export GSLAM_KF_Z_ADMIT_DIAG=1
export GSLAM_BA_D5_DIR="$OUT/ba_d5"
export GSLAM_MAP_MGMT_AUDIT_DIR="$AUDIT"
export GSLAM_MAP_MGMT_AUDIT_ENABLE=1
export GSLAM_LANDMARK_AUDIT_DIR="$OUT/landmark_audit"
export GSLAM_BA_AUDIT_DIR="$OUT/ba_audit"

echo "LC-CONSISTENCY-A4.2 ${RUN_ID} exclusion=${EXCL_PX}px sync=ON out=${OUT}" | tee "$OUT/run_meta.txt"
{
  echo "run_id=${RUN_ID}"
  echo "series=GSLAM-LC-CONSISTENCY-A4.2.2"
  echo "Init_min_distance_to_previous_mapped_features=${EXCL_PX}"
  echo "enable_visual_loop_closure=true"
  echo "apply_visual_loop_corrections=true"
  echo "apply_ba_corrections=false"
  echo "discrete_synchronization=ON (A3 binaries)"
  echo "Init_hybrid_kf_z_absolute_enable=true (YAML/default; no env override)"
  echo "GSLAM_MAP_MGMT_AUDIT_DIR=${GSLAM_MAP_MGMT_AUDIT_DIR}"
  echo "GSLAM_LANDMARK_AUDIT_DIR=${GSLAM_LANDMARK_AUDIT_DIR}"
  echo "GSLAM_BA_AUDIT_DIR=${GSLAM_BA_AUDIT_DIR}"
  echo "GSLAM_BA_D5_DIR=${GSLAM_BA_D5_DIR}"
  echo "world=${WORLD}"
  echo "params=${PARAMS}"
  echo "plan=${PLAN}"
  echo "traj_v_max_xy=5.0"
  echo "tracker_v_max_xy=5.5"
  echo "association_horizon_kf=20"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "git_head=$(cd "$ROOT" && git rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "globalslam_so_mtime=$(stat -c %y "$ROOT/install/globalslam/lib/libglobalslam_component.so" 2>/dev/null || echo unknown)"
  echo "localslam_so_mtime=$(stat -c %y "$ROOT/install/localslam/lib/liblocalslam_component.so" 2>/dev/null || echo unknown)"
} >> "$OUT/run_meta.txt"

(
  cd "$ROOT"
  ./launch4.sh --world "$WORLD" \
    --params-file "$PARAMS" \
    --traj-v-max-xy 5.0 \
    --tracker-v-max-xy 5.5 \
    --globalslam true \
    --enable-tag-loop-closure false \
    --enable-visual-loop-closure true \
    --apply-ba-corrections false \
    --apply-tag-loop-corrections false \
    --apply-visual-loop-corrections true \
    --auto-start-plan true \
    --control-plan "$PLAN" \
    > "$OUT/launch_mission.log" 2>&1
) &
LAUNCH_PID=$!
echo "launch_pid=${LAUNCH_PID}" >> "$OUT/run_meta.txt"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u

echo "waiting for /odometry ..."
TOPICS_OK=0
for i in $(seq 1 180); do
  if ros2 topic list 2>/dev/null | grep -qx /odometry; then
    echo "topics up after ${i}s"
    TOPICS_OK=1
    break
  fi
  sleep 1
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "error: launch exited early" >&2
    tail -80 "$OUT/launch_mission.log" >&2 || true
    exit 1
  fi
done
if [[ "$TOPICS_OK" -ne 1 ]]; then
  echo "error: /odometry never appeared" >&2
  exit 1
fi

for i in $(seq 1 120); do
  if rg -q "hybrid_z_abs=|TakeOff: unified|control-> TakeOff" "$OUT/launch_mission.log" 2>/dev/null; then
    echo "ready/plan after ${i}s"
    break
  fi
  sleep 1
done

EFFECTIVE_OK=0
FEEDBACK_OK=0
for i in $(seq 1 180); do
  if [[ -f "$AUDIT/per_keyframe.csv" ]] && [[ $(wc -l < "$AUDIT/per_keyframe.csv") -ge 3 ]]; then
    EFF2=$(python3 - <<PY
import csv
from pathlib import Path
p=Path("$AUDIT/per_keyframe.csv")
rows=list(csv.DictReader(p.open()))
print(rows[1]["min_distance_px"] if len(rows)>1 else rows[0]["min_distance_px"])
PY
)
    echo "effective min_distance_px=${EFF2} (expected ${EXCL_PX})"
    if [[ "$EFF2" == "${EXCL_PX}" || "$EFF2" == "${EXCL_PX}.0" || "$EFF2" == "${EXCL_PX}.000000" ]]; then
      EFFECTIVE_OK=1
    else
      echo "error: effective exclusion ${EFF2} != expected ${EXCL_PX}" >&2
      kill -TERM "$LAUNCH_PID" 2>/dev/null || true
      exit 1
    fi
    break
  fi
  sleep 1
done

if rg -q "vis=1/fb=1" "$OUT/launch_mission.log" 2>/dev/null; then
  FEEDBACK_OK=1
fi
echo "effective_param_verified=${EFFECTIVE_OK}" >> "$OUT/run_meta.txt"
echo "feedback_enable_verified=${FEEDBACK_OK}" >> "$OUT/run_meta.txt"
echo "feedback_enable_verified=${FEEDBACK_OK} (log matrix)"

python3 "$ROOT/tools/analysis/a3c_record_trajectories.py" \
  --out-dir "$OUT" --max-sec 3600 --stop-file /tmp/a42_recorder_stop \
  > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
echo "recorder_pid=${REC_PID}" >> "$OUT/run_meta.txt"

echo "waiting for plan completion ..."
DONE=0
for i in $(seq 1 2400); do
  if rg -q "no more commands to execute" "$OUT/launch_mission.log" 2>/dev/null; then
    DONE=1
    echo "mission complete at poll ${i}"
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "launch ended before completion marker"
    break
  fi
  sleep 1
done

sleep 3
touch /tmp/a42_recorder_stop
wait "$REC_PID" 2>/dev/null || true

pkill -TERM -P "$LAUNCH_PID" 2>/dev/null || true
kill -TERM "$LAUNCH_PID" 2>/dev/null || true
sleep 2
pkill -TERM -f 'webots-bin|component_container.*quad_experiment|robot_launch.py|quad_experiment.launch' 2>/dev/null || true
sleep 2
pkill -KILL -f 'webots-bin|component_container.*quad_experiment' 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

{
  echo "finished_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "mission_done_marker=${DONE}"
  echo "per_keyframe_rows=$(wc -l < "$AUDIT/per_keyframe.csv" 2>/dev/null || echo 0)"
  echo "--- sync / LC excerpts ---"
  rg -n "estimator-only|no control pause|prepare_pause|pause_not_acknowledged|park for coordination|SYNC_REQUEST|LOCAL_APPLIED|REFERENCE_REBASED|SYNC_COMPLETE|SYNC_FAILED|STALE_KF_REJECTED|FIRST_POST_SYNC_KF|SYNC unknown|SYNC_ADMIT|GlobalLoopCommitted|visual LC significance|committed=1|LSposUpdate|CorrectionApplied|pause_not_acknowledged|apply_pending|park for coordination|G6d|vis=.*/fb=" \
    "$OUT/launch_mission.log" | head -200 || true
} >> "$OUT/run_meta.txt"

set +u
python3 "$ROOT/tools/analysis/a3c_analyze_ate.py" --run-dir "$OUT" \
  > "$OUT/ate_summary.json.print" 2>&1 || true
python3 "$ROOT/tools/analysis/map_mgmt_a2_analyze.py" --run-dir "$OUT" \
  --evidence-dir "$OUT/analysis_tmp" \
  > "$OUT/analyze.log" 2>&1 || true
mkdir -p "$OUT/analysis_tmp/tables" "$OUT/analysis_tmp/figures"
if [[ -f "$OUT/analysis_tmp/tables/summary.json" ]]; then
  cp "$OUT/analysis_tmp/tables/summary.json" "$OUT/summary.json" || true
  cp "$OUT/analysis_tmp/tables/per_keyframe_enriched.csv" "$OUT/per_keyframe_enriched.csv" || true
fi
set -u

echo "LC-CONSISTENCY-A4.2 ${RUN_ID} finished done=${DONE} exclusion=${EXCL_PX}"
