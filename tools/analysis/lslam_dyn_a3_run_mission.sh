#!/usr/bin/env bash
# LSLAM-DYNAMIC-CONSISTENCY-A3 / A3.1: instrumented diagnostic mission runner.
# Usage: lslam_dyn_a3_run_mission.sh <run_id> <params_yaml> [plan_file]
# Env overrides:
#   LSLAM_DYN_EVID  evidence pack root (default .../lslam_dynamic_consistency_a3)
#   LSLAM_DYN_PLAN  control plan path (overrides 3rd arg / default 100m plan)
# Example: lslam_dyn_a3_run_mission.sh run_a3a_jac_off_sigma001 \
#   config/experiments/quad_params_lslam_dyn_a3a.yaml
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ID="${1:?run_id required}"
PARAMS_REL="${2:?params yaml required}"
PLAN_ARG="${3:-}"
if [[ "$PARAMS_REL" = /* ]]; then
  PARAMS="$PARAMS_REL"
else
  PARAMS="$ROOT/$PARAMS_REL"
fi
EVID_ROOT="${LSLAM_DYN_EVID:-$ROOT/docs/evidence/lslam_dynamic_consistency_a3}"
OUT="$EVID_ROOT/raw/${RUN_ID}"
mkdir -p "$OUT" "$OUT/ba_d5" "$OUT/landmark_audit" "$OUT/ba_audit" "$OUT/map_mgmt_audit"
: > "$OUT/launch_mission.log"
rm -f /tmp/lslam_dyn_a3_recorder_stop

if [[ ! -f "$PARAMS" ]]; then
  echo "error: params file not found: $PARAMS" >&2
  exit 1
fi

if [[ -n "${LSLAM_DYN_PLAN:-}" ]]; then
  PLAN="$LSLAM_DYN_PLAN"
elif [[ -n "$PLAN_ARG" ]]; then
  if [[ "$PLAN_ARG" = /* ]]; then PLAN="$PLAN_ARG"; else PLAN="$ROOT/$PLAN_ARG"; fi
else
  PLAN="$ROOT/control_plan_lslam_dyn_a3_rect100.txt"
fi
if [[ ! -f "$PLAN" ]]; then
  echo "error: plan not found: $PLAN" >&2
  exit 1
fi
WORLD="earth_aerial_a3d0_380_extended.wbt"
DIAG_CSV="$OUT/lslam_dynamic_diag.csv"
rm -f "$DIAG_CSV"

# Verify experimental factors present in overlay (skip comment lines)
JAC=$(rg -n "^\s*Visual_update_attitude_update:" "$PARAMS" | head -1 || true)
SIG=$(rg -n "^\s*Sigma_att_update:" "$PARAMS" | head -1 || true)
SF=$(rg -n "^\s*Slam_feedback:" "$PARAMS" | head -1 || true)
LC=$(rg -n "^\s*Enable_visual_loop_closure:" "$PARAMS" | head -1 || true)
# Fail fast if overlay factors are missing
echo "$JAC" | rg -q "Visual_update_attitude_update:" || { echo "error: missing Visual_update_attitude_update" >&2; exit 1; }
echo "$SIG" | rg -q "Sigma_att_update:" || { echo "error: missing Sigma_att_update" >&2; exit 1; }

export LSLAM_DYNAMIC_DIAG=1
export LSLAM_DYNAMIC_DIAG_PATH="$DIAG_CSV"
export GSLAM_KF_Z_ADMIT_DIAG=1
export GSLAM_BA_D5_DIR="$OUT/ba_d5"
export GSLAM_LANDMARK_AUDIT_DIR="$OUT/landmark_audit"
export GSLAM_BA_AUDIT_DIR="$OUT/ba_audit"
export GSLAM_MAP_MGMT_AUDIT_DIR="$OUT/map_mgmt_audit"
export GSLAM_MAP_MGMT_AUDIT_ENABLE=1
unset GSLAM_INIT_HYBRID_KF_Z_ABS || true

{
  echo "run_id=${RUN_ID}"
  echo "series=LSLAM-DYNAMIC-CONSISTENCY-A3"
  echo "params=${PARAMS}"
  echo "plan=${PLAN}"
  echo "world=${WORLD}"
  echo "traj_v_max_xy=5.0"
  echo "tracker_v_max_xy=5.5"
  echo "LSLAM_DYNAMIC_DIAG=1"
  echo "LSLAM_DYNAMIC_DIAG_PATH=${DIAG_CSV}"
  echo "apply_ba_corrections=false"
  echo "apply_visual_loop_corrections=false"
  echo "enable_visual_loop_closure=false"
  echo "yaml_Visual_update_attitude_update_line=${JAC}"
  echo "yaml_Sigma_att_update_line=${SIG}"
  echo "yaml_Slam_feedback_line=${SF}"
  echo "yaml_Enable_visual_loop_closure_line=${LC}"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "git_head=$(cd "$ROOT" && git rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_dirty=$(cd "$ROOT" && git status --porcelain | wc -l)"
} | tee "$OUT/run_meta.txt"

(
  cd "$ROOT"
  ./launch4.sh --world "$WORLD" \
    --params-file "$PARAMS" \
    --traj-v-max-xy 5.0 \
    --tracker-v-max-xy 5.5 \
    --globalslam true \
    --enable-tag-loop-closure false \
    --enable-visual-loop-closure false \
    --apply-ba-corrections false \
    --apply-tag-loop-corrections false \
    --apply-visual-loop-corrections false \
    --auto-start-plan true \
    --control-plan "$PLAN" \
    > "$OUT/launch_mission.log" 2>&1
) &
LAUNCH_PID=$!
echo "launch_pid=${LAUNCH_PID}" | tee -a "$OUT/run_meta.txt"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u

echo "waiting for /odometry ..."
TOPICS_OK=0
for i in $(seq 1 240); do
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

# Capture effective LocalSLAM params from launch log once available
for i in $(seq 1 120); do
  if rg -q "Visual_update_attitude_update|Sigma_att_update|TakeOff|StartSLAM" \
      "$OUT/launch_mission.log" 2>/dev/null; then
    break
  fi
  sleep 1
done

python3 "$ROOT/tools/analysis/a3c_record_trajectories.py" \
  --out-dir "$OUT" --max-sec 3600 --stop-file /tmp/lslam_dyn_a3_recorder_stop \
  > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
echo "recorder_pid=${REC_PID}" | tee -a "$OUT/run_meta.txt"

echo "waiting for plan completion (expect ~3-8 min for 100x100 @ 5 m/s) ..."
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
  # progress heartbeat every ~60s
  if (( i % 60 == 0 )); then
    echo "still running poll=${i}s diag_bytes=$(wc -c < "$DIAG_CSV" 2>/dev/null || echo 0)"
  fi
  sleep 1
done

sleep 3
touch /tmp/lslam_dyn_a3_recorder_stop
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
  echo "diag_csv_bytes=$(wc -c < "$DIAG_CSV" 2>/dev/null || echo 0)"
  echo "diag_csv_rows=$(wc -l < "$DIAG_CSV" 2>/dev/null || echo 0)"
  rg -n "Visual_update_attitude_update|Sigma_att_update|Slam_feedback|Prediction_model|Speed_update|hybrid_z_abs=|no more commands" \
    "$OUT/launch_mission.log" | head -120 || true
} | tee -a "$OUT/run_meta.txt"

set +u
python3 "$ROOT/tools/analysis/a3c_analyze_ate.py" --run-dir "$OUT" \
  > "$OUT/ate_summary.json.print" 2>&1 || true
set -u

echo "A3 mission ${RUN_ID} finished done=${DONE} out=${OUT}"
