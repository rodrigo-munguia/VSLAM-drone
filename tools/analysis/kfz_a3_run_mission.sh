#!/usr/bin/env bash
# KF-Z-A3: vertical-drift C/T — full-relative (R) vs hybrid Z-abs (H).
# Visual LC OFF. Usage: kfz_a3_run_mission.sh <run_R|run_H> <hybrid 0|1>
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ID="${1:?run id e.g. run_R}"
HYBRID="${2:?hybrid 0|1}"
OUT="$ROOT/docs/evidence/gslam_kf_z_a3/raw/${RUN_ID}"
mkdir -p "$OUT" "$OUT/ba_d5"
: > "$OUT/launch_mission.log"
rm -f /tmp/a3c_recorder_stop

PARAMS="$ROOT/config/quad_params_a3d0_tfa300l.yaml"
PLAN="$ROOT/control_plan_a3d0_rect380.txt"
WORLD="earth_aerial_a3d0_380.wbt"

# Hybrid OFF by default in YAML; force via env for both clarity and H enable.
export GSLAM_INIT_HYBRID_KF_Z_ABS="$HYBRID"
export GSLAM_KF_Z_ADMIT_DIAG=1
export GSLAM_BA_D5_DIR="$OUT/ba_d5"

echo "KF-Z-A3 ${RUN_ID} hybrid=${HYBRID} out=${OUT}" | tee "$OUT/run_meta.txt"
{
  echo "run_id=${RUN_ID}"
  echo "Init_hybrid_kf_z_absolute_enable=${HYBRID}"
  echo "GSLAM_INIT_HYBRID_KF_Z_ABS=${GSLAM_INIT_HYBRID_KF_Z_ABS}"
  echo "GSLAM_KF_Z_ADMIT_DIAG=1"
  echo "GSLAM_BA_D5_DIR=${GSLAM_BA_D5_DIR}"
  echo "enable_visual_loop_closure=false"
  echo "enable_tag_loop_closure=false"
  echo "world=${WORLD}"
  echo "params=${PARAMS}"
  echo "plan=${PLAN}"
  echo "traj_v_max_xy=5.0"
  echo "tracker_v_max_xy=5.5"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} >> "$OUT/run_meta.txt"

(
  cd "$ROOT"
  # Env must reach component_container — launch inherits this shell's env.
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
    exit 1
  fi
done
if [[ "$TOPICS_OK" -ne 1 ]]; then
  echo "error: /odometry never appeared" >&2
  exit 1
fi

for i in $(seq 1 120); do
  if rg -q "TakeOff: unified|control-> TakeOff|hybrid_z_abs=" "$OUT/launch_mission.log" 2>/dev/null; then
    echo "plan/ready after ${i}s"
    break
  fi
  sleep 1
done

# Re-export for recorder process (diagnostics are in launch child).
python3 "$ROOT/tools/analysis/a3c_record_trajectories.py" \
  --out-dir "$OUT" --max-sec 3600 --stop-file /tmp/a3c_recorder_stop \
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
touch /tmp/a3c_recorder_stop
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
  rg -n "hybrid_z_abs=|relative_kf_pose=|G6d source matrix" "$OUT/launch_mission.log" | head -5 || true
} >> "$OUT/run_meta.txt"

set +u
python3 "$ROOT/tools/analysis/a3c_analyze_ate.py" --run-dir "$OUT" \
  > "$OUT/ate_summary.json.print" 2>&1 || true
set -u

echo "KF-Z-A3 ${RUN_ID} finished done=${DONE}"
