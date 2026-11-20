#!/usr/bin/env bash
# Launch one A3c Webots mission + trajectory recorder.
# Usage: a3c_run_mission.sh <run_id> <enable_visual true|false>
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ID="${1:?run id e.g. run_C}"
VIS="${2:?enable_visual_loop_closure true|false}"
OUT="$ROOT/docs/evidence/gslam_visual_loop_a3c/raw/${RUN_ID}"
mkdir -p "$OUT"
: > "$OUT/launch_mission.log"
rm -f /tmp/a3c_recorder_stop

PARAMS="$ROOT/config/quad_params_a3c_tfa300l.yaml"
PLAN="$ROOT/control_plan_a3c_extended.txt"

echo "A3c ${RUN_ID} visual=${VIS} out=${OUT}" | tee "$OUT/run_meta.txt"
{
  echo "run_id=${RUN_ID}"
  echo "enable_visual_loop_closure=${VIS}"
  echo "params=${PARAMS}"
  echo "plan=${PLAN}"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} >> "$OUT/run_meta.txt"

# Launch mission
(
  cd "$ROOT"
  ./launch4.sh --globalslam true \
    --params-file "$PARAMS" \
    --enable-tag-loop-closure false \
    --enable-visual-loop-closure "$VIS" \
    --apply-ba-corrections false \
    --apply-tag-loop-corrections false \
    --apply-visual-loop-corrections false \
    --auto-start-plan true \
    --control-plan "$PLAN" \
    > "$OUT/launch_mission.log" 2>&1
) &
LAUNCH_PID=$!
echo "launch_pid=${LAUNCH_PID}" >> "$OUT/run_meta.txt"

# Wait for ROS / topics (ament setup.bash is not nounset-safe)
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/install/setup.bash"
fi
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

# Also wait for ControlQuad plan start
for i in $(seq 1 120); do
  if rg -q "TakeOff: unified|control-> TakeOff" "$OUT/launch_mission.log" 2>/dev/null; then
    echo "plan started after ${i}s"
    break
  fi
  sleep 1
done

python3 "$ROOT/tools/analysis/a3c_record_trajectories.py" \
  --out-dir "$OUT" --max-sec 3600 --stop-file /tmp/a3c_recorder_stop \
  > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
echo "recorder_pid=${REC_PID}" >> "$OUT/run_meta.txt"

# Wait for mission completion
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

# Stop launch stack
pkill -TERM -P "$LAUNCH_PID" 2>/dev/null || true
kill -TERM "$LAUNCH_PID" 2>/dev/null || true
sleep 2
pkill -TERM -f 'webots-bin' 2>/dev/null || true
pkill -TERM -f 'component_container.*quad_experiment' 2>/dev/null || true
pkill -TERM -f 'quad_experiment.launch.py' 2>/dev/null || true
pkill -TERM -f 'robot_launch.py' 2>/dev/null || true
sleep 2
pkill -KILL -f 'webots-bin' 2>/dev/null || true
pkill -KILL -f 'component_container.*quad_experiment' 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

{
  echo "finished_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "mission_done_marker=${DONE}"
} >> "$OUT/run_meta.txt"

set +u
python3 "$ROOT/tools/analysis/a3c_analyze_ate.py" --run-dir "$OUT" \
  > "$OUT/ate_summary.json.print" 2>&1 || true
set -u

echo "A3c ${RUN_ID} finished done=${DONE}"
