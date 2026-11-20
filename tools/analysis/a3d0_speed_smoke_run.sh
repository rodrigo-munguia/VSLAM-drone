#!/usr/bin/env bash
# A3d speed smoke at ~5 m/s: straight + corner with LocalSLAM on.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$ROOT/docs/evidence/gslam_visual_loop_a3d0/raw/speed_smoke"
mkdir -p "$OUT"
: > "$OUT/launch_mission.log"
rm -f /tmp/a3d0_speed_stop

{
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "world=earth_aerial_a3d0_300.wbt"
  echo "plan=control_plan_a3d0_speed_smoke.txt"
  echo "traj_v_max_xy=5.0 tracker_v_max_xy=5.5"
} > "$OUT/run_meta.txt"

(
  cd "$ROOT"
  ./launch4.sh --world earth_aerial_a3d0_300.wbt \
    --params-file "$ROOT/config/quad_params_a3d0_tfa300l.yaml" \
    --traj-v-max-xy 5.0 \
    --tracker-v-max-xy 5.5 \
    --globalslam true \
    --enable-tag-loop-closure false \
    --enable-visual-loop-closure false \
    --apply-ba-corrections false \
    --apply-tag-loop-corrections false \
    --apply-visual-loop-corrections false \
    --auto-start-plan true \
    --control-plan "$ROOT/control_plan_a3d0_speed_smoke.txt" \
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
for i in $(seq 1 180); do
  if ros2 topic list 2>/dev/null | grep -qx /odometry; then
    echo "topics up after ${i}s"
    break
  fi
  sleep 1
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "error: launch exited early" >&2
    exit 1
  fi
done

python3 "$ROOT/tools/analysis/a3d0_speed_sampler.py" \
  --out-dir "$OUT" --stop-file /tmp/a3d0_speed_stop --max-sec 900 \
  > "$OUT/sampler.log" 2>&1 &
SAMP_PID=$!
echo "sampler_pid=${SAMP_PID}" >> "$OUT/run_meta.txt"

echo "waiting for speed-smoke completion..."
DONE=0
for i in $(seq 1 900); do
  if rg -q "no more commands to execute" "$OUT/launch_mission.log" 2>/dev/null; then
    DONE=1
    echo "complete at poll ${i}"
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    break
  fi
  sleep 1
done

sleep 2
touch /tmp/a3d0_speed_stop
wait "$SAMP_PID" 2>/dev/null || true

pkill -TERM -P "$LAUNCH_PID" 2>/dev/null || true
kill -TERM "$LAUNCH_PID" 2>/dev/null || true
sleep 2
pkill -TERM -f 'webots-bin|component_container.*quad_experiment|robot_launch.py|quad_experiment.launch' 2>/dev/null || true
sleep 1
pkill -KILL -f 'webots-bin|component_container.*quad_experiment' 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

{
  echo "finished_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "mission_done_marker=${DONE}"
} >> "$OUT/run_meta.txt"

python3 "$ROOT/tools/analysis/a3d0_speed_analyze.py" --out-dir "$OUT" --target-v 5.0 \
  | tee "$OUT/speed_verdict.txt"

echo "A3d-0 speed smoke finished done=${DONE}"
