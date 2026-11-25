#!/usr/bin/env bash
# A3d-0 smoke: spawn at (-190,-190), TakeOff, hop +20 m mission-X, return.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$ROOT/docs/evidence/gslam_visual_loop_a3d0/raw/smoke"
mkdir -p "$OUT"
: > "$OUT/launch_mission.log"
rm -f /tmp/a3d0_smoke_stop

{
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "world=earth_aerial_a3d0_380_extended.wbt"
  echo "plan=control_plan_a3d0_smoke.txt"
  echo "expect_spawn_world=(-190,-190,15.75)"
  echo "expect_hop_mission=(20,0,60) -> world≈(-170,-190)"
} > "$OUT/run_meta.txt"

(
  cd "$ROOT"
  ./launch4.sh --world earth_aerial_a3d0_380_extended.wbt \
    --params-file "$ROOT/config/quad_params_a3d0_tfa300l.yaml" \
    --globalslam true \
    --enable-tag-loop-closure false \
    --enable-visual-loop-closure false \
    --apply-ba-corrections false \
    --apply-tag-loop-corrections false \
    --apply-visual-loop-corrections false \
    --auto-start-plan true \
    --control-plan "$ROOT/control_plan_a3d0_smoke.txt" \
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

echo "waiting for topics..."
for i in $(seq 1 180); do
  if ros2 topic list 2>/dev/null | grep -qx /GT_topic; then
    echo "GT up after ${i}s"
    break
  fi
  sleep 1
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "error: launch exited early" >&2
    exit 1
  fi
done

python3 "$ROOT/tools/analysis/a3d0_gt_sampler.py" \
  --out "$OUT/gt_samples.csv" --stop-file /tmp/a3d0_smoke_stop --max-sec 600 \
  > "$OUT/gt_sampler.log" 2>&1 &
SAMP_PID=$!
echo "sampler_pid=${SAMP_PID}" >> "$OUT/run_meta.txt"

echo "waiting for smoke plan completion..."
DONE=0
for i in $(seq 1 600); do
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
touch /tmp/a3d0_smoke_stop
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

python3 "$ROOT/tools/analysis/a3d0_smoke_analyze.py" --out-dir "$OUT" | tee "$OUT/smoke_verdict.txt"

echo "A3d-0 smoke finished done=${DONE}"
