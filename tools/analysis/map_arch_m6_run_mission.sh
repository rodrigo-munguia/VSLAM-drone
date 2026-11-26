#!/usr/bin/env bash
# GSLAM-MAP-ARCH-M6: comparative Webots mission runner.
# Usage: map_arch_m6_run_mission.sh <run_id> <workspace_root> [params_yaml]
# Preserves evidence under docs/evidence/gslam_map_arch_m6/raw/<run_id>
set -euo pipefail
RUN_ID="${1:?run_id required}"
ROOT="$(cd "${2:?workspace_root required}" && pwd)"
PARAMS="${3:-$ROOT/config/experiments/quad_params_a3_2a_exclusion10.yaml}"
# Evidence always under the candidate (main) tree evidence pack.
EVID_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/docs/evidence/gslam_map_arch_m6"
OUT="${EVID_ROOT}/raw/${RUN_ID}"
AUDIT="$OUT/map_mgmt_audit"
IDINV="$OUT/identity_invariant"
mkdir -p "$OUT" "$OUT/ba_d5" "$AUDIT" "$OUT/landmark_audit" "$OUT/ba_audit" "$IDINV" "$OUT/ba_replay"
: > "$OUT/launch_mission.log"
STOP_FILE="/tmp/m6_${RUN_ID}_recorder_stop"
rm -f "$STOP_FILE" /tmp/a3c_recorder_stop

if [[ ! -f "$PARAMS" ]]; then
  echo "error: params not found: $PARAMS" >&2
  exit 1
fi
if [[ ! -x "$ROOT/launch4.sh" ]]; then
  echo "error: launch4.sh missing in $ROOT" >&2
  exit 1
fi
if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  echo "error: install/setup.bash missing in $ROOT — build first" >&2
  exit 1
fi

PLAN="$ROOT/control_plan_a3f_rect380_revisit40.txt"
WORLD="earth_aerial_a3d0_380_extended.wbt"
EXCL_PX=10

unset GSLAM_INIT_HYBRID_KF_Z_ABS || true
export GSLAM_KF_Z_ADMIT_DIAG=1
export GSLAM_BA_D5_DIR="$OUT/ba_d5"
export GSLAM_MAP_MGMT_AUDIT_DIR="$AUDIT"
export GSLAM_MAP_MGMT_AUDIT_ENABLE=1
export GSLAM_LANDMARK_AUDIT_DIR="$OUT/landmark_audit"
export GSLAM_BA_AUDIT_DIR="$OUT/ba_audit"
export GSLAM_IDENTITY_INVARIANT_DIR="$IDINV"
export GSLAM_BA_REPLAY_DIR="$OUT/ba_replay"

GS_SO="$ROOT/install/globalslam/lib/libglobalslam_component.so"
{
  echo "run_id=${RUN_ID}"
  echo "series=GSLAM-MAP-ARCH-M6"
  echo "workspace_root=${ROOT}"
  echo "git_head=$(cd "$ROOT" && git rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_describe=$(cd "$ROOT" && git describe --always --dirty 2>/dev/null || echo unknown)"
  echo "globalslam_so=${GS_SO}"
  echo "globalslam_so_sha256=$(sha256sum "$GS_SO" 2>/dev/null | awk '{print $1}' || echo missing)"
  echo "globalslam_so_mtime=$(stat -c %y "$GS_SO" 2>/dev/null || echo unknown)"
  echo "params=${PARAMS}"
  echo "world=${WORLD}"
  echo "plan=${PLAN}"
  echo "Init_min_distance_to_previous_mapped_features=${EXCL_PX}"
  echo "enable_visual_loop_closure=true"
  echo "apply_visual_loop_corrections=true"
  echo "apply_ba_corrections=false"
  echo "Slam_feedback=false"
  echo "landmark_fusion=OFF"
  echo "GSLAM_IDENTITY_INVARIANT_DIR=${GSLAM_IDENTITY_INVARIANT_DIR}"
  echo "GSLAM_BA_REPLAY_DIR=${GSLAM_BA_REPLAY_DIR}"
  echo "traj_v_max_xy=5.0"
  echo "tracker_v_max_xy=5.5"
  echo "association_horizon_kf=20"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} | tee "$OUT/run_meta.txt"

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

# Binary provenance check from running process maps when available
sleep 2
python3 - <<PY >> "$OUT/run_meta.txt" 2>/dev/null || true
import os, glob
want = os.path.realpath("$GS_SO")
print(f"expected_globalslam_realpath={want}")
for maps in glob.glob("/proc/*/maps"):
    try:
        txt = open(maps).read()
    except Exception:
        continue
    if "libglobalslam_component.so" in txt:
        for line in txt.splitlines():
            if "libglobalslam_component.so" in line:
                path = line.split()[-1]
                print(f"loaded_globalslam_maps={path}")
                print(f"loaded_matches_expected={int(os.path.realpath(path)==want)}")
                raise SystemExit
print("loaded_globalslam_maps=NOT_FOUND")
PY

RECORDER="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/tools/analysis/a3c_record_trajectories.py"
python3 "$RECORDER" \
  --out-dir "$OUT" --max-sec 3600 --stop-file "$STOP_FILE" \
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
touch "$STOP_FILE"
wait "$REC_PID" 2>/dev/null || true

pkill -TERM -P "$LAUNCH_PID" 2>/dev/null || true
kill -TERM "$LAUNCH_PID" 2>/dev/null || true
sleep 2
pkill -TERM -f 'webots-bin|component_container.*quad_experiment|robot_launch.py|quad_experiment.launch' 2>/dev/null || true
sleep 2
pkill -KILL -f 'webots-bin|component_container.*quad_experiment' 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

MAIN="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
{
  echo "finished_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "mission_done_marker=${DONE}"
  echo "per_keyframe_rows=$(wc -l < "$AUDIT/per_keyframe.csv" 2>/dev/null || echo 0)"
  echo "invariant_samples=$(wc -l < "$IDINV/invariant_samples.csv" 2>/dev/null || echo 0)"
  echo "invariant_violations=$(wc -l < "$IDINV/invariant_violations.csv" 2>/dev/null || echo 0)"
  echo "--- sync / LC excerpts ---"
  rg -n "SYNC_REQUEST|LOCAL_APPLIED|REFERENCE_REBASED|SYNC_COMPLETE|FIRST_POST_SYNC_KF|GlobalLoopCommitted|visual LC: matched|committed=1|vis=.*/fb=|estimator-only|CoordPause|pause_not_acknowledged" \
    "$OUT/launch_mission.log" | head -200 || true
} >> "$OUT/run_meta.txt"

set +u
python3 "$MAIN/tools/analysis/a3c_analyze_ate.py" --run-dir "$OUT" \
  > "$OUT/ate_summary.json.print" 2>&1 || true
python3 "$MAIN/tools/analysis/map_mgmt_a2_analyze.py" --run-dir "$OUT" \
  --evidence-dir "$OUT/analysis_tmp" \
  > "$OUT/analyze.log" 2>&1 || true
mkdir -p "$OUT/analysis_tmp/tables" "$OUT/analysis_tmp/figures"
if [[ -f "$OUT/analysis_tmp/tables/summary.json" ]]; then
  cp "$OUT/analysis_tmp/tables/summary.json" "$OUT/summary.json" || true
  cp "$OUT/analysis_tmp/tables/per_keyframe_enriched.csv" "$OUT/per_keyframe_enriched.csv" || true
fi
set -u

echo "MAP-ARCH-M6 ${RUN_ID} finished done=${DONE} root=${ROOT}"
