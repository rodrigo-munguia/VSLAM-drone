#!/usr/bin/env bash
# Run A3a → A3b → A3c sequentially (identical plan/world).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNNER="$ROOT/tools/analysis/lslam_dyn_a3_run_mission.sh"
EVID="$ROOT/docs/evidence/lslam_dynamic_consistency_a3"
mkdir -p "$EVID/raw" "$EVID/figures" "$EVID/tables"
cp -f "$ROOT/control_plan_lslam_dyn_a3_rect100.txt" "$EVID/raw/control_plan_lslam_dyn_a3_rect100.txt"

# Ensure no leftover sim
pkill -TERM -f 'webots-bin|component_container.*quad_experiment|robot_launch.py|quad_experiment.launch' 2>/dev/null || true
sleep 2
pkill -KILL -f 'webots-bin|component_container.*quad_experiment' 2>/dev/null || true
sleep 1

declare -a JOBS=(
  "run_a3a_jac_off_sigma001|config/experiments/quad_params_lslam_dyn_a3a.yaml"
  "run_a3b_jac_on_sigma001|config/experiments/quad_params_lslam_dyn_a3b.yaml"
  "run_a3c_jac_on_sigma050|config/experiments/quad_params_lslam_dyn_a3c.yaml"
)

for job in "${JOBS[@]}"; do
  IFS='|' read -r RID PARAMS <<<"$job"
  echo "========== START ${RID} =========="
  "$RUNNER" "$RID" "$PARAMS" | tee "$EVID/raw/${RID}_console.log"
  echo "========== DONE ${RID} =========="
  pkill -TERM -f 'webots-bin|component_container.*quad_experiment' 2>/dev/null || true
  sleep 3
  pkill -KILL -f 'webots-bin|component_container.*quad_experiment' 2>/dev/null || true
  sleep 2
done

python3 "$ROOT/tools/analysis/lslam_dyn_a3_analyze.py" --evidence-dir "$EVID" \
  | tee "$EVID/tables/analyze_console.log"

echo "campaign complete"
