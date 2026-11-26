#!/usr/bin/env bash
# LSLAM-DYNAMIC-CONSISTENCY-A3.1: single A3b mission on 380×380 m @ 40 m.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export LSLAM_DYN_EVID="$ROOT/docs/evidence/lslam_dynamic_consistency_a3_1"
export LSLAM_DYN_PLAN="$ROOT/control_plan_lslam_dyn_a3_1_rect380.txt"
mkdir -p "$LSLAM_DYN_EVID/raw" "$LSLAM_DYN_EVID/figures" "$LSLAM_DYN_EVID/tables"
cp -f "$LSLAM_DYN_PLAN" "$LSLAM_DYN_EVID/raw/control_plan_lslam_dyn_a3_1_rect380.txt"
cp -f "$ROOT/config/experiments/quad_params_lslam_dyn_a3b.yaml" \
  "$LSLAM_DYN_EVID/raw/quad_params_lslam_dyn_a3b.yaml" 2>/dev/null || true

# Ensure clean sim
pkill -TERM -f 'webots-bin|component_container.*quad_experiment|a3c_record' 2>/dev/null || true
sleep 2
pkill -KILL -f 'webots-bin|component_container.*quad_experiment' 2>/dev/null || true
sleep 1

echo "A3.1 starting: A3b params, 380m@40m plan, evidence=$LSLAM_DYN_EVID"
"$ROOT/tools/analysis/lslam_dyn_a3_run_mission.sh" \
  run_a3_1_jac_on_sigma001_rect380 \
  config/experiments/quad_params_lslam_dyn_a3b.yaml \
  control_plan_lslam_dyn_a3_1_rect380.txt \
  | tee "$LSLAM_DYN_EVID/raw/run_a3_1_jac_on_sigma001_rect380_console.log"

echo "A3.1 mission runner finished"
