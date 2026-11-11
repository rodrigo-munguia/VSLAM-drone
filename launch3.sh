#!/usr/bin/env bash
# launch3.sh — integrated experiment using optimized velocity-free LocalSLAM.
#
# Reuses ./launch2.sh unchanged. Default params file:
#   <repo>/config/quad_params_no_velocity.yaml
# (Speed_update=false, min_vel=-1, Sigma_a=4, Sigma_cam_uv=10; LSLAM-NOV-VEL-T2)
#
# Explicit --params-file from the caller overrides that default.
# All other arguments are forwarded to launch2.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH2="${SCRIPT_DIR}/launch2.sh"
DEFAULT_PARAMS="${SCRIPT_DIR}/config/quad_params_no_velocity.yaml"

if [[ ! -x "$LAUNCH2" && ! -f "$LAUNCH2" ]]; then
  echo "error: launch2.sh not found at $LAUNCH2" >&2
  exit 1
fi
if [[ ! -f "$DEFAULT_PARAMS" ]]; then
  echo "error: default velocity-free YAML not found: $DEFAULT_PARAMS" >&2
  exit 1
fi

params_file_set=0
forward=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --params-file)
      if [[ $# -lt 2 ]]; then
        echo "error: --params-file requires a path argument" >&2
        exit 1
      fi
      forward+=(--params-file "$2")
      params_file_set=1
      shift 2
      ;;
    --params-file=*)
      forward+=("$1")
      params_file_set=1
      shift
      ;;
    *)
      forward+=("$1")
      shift
      ;;
  esac
done

if [[ "$params_file_set" -eq 0 ]]; then
  forward=(--params-file "$DEFAULT_PARAMS" "${forward[@]}")
fi

exec "$LAUNCH2" "${forward[@]}"
