#!/usr/bin/env bash
# launch4.sh — TFA300-L laser-assisted, velocity-free LocalSLAM (LSLAM-RANGE-B4).
#
# Thin wrapper around ./launch2.sh. Defaults:
#   --params-file <repo>/config/quad_params_no_velocity_tfa300l.yaml
#   --range-sensor tfa300l
#   --world earth_aerial.wbt
#
# Equivalent intent to:
#   ./launch3.sh --range-sensor tfa300l \
#     --params-file "$PWD/config/quad_params_no_velocity_tfa300l.yaml" \
#     --world earth_aerial.wbt
#
# Explicit --params-file / --range-sensor / --world from the caller override the
# defaults (no duplicate flags). Inconsistent combinations are rejected.
# All other arguments are forwarded unchanged (including --control-plan and
# --auto-start-plan). Plans do not auto-start by default; use EKF-Operator 'a'
# (or pass --auto-start-plan true).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH2="${SCRIPT_DIR}/launch2.sh"
DEFAULT_PARAMS="${SCRIPT_DIR}/config/quad_params_no_velocity_tfa300l.yaml"
DEFAULT_RANGE_SENSOR="tfa300l"
DEFAULT_WORLD="earth_aerial.wbt"

if [[ ! -x "$LAUNCH2" && ! -f "$LAUNCH2" ]]; then
  echo "error: launch2.sh not found at $LAUNCH2" >&2
  exit 1
fi
if [[ ! -f "$DEFAULT_PARAMS" ]]; then
  echo "error: default laser YAML not found: $DEFAULT_PARAMS" >&2
  exit 1
fi

params_file_set=0
range_sensor_set=0
world_set=0
PARAMS_FILE=""
RANGE_SENSOR=""
WORLD=""
forward=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --params-file)
      if [[ $# -lt 2 ]]; then
        echo "error: --params-file requires a path argument" >&2
        exit 1
      fi
      forward+=(--params-file "$2")
      PARAMS_FILE="$2"
      params_file_set=1
      shift 2
      ;;
    --params-file=*)
      forward+=("$1")
      PARAMS_FILE="${1#--params-file=}"
      params_file_set=1
      shift
      ;;
    --range-sensor)
      if [[ $# -lt 2 ]]; then
        echo "error: --range-sensor requires sonar|tfa300l" >&2
        exit 1
      fi
      forward+=(--range-sensor "$2")
      RANGE_SENSOR="$2"
      range_sensor_set=1
      shift 2
      ;;
    --range-sensor=*)
      forward+=("$1")
      RANGE_SENSOR="${1#--range-sensor=}"
      range_sensor_set=1
      shift
      ;;
    --world)
      if [[ $# -lt 2 ]]; then
        echo "error: --world requires a NAME.wbt argument" >&2
        exit 1
      fi
      forward+=(--world "$2")
      WORLD="$2"
      world_set=1
      shift 2
      ;;
    --world=*)
      forward+=("$1")
      WORLD="${1#--world=}"
      world_set=1
      shift
      ;;
    *)
      forward+=("$1")
      shift
      ;;
  esac
done

if [[ "$params_file_set" -eq 0 ]]; then
  PARAMS_FILE="$DEFAULT_PARAMS"
  forward=(--params-file "$DEFAULT_PARAMS" "${forward[@]}")
fi
if [[ "$range_sensor_set" -eq 0 ]]; then
  RANGE_SENSOR="$DEFAULT_RANGE_SENSOR"
  forward=(--range-sensor "$DEFAULT_RANGE_SENSOR" "${forward[@]}")
fi
if [[ "$world_set" -eq 0 ]]; then
  WORLD="$DEFAULT_WORLD"
  forward=(--world "$DEFAULT_WORLD" "${forward[@]}")
fi

# Reject silent inconsistent sensor/config combinations.
base_params="$(basename -- "$PARAMS_FILE")"
if [[ "$RANGE_SENSOR" == "sonar" && "$base_params" == *tfa300l* ]]; then
  echo "error: launch4: --range-sensor sonar is incompatible with laser params file:" >&2
  echo "  $PARAMS_FILE" >&2
  echo "  Use launch2/3 for sonar, or omit --range-sensor for launch4 defaults." >&2
  exit 1
fi
if [[ "$RANGE_SENSOR" == "tfa300l" && "$base_params" == "quad_params.yaml" ]]; then
  echo "error: launch4: --range-sensor tfa300l with velocity-aided sonar baseline" >&2
  echo "  ($PARAMS_FILE) is rejected to avoid mixing profiles." >&2
  echo "  Use ./launch2.sh --range-sensor tfa300l for that combination." >&2
  exit 1
fi

echo "launch4: range_sensor=$RANGE_SENSOR params_file=$PARAMS_FILE world=$WORLD"
exec "$LAUNCH2" "${forward[@]}"
