#!/usr/bin/env bash
# Canonical integrated experiment launcher (Webots + app + viewer + operator).
#
# Overlay selection:
#   EKF_INSTALL_BASE   absolute or repo-relative install prefix
#   default: install/  (conventional colcon install tree)
#
# Examples:
#   ./launch2.sh
#   ./launch2.sh --viewer plot2
#   ./launch2.sh --viewer rviz2
#   ./launch2.sh --viewer none
#   ./launch2.sh --globalslam true
#   ./launch2.sh --globalslam true --global-corrections true
#   ./launch2.sh --globalslam true --apply-tag-loop-corrections true
#   ./launch2.sh --globalslam true --apply-ba-corrections false \
#       --apply-tag-loop-corrections true
#   ./launch2.sh --world mars.wbt --slam-feedback false
#   EKF_INSTALL_BASE=install ./launch2.sh
#
# Standalone Local SLAM (no Webots) remains:
#   ros2 launch localslam localslam.launch.py ...
# Standalone Global SLAM:
#   ros2 launch globalslam globalslam.launch.py
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

WORLD="mars.wbt"
VIEWER="plot2"
GLOBALSLAM="false"
# Empty = omit launch override (YAML Apply_global_corrections / Apply_* win).
GLOBAL_CORRECTIONS=""
ENABLE_GLOBAL_BA=""
ENABLE_TAG_LOOP=""
ENABLE_VISUAL_LOOP=""
APPLY_BA_CORR=""
APPLY_TAG_CORR=""
APPLY_VISUAL_CORR=""
SLAM_FEEDBACK=""
SLAM_FEEDBACK_IFACE=""
PARAMS_FILE=""
CONTROL_PLAN=""
RANGE_SENSOR="sonar"
NO_WEBOTS="false"
NO_KEYBOARD="false"
# Default false: operator starts the plan with 'a' in EKF-Operator.
# Pass --auto-start-plan true to start without the keyboard.
AUTO_START_PLAN="false"
# R5/R5a: optional trajectory / tracker overrides (defaults = validated R4e).
TRAJ_V_MAX=""
TRAJ_V_MAX_XY=""
TRAJ_V_MAX_Z=""
TRAJ_A_MAX=""
TRAJ_J_MAX=""
TRACKER_V_MAX_XY=""
TRACKER_V_MAX_Z=""

usage() {
  cat <<EOF
Usage: ./launch2.sh [options]

  --world NAME.wbt           Webots world (default: mars.wbt)
  --viewer plot2|rviz2|none  Visualization (default: plot2)
  --globalslam true|false    Opt-in Global SLAM (default: false)
  --global-corrections true|false
                             Compatibility master. Omitted = leave YAML.
                             true: force Apply_* for each enabled source
                             (never enables visual processing). Explicit
                             --apply-*-corrections false with master true → error.
                             false: clear Apply_global_corrections only.
                             Requires --globalslam true when true.
  --enable-global-ba true|false
  --enable-tag-loop-closure true|false
  --enable-visual-loop-closure true|false
                             Per-source global processing. Omitted = YAML
                             (shipped: ba=true, tag=true, visual=false).
                             Tag+visual both true → reject.
  --apply-ba-corrections true|false
  --apply-tag-loop-corrections true|false
  --apply-visual-loop-corrections true|false
                             Per-source Local SLAM feedback. Omitted = YAML
                             (shipped: all false). Feedback requires its
                             source processing. Effective Apply_*=true →
                             estimator-only Local feedback (A4.1; no CoordPause).
  --slam-feedback true|false Override control Slam_feedback (nav pose; NOT corrections).
                             Omitted = YAML.
  --slam-feedback-interface odometry|legacy
                             Omitted = YAML.
  --params-file PATH         Absolute YAML for bringup (default: installed quad_params.yaml)
  --control-plan PATH        Absolute path override for Control_plan_path
  --auto-start-plan true|false
                             Call /control_run_service after bringup (same as
                             operator key 'a'). Default: false — use EKF-Operator.
  --range-sensor sonar|tfa300l
                             Range source for LocalSLAM ~/downward_range
                             (default: sonar). tfa300l selects /laser_range
                             and merges config/range_sensor_tfa300l.yaml
                             gates only (does not change Speed_update/Σ_a/uv).
  --no-plot2                 Compatibility alias for --viewer none
  --no-webots                Do not spawn Webots (app+keyboard only)
  --no-keyboard              Do not spawn operator terminal
  --traj-v-max M/S           Legacy isotropic traj vmax (default 5.0; prefer xy/z)
  --traj-v-max-xy M/S        Traj XY vmax (production default 5.0; prior 3.0 selectable)
  --traj-v-max-z M/S         Traj Z vmax (production default 1.5; ≤0 → inherit)
  --traj-a-max M/S2          Trajectory amax (default 0.8)
  --traj-j-max M/S3          Trajectory jmax (default 2.0)
  --tracker-v-max-xy M/S     Tracker XY v_sp sat (production default 5.5; prior 3.5)
  --tracker-v-max-z M/S      Tracker Z v_sp sat (default 1.5)
  -h, --help                 Show this help

Environment:
  EKF_INSTALL_BASE   Install overlay under repo (default: install)
  WEBOTS_HOME        Webots install (auto-detected if unset)
  ROS_DISTRO         Expected jazzy
  GTSAM / LD_LIBRARY_PATH   Auto-configured when --globalslam true
                            (tools/ensure_gtsam_runtime.sh; see gtsam_prefix.env.example)
EOF
}

normalize_bool() {
  case "$1" in
    true|1|yes) echo true ;;
    false|0|no) echo false ;;
    *) return 1 ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world) WORLD="$2"; shift 2 ;;
    --viewer)
      VIEWER="$2"
      shift 2
      ;;
    --globalslam) GLOBALSLAM="$2"; shift 2 ;;
    --global-corrections) GLOBAL_CORRECTIONS="$2"; shift 2 ;;
    --enable-global-ba) ENABLE_GLOBAL_BA="$2"; shift 2 ;;
    --enable-tag-loop-closure) ENABLE_TAG_LOOP="$2"; shift 2 ;;
    --enable-visual-loop-closure) ENABLE_VISUAL_LOOP="$2"; shift 2 ;;
    --apply-ba-corrections) APPLY_BA_CORR="$2"; shift 2 ;;
    --apply-tag-loop-corrections) APPLY_TAG_CORR="$2"; shift 2 ;;
    --apply-visual-loop-corrections) APPLY_VISUAL_CORR="$2"; shift 2 ;;
    --slam-feedback) SLAM_FEEDBACK="$2"; shift 2 ;;
    --slam-feedback-interface) SLAM_FEEDBACK_IFACE="$2"; shift 2 ;;
    --params-file) PARAMS_FILE="$2"; shift 2 ;;
    --control-plan) CONTROL_PLAN="$2"; shift 2 ;;
    --traj-v-max) TRAJ_V_MAX="$2"; shift 2 ;;
    --traj-v-max-xy) TRAJ_V_MAX_XY="$2"; shift 2 ;;
    --traj-v-max-z) TRAJ_V_MAX_Z="$2"; shift 2 ;;
    --traj-a-max) TRAJ_A_MAX="$2"; shift 2 ;;
    --traj-j-max) TRAJ_J_MAX="$2"; shift 2 ;;
    --tracker-v-max-xy) TRACKER_V_MAX_XY="$2"; shift 2 ;;
    --tracker-v-max-z) TRACKER_V_MAX_Z="$2"; shift 2 ;;
    --range-sensor) RANGE_SENSOR="$2"; shift 2 ;;
    --auto-start-plan)
      if [[ $# -lt 2 ]]; then
        echo "error: --auto-start-plan requires true|false" >&2
        exit 1
      fi
      AUTO_START_PLAN="$2"
      shift 2
      ;;
    --auto-start-plan=*)
      AUTO_START_PLAN="${1#--auto-start-plan=}"
      shift
      ;;
    --no-plot2) VIEWER="none"; shift ;;
    --no-webots) NO_WEBOTS="true"; shift ;;
    --no-keyboard) NO_KEYBOARD="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

if ! AUTO_START_PLAN="$(normalize_bool "$AUTO_START_PLAN")"; then
  echo "error: --auto-start-plan must be true|false (got: $AUTO_START_PLAN)"
  exit 1
fi

case "$RANGE_SENSOR" in
  sonar|tfa300l) ;;
  *)
    echo "error: --range-sensor must be sonar|tfa300l (got: $RANGE_SENSOR)"
    exit 1
    ;;
esac

if ! GLOBALSLAM="$(normalize_bool "$GLOBALSLAM")"; then
  echo "error: --globalslam must be true|false (got: $GLOBALSLAM)"
  exit 1
fi

normalize_opt_bool() {
  local name="$1" val="$2"
  if [[ -z "$val" ]]; then
    echo ""
    return 0
  fi
  if ! normalize_bool "$val" >/dev/null; then
    echo "error: $name must be true|false (got: $val)" >&2
    exit 1
  fi
  normalize_bool "$val"
}
if [[ -n "$GLOBAL_CORRECTIONS" ]]; then
  if ! GLOBAL_CORRECTIONS="$(normalize_bool "$GLOBAL_CORRECTIONS")"; then
    echo "error: --global-corrections must be true|false (got: $GLOBAL_CORRECTIONS)"
    exit 1
  fi
fi
ENABLE_GLOBAL_BA="$(normalize_opt_bool --enable-global-ba "$ENABLE_GLOBAL_BA")"
ENABLE_TAG_LOOP="$(normalize_opt_bool --enable-tag-loop-closure "$ENABLE_TAG_LOOP")"
ENABLE_VISUAL_LOOP="$(normalize_opt_bool --enable-visual-loop-closure "$ENABLE_VISUAL_LOOP")"
APPLY_BA_CORR="$(normalize_opt_bool --apply-ba-corrections "$APPLY_BA_CORR")"
APPLY_TAG_CORR="$(normalize_opt_bool --apply-tag-loop-corrections "$APPLY_TAG_CORR")"
APPLY_VISUAL_CORR="$(normalize_opt_bool --apply-visual-loop-corrections "$APPLY_VISUAL_CORR")"

# YAML baselines for omitted flags (CLI > YAML; never invent hard-coded enables).
YAML_GSLAM="$ROOT/config/quad_params.yaml"
yaml_gslam_bool() {
  local key="$1" default="$2"
  if [[ ! -f "$YAML_GSLAM" ]]; then
    echo "$default"
    return 0
  fi
  python3 - "$YAML_GSLAM" "$key" "$default" <<'PY'
import sys, yaml
path, key, default = sys.argv[1], sys.argv[2], sys.argv[3]
with open(path) as f:
    raw = yaml.safe_load(f) or {}
params = (raw.get('/gslam') or {}).get('ros__parameters') or {}
v = params.get(key, default)
print('true' if str(v).lower() in ('1', 'true', 'yes') else 'false')
PY
}
EFF_BA="${ENABLE_GLOBAL_BA:-$(yaml_gslam_bool Enable_global_ba true)}"
EFF_TAG="${ENABLE_TAG_LOOP:-$(yaml_gslam_bool Enable_tag_loop_closure true)}"
EFF_VIS="${ENABLE_VISUAL_LOOP:-$(yaml_gslam_bool Enable_visual_loop_closure false)}"
EFF_APPLY_BA="${APPLY_BA_CORR:-$(yaml_gslam_bool Apply_ba_corrections false)}"
EFF_APPLY_TAG="${APPLY_TAG_CORR:-$(yaml_gslam_bool Apply_tag_loop_corrections false)}"
EFF_APPLY_VIS="${APPLY_VISUAL_CORR:-$(yaml_gslam_bool Apply_visual_loop_corrections false)}"
if [[ "$GLOBAL_CORRECTIONS" == "true" ]]; then
  [[ "$EFF_BA" == "true" ]] && EFF_APPLY_BA=true
  [[ "$EFF_TAG" == "true" ]] && EFF_APPLY_TAG=true
  [[ "$EFF_VIS" == "true" ]] && EFF_APPLY_VIS=true
fi

if [[ "$GLOBAL_CORRECTIONS" == "true" && "$GLOBALSLAM" != "true" ]]; then
  echo "error: --global-corrections true requires --globalslam true"
  exit 1
fi
if [[ "$EFF_TAG" == "true" && "$EFF_VIS" == "true" ]]; then
  echo "error: Enable_tag_loop_closure and Enable_visual_loop_closure cannot both be true"
  echo "  (no simultaneous loop-source arbitration in this slice). Pick one."
  exit 1
fi
if [[ "$EFF_APPLY_BA" == "true" && "$EFF_BA" != "true" ]]; then
  echo "error: Apply_ba_corrections requires Enable_global_ba"
  exit 1
fi
if [[ "$EFF_APPLY_TAG" == "true" && "$EFF_TAG" != "true" ]]; then
  echo "error: Apply_tag_loop_corrections requires Enable_tag_loop_closure"
  exit 1
fi
if [[ "$EFF_APPLY_VIS" == "true" && "$EFF_VIS" != "true" ]]; then
  echo "error: Apply_visual_loop_corrections requires Enable_visual_loop_closure"
  exit 1
fi
# Master + explicit per-source false is a hard conflict (no silent override).
if [[ "$GLOBAL_CORRECTIONS" == "true" ]]; then
  if [[ "$APPLY_BA_CORR" == "false" && "$EFF_BA" == "true" ]]; then
    echo "error: contradictory: --global-corrections true with --apply-ba-corrections false"
    exit 1
  fi
  if [[ "$APPLY_TAG_CORR" == "false" && "$EFF_TAG" == "true" ]]; then
    echo "error: contradictory: --global-corrections true with --apply-tag-loop-corrections false"
    exit 1
  fi
  if [[ "$APPLY_VISUAL_CORR" == "false" && "$EFF_VIS" == "true" ]]; then
    echo "error: contradictory: --global-corrections true with --apply-visual-loop-corrections false"
    exit 1
  fi
fi
CLI_APPLY=false
if [[ "$GLOBAL_CORRECTIONS" == "true" || "$APPLY_BA_CORR" == "true" || "$APPLY_TAG_CORR" == "true" || "$APPLY_VISUAL_CORR" == "true" ]]; then
  CLI_APPLY=true
fi
if [[ "$CLI_APPLY" == "true" && "$GLOBALSLAM" != "true" ]]; then
  echo "error: Local feedback / --global-corrections true requires --globalslam true"
  exit 1
fi
ANY_APPLY=false
if [[ "$EFF_APPLY_BA" == "true" || "$EFF_APPLY_TAG" == "true" || "$EFF_APPLY_VIS" == "true" || "$GLOBAL_CORRECTIONS" == "true" ]]; then
  ANY_APPLY=true
fi

case "$VIEWER" in
  plot2|rviz2|none) ;;
  *)
    echo "error: --viewer must be plot2, rviz2, or none (got: $VIEWER)"
    exit 1
    ;;
esac

INSTALL_REL="${EKF_INSTALL_BASE:-install}"
if [[ "$INSTALL_REL" = /* ]]; then
  INSTALL_DIR="$INSTALL_REL"
else
  INSTALL_DIR="$ROOT/$INSTALL_REL"
fi

if [[ ! -f "$INSTALL_DIR/setup.bash" ]]; then
  echo "error: overlay missing: $INSTALL_DIR/setup.bash"
  echo "Build the integrated stack into that prefix, e.g.:"
  echo "  source /opt/ros/jazzy/setup.bash"
  echo "  colcon build \\"
  echo "    --packages-select interfaces localslam webotbridge control \\"
  echo "      ekf_monoslam_bringup webot_package plot2 keyboard \\"
  echo "    --build-base build --install-base install"
  echo "With Global SLAM, also build globalslam (GTSAM on CMAKE_PREFIX_PATH)."
  echo "Or set EKF_INSTALL_BASE to an existing overlay."
  exit 1
fi

if [[ "$GLOBALSLAM" == "true" ]]; then
  # Merged install: install/share/globalslam
  # Isolated install: install/globalslam/share/globalslam
  if [[ ! -d "$INSTALL_DIR/share/globalslam" &&
        ! -d "$INSTALL_DIR/globalslam/share/globalslam" ]]; then
    echo "error: --globalslam true but globalslam is not in $INSTALL_DIR"
    echo "Build globalslam with GTSAM visible, e.g.:"
    echo "  # shellcheck source=tools/ensure_gtsam_runtime.sh"
    echo "  source tools/ensure_gtsam_runtime.sh && ekf_ensure_gtsam_runtime \"\$PWD\""
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  colcon build --packages-select interfaces globalslam \\"
    echo "    --build-base build --install-base install"
    exit 1
  fi
  # Runtime: resolve libgtsam.so.4 before loading the gslam component.
  # shellcheck source=tools/ensure_gtsam_runtime.sh
  source "$ROOT/tools/ensure_gtsam_runtime.sh"
  ekf_ensure_gtsam_runtime "$ROOT"
fi

# ROS setup scripts reference optional unset vars; relax -u while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$INSTALL_DIR/setup.bash"
set -u

if [[ -z "${WEBOTS_HOME:-}" ]]; then
  for candidate in /usr/local/webots /snap/webots/current/usr/share/webots; do
    if [[ -x "$candidate/webots" ]]; then
      export WEBOTS_HOME="$candidate"
      break
    fi
  done
fi

pick_terminal() {
  if command -v gnome-terminal >/dev/null 2>&1; then
    echo gnome-terminal
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    echo xfce4-terminal
  elif command -v konsole >/dev/null 2>&1; then
    echo konsole
  elif command -v mate-terminal >/dev/null 2>&1; then
    echo mate-terminal
  elif command -v xterm >/dev/null 2>&1; then
    echo xterm
  else
    echo ""
  fi
}

TERM_EMU="$(pick_terminal)"

open_term() {
  local title="$1"
  local cmd="$2"
  case "$TERM_EMU" in
    gnome-terminal)
      gnome-terminal --title="$title" -- bash -lc "$cmd; echo; echo '[exit] press enter'; read -r" &
      ;;
    xfce4-terminal)
      xfce4-terminal --title="$title" -e "bash -lc '$cmd; echo; read -r'" &
      ;;
    konsole)
      konsole --new-tab -p tabtitle="$title" -e bash -lc "$cmd; echo; read -r" &
      ;;
    mate-terminal)
      mate-terminal --title="$title" -e "bash -lc '$cmd; echo; read -r'" &
      ;;
    xterm)
      xterm -T "$title" -e bash -lc "$cmd; echo; read -r" &
      ;;
    *)
      echo "error: no supported terminal emulator (tried gnome-terminal, xfce4-terminal, konsole, mate-terminal, xterm)"
      exit 1
      ;;
  esac
}

# Propagate GTSAM lib path into helper terminals when Global SLAM is on
# (component_container inherits this shell's environment directly).
SETUP_CMD="source /opt/ros/jazzy/setup.bash; source '$INSTALL_DIR/setup.bash'; export WEBOTS_HOME='${WEBOTS_HOME:-}'; export EKF_VIEWER='$VIEWER'"
if [[ "$GLOBALSLAM" == "true" ]]; then
  SETUP_CMD+="; export GTSAM_PREFIX='${GTSAM_PREFIX:-}'; export LD_LIBRARY_PATH='${LD_LIBRARY_PATH:-}'; export CMAKE_PREFIX_PATH='${CMAKE_PREFIX_PATH:-}'"
fi

echo "=== EKF MonoSLAM integrated launch ==="
echo "ROOT=$ROOT"
echo "overlay=$INSTALL_DIR"
echo "WEBOTS_HOME=${WEBOTS_HOME:-unset}"
echo "terminal=$TERM_EMU"
echo "world=$WORLD viewer=$VIEWER"

if [[ "$NO_WEBOTS" != "true" ]]; then
  if ! python3 -c "import webots_ros2_driver" 2>/dev/null; then
    echo "error: webots_ros2_driver not importable. Install:"
    echo "  sudo apt-get install ros-jazzy-webots-ros2"
    exit 1
  fi
  if [[ -z "${WEBOTS_HOME:-}" || ! -x "${WEBOTS_HOME}/webots" ]]; then
    echo "error: Webots binary not found. Install R2025a and set WEBOTS_HOME."
    exit 1
  fi
fi

USE_PLOT2="false"
if [[ "$VIEWER" == "plot2" ]]; then
  USE_PLOT2="true"
  if ! pkg-config --exists glfw3 2>/dev/null; then
    echo "error: GLFW development package missing (required by plot2)."
    echo "Install: sudo apt-get install libglfw3-dev"
    echo "Or relaunch with --viewer none|rviz2"
    exit 1
  fi
  if ! ros2 pkg prefix plot2 >/dev/null 2>&1; then
    echo "error: plot2 not in overlay $INSTALL_DIR. Rebuild with plot2 or use --viewer none|rviz2."
    exit 1
  fi
fi

if [[ "$VIEWER" == "rviz2" ]]; then
  if ! ros2 pkg prefix rviz2 >/dev/null 2>&1; then
    echo "error: rviz2 is not available in this ROS environment."
    echo "Install: sudo apt-get install ros-jazzy-rviz2"
    echo "No silent fallback to Plot2."
    exit 1
  fi
  if ! ros2 pkg prefix ekf_monoslam_bringup >/dev/null 2>&1; then
    echo "error: ekf_monoslam_bringup missing (needed for RViz config)."
    exit 1
  fi
fi

PIDS=()
cleanup() {
  echo
  echo "Shutting down launch2-owned processes..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  pkill -P $$ 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "$NO_WEBOTS" != "true" ]]; then
  WEBOTS_LAUNCH_ARGS=("world:=$WORLD")
  [[ -n "$TRAJ_V_MAX" ]] && WEBOTS_LAUNCH_ARGS+=("traj_v_max:=$TRAJ_V_MAX")
  [[ -n "$TRAJ_V_MAX_XY" ]] && WEBOTS_LAUNCH_ARGS+=("traj_v_max_xy:=$TRAJ_V_MAX_XY")
  [[ -n "$TRAJ_V_MAX_Z" ]] && WEBOTS_LAUNCH_ARGS+=("traj_v_max_z:=$TRAJ_V_MAX_Z")
  [[ -n "$TRAJ_A_MAX" ]] && WEBOTS_LAUNCH_ARGS+=("traj_a_max:=$TRAJ_A_MAX")
  [[ -n "$TRAJ_J_MAX" ]] && WEBOTS_LAUNCH_ARGS+=("traj_j_max:=$TRAJ_J_MAX")
  [[ -n "$TRACKER_V_MAX_XY" ]] && WEBOTS_LAUNCH_ARGS+=("tracker_v_max_xy:=$TRACKER_V_MAX_XY")
  [[ -n "$TRACKER_V_MAX_Z" ]] && WEBOTS_LAUNCH_ARGS+=("tracker_v_max_z:=$TRACKER_V_MAX_Z")
  # Export for any nested tooling; robot_launch also mirrors into controller env.
  [[ -n "$TRAJ_V_MAX" ]] && export WEBOT_TRAJ_V_MAX="$TRAJ_V_MAX"
  [[ -n "$TRAJ_V_MAX_XY" ]] && export WEBOT_TRAJ_V_MAX_XY="$TRAJ_V_MAX_XY"
  [[ -n "$TRAJ_V_MAX_Z" ]] && export WEBOT_TRAJ_V_MAX_Z="$TRAJ_V_MAX_Z"
  [[ -n "$TRAJ_A_MAX" ]] && export WEBOT_TRAJ_A_MAX="$TRAJ_A_MAX"
  [[ -n "$TRAJ_J_MAX" ]] && export WEBOT_TRAJ_J_MAX="$TRAJ_J_MAX"
  [[ -n "$TRACKER_V_MAX_XY" ]] && export WEBOT_TRACKER_V_MAX_XY="$TRACKER_V_MAX_XY"
  [[ -n "$TRACKER_V_MAX_Z" ]] && export WEBOT_TRACKER_V_MAX_Z="$TRACKER_V_MAX_Z"
  open_term "EKF-Webots" "$SETUP_CMD; ros2 launch webot_package robot_launch.py ${WEBOTS_LAUNCH_ARGS[*]}"
  sleep 2
fi

if [[ "$NO_KEYBOARD" != "true" ]]; then
  if ros2 pkg prefix keyboard >/dev/null 2>&1; then
    open_term "EKF-Operator" "$SETUP_CMD; ros2 run keyboard keyboard_node"
  else
    echo "warn: keyboard package missing in overlay; operator menu will not start"
  fi
fi

if [[ "$VIEWER" == "rviz2" ]]; then
  open_term "EKF-RViz2" "$SETUP_CMD; ros2 launch ekf_monoslam_bringup rviz2_experiment.launch.py"
fi

LAUNCH_ARGS=(
  use_plot2:="$USE_PLOT2"
  use_globalslam:="$GLOBALSLAM"
  range_sensor:="$RANGE_SENSOR"
)
echo "range_sensor=$RANGE_SENSOR"
# Omit empty overrides so ROS launch keeps YAML (CLI > YAML > node default).
[[ -n "$GLOBAL_CORRECTIONS" ]] && LAUNCH_ARGS+=(global_corrections:="$GLOBAL_CORRECTIONS")[[ -n "$ENABLE_GLOBAL_BA" ]] && LAUNCH_ARGS+=(enable_global_ba:="$ENABLE_GLOBAL_BA")
[[ -n "$ENABLE_TAG_LOOP" ]] && LAUNCH_ARGS+=(enable_tag_loop_closure:="$ENABLE_TAG_LOOP")
[[ -n "$ENABLE_VISUAL_LOOP" ]] && LAUNCH_ARGS+=(enable_visual_loop_closure:="$ENABLE_VISUAL_LOOP")
[[ -n "$APPLY_BA_CORR" ]] && LAUNCH_ARGS+=(apply_ba_corrections:="$APPLY_BA_CORR")
[[ -n "$APPLY_TAG_CORR" ]] && LAUNCH_ARGS+=(apply_tag_loop_corrections:="$APPLY_TAG_CORR")
[[ -n "$APPLY_VISUAL_CORR" ]] && LAUNCH_ARGS+=(apply_visual_loop_corrections:="$APPLY_VISUAL_CORR")
if [[ -n "$SLAM_FEEDBACK" ]]; then
  LAUNCH_ARGS+=(slam_feedback:="$SLAM_FEEDBACK")
fi
if [[ -n "$SLAM_FEEDBACK_IFACE" ]]; then
  LAUNCH_ARGS+=(slam_feedback_interface:="$SLAM_FEEDBACK_IFACE")
fi
if [[ -n "$CONTROL_PLAN" ]]; then
  LAUNCH_ARGS+=(control_plan:="$CONTROL_PLAN")
fi
if [[ -n "$PARAMS_FILE" ]]; then
  if [[ ! -f "$PARAMS_FILE" ]]; then
    echo "error: --params-file not found: $PARAMS_FILE"
    exit 1
  fi
  # Require absolute path so composable nodes resolve independently of cwd.
  if [[ "$PARAMS_FILE" != /* ]]; then
    PARAMS_FILE="$(cd "$(dirname "$PARAMS_FILE")" && pwd)/$(basename "$PARAMS_FILE")"
  fi
  LAUNCH_ARGS+=(params_file:="$PARAMS_FILE")
  echo "params_file=$PARAMS_FILE"
fi

FB_DESC="YAML default"
if [[ -n "$SLAM_FEEDBACK" ]]; then
  if [[ "$SLAM_FEEDBACK" == "true" || "$SLAM_FEEDBACK" == "1" || "$SLAM_FEEDBACK" == "yes" ]]; then
    FB_DESC="odometry (${SLAM_FEEDBACK_IFACE:-odometry})"
  else
    FB_DESC="gt (Webots GPS)"
  fi
fi

if [[ "$GLOBALSLAM" == "true" ]]; then
  MASTER_DESC="${GLOBAL_CORRECTIONS:-YAML}"
  echo "Global SLAM: ON"
  echo "  G6d matrix (CLI>YAML): ba=${EFF_BA}/fb=${EFF_APPLY_BA} tag=${EFF_TAG}/fb=${EFF_APPLY_TAG} vis=${EFF_VIS}/fb=${EFF_APPLY_VIS} master=${MASTER_DESC}"
  echo "  nav_feedback_mode=$FB_DESC (--slam-feedback is NOT Local corrections)"
  if [[ "$ANY_APPLY" == "true" ]]; then
    echo "  estimator-only Local feedback ON (Coordinate_mode=offline_bypass; no CoordPause)"
  else
    echo "  Local feedback OFF; Path/cloud on /gslam/global_* ; GlobalLoopCommitted informational"
  fi
else
  echo "Global SLAM: OFF (local-only default)"
fi

if [[ "$AUTO_START_PLAN" == "true" ]]; then
  echo "auto-start-plan=true (will call /control_run_service when ready; no manual 'a')"
  (
    # Wait for control service, then start the plan once.
    # Use cmd 104 == 'h' (start/resume), NOT 114 == 'r' (toggle): a duplicate
    # call must not stop the plan if a stale auto-start races the new one.
    for _i in $(seq 1 90); do
      if ros2 service list 2>/dev/null | grep -q '^/control_run_service$'; then
        sleep 3
        echo "auto-start-plan: requesting Run control plan..."
        ros2 service call /control_run_service interfaces/srv/SimpleServ "{cmd: 104}" \
          || echo "auto-start-plan: service call failed" >&2
        exit 0
      fi
      sleep 1
    done
    echo "auto-start-plan: timed out waiting for /control_run_service" >&2
  ) &
  PIDS+=("$!")
else
  echo "auto-start-plan=false (press 'a' in the operator terminal to run the plan)"
fi

echo "Starting application container (status in this terminal)..."
# Foreground: component logs stay in the launching terminal (status terminal).
ros2 launch ekf_monoslam_bringup quad_experiment.launch.py "${LAUNCH_ARGS[@]}"
