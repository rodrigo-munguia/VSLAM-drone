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
#   ./launch2.sh --world mars.wbt --slam-feedback false
#   EKF_INSTALL_BASE=install ./launch2.sh
#
# Standalone Local SLAM (no Webots) remains:
#   ros2 launch localslam localslam.launch.py ...
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

WORLD="mars.wbt"
VIEWER="plot2"
SLAM_FEEDBACK=""
SLAM_FEEDBACK_IFACE=""
PARAMS_FILE=""
CONTROL_PLAN=""
NO_WEBOTS="false"
NO_KEYBOARD="false"

usage() {
  cat <<EOF
Usage: ./launch2.sh [options]

  --world NAME.wbt           Webots world (default: mars.wbt)
  --viewer plot2|rviz2|none  Visualization (default: plot2)
  --slam-feedback true|false Override control Slam_feedback
  --slam-feedback-interface odometry|legacy
  --params-file PATH         Unused placeholder (edit config/ + reinstall bringup)
  --control-plan PATH        Absolute path override for Control_plan_path
  --no-plot2                 Compatibility alias for --viewer none
  --no-webots                Do not spawn Webots (app+keyboard only)
  --no-keyboard              Do not spawn operator terminal
  -h, --help                 Show this help

Environment:
  EKF_INSTALL_BASE   Install overlay under repo (default: install)
  WEBOTS_HOME        Webots install (auto-detected if unset)
  ROS_DISTRO         Expected jazzy
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world) WORLD="$2"; shift 2 ;;
    --viewer)
      VIEWER="$2"
      shift 2
      ;;
    --slam-feedback) SLAM_FEEDBACK="$2"; shift 2 ;;
    --slam-feedback-interface) SLAM_FEEDBACK_IFACE="$2"; shift 2 ;;
    --params-file) PARAMS_FILE="$2"; shift 2 ;;
    --control-plan) CONTROL_PLAN="$2"; shift 2 ;;
    --no-plot2) VIEWER="none"; shift ;;
    --no-webots) NO_WEBOTS="true"; shift ;;
    --no-keyboard) NO_KEYBOARD="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

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
  echo "Or set EKF_INSTALL_BASE to an existing overlay."
  exit 1
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

SETUP_CMD="source /opt/ros/jazzy/setup.bash; source '$INSTALL_DIR/setup.bash'; export WEBOTS_HOME='${WEBOTS_HOME:-}'; export EKF_VIEWER='$VIEWER'"

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
  open_term "EKF-Webots" "$SETUP_CMD; ros2 launch webot_package robot_launch.py world:=$WORLD"
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

LAUNCH_ARGS=(use_plot2:="$USE_PLOT2")
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
  echo "note: --params-file ignored; edit config/quad_params.yaml and rebuild/reinstall ekf_monoslam_bringup"
fi

echo "Starting application container (status in this terminal)..."
# Foreground: component logs stay in the launching terminal (status terminal).
ros2 launch ekf_monoslam_bringup quad_experiment.launch.py "${LAUNCH_ARGS[@]}"
