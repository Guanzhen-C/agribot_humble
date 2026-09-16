#!/usr/bin/env bash

set -Eeuo pipefail

WORKSPACE="${AGRIBOT_ROS_WORKSPACE:-$HOME/agribot_ws/ros2_ws}"
EXPORT_DIR=""
GUI="${AGRIBOT_SIM_GUI:-true}"
RVIZ="${AGRIBOT_SIM_RVIZ:-true}"
RUN_WAYPOINTS="${AGRIBOT_SIM_RUN_WAYPOINTS:-false}"
PREPARE_ONLY=false
STOP_ONLY=false

usage() {
  cat <<'EOF'
Usage:
  unity_sync_and_simulate.sh --export-dir PATH [options]
  unity_sync_and_simulate.sh --stop

Options:
  --workspace PATH       ROS 2 workspace (default: ~/agribot_ws/ros2_ws)
  --gui true|false       Open Gazebo GUI (default: true)
  --rviz true|false      Open RViz (default: true)
  --run-waypoints BOOL   Run configured waypoints (default: false)
  --prepare-only         Import and build without restarting simulation
  --stop                 Stop the simulation managed by this script
  -h, --help             Show this help
EOF
}

while (($#)); do
  case "$1" in
    --export-dir)
      EXPORT_DIR="${2:?missing value for --export-dir}"
      shift 2
      ;;
    --workspace)
      WORKSPACE="${2:?missing value for --workspace}"
      shift 2
      ;;
    --gui)
      GUI="${2:?missing value for --gui}"
      shift 2
      ;;
    --rviz)
      RVIZ="${2:?missing value for --rviz}"
      shift 2
      ;;
    --run-waypoints)
      RUN_WAYPOINTS="${2:?missing value for --run-waypoints}"
      shift 2
      ;;
    --prepare-only)
      PREPARE_ONLY=true
      shift
      ;;
    --stop)
      STOP_ONLY=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

for value in "$GUI" "$RVIZ" "$RUN_WAYPOINTS"; do
  if [[ "$value" != "true" && "$value" != "false" ]]; then
    echo "ERROR: boolean arguments must be true or false: $value" >&2
    exit 2
  fi
done

STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/agribot-unity-sim"
PID_FILE="$STATE_DIR/simulation.pid"
LOCK_FILE="$STATE_DIR/sync.lock"
STATUS_FILE="$STATE_DIR/status"
LOG_FILE="$STATE_DIR/latest.log"
mkdir -p "$STATE_DIR"
: >"$LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*"
}

set_status() {
  printf '%s\n' "$1" >"$STATUS_FILE"
}

managed_process_is_running() {
  [[ -s "$PID_FILE" ]] || return 1
  local pid command
  pid="$(cat "$PID_FILE")"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  command="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
  [[ "$command" == *"configured_ackermann_sim.launch.py"* ]]
}

stop_managed_simulation() {
  if ! managed_process_is_running; then
    rm -f "$PID_FILE"
    log "没有正在运行的 Unity 受管仿真。"
    return 0
  fi

  local pid
  pid="$(cat "$PID_FILE")"
  log "正在停止上一轮 Unity 受管仿真（PID $pid）..."
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.2
  done
  if kill -0 "$pid" 2>/dev/null; then
    log "上一轮仿真未及时退出，发送 TERM。"
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  fi
  rm -f "$PID_FILE"
}

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "ERROR: another Unity-to-ROS synchronization is already running." >&2
  exit 3
fi

if [[ "$STOP_ONLY" == "true" ]]; then
  stop_managed_simulation
  set_status "stopped"
  exit 0
fi

if [[ -z "$EXPORT_DIR" ]]; then
  echo "ERROR: --export-dir is required." >&2
  usage >&2
  exit 2
fi

WORKSPACE="$(realpath "$WORKSPACE")"
EXPORT_DIR="$(realpath "$EXPORT_DIR")"
CONFIG_JSON="$EXPORT_DIR/vehicle_config.json"
CONFIG_MESH="$EXPORT_DIR/vehicle_visual.obj"
TOOLS_SOURCE="$WORKSPACE/src/agribot_vehicle_config_tools"
DESCRIPTION_SOURCE="$WORKSPACE/src/agribot_vehicle_description"
CANONICAL_CONFIG="$DESCRIPTION_SOURCE/config/vehicles/ackermann_current/vehicle_config.json"
GENERATED_OUTPUT="$DESCRIPTION_SOURCE/generated/ackermann_current"

for required in \
  /opt/ros/humble/setup.bash \
  "$CONFIG_JSON" \
  "$CONFIG_MESH" \
  "$TOOLS_SOURCE/agribot_vehicle_config_tools/cli.py" \
  "$DESCRIPTION_SOURCE/launch/configured_ackermann_sim.launch.py"; do
  if [[ ! -e "$required" ]]; then
    echo "ERROR: required file is missing: $required" >&2
    set_status "failed: missing $required"
    exit 4
  fi
done

set_status "preparing"
log "Unity 导出目录：$EXPORT_DIR"
log "ROS 工作区：$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
set -u
cd "$WORKSPACE"

log "校验 Unity 配置并生成 ROS 文件..."
PYTHONPATH="$TOOLS_SOURCE${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m agribot_vehicle_config_tools.cli import-unity \
    "$EXPORT_DIR" \
    --canonical "$CANONICAL_CONFIG" \
    --workspace-src "$WORKSPACE/src" \
    --output "$GENERATED_OUTPUT"

log "增量编译车辆配置工具和描述包..."
colcon build \
  --symlink-install \
  --parallel-workers "${AGRIBOT_BUILD_WORKERS:-4}" \
  --packages-select agribot_vehicle_config_tools agribot_vehicle_description

set +u
source "$WORKSPACE/install/setup.bash"
set -u
ros2 pkg prefix agribot_vehicle_description >/dev/null
ros2 launch agribot_vehicle_description configured_ackermann_sim.launch.py \
  --show-args >/dev/null

if [[ "$PREPARE_ONLY" == "true" ]]; then
  set_status "prepared"
  log "配置导入和编译完成；按要求未启动仿真。"
  exit 0
fi

# The old simulation is stopped only after validation and build succeed.
stop_managed_simulation
flock -u 9
exec 9>&-

export ROS_DOMAIN_ID="${AGRIBOT_SIM_ROS_DOMAIN_ID:-37}"
export ROS_LOCALHOST_ONLY="${AGRIBOT_SIM_ROS_LOCALHOST_ONLY:-1}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

set_status "starting"
log "启动 Gazebo 和 RViz（ROS_DOMAIN_ID=$ROS_DOMAIN_ID）..."
setsid ros2 launch agribot_vehicle_description configured_ackermann_sim.launch.py \
  gui:="$GUI" \
  rviz:="$RVIZ" \
  run_waypoints:="$RUN_WAYPOINTS" &
SIM_PID=$!
printf '%s\n' "$SIM_PID" >"$PID_FILE"
set_status "running: $SIM_PID"
log "仿真已启动（PID $SIM_PID）。再次从 Unity 导出时会自动切换到新配置。"

cleanup_pid_file() {
  if [[ -s "$PID_FILE" && "$(cat "$PID_FILE")" == "$SIM_PID" ]]; then
    rm -f "$PID_FILE"
  fi
}

forward_signal() {
  kill -INT -- "-$SIM_PID" 2>/dev/null || true
}

trap cleanup_pid_file EXIT
trap forward_signal HUP INT TERM
set +e
wait "$SIM_PID"
result=$?
set -e
set_status "stopped: $result"
log "仿真进程已退出（状态 $result）。"
exit "$result"
