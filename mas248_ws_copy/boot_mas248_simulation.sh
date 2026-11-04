#!/usr/bin/env bash
set -Eeuo pipefail

# Simulation-only runner for the MAS248 image-to-UR5e demo.
# Uses mas248_ur5e_moveit_config's mock ros2_control model. It never launches
# the separate mas248_ur5e_real physical-driver package.

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
IMAGE="${1:-$WS/src/mas248_ur5e_demos/images}"
TARGETS="/tmp/mas248_targets.json"
CTRL_LOG="/tmp/mas248_sim_ros2_control.log"
MOVEIT_LOG="/tmp/mas248_sim_moveit.log"
TRACE_LOG="/tmp/mas248_sim_tcp_trace.log"
CTRL_PID=""
MOVEIT_PID=""
TRACE_PID=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "$TRACE_PID" "$MOVEIT_PID" "$CTRL_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$TRACE_PID" "$MOVEIT_PID" "$CTRL_PID"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ROS setup scripts may reference optional variables that are unset. Source
# them with Bash nounset disabled, then restore strict checking for this runner.
source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 Humble setup not found: $ROS_SETUP" >&2
  exit 1
fi
if [[ ! -d "$WS/src/mas248_ur5e_moveit_config" || ! -d "$WS/src/mas248_ur5e_demos" ]]; then
  echo "Run this script from ~/mas248_ws (or copy it there). Expected both packages under src/." >&2
  exit 1
fi
MOCK_XACRO="$WS/src/mas248_ur5e_moveit_config/urdf/ur5e_workcell.urdf.xacro"
if [[ ! -f "$MOCK_XACRO" ]] || ! grep -q 'use_fake_hardware="true"' "$MOCK_XACRO"; then
  echo "Simulation safety check failed: expected Humble use_fake_hardware in $MOCK_XACRO" >&2
  exit 1
fi
if [[ -d "$WS/src/mas248_ur5e_real" ]]; then
  echo "Simulation mode: mas248_ur5e_real detected and intentionally excluded; it launches the physical UR driver."
fi
if [[ -d "$IMAGE" ]]; then
  IMAGE_DIR="$IMAGE"
  if [[ -f "$IMAGE_DIR/pumpkin_outline.jpg" ]]; then
    IMAGE="$IMAGE_DIR/pumpkin_outline.jpg"
  else
    shopt -s nullglob nocaseglob
    IMAGE_CANDIDATES=("$IMAGE_DIR"/*.png "$IMAGE_DIR"/*.jpg "$IMAGE_DIR"/*.jpeg "$IMAGE_DIR"/*.bmp)
    shopt -u nullglob nocaseglob
    if [[ ${#IMAGE_CANDIDATES[@]} -eq 1 ]]; then
      IMAGE="${IMAGE_CANDIDATES[0]}"
    elif [[ ${#IMAGE_CANDIDATES[@]} -eq 0 ]]; then
      echo "No supported images found in: $IMAGE_DIR" >&2
      echo "Supported formats: PNG, JPG, JPEG, BMP." >&2
      exit 1
    else
      echo "More than one image found in $IMAGE_DIR. Pass the specific image file:" >&2
      printf '  %s\n' "${IMAGE_CANDIDATES[@]}" >&2
      exit 1
    fi
  fi
elif [[ ! -f "$IMAGE" ]]; then
  echo "Image file or folder not found: $IMAGE" >&2
  echo "Usage: $0 [path/to/image-or-image-folder]" >&2
  exit 1
fi
IMAGE="$(realpath "$IMAGE")"
DIMENSIONS_FILE="$WS/src/mas248_ur5e_moveit_config/config/workcell_dimensions.yaml"
echo "Using image: $IMAGE"

echo "Sourcing ROS 2 Humble and building only the mock config and demo packages..."
source_setup "$ROS_SETUP"
cd "$WS"
colcon build --symlink-install --packages-select mas248_ur5e_moveit_config mas248_ur5e_demos
source_setup "$WS/install/setup.bash"

echo "Checking installed xacro and drawing command..."
XACRO="$WS/install/mas248_ur5e_moveit_config/share/mas248_ur5e_moveit_config/urdf/ur5e_workcell.urdf.xacro"
[[ -f "$XACRO" ]] || { echo "Installed xacro missing: $XACRO" >&2; exit 1; }
ros2 pkg executables mas248_ur5e_demos | grep -q 'run_picture' || {
  echo "run_picture command is not installed. Check setup.py and rebuild." >&2
  exit 1
}

EXISTING_MOVE_GROUP_PIDS="$(pgrep -x move_group || true)"
EXISTING_CONTROL_PIDS="$(pgrep -f '[r]os2_control_node' || true)"
if [[ -n "$EXISTING_MOVE_GROUP_PIDS$EXISTING_CONTROL_PIDS" ]]; then
  echo "Existing MoveIt or ros2_control processes were found; stop them before starting this simulation." >&2
  [[ -z "$EXISTING_MOVE_GROUP_PIDS" ]] || echo "move_group PIDs: $EXISTING_MOVE_GROUP_PIDS" >&2
  [[ -z "$EXISTING_CONTROL_PIDS" ]] || echo "ros2_control_node PIDs: $EXISTING_CONTROL_PIDS" >&2
  exit 1
fi

echo "Starting mock controllers (log: $CTRL_LOG)..."
setsid ros2 launch mas248_ur5e_moveit_config ros2_control.launch.py >"$CTRL_LOG" 2>&1 &
CTRL_PID=$!
sleep 5
if ! kill -0 "$CTRL_PID" 2>/dev/null; then
  echo "Controller launch stopped early. Recent log:" >&2
  tail -60 "$CTRL_LOG" >&2
  exit 1
fi

echo "Starting MoveIt and RViz (log: $MOVEIT_LOG)..."
setsid ros2 launch mas248_ur5e_moveit_config moveit.launch.py >"$MOVEIT_LOG" 2>&1 &
MOVEIT_PID=$!

echo "Waiting for MoveIt services..."
ready=0
for _ in $(seq 1 60); do
  if ! kill -0 "$CTRL_PID" 2>/dev/null || ! kill -0 "$MOVEIT_PID" 2>/dev/null; then
    echo "A launch process stopped. Controller log: $CTRL_LOG; MoveIt log: $MOVEIT_LOG" >&2
    tail -40 "$CTRL_LOG" >&2
    tail -40 "$MOVEIT_LOG" >&2
    exit 1
  fi
  if ros2 service list 2>/dev/null | grep -q '^/compute_cartesian_path$' \
    && ros2 action list 2>/dev/null | grep -q '^/move_action$' \
    && ros2 action list 2>/dev/null | grep -q '^/execute_trajectory$'; then
    ready=1
    break
  fi
  sleep 2
done
if [[ "$ready" != 1 ]]; then
  echo "MoveIt services did not become ready. Recent logs:" >&2
  tail -60 "$CTRL_LOG" >&2
  tail -60 "$MOVEIT_LOG" >&2
  exit 1
fi

echo "Checking ROS stack endpoints..."
STACK_STATUS="$(ros2 run mas248_ur5e_demos check_stack 2>&1)" || {
  echo "$STACK_STATUS" >&2
  exit 1
}
echo "$STACK_STATUS"
if grep -q '\[MISS\]' <<<"$STACK_STATUS"; then
  echo "Some ROS endpoints are missing; stopping before motion. See $CTRL_LOG and $MOVEIT_LOG." >&2
  exit 1
fi

echo "Reading paper location and drawing width from $DIMENSIONS_FILE..."
read -r IMAGE_WIDTH CENTER_X CENTER_Y CENTER_Z < <(python3 -c 'import sys,yaml; p=yaml.safe_load(open(sys.argv[1]))["paper"]; print(p["drawing_width_m"],p["center_x_m"],p["center_y_m"],p["surface_z_m"])' "$DIMENSIONS_FILE")
echo "Converting image to target poses..."
ros2 run mas248_ur5e_demos image_to_targets "$IMAGE" \
  --width-m "$IMAGE_WIDTH" --center-x "$CENTER_X" --center-y "$CENTER_Y" --center-z "$CENTER_Z" \
  --max-contours 4 --max-points 40 --open-stages --output "$TARGETS"
[[ -s "$TARGETS" ]] || { echo "Target file was not created: $TARGETS" >&2; exit 1; }

echo "Starting the live TCP trace publisher in RViz (log: $TRACE_LOG)..."
setsid ros2 run mas248_ur5e_demos show_targets_rviz --json "$TARGETS" >"$TRACE_LOG" 2>&1 &
TRACE_PID=$!
sleep 2
if ! kill -0 "$TRACE_PID" 2>/dev/null; then
  echo "Trace publisher stopped early. Recent log:" >&2
  tail -60 "$TRACE_LOG" >&2
  exit 1
fi
echo "RViz target markers: /mas248/drawing_targets; live TCP trace: /mas248/tcp_path"

echo "Planning all strokes (plan only; no movement yet)..."
ros2 run mas248_ur5e_demos run_picture --targets "$TARGETS"

echo
read -r -p "Preflight succeeded. Execute on the supplied MOCK controller? Type yes to continue: " answer
if [[ "$answer" == "yes" ]]; then
  ros2 run mas248_ur5e_demos run_picture --targets "$TARGETS" --execute
else
  echo "Execution skipped. The successful plan is complete; robot remains uncommanded."
fi

echo "Demo finished. Shutting down the ROS launch processes."
