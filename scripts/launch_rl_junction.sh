#!/usr/bin/env bash
# Separate Gazebo world. Does not start patrol, manual control or hardware UART.
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(dirname "$DIR")"
GUI=false
if [[ "${1:-}" == --gui ]]; then GUI=true; shift; fi
if (( $# )); then echo "Usage: bash $0 [--gui]" >&2; exit 2; fi
set +u
source /opt/ros/humble/setup.bash
if [[ -r "$PROJECT/install/setup.bash" ]]; then source "$PROJECT/install/setup.bash"; fi
set -u
export ROS_DOMAIN_ID=42
export IGN_PARTITION=4wd_rl_junction
export GZ_PARTITION="$IGN_PARTITION"
if [[ -r /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
export IGN_GAZEBO_RESOURCE_PATH="$PROJECT/pig_model/models:$PROJECT/pig_model:${IGN_GAZEBO_RESOURCE_PATH:-}"
exec 9>/tmp/4wd_rl_junction.lock
flock -n 9 || { echo "RL junction environment already running" >&2; exit 1; }
mkdir -p "$PROJECT/logs/rl_junction"
RUN_DIR=$(mktemp -d "$PROJECT/logs/rl_junction/run.XXXXXX")
PIDS=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    sleep 1
    for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
background() {
    local log="$1"; shift
    setsid "$@" >"$RUN_DIR/$log" 2>&1 9>&- &
    PIDS+=("$!")
}
WORLD=pig_pen_16units_world
/usr/bin/python3 "$PROJECT/rl_junction/prepare_scene.py" --world "$PROJECT/worlds/pig_pen_16units.world" \
    --robot "$PROJECT/turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf" \
    --output-dir "$RUN_DIR"
echo "Training world: $WORLD (original pig pen); domain=42; partition=$IGN_PARTITION"
echo "Logs: $RUN_DIR"
ARGS=(-s -r)
if [[ "$GUI" == false ]]; then ARGS+=(--headless-rendering); fi
background server.log ign gazebo "${ARGS[@]}" "$RUN_DIR/training.world"
SERVER=${PIDS[0]}
sleep 3
background bridge.log ros2 run ros_gz_bridge parameter_bridge \
    "/world/$WORLD/control@ros_gz_interfaces/srv/ControlWorld" \
    "/world/$WORLD/set_pose@ros_gz_interfaces/srv/SetEntityPose" \
    '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan' \
    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
    '/model/wheeltec_mini/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
    '/model/wheeltec_mini/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose' \
    "/world/$WORLD/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock" \
    --ros-args -r /model/wheeltec_mini/odometry:=/odom \
    -r /model/wheeltec_mini/pose:=/rl/ground_truth \
    -r /world/$WORLD/clock:=/clock
timeout 45s ros2 run ros_gz_sim create -world "$WORLD" -name wheeltec_mini \
    -file "$RUN_DIR/training.urdf" -x 0 -y -8.18 -z .05 -Y 1.5707963267948966 \
    >"$RUN_DIR/spawn.log" 2>&1 9>&-
if [[ "$GUI" == true ]]; then background gui.log ign gazebo -g; fi
echo "Scene launched. Collect FSM demonstrations with rl_junction/collect_demos.py in domain 42."
echo "Train MLP+BC offline with rl_junction/train_bc.py after data validation."
echo "Ctrl-C stops only this environment's process groups."
wait -n "${PIDS[@]}"
