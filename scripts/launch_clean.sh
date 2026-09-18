#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEADLESS=false
ENV_ID=""
TARGET_RTF=""
LIDAR_ONLY=false
TEMP_SCENE_DIR=""
WORLD_FILE="$DIR/../worlds/pig_pen_16units.world"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless) HEADLESS=true; shift ;;
        --env-id)   ENV_ID="$2"; shift 2 ;;
        --rtf) TARGET_RTF="$2"; shift 2 ;;
        --lidar-only) LIDAR_ONLY=true; shift ;;
        *)          WORLD_FILE="$1"; shift ;;
    esac
done

LOG_DIR="$DIR/../logs"
if [[ -n "$ENV_ID" ]]; then
    if [[ ! "$ENV_ID" =~ ^[0-9]+$ ]] || (( ENV_ID > 100 )); then
        echo "--env-id must be a ROS domain number between 0 and 100" >&2
        exit 1
    fi
    export ROS_DOMAIN_ID="$ENV_ID"
    export IGN_PARTITION="${IGN_PARTITION:-4wd_env_$ENV_ID}"
    export GZ_PARTITION="$IGN_PARTITION"
    LOG_DIR="$LOG_DIR/env_$ENV_ID"
    exec 9>"/tmp/4wd_ros_domain_${ENV_ID}.lock"
    flock -n 9 || { echo "ROS domain $ENV_ID is already used by a 4wd simulation" >&2; exit 1; }
fi
MODEL="wheeltec_mini"
URDF="$DIR/../turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf"
RVIZ_CONFIG="$DIR/../rviz/wheeltec.rviz"

mkdir -p "$LOG_DIR"

if [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi
if [[ -f "$DIR/../install/setup.bash" ]]; then
    set +u
    source "$DIR/../install/setup.bash"
    set -u
fi

PIG_MODEL_PKG_DIR="${PIG_MODEL_PKG_DIR:-$HOME/Desktop/4wd/pig_model}"
if [[ -d "$PIG_MODEL_PKG_DIR" ]]; then
    export IGN_GAZEBO_RESOURCE_PATH="$PIG_MODEL_PKG_DIR/models:$(dirname "$PIG_MODEL_PKG_DIR"):${IGN_GAZEBO_RESOURCE_PATH:-}"
    export GZ_SIM_RESOURCE_PATH="$PIG_MODEL_PKG_DIR/models:$(dirname "$PIG_MODEL_PKG_DIR"):${GZ_SIM_RESOURCE_PATH:-}"
    echo "pig_model 資源路徑已加入: $PIG_MODEL_PKG_DIR"
else
    echo "警告: 找不到 pig_model 路徑 ($PIG_MODEL_PKG_DIR)" >&2
fi

export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export IGN_TRANSPORT_TOPIC_STATISTICS=0

if [[ -n "$TARGET_RTF" || "$LIDAR_ONLY" == "true" ]]; then
    ORIGINAL_WORLD_DIR="$(cd "$(dirname "$WORLD_FILE")" && pwd)"
    TEMP_SCENE_DIR="$(mktemp -d /tmp/4wd-rl-scene.XXXXXX)"
    PREPARE_ARGS=(--world "$WORLD_FILE" --robot "$URDF" --output-dir "$TEMP_SCENE_DIR")
    if [[ -n "$TARGET_RTF" ]]; then PREPARE_ARGS+=(--rtf "$TARGET_RTF"); fi
    if [[ "$LIDAR_ONLY" == "true" ]]; then PREPARE_ARGS+=(--lidar-only); fi
    if ! python3 "$DIR/prepare_training_scene.py" "${PREPARE_ARGS[@]}"; then
        rm -rf -- "$TEMP_SCENE_DIR"
        exit 1
    fi
    export IGN_GAZEBO_RESOURCE_PATH="$ORIGINAL_WORLD_DIR:${IGN_GAZEBO_RESOURCE_PATH:-}"
    WORLD_FILE="$TEMP_SCENE_DIR/training.world"
    URDF="$TEMP_SCENE_DIR/training.urdf"
fi

CHILD_PIDS=()
start_background() {
    local log_name="$1"
    shift
    setsid "$@" > "$LOG_DIR/$log_name" 2>&1 &
    LAST_PID=$!
    CHILD_PIDS+=("$LAST_PID")
}
cleanup() {
    trap - EXIT INT TERM
    echo "Stopping this simulation's process groups..."
    for pid in "${CHILD_PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    sleep 1
    for pid in "${CHILD_PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    if [[ -n "$TEMP_SCENE_DIR" ]]; then rm -rf -- "$TEMP_SCENE_DIR"; fi
    echo "Cleanup complete!"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "World:    $WORLD_FILE"
echo "Logs:     $LOG_DIR"
echo "Headless: $HEADLESS"
echo "ROS domain: ${ROS_DOMAIN_ID:-0}; Gazebo partition: ${IGN_PARTITION:-default}"

echo "Starting ign gazebo server..."
GZ_ARGS=(-s -r)
if [[ "$HEADLESS" == "true" ]]; then GZ_ARGS+=(--headless-rendering); fi
start_background gz_sim_server.log ign gazebo "${GZ_ARGS[@]}" "$WORLD_FILE"
GZ_PID=$LAST_PID
sleep 3

if [[ "$HEADLESS" == "false" ]]; then
    echo "Starting ign gazebo GUI..."
    start_background gz_sim_gui.log env QSG_RENDER_LOOP=basic LIBGL_ALWAYS_SOFTWARE=0 __GLX_VENDOR_LIBRARY_NAME=nvidia QSG_RHI_BACKEND=opengl ign gazebo -g
    GZ_GUI_PID=$LAST_PID
    sleep 2
fi

if [[ -f "$DIR/pig_behavior.py" ]]; then
    echo "Starting pig behavior controller..."
    start_background pig_behavior.log python3 -u "$DIR/pig_behavior.py" --world "$WORLD_FILE"
fi

echo "Starting ros_gz_bridge..."
start_background ros_gz_bridge.log ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
    /model/${MODEL}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    --ros-args \
    -r /model/${MODEL}/odometry:=/odom \
    -r /scan:=/scan \
    -r /cmd_vel:=/cmd_vel

if [[ "$HEADLESS" == "false" && "$LIDAR_ONLY" == "false" ]]; then
    start_background ros_gz_bridge_camera.log ros2 run ros_gz_bridge parameter_bridge \
        /camera@sensor_msgs/msg/Image[gz.msgs.Image \
        /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
        /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
        /camera_right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
        --ros-args \
        -r /camera:=/camera/depth/image_raw \
        -r /camera_info:=/camera/depth/camera_info \
        -r /camera/color/image_raw:=/camera/color/image_raw \
        -r /camera_right/image_raw:=/camera_right/image_raw
fi

sleep 1

if [[ -f "$URDF" && -x "$DIR/spawn_robot.sh" ]]; then
    echo "Spawning robot..."
    start_background spawn.log bash "$DIR/spawn_robot.sh" --file "$URDF" --model "$MODEL" \
        --pos 0 -11.3 0.05 --yaw 1.5708
fi

if [[ "$HEADLESS" == "false" && "$LIDAR_ONLY" == "false" ]] && [[ -f "$DIR/aruco_face_detector.py" ]]; then
    echo "Starting ArUco face detector..."
    start_background aruco_face_detector.log python3 -u "$DIR/aruco_face_detector.py"
fi

if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix robot_state_publisher >/dev/null 2>&1; then
    echo "Starting robot_state_publisher..."
    start_background robot_state_publisher.log ros2 run robot_state_publisher robot_state_publisher --ros-args \
        -p robot_description:="$(tr '\n' ' ' < "$URDF")"
fi

echo "Starting static TF publishers..."
start_background tf_odom_base_link.log ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
start_background tf_base_depth_camera.log ros2 run tf2_ros static_transform_publisher 0.2 0 0.095 0.5 -0.5 0.5 -0.5 \
    base_link wheeltec_mini/base_link/depth_camera
start_background tf_base_lidar.log ros2 run tf2_ros static_transform_publisher 0 0 0.245 0 0 0 \
    base_link wheeltec_mini/base_link/lidar
start_background tf_base_right_camera.log ros2 run tf2_ros static_transform_publisher 0 -0.15 0.15 -1.5708 0 0 \
    base_link right_camera_link

if [[ "$HEADLESS" == "false" && "$LIDAR_ONLY" == "false" ]]; then
    if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix depth_image_proc >/dev/null 2>&1; then
        echo "Starting depth_image_proc..."
        start_background depth_image_proc.log ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
            -r __ns:=/camera/depth -r image_rect:=image_raw -r points:=points
        if [[ -f "$DIR/pointcloud_qos_relay.py" ]]; then
            start_background pointcloud_qos_relay.log python3 "$DIR/pointcloud_qos_relay.py"
        fi
    fi
    if command -v rviz2 >/dev/null 2>&1 && [[ -f "$RVIZ_CONFIG" ]]; then
        echo "Starting RViz..."
        start_background rviz.log rviz2 -d "$RVIZ_CONFIG"
    fi
fi

echo "Done. Press Ctrl-C to exit and cleanup."
wait -n "$GZ_PID" || true
