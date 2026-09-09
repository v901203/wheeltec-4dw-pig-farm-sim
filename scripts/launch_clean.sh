#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEADLESS=false
ENV_ID=""
WORLD_FILE="$DIR/../worlds/pig_pen_16units.world"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless) HEADLESS=true; shift ;;
        --env-id)   ENV_ID="$2"; shift 2 ;;
        *)          WORLD_FILE="$1"; shift ;;
    esac
done

LOG_DIR="$DIR/../logs"
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
export IGN_TRANSPORT_TOPIC_STATISTICS=0

cleanup() {
    echo "======================================"
    echo "Initiating Cleanup..."
    echo "======================================"
    pkill -f "ros_gz_bridge"         || true
    pkill -f "robot_state_publisher" || true
    pkill -f "depth_image_proc"      || true
    pkill -f "pointcloud_qos_relay"  || true
    pkill -f "spawn_robot.sh"        || true
    pkill -f "pig_behavior.py"       || true
    pkill -f "aruco_face_detector"   || true
    pkill -f "static_transform_publisher" || true
    sleep 1
    pkill -9 -f "ign gazebo" || true
    killall -9 ign 2>/dev/null || true
    echo "Cleanup complete!"
}
trap cleanup EXIT

echo "World:    $WORLD_FILE"
echo "Logs:     $LOG_DIR"
echo "Headless: $HEADLESS"

echo "Starting ign gazebo server..."
ign gazebo -s -r "$WORLD_FILE" &> "$LOG_DIR/gz_sim_server.log" &
GZ_PID=$!
sleep 3

if [[ "$HEADLESS" == "false" ]]; then
    echo "Starting ign gazebo GUI..."
    ign gazebo -g &> "$LOG_DIR/gz_sim_gui.log" &
    GZ_GUI_PID=$!
    sleep 2
fi

if [[ "$HEADLESS" == "false" ]] && [[ -f "$DIR/pig_behavior.py" ]]; then
    echo "Starting pig behavior controller..."
    python3 -u "$DIR/pig_behavior.py" --world "$WORLD_FILE" &> "$LOG_DIR/pig_behavior.log" &
fi

echo "Starting ros_gz_bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
    /model/${MODEL}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    --ros-args \
    -r /model/${MODEL}/odometry:=/odom \
    -r /scan:=/scan \
    -r /cmd_vel:=/cmd_vel \
    &> "$LOG_DIR/ros_gz_bridge.log" &

if [[ "$HEADLESS" == "false" ]]; then
    ros2 run ros_gz_bridge parameter_bridge \
        /camera@sensor_msgs/msg/Image[gz.msgs.Image \
        /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
        /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
        /camera_right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
        --ros-args \
        -r /camera:=/camera/depth/image_raw \
        -r /camera_info:=/camera/depth/camera_info \
        -r /camera/color/image_raw:=/camera/color/image_raw \
        -r /camera_right/image_raw:=/camera_right/image_raw \
        &> "$LOG_DIR/ros_gz_bridge_camera.log" &
fi

sleep 1

if [[ -f "$URDF" && -x "$DIR/spawn_robot.sh" ]]; then
    echo "Spawning robot..."
    bash "$DIR/spawn_robot.sh" --file "$URDF" --model "$MODEL" \
        --pos 0 -14 0.05 --yaw 1.5708 \
        &> "$LOG_DIR/spawn.log" &
fi

if [[ "$HEADLESS" == "false" ]] && [[ -f "$DIR/aruco_face_detector.py" ]]; then
    echo "Starting ArUco face detector..."
    python3 -u "$DIR/aruco_face_detector.py" &> "$LOG_DIR/aruco_face_detector.log" &
fi

if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix robot_state_publisher >/dev/null 2>&1; then
    echo "Starting robot_state_publisher..."
    ros2 run robot_state_publisher robot_state_publisher --ros-args \
        -p robot_description:="$(tr '\n' ' ' < "$URDF")" \
        &> "$LOG_DIR/robot_state_publisher.log" &
fi

echo "Starting static TF publishers..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link \
    &> "$LOG_DIR/tf_odom_base_link.log" &
ros2 run tf2_ros static_transform_publisher 0.2 0 0.095 0.5 -0.5 0.5 -0.5 \
    base_link wheeltec_mini/base_link/depth_camera \
    &> "$LOG_DIR/tf_base_depth_camera.log" &
ros2 run tf2_ros static_transform_publisher 0 0 0.245 0 0 0 \
    base_link wheeltec_mini/base_link/lidar \
    &> "$LOG_DIR/tf_base_lidar.log" &
ros2 run tf2_ros static_transform_publisher 0 -0.15 0.15 -1.5708 0 0 \
    base_link right_camera_link \
    &> "$LOG_DIR/tf_base_right_camera.log" &

if [[ "$HEADLESS" == "false" ]]; then
    if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix depth_image_proc >/dev/null 2>&1; then
        echo "Starting depth_image_proc..."
        ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
            -r __ns:=/camera/depth -r image_rect:=image_raw -r points:=points \
            &> "$LOG_DIR/depth_image_proc.log" &
        if [[ -f "$DIR/pointcloud_qos_relay.py" ]]; then
            python3 "$DIR/pointcloud_qos_relay.py" &> "$LOG_DIR/pointcloud_qos_relay.log" &
        fi
    fi
    if command -v rviz2 >/dev/null 2>&1 && [[ -f "$RVIZ_CONFIG" ]]; then
        echo "Starting RViz..."
        rviz2 -d "$RVIZ_CONFIG" &> "$LOG_DIR/rviz.log" &
    fi
fi

echo "Done. Press Ctrl-C to exit and cleanup."
wait -n "$GZ_PID" || true
