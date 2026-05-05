#!/usr/bin/env bash
set -euo pipefail

# Launch Gazebo world + ros_gz_bridge parameter_bridge + spawn robot
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="$DIR/../worlds/pig_pen_8units(lv4).world"
LOG_DIR="$DIR/../logs"

MODEL="wheeltec_mini"
DEFAULT_URDF="$DIR/../turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf"

mkdir -p "$LOG_DIR"

if [ -f "/opt/ros/humble/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

echo "Starting Gazebo world: $WORLD_FILE (logs: $LOG_DIR/gz_sim.log)"
gz sim "$WORLD_FILE" -r &> "$LOG_DIR/gz_sim.log" &
GZ_PID=$!

sleep 3

# Start ros_gz_bridge parameter_bridge with typical RL topics
BRIDGE_LOG="$LOG_DIR/ros_gz_bridge.log"
echo "Starting ros_gz_bridge (logs: $BRIDGE_LOG)"

# Topics mapped here (adjust names if your model uses different topic paths)
PARAMS=(
  "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
  "/camera@sensor_msgs/msg/Image[gz.msgs.Image"
  "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
  "/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist"
  "/model/${MODEL}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"
)

# Add remappings so ROS2 sees canonical topics (useful for RL/env standardization)
ros2 run ros_gz_bridge parameter_bridge "${PARAMS[@]}" --ros-args \
  -r /model/${MODEL}/odometry:=/odom \
  -r /scan:=/scan \
  -r /camera:=/camera/depth/image_raw \
  -r /camera_info:=/camera/depth/camera_info \
  -r /cmd_vel:=/cmd_vel &> "$BRIDGE_LOG" &
BRIDGE_PID=$!

sleep 1

# Spawn robot
SPAWN_SCRIPT="$DIR/spawn_robot.sh"
if [ -x "$SPAWN_SCRIPT" ] || [ -f "$SPAWN_SCRIPT" ]; then
  if [ -f "$DEFAULT_URDF" ]; then
    echo "Spawning robot from $DEFAULT_URDF (model: $MODEL)"
    bash "$SPAWN_SCRIPT" --file "$DEFAULT_URDF" --model "$MODEL" --pos -7 0 0.01 --yaw 0 &> "$LOG_DIR/spawn.log" &
    SPAWN_PID=$!
  else
    echo "Default URDF not found ($DEFAULT_URDF)."
  fi
else
  echo "Spawn script not found: $SPAWN_SCRIPT"
fi

# Try to run depth_image_proc point cloud converter (if available)
if command -v ros2 >/dev/null 2>&1; then
  if [ -f "$DEFAULT_URDF" ] && ros2 pkg prefix robot_state_publisher >/dev/null 2>&1; then
    ROBOT_STATE_PARAMS="$LOG_DIR/robot_state_publisher.params.yaml"
    {
      echo "robot_state_publisher:"
      echo "  ros__parameters:"
      echo "    robot_description: |"
      sed 's/^/      /' "$DEFAULT_URDF"
    } > "$ROBOT_STATE_PARAMS"
    echo "Starting robot_state_publisher (logs: $LOG_DIR/robot_state_publisher.log)"
    ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$ROBOT_STATE_PARAMS" &> "$LOG_DIR/robot_state_publisher.log" &
    RSP_PID=$!

    echo "Starting static TF odom -> base_link (logs: $LOG_DIR/tf_odom_base_link.log)"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &> "$LOG_DIR/tf_odom_base_link.log" &
    TF_BASE_PID=$!

    echo "Starting static TF base_link -> wheeltec_mini/base_link/depth_camera (logs: $LOG_DIR/tf_base_depth_camera.log)"
    ros2 run tf2_ros static_transform_publisher 0.2 0 0.095 0.5 -0.5 0.5 -0.5 base_link wheeltec_mini/base_link/depth_camera &> "$LOG_DIR/tf_base_depth_camera.log" &
    TF_DEPTH_PID=$!

    echo "Starting static TF base_link -> wheeltec_mini/base_link/lidar (logs: $LOG_DIR/tf_base_lidar.log)"
    ros2 run tf2_ros static_transform_publisher 0 0 0.245 0 0 0 base_link wheeltec_mini/base_link/lidar &> "$LOG_DIR/tf_base_lidar.log" &
    TF_LIDAR_PID=$!
  fi

  if [ -x "$DIR/publish_robot_description.py" ] && [ -f "$DEFAULT_URDF" ]; then
    echo "Starting robot_description publisher (logs: $LOG_DIR/robot_description.log)"
    "$DIR/publish_robot_description.py" "$DEFAULT_URDF" &> "$LOG_DIR/robot_description.log" &
    ROBOT_DESC_PID=$!
  fi

  if ros2 pkg prefix depth_image_proc >/dev/null 2>&1; then
    echo "Starting depth_image_proc point_cloud_xyz_node -> /camera/depth/points (logs: $LOG_DIR/depth_image_proc.log)"
    ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
      -r __ns:=/camera/depth \
      -r image_rect:=image_raw \
      -r points:=points &> "$LOG_DIR/depth_image_proc.log" &
    DEPTH_PROC_PID=$!
  else
    echo "depth_image_proc not found; to enable pointcloud generation install ros-humble-depth-image-proc or equivalent" >&2
  fi
fi

echo "Launched processes:"
echo " - gz sim PID $GZ_PID (log: $LOG_DIR/gz_sim.log)"
echo " - ros_gz_bridge PID $BRIDGE_PID (log: $BRIDGE_LOG)"
if [ -n "${SPAWN_PID:-}" ]; then echo " - spawn PID $SPAWN_PID (log: $LOG_DIR/spawn.log)"; fi
if [ -n "${RSP_PID:-}" ]; then echo " - robot_state_publisher PID $RSP_PID (log: $LOG_DIR/robot_state_publisher.log)"; fi
if [ -n "${TF_BASE_PID:-}" ]; then echo " - static TF base PID $TF_BASE_PID (log: $LOG_DIR/tf_odom_base_link.log)"; fi
if [ -n "${TF_DEPTH_PID:-}" ]; then echo " - static TF depth PID $TF_DEPTH_PID (log: $LOG_DIR/tf_base_depth_camera.log)"; fi
if [ -n "${TF_LIDAR_PID:-}" ]; then echo " - static TF lidar PID $TF_LIDAR_PID (log: $LOG_DIR/tf_base_lidar.log)"; fi

echo "All logs: $LOG_DIR"
echo "When finished, kill these PIDs if needed: $GZ_PID ${BRIDGE_PID:-} ${SPAWN_PID:-} ${RSP_PID:-} ${TF_BASE_PID:-} ${TF_DEPTH_PID:-} ${TF_LIDAR_PID:-}"

exit 0
