#!/usr/bin/env bash
set -euo pipefail

# Clean launch helper for Gazebo + ros_gz_bridge + robot + TF + pointcloud + RViz
# Usage: ./launch_clean.sh [world_file]

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="${1:-$DIR/../worlds/pig_pen_8units(lv4).world}"
LOG_DIR="$DIR/../logs"

MODEL="wheeltec_mini"
URDF="$DIR/../turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf"
RVIZ_CONFIG="$DIR/../rviz/wheeltec.rviz"

mkdir -p "$LOG_DIR"

if [ -f "/opt/ros/humble/setup.bash" ]; then
  # Avoid 'set -u' causing unbound variable errors inside ROS setup
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

cleanup() {
  echo "Cleaning up..."
  pkill -f "ros_gz_bridge" || true
  pkill -f "gz sim" || true
  pkill -f robot_description_publisher || true
  pkill -f publish_robot_description.py || true
  pkill -f robot_state_publisher || true
  pkill -f static_transform_publisher || true
  pkill -f depth_image_proc || true
  pkill -f pointcloud_qos_relay.py || true
}
trap cleanup EXIT

echo "World: $WORLD_FILE"
echo "Logs: $LOG_DIR"

echo "Starting gz sim..."
gz sim "$WORLD_FILE" -r &> "$LOG_DIR/gz_sim.log" &
GZ_PID=$!
sleep 2

echo "Starting ros_gz_bridge with remaps..."
BRIDGE_LOG="$LOG_DIR/ros_gz_bridge.log"
# Kill any old bridge/processes that may be left running from previous runs
pkill -f ros_gz_bridge || true
pkill -f depth_image_proc || true
pkill -f robot_description_publisher || true
pkill -f publish_robot_description.py || true
pkill -f pointcloud_qos_relay.py || true
sleep 0.5

PARAMS=(
  "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
  "/camera@sensor_msgs/msg/Image[gz.msgs.Image"
  "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
  "/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist"
  "/model/${MODEL}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"
)

ros2 run ros_gz_bridge parameter_bridge "${PARAMS[@]}" --ros-args \
  -r /model/${MODEL}/odometry:=/odom \
  -r /scan:=/scan \
  -r /camera:=/camera/depth/image_raw \
  -r /camera_info:=/camera/depth/camera_info \
  -r /cmd_vel:=/cmd_vel &> "$BRIDGE_LOG" &
BRIDGE_PID=$!

sleep 1

if [ -f "$URDF" ]; then
  echo "Spawning robot: $URDF"
  if [ -x "$DIR/spawn_robot.sh" ]; then
    bash "$DIR/spawn_robot.sh" --file "$URDF" --model "$MODEL" --pos -7 0 0.01 --yaw 0 &> "$LOG_DIR/spawn.log" &
    SPAWN_PID=$!
    sleep 1
  else
    echo "Warning: spawn script not found or not executable: $DIR/spawn_robot.sh"
  fi
else
  echo "Warning: URDF not found: $URDF"
fi

# Start robot_description publisher + robot_state_publisher
if [ -f "$URDF" ]; then
  if [ -x "$DIR/publish_robot_description.py" ]; then
    echo "Starting robot_description publisher..."
    "$DIR/publish_robot_description.py" "$URDF" &> "$LOG_DIR/robot_description.log" &
    ROBOT_DESC_PID=$!
    sleep 0.5
  fi

  if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix robot_state_publisher >/dev/null 2>&1; then
    ROBOT_STATE_PARAMS="$LOG_DIR/robot_state_publisher.params.yaml"
    {
      echo "robot_state_publisher:"
      echo "  ros__parameters:"
      echo "    robot_description: |"
      sed 's/^/      /' "$URDF"
    } > "$ROBOT_STATE_PARAMS"
    echo "Starting robot_state_publisher..."
    ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$ROBOT_STATE_PARAMS" &> "$LOG_DIR/robot_state_publisher.log" &
    RSP_PID=$!
    sleep 0.5

    echo "Starting static TF publishers (odom -> base_link, base_link -> sensors)..."
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &> "$LOG_DIR/tf_odom_base_link.log" &
    # Match sensor frames used by Gazebo topics:
    # /camera/* frame_id: wheeltec_mini/base_link/depth_camera
    # /scan frame_id: wheeltec_mini/base_link/lidar
    # Camera frame appears rotated relative to lidar/base axes.
    # Quaternion below maps camera axes to the observed lidar/base convention.
    ros2 run tf2_ros static_transform_publisher 0.2 0 0.095 0.5 -0.5 0.5 -0.5 base_link wheeltec_mini/base_link/depth_camera &> "$LOG_DIR/tf_base_depth_camera.log" &
    ros2 run tf2_ros static_transform_publisher 0 0 0.245 0 0 0 base_link wheeltec_mini/base_link/lidar &> "$LOG_DIR/tf_base_lidar.log" &
  fi
fi

# Start depth_image_proc pointcloud if installed
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix depth_image_proc >/dev/null 2>&1; then
  echo "Starting depth_image_proc point_cloud_xyz_node..."
  ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
    -r __ns:=/camera/depth \
    -r image_rect:=image_raw \
    -r points:=points &> "$LOG_DIR/depth_image_proc.log" &
  DEPTH_PROC_PID=$!

  # Relay BEST_EFFORT pointcloud to a RELIABLE topic for RViz compatibility.
  if [ -f "$DIR/pointcloud_qos_relay.py" ]; then
    echo "Starting pointcloud QoS relay..."
    python3 "$DIR/pointcloud_qos_relay.py" &> "$LOG_DIR/pointcloud_qos_relay.log" &
    PC_RELAY_PID=$!
  fi
else
  echo "Note: depth_image_proc not installed; skip pointcloud generation (install ros-humble-depth-image-proc)." >&2
fi

sleep 1

echo "Launched processes (PIDs): gz:$GZ_PID bridge:$BRIDGE_PID spawn:${SPAWN_PID:-} rsp:${RSP_PID:-} depth:${DEPTH_PROC_PID:-} relay:${PC_RELAY_PID:-}"
echo "Logs in: $LOG_DIR"

if [ -f "$RVIZ_CONFIG" ]; then
  echo "Starting RViz with config: $RVIZ_CONFIG"
  rviz2 -d "$RVIZ_CONFIG" &
fi

echo "Done. Press Ctrl-C to exit and cleanup."
wait
