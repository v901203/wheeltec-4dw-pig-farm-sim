#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="${1:-$DIR/../worlds/pig_pen_8units(lv4).world}"
LOG_DIR="$DIR/../logs"
MODEL="wheeltec_mini"
URDF="$DIR/../turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf"
RVIZ_CONFIG="$DIR/../rviz/wheeltec.rviz"

mkdir -p "$LOG_DIR"

if [[ -f /opt/ros/humble/setup.bash ]]; then
	set +u
	# shellcheck disable=SC1091
	source /opt/ros/humble/setup.bash
	set -u
fi

if [[ -f "$DIR/../install/setup.bash" ]]; then
	set +u
	# shellcheck disable=SC1091
	source "$DIR/../install/setup.bash"
	set -u
fi

cleanup() {
	echo "Cleaning up..."
	pkill -f "ros_gz_bridge" || true
	pkill -f "gz sim" || true
	pkill -f "robot_state_publisher" || true
	pkill -f "depth_image_proc" || true
	pkill -f "pointcloud_qos_relay.py" || true
	pkill -f "publish_robot_description.py" || true
	pkill -f "spawn_robot.sh" || true
}
trap cleanup EXIT

echo "World: $WORLD_FILE"
echo "Logs: $LOG_DIR"

echo "Starting gz sim..."
gz sim "$WORLD_FILE" -r &> "$LOG_DIR/gz_sim.log" &
GZ_PID=$!
sleep 3

echo "Starting ros_gz_bridge with remaps..."
ros2 run ros_gz_bridge parameter_bridge \
	/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
	/camera@sensor_msgs/msg/Image[gz.msgs.Image \
	/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
	/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
	/camera_right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
	/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist \
	/model/${MODEL}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
	--ros-args \
	-r /model/${MODEL}/odometry:=/odom \
	-r /scan:=/scan \
	-r /camera:=/camera/depth/image_raw \
	-r /camera_info:=/camera/depth/camera_info \
	-r /camera/color/image_raw:=/camera/color/image_raw \
	-r /camera_right/image_raw:=/camera_right/image_raw \
	-r /cmd_vel:=/cmd_vel \
	&> "$LOG_DIR/ros_gz_bridge.log" &

sleep 1

if [[ -f "$URDF" && -x "$DIR/spawn_robot.sh" ]]; then
	echo "Spawning robot: $URDF"
	bash "$DIR/spawn_robot.sh" --file "$URDF" --model "$MODEL" --pos -7 0 0.01 --yaw 0 \
		&> "$LOG_DIR/spawn.log" &
fi

if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix robot_state_publisher >/dev/null 2>&1; then
	echo "Starting robot_state_publisher..."
	ros2 run robot_state_publisher robot_state_publisher --ros-args \
		-p robot_description:="$(tr '\n' ' ' < "$URDF")" \
		&> "$LOG_DIR/robot_state_publisher.log" &
fi

echo "Starting static TF publishers (odom -> base_link, base_link -> sensors)..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &> "$LOG_DIR/tf_odom_base_link.log" &
ros2 run tf2_ros static_transform_publisher 0.2 0 0.095 0.5 -0.5 0.5 -0.5 base_link wheeltec_mini/base_link/depth_camera &> "$LOG_DIR/tf_base_depth_camera.log" &
ros2 run tf2_ros static_transform_publisher 0 0 0.245 0 0 0 base_link wheeltec_mini/base_link/lidar &> "$LOG_DIR/tf_base_lidar.log" &

# 關鍵：必須確保右側相機的 TF 也正確宣告，否則會影響整個感測器樹的連線
ros2 run tf2_ros static_transform_publisher 0 -0.15 0.15 -1.5708 0 0 base_link right_camera_link &> "$LOG_DIR/tf_base_right_camera.log" &

if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix depth_image_proc >/dev/null 2>&1; then
	echo "Starting depth_image_proc point_cloud_xyz_node..."
	ros2 run depth_image_proc point_cloud_xyz_node --ros-args -r __ns:=/camera/depth -r image_rect:=image_raw -r points:=points &> "$LOG_DIR/depth_image_proc.log" &
	DEPTH_PROC_PID=$!

	if [[ -f "$DIR/pointcloud_qos_relay.py" ]]; then
		echo "Starting pointcloud QoS relay..."
		python3 "$DIR/pointcloud_qos_relay.py" &> "$LOG_DIR/pointcloud_qos_relay.log" &
		PC_RELAY_PID=$!
	fi
else
	echo "Note: depth_image_proc not installed; skip pointcloud generation (install ros-humble-depth-image-proc)." >&2
fi

echo "Starting RViz with config: $RVIZ_CONFIG"
if command -v rviz2 >/dev/null 2>&1 && [[ -f "$RVIZ_CONFIG" ]]; then
	rviz2 -d "$RVIZ_CONFIG" &> "$LOG_DIR/rviz.log" &
fi

echo "Done. Press Ctrl-C to exit and cleanup."
wait -n "$GZ_PID" || true