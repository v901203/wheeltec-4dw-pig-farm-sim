#!/usr/bin/env bash
set -euo pipefail

# Spawn a robot model into Gazebo (ROS2 + ros_gz_sim create).
# Supports spawning from a file (recommended) or from topic `robot_description`.
# Usage examples:
#   ./scripts/spawn_robot.sh --model mybot --pos 0 0 0 --yaw 0
#   ./scripts/spawn_robot.sh --file path/to/robot.urdf --model mybot --pos 0 0 0 --yaw 0

MODEL="robot"
MODE="topic" # or 'file'
FILE=""
X=0
Y=0
Z=0
YAW=0

usage() {
  cat <<EOF
Usage: $0 [--file PATH] [--model NAME] [--pos X Y Z] [--yaw RADIANS]

If --file is given, the URDF file will be used. Otherwise the script uses
the topic 'robot_description'.
EOF
}

if [ "$#" -eq 0 ]; then
  usage
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    --file) MODE=file; FILE="$2"; shift 2;;
    --model) MODEL="$2"; shift 2;;
    --pos) X="$2"; Y="$3"; Z="$4"; shift 4;;
    --yaw) YAW="$2"; shift 2;;
    --help) usage; exit 0;;
    *) echo "Unknown arg: $1"; usage; exit 1;;
  esac
done

if [ -f "/opt/ros/humble/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Error: ros2 command not found. Install ROS2 or source your ROS2 setup first."
  exit 1
fi

if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "Error: ros_gz_sim package not found."
  echo "Install it (Ubuntu): sudo apt install ros-humble-ros-gz-sim"
  exit 1
fi

if [ "$MODE" = "file" ]; then
  echo "Spawning model from file: $FILE (model name: $MODEL)"
  exec ros2 run ros_gz_sim create -name "$MODEL" -x "$X" -y "$Y" -z "$Z" -Y "$YAW" -file "$FILE"
else
  echo "Spawning model from topic 'robot_description' (model name: $MODEL)"
  exec ros2 run ros_gz_sim create -name "$MODEL" -x "$X" -y "$Y" -z "$Z" -Y "$YAW" -topic robot_description
fi
