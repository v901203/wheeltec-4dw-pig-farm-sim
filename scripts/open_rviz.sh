#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$DIR/../logs"
mkdir -p "$LOG_DIR"

if [ -f "/opt/ros/humble/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

echo "Checking required topics (/scan, /odom, /camera/depth/image_raw)..."
sleep 0.5
TOPICS=$(ros2 topic list || true)

if echo "$TOPICS" | grep -q "/scan"; then
  echo " - /scan: OK"
else
  echo " - /scan: MISSING" >&2
fi

if echo "$TOPICS" | grep -q "/odom"; then
  echo " - /odom: OK"
else
  echo " - /odom: MISSING" >&2
fi

if echo "$TOPICS" | grep -q "/camera/depth/image_raw"; then
  echo " - /camera/depth/image_raw: OK"
else
  echo " - /camera/depth/image_raw: MISSING (optional)"
fi

echo
echo "Launching RViz2... (logs -> $LOG_DIR/rviz2.log)"
rviz2 &> "$LOG_DIR/rviz2.log" &
RVIZ_PID=$!

cat <<EOF
RViz started (PID $RVIZ_PID).

Quick setup in RViz:
- Global Options -> Fixed Frame: set to 'odom' (or 'base_link' if odom missing)
- Add -> By topic -> choose '/scan' (LaserScan)
- Add -> By topic -> choose '/camera/depth/image_raw' (Image) or PointCloud if available
- Add -> RobotModel and set Description Source: 'robot_description' or load URDF manually

Logs: $LOG_DIR/rviz2.log
EOF

exit 0
