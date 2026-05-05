#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$DIR/../logs"

echo "== Depth / PointCloud diagnostic =="

echo "\n-- ROS2 topics (filtered) --"
ros2 topic list 2>/dev/null | grep -E "camera|depth|points|scan|odom" || true

echo "\n-- Topic hz (image) --"
if ros2 topic list 2>/dev/null | grep -q '/camera/depth/image_raw'; then
  ros2 topic hz /camera/depth/image_raw --window 2 || true
else
  echo "/camera/depth/image_raw not present"
fi

echo "\n-- Topic hz (points) --"
if ros2 topic list 2>/dev/null | grep -q '/camera/depth/points'; then
  ros2 topic hz /camera/depth/points --window 2 || true
else
  echo "/camera/depth/points not present"
fi

echo "\n-- Sample messages (headers) --"
for t in /camera/depth/camera_info /camera/depth/image_raw /camera/depth/points; do
  echo "\n>>> $t <<<"
  ros2 topic echo --once "$t" 2>/dev/null || echo "(no message on $t)"
done

echo "\n-- Processes --"
ps aux | egrep 'ros_gz_bridge|gz sim|depth_image_proc|robot_state_publisher' || true

echo "\n-- Check logs (last 200 lines if exist) --"
for f in "$LOG_DIR/depth_image_proc.log" "$LOG_DIR/ros_gz_bridge.log" "$LOG_DIR/gz_sim.log"; do
  echo "\n--- $f ---"
  if [ -f "$f" ]; then
    tail -n 200 "$f" || true
  else
    echo "(no log file)"
  fi
done

echo "\n-- TF check --"
ros2 run tf2_ros tf2_echo odom wheeltec_mini/base_link/depth_camera 2>/dev/null || echo "(tf lookup failed)"

cat <<'EOF'

Diagnostic finished. Suggestions:
- If /camera/depth/image_raw exists but /camera/depth/points doesn't: ensure depth_image_proc is running and subscribing to the correct namespaces.
- If camera_info timestamps mismatch: check ros_gz_bridge remap and remove any custom camera_info publisher.
- If rviz plugins fail: make sure ROS environment is sourced before launching rviz (source /opt/ros/humble/setup.bash).
EOF
