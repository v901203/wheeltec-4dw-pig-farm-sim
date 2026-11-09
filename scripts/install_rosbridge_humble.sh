#!/usr/bin/env bash
# Run on the Xavier HOST; compile an isolated overlay in existing robot-dev.
set -euo pipefail
container="${ROBOT_CONTAINER:-robot-dev}"
if [[ "${1:-}" == --help ]]; then
    echo '在 Xavier 主機執行：bash install_rosbridge_humble.sh；不啟動服務或車子。'
    exit
fi
[[ $# == 0 ]] || { echo '不支援的參數' >&2; exit 2; }
[[ "$(docker inspect -f '{{.State.Running}}' "$container")" == true ]] || {
    echo "容器 $container 未運行，請先確認。" >&2; exit 1;
}
docker exec -i -u root "$container" bash --noprofile --norc -s <<'CONTAINER'
set -eo pipefail
# Source only the custom Humble underlay, never the host/Foxy workspace.
ros_setup=
for candidate in /opt/ros/humble/install/setup.bash /opt/ros/humble/setup.bash; do
    if [ -r "$candidate" ]; then ros_setup="$candidate"; break; fi
done
[ -n "$ros_setup" ] || { echo '容器找不到 Humble 環境' >&2; exit 1; }
source "$ros_setup"
[ "${ROS_DISTRO:-}" = humble ] || { echo '不是 Humble，停止安裝。' >&2; exit 1; }
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
command -v colcon >/dev/null || { echo '缺少 colcon，請貼回此訊息。' >&2; exit 1; }
python3 - <<'PY'
import sys
import rclpy
from rclpy.utilities import get_rmw_implementation_identifier
print('Python:', sys.version)
print('rclpy:', rclpy.__file__)
assert get_rmw_implementation_identifier() == 'rmw_fastrtps_cpp'
PY
# Only distro-neutral Focal dependencies; no ros-humble binary apt packages.
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends git ca-certificates cmake build-essential \
    python3-tornado python3-twisted python3-bson python3-pil
source_dir=/workspace/src_humble_rosbridge/rosbridge_suite
revision=51fadc5457eadf1a55fe0ef8677cd88dde66da5b
mkdir -p /workspace/src_humble_rosbridge
if [ ! -e "$source_dir" ]; then
    git clone --depth 1 --branch 1.3.2 \
        https://github.com/RobotWebTools/rosbridge_suite.git "$source_dir"
fi
[ -d "$source_dir/.git" ] || { echo '來源目錄已存在但不是 git，未覆寫。' >&2; exit 1; }
[ "$(git -C "$source_dir" rev-parse HEAD)" = "$revision" ] || {
    echo '來源版本不是指定的 1.3.2，未重設或覆寫，請貼回訊息。' >&2; exit 1;
}
[ -z "$(git -C "$source_dir" status --porcelain)" ] || {
    echo '原始碼已有修改，停止以免混用。' >&2; exit 1;
}
cd /workspace
colcon --log-base log_humble_rosbridge build \
    --base-paths "$source_dir" \
    --packages-up-to rosbridge_server rosapi \
    --build-base build_humble_rosbridge \
    --install-base install_humble_rosbridge \
    --executor sequential \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source /workspace/install_humble_rosbridge/local_setup.bash
ros2 pkg prefix rosbridge_server
ros2 pkg executables rosbridge_server
python3 - <<'PY'
import tornado, bson
from PIL import Image
import rosbridge_server
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_msgs.msg import ConnectedClient
from rosapi_msgs.srv import Topics
print('rosbridge 模組與產生的 ROS 介面載入成功。')
PY
echo 'rosbridge 已建置到 /workspace/install_humble_rosbridge；尚未啟動 9090 或任何車輛服務。'
CONTAINER
