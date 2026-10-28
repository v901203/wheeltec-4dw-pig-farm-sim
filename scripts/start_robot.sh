#!/usr/bin/env bash
# Run on the Xavier HOST. Never run the legacy mecanum controller.
set -euo pipefail

ACTION="${1:-status}"
case "$ACTION" in
    setup-network|start|start-base|test-safe|start-patrol|start-modes|status|stop) ;;
    *) echo "Usage: bash $0 {setup-network|start|start-base|test-safe|start-patrol|start-modes|status|stop}" >&2; exit 2 ;;
esac

# Configure only the host interface connected to the radar. Never flush existing
# addresses, replace a default route, or write the radar's internal settings.
setup_lidar_network() {
    local iface="${RADAR_INTERFACE:-eth0}"
    local host_ip="${RADAR_HOST_IP:-192.168.1.100/24}"
    local compat_ip="${RADAR_COMPAT_IP-192.168.1.102/24}"
    local device_ip="${RADAR_DEVICE_IP:-192.168.1.200}"
    local wait_s="${RADAR_LINK_WAIT_SEC:-30}"
    local deadline link address route
    command -v ip >/dev/null || { echo "缺少 ip 指令，請安裝 iproute2。" >&2; return 1; }
    [[ "$iface" =~ ^[a-zA-Z0-9_.:-]+$ && "$iface" != lo ]] || {
        echo "無效的雷達網卡名稱：$iface" >&2; return 1;
    }
    [[ "$wait_s" =~ ^[0-9]{1,3}$ ]] && ((10#$wait_s <= 300)) || {
        echo "RADAR_LINK_WAIT_SEC 必須為 0～300 秒。" >&2; return 1;
    }
    # Validate IPv4 input before requesting privileges or changing the interface.
    valid_ipv4() {
        local octet
        local -a octets
        [[ "$1" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]] || return 1
        IFS=. read -r -a octets <<< "$1"
        for octet in "${octets[@]}"; do ((10#$octet <= 255)) || return 1; done
    }
    valid_cidr() {
        [[ "$1" == */* ]] && valid_ipv4 "${1%/*}" &&
            [[ "${1#*/}" =~ ^[0-9]{1,2}$ ]] && ((10#${1#*/} <= 32))
    }
    valid_cidr "$host_ip" && valid_ipv4 "$device_ip" &&
        { [ -z "$compat_ip" ] || valid_cidr "$compat_ip"; } || {
        echo "雷達 IPv4 設定格式不正確。" >&2; return 1;
    }
    network_ip() {
        if (( EUID == 0 )); then
            ip "$@"
        elif [ -t 0 ]; then
            sudo ip "$@"
        else
            sudo -n ip "$@" || {
                echo "無權設定雷達網路；systemd 請加入 root 執行的 setup-network 前置步驟，詳見 README。" >&2
                return 1
            }
        fi
    }
    deadline=$((SECONDS + 10#$wait_s))
    until ip link show dev "$iface" >/dev/null 2>&1; do
        if (( SECONDS >= deadline )); then
            echo "找不到雷達網卡 $iface。" >&2; return 1
        fi
        sleep 1
    done
    link=$(ip -o link show dev "$iface")
    if ! grep -Eq '<([^>]*,)?UP(,|>)' <<< "$link"; then
        network_ip link set dev "$iface" up || return 1
    fi
    until ip -o link show dev "$iface" | grep -q 'LOWER_UP'; do
        if (( SECONDS >= deadline )); then
            echo "雷達網卡 $iface 無實體連線；請檢查供電與網線。" >&2; return 1
        fi
        sleep 1
    done
    for address in "$host_ip" "$compat_ip"; do
        [ -n "$address" ] || continue
        if ! ip -o -4 addr show dev "$iface" | awk '{print $4}' | grep -Fxq "$address"; then
            network_ip addr add "$address" dev "$iface" || return 1
        fi
    done
    route=$(ip -4 route get "$device_ip") || return 1
    if [[ " $route " != *" dev $iface "* ]]; then
        echo "雷達 $device_ip 的路由沒有走 $iface，拒絕啟動：$route" >&2
        return 1
    fi
    echo "雷達網路已就緒：$iface，主機 $host_ip，相容位址 ${compat_ip:-無}，雷達 $device_ip"
}

if [ "$ACTION" = setup-network ]; then
    setup_lidar_network
    exit
fi

CONTAINER="${ROBOT_CONTAINER:-robot-dev}"
if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "找不到容器 $CONTAINER；請先準備 Humble、workspace 掛載、雷達網路及串口。" >&2
    exit 1
fi
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != true ]; then
    echo "容器 $CONTAINER 未執行；請確認其啟動設定後，手動 docker start。" >&2
    exit 1
fi
case "$ACTION" in
    start|test-safe|start-patrol|start-modes)
        if [ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$CONTAINER")" != host ]; then
            echo "雷達啟動需要容器使用 host 網路；請檢查 robot-dev 建立設定。" >&2
            exit 1
        fi
        setup_lidar_network
        ;;
esac
script_path="${BASH_SOURCE[0]}"
if [ "$ACTION" = start-patrol ] || [ "$ACTION" = start-base ] || [ "$ACTION" = start-modes ]; then
    # docker exec without a TTY does not reliably relay terminal signals to the
    # container process. Issue an explicit, separate stop request on interrupt.
    trap 'trap - INT TERM; bash "$script_path" stop || true; exit 130' INT
    trap 'trap - INT TERM; bash "$script_path" stop || true; exit 143' TERM
fi

# No automatic apt install, privileged container creation,
# chassis motion during status, or broad pkill/docker stop operations.
docker exec -i "$CONTAINER" bash -s -- "$ACTION" \
    "${ROBOT_WS_CONTAINER:-/workspace}" "${ROBOT_PORT:-/dev/ttyACM0}" \
    "${ROBOT_SCAN_TOPIC:-/x10/scan}" "${ROBOT_ROS_SETUP:-}" \
    "${ROBOT_PROJECT_CONTAINER:-/workspace/pig_patrol}" <<'CONTAINER_SCRIPT'
set -euo pipefail
action="$1"; workspace="$2"; port="$3"; scan_topic="$4"; ros_setup="$5"
project="${6:-$workspace}"
fsm="$project/rl_pig_pen/fsm_lidar_patrol.py"
if [ "$action" = start-modes ]; then fsm="$project/rl_pig_pen/control_modes.py"; fi
config="$project/rl_pig_pen/patrol_config.json"
bridge_config="$project/wheeltec_uart_bridge/config/uart_bridge.yaml"
# Compatibility for the original workspace layout when no project is supplied.
if [ "$#" -lt 6 ]; then
    bridge_config="$workspace/src/wheeltec_uart_bridge/config/uart_bridge.yaml"
fi
runtime=/tmp/pig_pen_start_robot
mkdir -p "$runtime"

if [ -z "$ros_setup" ]; then
    for candidate in /opt/ros/humble/install/setup.bash /opt/ros/humble/setup.bash; do
        if [ -r "$candidate" ]; then ros_setup="$candidate"; break; fi
    done
fi
if [ ! -r "$ros_setup" ] || [ ! -r "$workspace/install/local_setup.bash" ]; then
    echo "缺少 ROS 環境或 $workspace/install/local_setup.bash；請先在容器內編譯。" >&2
    exit 1
fi
if [ "$action" != stop ] && [ "$action" != status ] && [ "$action" != start-base ]; then
    [ -r "$fsm" ] && [ -r "$config" ] || {
        echo "找不到桌面豬舍專案：$project/rl_pig_pen；請確認 Docker 有掛載 /home/yu/Desktop/pig_patrol。" >&2
        exit 1
    }
fi
set +u
source "$ros_setup"
source "$workspace/install/local_setup.bash"
set -u

# Save Linux process start time with the PID to avoid signalling a recycled PID.
process_start() {
    local stat
    [ -r "/proc/$1/stat" ] || return 1
    stat=$(cat "/proc/$1/stat")
    stat=${stat##*) }
    awk '{print $20}' <<< "$stat"
}
record_pid() {
    local started
    started=$(process_start "$2") || return 1
    printf '%s %s\n' "$2" "$started" > "$runtime/$1.pid"
}
owned_pid() {
    local pid started current
    [ -r "$runtime/$1.pid" ] || return 1
    read -r pid started < "$runtime/$1.pid" || return 1
    [[ "$pid" =~ ^[0-9]+$ && "$started" =~ ^[0-9]+$ ]] || return 1
    current=$(process_start "$pid") || return 1
    [ "$current" = "$started" ] && kill -0 "$pid" 2>/dev/null || return 1
    # A zombie cannot send commands and should not hold up shutdown.
    [ "$(awk '{print $3}' "/proc/$pid/stat")" != Z ] || return 1
    printf '%s\n' "$pid"
}
stop_owned() {
    local name="$1" pid
    if pid=$(owned_pid "$name"); then
        kill -TERM "$pid" 2>/dev/null || true
        for _ in {1..30}; do
            if ! owned_pid "$name" >/dev/null; then break; fi
            sleep .1
        done
        if pid=$(owned_pid "$name"); then
            echo "$name 未正常結束，終止本腳本記錄的 PID $pid；仍需確認車子停止。" >&2
            kill -KILL "$pid" 2>/dev/null || true
        fi
    fi
}
send_zero() {
    # Best effort only; neither receipt nor physical stopping is acknowledged.
    timeout --signal=INT --kill-after=1s 3s ros2 topic pub -r 10 \
        /cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
        >/dev/null 2>&1 9>&- || true
}
stop_control() {
    stop_owned fsm
    if owned_pid bridge >/dev/null; then
        send_zero
        stop_owned bridge
    fi
    echo "已停止本腳本管理的控制程序並嘗試送零速度；請現場確認停穩。雷達與容器保留。"
}
if [ "$action" = stop ]; then
    touch "$runtime/stop_requested"
    stop_control
    exit 0
fi
if [ "$action" = status ]; then
    echo "FSM: $fsm"
    echo "巡邏設定: $config"
    echo "UART 設定: $bridge_config；串口: $port；雷達: $scan_topic"
    for name in lidar bridge fsm; do
        if pid=$(owned_pid "$name"); then echo "$name PID=$pid"; else echo "$name: 無本腳本管理的執行程序"; fi
    done
    if [ -r "$runtime/latest_log_dir" ]; then cat "$runtime/latest_log_dir"; fi
    ros2 node list 9>&-
    ros2 topic info /cmd_vel 9>&- || true
    ros2 param get /wheeltec_uart_bridge max_linear_x 9>&- || true
    ros2 param get /wheeltec_uart_bridge max_angular_z 9>&- || true
    exit 0
fi

# Hold a container-wide lock throughout a managed run. stop remains callable.
# ROS CLI can create a persistent daemon. Close fd 9 for every ROS invocation,
# including timeout-wrapped commands, so that daemon cannot retain this lock.
exec 9>"$runtime/control.lock"
flock -n 9 || { echo "已有本腳本的啟動／控制工作執行中。" >&2; exit 1; }
rm -f "$runtime/stop_requested"
check_cancelled() {
    if [ -e "$runtime/stop_requested" ]; then
        echo "已收到 stop，取消啟動。" >&2
        return 1
    fi
}
log_root="$workspace/logs/robot"
mkdir -p "$log_root"
log_dir=$(mktemp -d "$log_root/$(date +%Y%m%d_%H%M%S)-XXXXXX")
printf '%s\n' "$log_dir" > "$runtime/latest_log_dir"
echo "日誌目錄：$log_dir"

scan_ready() {
    # A discovered topic alone is not proof that scans are arriving.
    timeout --signal=INT --kill-after=1s 5s ros2 topic echo "$scan_topic" \
        sensor_msgs/msg/LaserScan --once --qos-reliability best_effort \
        >/dev/null 2>&1 9>&-
}
start_lidar_motor() {
    # M10P may ignore a one-shot command while its UDP control socket is
    # recovering; repeat the command briefly so the hardware receives it.
    timeout --signal=INT --kill-after=1s 4s ros2 topic pub -r 10 \
        /x10/motor_control std_msgs/msg/Int8 '{data: 1}' \
        >/dev/null 2>&1 9>&- || true
}
ensure_lidar() {
    if scan_ready; then return 0; fi
    # A running driver may have left the lidar motor stopped after a restart.
    if owned_pid lidar >/dev/null ||
       pgrep -f '[l]slidar_driver_node|[l]slidar_m10p_launch.py' >/dev/null; then
        echo "雷達 driver 已存在但沒有掃描；嘗試啟動 M10P 馬達。"
        start_lidar_motor
        for _ in {1..5}; do
            if scan_ready; then return 0; fi
        done
        echo "雷達仍有發布者但收不到 LaserScan；請檢查馬達旋轉、UDP 2368/2369 與驅動日誌。" >&2
        return 1
    fi
    if ros2 topic info "$scan_topic" 2>/dev/null 9>&- | grep -Eq 'Publisher count: [1-9]'; then
        echo "已有雷達程序但掃描未就緒；不重複啟動。" >&2
        return 1
    fi
    ros2 launch lslidar_driver lslidar_m10p_launch.py >"$log_dir/lidar.log" 2>&1 9>&- &
    record_pid lidar "$!"
    sleep 1
    start_lidar_motor
    for _ in {1..5}; do if scan_ready; then return 0; fi; done
    echo "收不到 $scan_topic，詳見 $log_dir/lidar.log；雷達程序保留供排查。" >&2
    return 1
}
if [ "$action" = start ]; then
    ensure_lidar
    echo "雷達已收到掃描；未啟動巡邏或底盤控制。"
    exit 0
fi

assert_no_controller() {
    local processes
    processes=$(pgrep -af '[m]ecanum_driver|[o]bstacle_avoidance|[f]sm_patrol.py|[f]sm_lidar_patrol.py|[c]ontrol_modes.py|[w]heeltec_uart_bridge_node' || true)
    if [ -n "$processes" ]; then
        echo "已有控制程式，請先安全停止；本腳本不會自動殺掉不屬於自己的程序：" >&2
        printf '%s\n' "$processes" >&2
        return 1
    fi
    python3 - <<'GRAPH_CHECK'
import time
import rclpy
rclpy.init()
node = rclpy.create_node('patrol_start_preflight')
try:
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    publishers = node.get_publishers_info_by_topic('/cmd_vel')
    if publishers:
        raise SystemExit('拒絕啟動：/cmd_vel 已有發布者：' +
                         ', '.join(p.node_namespace + '/' + p.node_name for p in publishers))
finally:
    node.destroy_node()
    rclpy.shutdown()
GRAPH_CHECK
}
assert_no_controller
if [ "$action" != start-base ]; then
    [ -r "$fsm" ] && [ -r "$config" ] || { echo "缺少 $fsm 或 $config" >&2; exit 1; }
    # Validate the actual config read by this exact Python entry point.
    PYTHONPATH="$project/rl_pig_pen${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'CONFIG_CHECK'
from navigation import CONFIG_FILE, load_config
c = load_config()
print(f'巡邏設定 {CONFIG_FILE}: cruise_speed={c.cruise_speed}, turn_speed={c.turn_speed}')
print(f'角柱遮罩: {c.lidar_ignore_sectors_deg}')
CONFIG_CHECK
    ensure_lidar
fi

if [ "$action" = test-safe ]; then
    echo "只啟動 FSM，速度輸出至 /test_cmd_vel，路段至 /test_patrol/state；不啟動 bridge。"
    # This is software isolation, not a substitute for physically disabling motors.
    timeout --signal=INT --kill-after=2s 8s python3 -u "$fsm" --ros-args \
        -p use_sim_time:=false -r /scan:="$scan_topic" \
        -r /cmd_vel:=/test_cmd_vel -r /patrol/state:=/test_patrol/state \
        >"$log_dir/test-safe.log" 2>&1 || result=$?
    cat "$log_dir/test-safe.log"
    result=${result:-0}
    [ "$result" = 0 ] || [ "$result" = 124 ]
    exit
fi

[ -r "$bridge_config" ] || { echo "缺少 $bridge_config；拒絕退回 C++ 預設限速。" >&2; exit 1; }
[ -r "$port" ] && [ -w "$port" ] || { echo "串口 $port 不存在或無讀寫權限。" >&2; exit 1; }
prefix=$(ros2 pkg prefix wheeltec_uart_bridge 9>&-)
bridge="$prefix/lib/wheeltec_uart_bridge/wheeltec_uart_bridge_node"
[ -x "$bridge" ] || { echo "請先在 Xavier Humble 容器內編譯 wheeltec_uart_bridge。" >&2; exit 1; }
# Recheck after potentially slow driver startup, immediately before enabling UART.
assert_no_controller
check_cancelled
viewer=
cleanup() {
    trap - EXIT INT TERM
    stop_control
    if [ -n "$viewer" ]; then kill "$viewer" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "UART 執行檔：$bridge；設定：$bridge_config；串口：$port"
"$bridge" --ros-args --params-file "$bridge_config" -p port:="$port" \
    >"$log_dir/bridge.log" 2>&1 9>&- &
record_pid bridge "$!"
ready=false
for _ in {1..50}; do
    if ! owned_pid bridge >/dev/null; then break; fi
    if grep -q 'Opened .* at 115200 8N1' "$log_dir/bridge.log"; then ready=true; break; fi
    sleep .1
done
if [ "$ready" != true ]; then
    cat "$log_dir/bridge.log" >&2
    echo "串口未確認開啟；不啟動巡邏。" >&2
    exit 1
fi
ros2 param get /wheeltec_uart_bridge max_linear_x 9>&-
ros2 param get /wheeltec_uart_bridge max_angular_z 9>&-
check_cancelled
if [ "$action" = start-patrol ] || [ "$action" = start-modes ]; then
    if [ "$action" = start-modes ]; then
        echo "啟動三模式控制，預設 STOP；使用 /control/set_mode 切換模式。"
    else
        echo "即將啟動完整巡邏，車子可能立即移動；Ctrl-C 或另一終端的 stop 可請求停止。"
    fi
    python3 -u "$fsm" --ros-args -p use_sim_time:=false -r /scan:="$scan_topic" \
        >"$log_dir/patrol.log" 2>&1 9>&- &
    record_pid fsm "$!"
    tail -n +1 -F "$log_dir/patrol.log" "$log_dir/bridge.log" 9>&- &
else
    echo "只有 UART bridge；未啟動巡邏，但其他新 /cmd_vel 指令仍可能驅動車子。"
    tail -n +1 -F "$log_dir/bridge.log" 9>&- &
fi
viewer=$!
while owned_pid bridge >/dev/null; do
    if [ -e "$runtime/stop_requested" ]; then break; fi
    if { [ "$action" = start-patrol ] || [ "$action" = start-modes ]; } && ! owned_pid fsm >/dev/null; then break; fi
    sleep .2
done
CONTAINER_SCRIPT
