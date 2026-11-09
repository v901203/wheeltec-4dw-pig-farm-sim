#!/usr/bin/env bash
# Read-only Xavier audit. No sudo authentication, service actions or network changes.
set -uo pipefail
if [[ "${1:-}" == --help ]]; then
    echo '用法：bash check_robot_autostart.sh；唯讀檢查開機／GUI／DDS／雷達網路設定。'
    exit 0
fi
container="${ROBOT_CONTAINER:-robot-dev}"
project="${ROBOT_PROJECT_HOST:-$HOME/Desktop/pig_patrol}"
launcher="${ROBOT_LAUNCHER_HOST:-$HOME/ros2_ws/start_robot.sh}"
section() { printf '\n【%s】\n' "$1"; }
section '開機服務設定與目前狀態'
systemctl is-enabled pig-pen-patrol.service 2>&1 || true
systemctl show pig-pen-patrol.service --no-pager \
    -p ActiveState -p SubState -p Result -p ExecStart -p NeedDaemonReload 2>&1 || true
systemctl show pig-pen-patrol.service -p Environment --value 2>/dev/null \
    | tr ' ' '\n' | grep -E '^ROBOT_(INITIAL_MODE|RMW|CONTAINER|PORT|SCAN_TOPIC|WS_CONTAINER)=' || true
section '主機啟動腳本'
if [[ -r "$launcher" ]]; then
    bash -n "$launcher" && echo 'Bash 語法正常'
    grep -nE 'local iface=|robot_rmw=|export RMW_IMPLEMENTATION|install_humble_|initial_mode=' "$launcher" || true
else
    echo "找不到：$launcher"
fi
section 'GUI 登入自動啟動'
startup="$HOME/.config/autostart/pig-pen-debug.desktop"
if [[ -r "$startup" ]]; then
    ls -ld "$HOME/.config/autostart" "$startup"
    grep -E '^(Type|Exec|Hidden|X-GNOME-Autostart-enabled|OnlyShowIn|NotShowIn)=' "$startup" || true
else
    echo "未安裝或無法讀取：$startup"
fi
[[ -r "$HOME/.config/autostart/pig-pen-terminal.desktop" ]] && echo '另有獨立日誌終端機的自動啟動設定'
ls -l "$project/scripts/patrol_debug_gui.py" 2>&1 || true
section '實車巡邏設定（檔案保存值）'
python3 - "$project/rl_pig_pen/patrol_config.json" <<'PY'
import json, sys
try:
    p = sys.argv[1]
    c = json.load(open(p))
    print('檔案：', p)
    for k in ('junctions', 'persistent_recovery', 'cruise_speed', 'turn_speed',
              'lidar_yaw', 'lidar_ignore_sectors_deg', 'vision_required'):
        print(k, '=', c.get(k, '(未設定，使用程式預設)'))
except (OSError, ValueError) as e:
    print('無法讀取設定：', e)
PY
section '雷達目前地址與路由（執行中狀態，不代表已保存）'
ip -br addr 2>&1 || true
ip route get 192.168.1.200 2>&1 || true
section 'NetworkManager 已保存的設定'
nmcli -f NAME,DEVICE connection show --active 2>&1 || true
# Inspect the active eth1 connection and both previously used profiles.
active=$(nmcli -g GENERAL.CONNECTION device show eth1 2>/dev/null || true)
declare -A seen_profiles=()
for profile in "$active" 'Profile 1' 'Profile 2'; do
    [[ -n "$profile" && "$profile" != -- ]] || continue
    [[ -z "${seen_profiles[$profile]+set}" ]] || continue
    seen_profiles["$profile"]=1
    echo "連線：$profile"
    nmcli -f connection.id,connection.interface-name,connection.autoconnect,connection.autoconnect-priority,ipv4.method,ipv4.addresses,ipv4.routes \
        connection show "$profile" 2>&1 || true
done
section 'GUI 免密碼權限（只查詢，不執行 stop/restart）'
for action in stop restart; do
    if sudo -n -l /usr/bin/systemctl "$action" pig-pen-patrol.service >/dev/null 2>&1; then
        echo "$action：sudo 規則允許（仍需執行時驗證 NOPASSWD）"
    else
        echo "$action：無法確認，請檢查 sudoers 規則"
    fi
done
section 'Docker 網路、預設環境及實際小車程序'
if docker inspect "$container" >/dev/null 2>&1; then
    docker inspect "$container" --format 'Network={{.HostConfig.NetworkMode}} Running={{.State.Running}}'
    docker inspect "$container" --format '{{range .Config.Env}}{{println .}}{{end}}' \
        | grep -E '^(ROS_DISTRO|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_LOCALHOST_ONLY)=' || true
    docker exec -i "$container" python3 - <<'PY'
from pathlib import Path
names = {'fsm_lidar_patrol.py', 'control_modes.py', 'wheeltec_uart_bridge_node', 'lslidar_driver_node'}
found = 0
for p in Path('/proc').iterdir():
    if not p.name.isdigit():
        continue
    try:
        cmd = (p/'cmdline').read_bytes().split(b'\0')
        tokens = [a.decode(errors='replace') for a in cmd if a]
        if not any(Path(a).name in names or Path(a).name.startswith('patrol_debug_gui.py') for a in tokens):
            continue
        env = dict(item.split('=', 1) for item in (p/'environ').read_bytes().decode(errors='replace').split('\0') if '=' in item)
        print('PID', p.name, ' '.join(tokens))
        print('  RMW=', env.get('RMW_IMPLEMENTATION', '(預設)'),
              'Domain=', env.get('ROS_DOMAIN_ID', '(預設 0)'),
              'localhost=', env.get('ROS_LOCALHOST_ONLY', '(預設)'))
        found += 1
    except (OSError, ProcessLookupError):
        continue
if not found:
    print('沒有找到正在運行的小車節點（或無讀取權限）。')
PY
else
    echo "無法讀取容器 $container（不存在或帳號無權限）。"
fi
section '本次開機巡邏日誌'
journalctl -u pig-pen-patrol.service -b -n 35 --no-pager 2>&1 || true
section '完成'
echo '以上只讀取設定；未啟動／停止服務、未改網路、未發布速度。'
echo '設定檔與執行中狀態需分開判讀；開機是否成功仍以本次開機日誌為準。'
