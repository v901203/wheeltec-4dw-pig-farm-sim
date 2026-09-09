#!/usr/bin/env bash
# launch_rl_training.sh
#
# 一鍵 RL 訓練腳本：
#   1. 啟動 Gazebo 模擬環境（包含車子）
#   2. 等待環境就緒
#   3. 首次使用時自動執行路徑點校準
#   4. 背景啟動 TensorBoard（http://localhost:6006）
#   5. 前景執行 PPO 訓練（Ctrl-C 安全中止）
#
# 跟 launch_clean.sh 的差別：
#   launch_clean.sh  → 只啟動模擬環境與車子，適合手動測試 / 遙控
#   launch_rl_training.sh → 在模擬環境上直接跑完整訓練 pipeline
#
# 用法：
#   cd ~/Desktop/4wd/scripts
#   ./launch_rl_training.sh

set -euo pipefail

# ── 路徑設定 ────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_DIR="$SCRIPT_DIR/../rl_pig_pen"
LOG_DIR="$SCRIPT_DIR/../logs"
WAYPOINTS="$RL_DIR/waypoints.json"

# ── 顏色輸出 ────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; NC='\033[0m'
info()  { echo -e "${GREEN}[RL]${NC} $*"; }
warn()  { echo -e "${YELLOW}[RL]${NC} $*"; }
error() { echo -e "${RED}[RL]${NC} $*"; }
step()  { echo -e "\n${CYAN}══════════════════════════════════${NC}"; echo -e "${CYAN} $*${NC}"; echo -e "${CYAN}══════════════════════════════════${NC}"; }

# ── 環境前置確認 ─────────────────────────────────────────────────────────
if [[ ! -f "$RL_DIR/pig_pen_env.py" ]]; then
    error "找不到 $RL_DIR/pig_pen_env.py"
    error "請先把 rl_pig_pen/ 資料夾放到 ~/Desktop/4wd/ 底下。"
    exit 1
fi
if [[ ! -f "$RL_DIR/train_ppo.py" ]]; then
    error "找不到 $RL_DIR/train_ppo.py"
    exit 1
fi

# ── Source ROS2 ──────────────────────────────────────────────────────────
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
export IGN_TRANSPORT_TOPIC_STATISTICS=0
if [[ -f "$SCRIPT_DIR/../install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$SCRIPT_DIR/../install/setup.bash"
fi
set -u

mkdir -p "$LOG_DIR"

# ── Cleanup（Ctrl-C 或正常結束時執行） ───────────────────────────────────
cleanup() {
    echo ""
    info "中止訓練，清理程序..."
    pkill -f "tensorboard"          2>/dev/null || true
    pkill -f "train_ppo.py"         2>/dev/null || true
    pkill -f "calibrate_waypoints"  2>/dev/null || true
    pkill -f "ros_gz_bridge"        2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "depth_image_proc"     2>/dev/null || true
    pkill -f "pig_behavior.py"      2>/dev/null || true
    pkill -f "aruco_face_detector"  2>/dev/null || true
    pkill -9 -f "ign gazebo"        2>/dev/null || true
    killall -9 ign 2>/dev/null || true
    info "清理完成。"
}
trap cleanup EXIT

# ════════════════════════════════════════════════════════════════════════
step "步驟 1／5：啟動 Gazebo 模擬環境"
# ════════════════════════════════════════════════════════════════════════
info "在背景執行 launch_clean.sh..."
bash "$SCRIPT_DIR/launch_clean.sh" --headless &> "$LOG_DIR/launch_clean_rl.log" &
LAUNCH_PID=$!
info "模擬環境 PID：$LAUNCH_PID  (log → $LOG_DIR/launch_clean_rl.log)"

# ════════════════════════════════════════════════════════════════════════
step "步驟 2／5：等待 Gazebo + ROS2 Bridge 就緒"
# ════════════════════════════════════════════════════════════════════════
TIMEOUT=120
ELAPSED=0
info "最多等待 ${TIMEOUT} 秒..."
while true; do
    # 嘗試收一筆 /scan 資料，成功代表 bridge 正常
    if timeout 4 ros2 topic echo /scan --once &>/dev/null; then
        break
    fi
    # 確認模擬環境程序還活著
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        error "launch_clean.sh 異常終止！請查看 $LOG_DIR/launch_clean_rl.log"
        exit 1
    fi
    sleep 4
    ELAPSED=$((ELAPSED + 4))
    if [[ $ELAPSED -ge $TIMEOUT ]]; then
        error "超時！/scan topic 未收到資料。"
        error "請查看：tail -50 $LOG_DIR/launch_clean_rl.log"
        exit 1
    fi
    info "  等待中... (${ELAPSED}s / ${TIMEOUT}s)"
done
info "環境就緒！(${ELAPSED}s)"
sleep 2   # 讓所有 bridge 再穩定一下

# ════════════════════════════════════════════════════════════════════════
step "步驟 3／5：路徑點校準"
# ════════════════════════════════════════════════════════════════════════
if [[ ! -f "$WAYPOINTS" ]] || grep -q "PLACEHOLDER" "$WAYPOINTS" 2>/dev/null; then
    warn "偵測到未校準的 waypoints.json，需要先校準路徑點。"
    warn "車子已在右端起點（第一個點直接按 Enter 即可）。"
    echo ""
    warn "請在另一個 Terminal 開啟 teleop 遙控車子："
    echo -e "  ${CYAN}ros2 run teleop_twist_keyboard teleop_twist_keyboard${NC}"
    echo ""
    read -rp "  teleop 開好後按 Enter 開始校準..." _
    echo ""
    cd "$RL_DIR"
    python3 calibrate_waypoints.py
    info "校準完成！路徑點已存入 $WAYPOINTS"
    echo ""
else
    info "已有校準完成的 waypoints.json，跳過校準。"
    info "  若要重新校準，刪除 $WAYPOINTS 後重新執行本腳本。"
fi

# ════════════════════════════════════════════════════════════════════════
step "步驟 4／5：啟動 TensorBoard（背景）"
# ════════════════════════════════════════════════════════════════════════
info "TensorBoard → http://localhost:6006"
tensorboard --logdir "$RL_DIR/logs" --host 0.0.0.0 --port 6006 \
    &> "$LOG_DIR/tensorboard.log" &
TB_PID=$!
sleep 1
if kill -0 "$TB_PID" 2>/dev/null; then
    info "TensorBoard 啟動成功 (PID $TB_PID)"
else
    warn "TensorBoard 啟動失敗（可能沒裝？），繼續訓練。"
fi

# ════════════════════════════════════════════════════════════════════════
step "步驟 5／5：開始 PPO 訓練"
# ════════════════════════════════════════════════════════════════════════
info "按 Ctrl-C 可安全中止訓練（最後一個 checkpoint 不會遺失）。"
info "TensorBoard：http://localhost:6006"
echo ""
cd "$RL_DIR"
python3 train_ppo.py

info "訓練結束。"
