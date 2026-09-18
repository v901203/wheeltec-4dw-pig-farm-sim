#!/usr/bin/env bash
# launch_rl_training.sh
#
# 一鍵 RL 訓練腳本：
#   1. 啟動 Gazebo 模擬環境（包含車子）
#   2. 等待環境就緒
#   3. 背景啟動 TensorBoard（http://localhost:6006）
#   4. 前景執行 PPO 訓練（Ctrl-C 安全中止）
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
TARGET_RTF=6
LIDAR_ONLY=true
HEADLESS=false
IN_TERMINAL=false
SAME_TERMINAL=false
ORIGINAL_ARGS=("$@")
TB_LOG_DIR="$RL_DIR/logs_corridor38"
TB_LOG_EXPLICIT=false
TRAIN_MODE=corridor
NUM_ENVS=1
DOMAIN_BASE=40
PARTITION_PREFIX="4wd_rl_$$"
TRAIN_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --rtf) TARGET_RTF="${2:?--rtf requires a positive number}"; shift 2 ;;
        --with-cameras) LIDAR_ONLY=false; shift ;;
        --headless) HEADLESS=true; shift ;;
        --gui) HEADLESS=false; shift ;;
        --in-terminal) IN_TERMINAL=true; shift ;;
        --same-terminal) SAME_TERMINAL=true; shift ;;
        --num-envs) NUM_ENVS="${2:?--num-envs requires a number}"; shift 2 ;;
        --num-envs=*) NUM_ENVS="${1#*=}"; shift ;;
        --domain-base) DOMAIN_BASE="${2:?--domain-base requires a number}"; shift 2 ;;
        --partition-prefix) PARTITION_PREFIX="${2:?--partition-prefix requires a name}"; shift 2 ;;
        --mode) TRAIN_MODE="${2:?--mode requires corridor or patrol}"; TRAIN_ARGS+=("$1" "$2"); shift 2 ;;
        --mode=*) TRAIN_MODE="${1#*=}"; TRAIN_ARGS+=("$1"); shift ;;
        --log-dir) TB_LOG_EXPLICIT=true; TB_LOG_DIR="${2:?--log-dir requires a path}"; TRAIN_ARGS+=("$1" "$2"); shift 2 ;;
        --log-dir=*) TB_LOG_EXPLICIT=true; TB_LOG_DIR="${1#*=}"; TRAIN_ARGS+=("$1"); shift ;;
        *) TRAIN_ARGS+=("$1"); shift ;;
    esac
done
if [[ ! "$NUM_ENVS" =~ ^[1-8]$ || ! "$DOMAIN_BASE" =~ ^[0-9]+$ ]] || (( DOMAIN_BASE + NUM_ENVS > 101 )); then
    echo "Use --num-envs 1..8 and ROS domains in 0..100" >&2
    exit 1
fi
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
for ((arg_index=0; arg_index<${#TRAIN_ARGS[@]}; arg_index++)); do
    case "${TRAIN_ARGS[$arg_index]}" in
        --config|--checkpoint-dir|--log-dir|--resume)
            if (( arg_index + 1 < ${#TRAIN_ARGS[@]} )); then
                path_value="${TRAIN_ARGS[$((arg_index + 1))]}"
                if [[ "$path_value" != /* && ( -e "$PROJECT_ROOT/$path_value" || "$path_value" == rl_pig_pen/* ) ]]; then
                    TRAIN_ARGS[$((arg_index + 1))]="$PROJECT_ROOT/$path_value"
                fi
                ((arg_index++))
            fi
            ;;
    esac
done
if [[ ! "$PARTITION_PREFIX" =~ ^[a-zA-Z0-9_-]+$ ]]; then
    echo "Invalid --partition-prefix" >&2
    exit 1
fi
if [[ "$TRAIN_MODE" != corridor && "$TRAIN_MODE" != patrol ]]; then
    echo "--mode must be corridor or patrol" >&2
    exit 1
fi
if [[ "$TB_LOG_EXPLICIT" == false && "$TRAIN_MODE" == patrol ]]; then
    TB_LOG_DIR="$RL_DIR/logs_patrol49_v2"
fi
if [[ "$TB_LOG_DIR" != /* ]]; then TB_LOG_DIR="$RL_DIR/$TB_LOG_DIR"; fi
# Validate before launching processes or installing the cleanup trap.
python3 -c 'import math,sys; v=float(sys.argv[1]); sys.exit(0 if math.isfinite(v) and v > 0 else 1)' "$TARGET_RTF" || {
    echo "--rtf 必須是正數" >&2
    exit 1
}

# Open the actual training shell, preserving the caller's Python environment.
# Pass arguments as argv, never interpolate user paths into shell command text.
if [[ "$IN_TERMINAL" == false && "$SAME_TERMINAL" == false && -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    if command -v gnome-terminal >/dev/null; then
        exec gnome-terminal --wait --title="PPO 訓練｜進度・數據・重生原因" \
            --working-directory="$SCRIPT_DIR/.." -- \
            env PATH="$PATH" VIRTUAL_ENV="${VIRTUAL_ENV:-}" PYTHONUNBUFFERED=1 \
            bash -c 'bash "$@"; result=$?; echo; echo "訓練程序已結束（代碼 $result）。按 Enter 關閉視窗。"; read -r; exit "$result"' \
            rl-training "$SCRIPT_DIR/launch_rl_training.sh" --in-terminal "${ORIGINAL_ARGS[@]}"
    else
        echo "找不到 gnome-terminal，將在目前終端機顯示訓練。"
    fi
fi
mkdir -p "$LOG_DIR"
CONSOLE_LOG="$LOG_DIR/rl_training_$(date +%Y%m%d_%H%M%S)_$$.log"
printf '%s\n' "$CONSOLE_LOG" > "$LOG_DIR/rl_training_console.path"
# Keep the log writer alive during Ctrl-C so the trainer can print and save.
exec > >(trap '' INT; exec tee -a "$CONSOLE_LOG") 2>&1
export PYTHONUNBUFFERED=1
echo "訓練終端紀錄：$CONSOLE_LOG"

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
SIM_PIDS=()
TRAIN_PID=""
TB_PID=""
cleanup() {
    trap - EXIT INT TERM
    info "中止訓練，清理本次程序..."
    if [[ -n "$TRAIN_PID" ]] && kill -0 "$TRAIN_PID" 2>/dev/null; then
        kill -INT "$TRAIN_PID" 2>/dev/null || true
        wait "$TRAIN_PID" 2>/dev/null || true
    fi
    if [[ -n "$TB_PID" ]]; then kill -TERM "$TB_PID" 2>/dev/null || true; fi
    for pid in "${SIM_PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    for pid in "${SIM_PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
    info "清理完成。"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ════════════════════════════════════════════════════════════════════════
step "步驟 1／4：啟動 Gazebo 模擬環境"
# ════════════════════════════════════════════════════════════════════════
info "在背景執行 launch_clean.sh..."
info "世界目標倍率：${TARGET_RTF}x（實際速度取決於模擬負載）"
LAUNCH_ARGS=(--rtf "$TARGET_RTF")
if [[ "$HEADLESS" == "true" ]]; then
    LAUNCH_ARGS+=(--headless)
else
    info "開啟 Gazebo 視窗（使用 --headless 才會關閉視窗）"
fi
if [[ "$LIDAR_ONLY" == "true" ]]; then LAUNCH_ARGS+=(--lidar-only); fi
for (( env_index=0; env_index<NUM_ENVS; env_index++ )); do
    if (( NUM_ENVS == 1 )); then
        bash "$SCRIPT_DIR/launch_clean.sh" "${LAUNCH_ARGS[@]}" > "$LOG_DIR/launch_clean_rl.log" 2>&1 &
    else
        domain=$((DOMAIN_BASE + env_index))
        partition="${PARTITION_PREFIX}_${env_index}"
        env ROS_DOMAIN_ID="$domain" IGN_PARTITION="$partition" GZ_PARTITION="$partition" \
            bash "$SCRIPT_DIR/launch_clean.sh" "${LAUNCH_ARGS[@]}" --env-id "$domain" \
            > "$LOG_DIR/launch_env_${env_index}.log" 2>&1 &
    fi
    SIM_PIDS+=("$!")
    info "環境 $env_index PID：${SIM_PIDS[$env_index]}"
done
TRAIN_ARGS+=(--num-envs "$NUM_ENVS" --domain-base "$DOMAIN_BASE" --partition-prefix "$PARTITION_PREFIX")

# ════════════════════════════════════════════════════════════════════════
step "步驟 2／4：等待 Gazebo + ROS2 Bridge 就緒"
# ════════════════════════════════════════════════════════════════════════
TIMEOUT=120
for (( env_index=0; env_index<NUM_ENVS; env_index++ )); do
    domain="${ROS_DOMAIN_ID:-0}"
    if (( NUM_ENVS > 1 )); then domain=$((DOMAIN_BASE + env_index)); fi
    info "等待環境 $env_index（ROS domain $domain）就緒，最多 $TIMEOUT 秒..."
    deadline=$((SECONDS + TIMEOUT))
    while true; do
        if ! kill -0 "${SIM_PIDS[$env_index]}" 2>/dev/null; then
            error "環境 $env_index 異常終止，請查看 logs/launch_env_${env_index}.log 或 launch_clean_rl.log"
            exit 1
        fi
        # Avoid ROS 2 daemon discovery here: it can be stale or unavailable
        # while Gazebo is already publishing valid sensor messages.
        if ROS_DOMAIN_ID="$domain" timeout 4 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --no-daemon >/dev/null 2>&1 && \
           ROS_DOMAIN_ID="$domain" timeout 4 ros2 topic echo /odom nav_msgs/msg/Odometry --once --no-daemon >/dev/null 2>&1; then
            break
        fi
        if (( SECONDS >= deadline )); then
            error "環境 $env_index 感測資料超時"
            exit 1
        fi
        sleep 2
    done
    info "環境 $env_index 已就緒"
done
sleep 2

# ════════════════════════════════════════════════════════════════════════
step "步驟 3／4：啟動 TensorBoard（背景）"
# ════════════════════════════════════════════════════════════════════════
info "TensorBoard → http://localhost:6006"
tensorboard --logdir "$TB_LOG_DIR" --host 0.0.0.0 --port 6006 \
    &> "$LOG_DIR/tensorboard.log" &
TB_PID=$!
sleep 1
if kill -0 "$TB_PID" 2>/dev/null; then
    info "TensorBoard 啟動成功 (PID $TB_PID)"
else
    warn "TensorBoard 啟動失敗（可能沒裝？），繼續訓練。"
fi

# ════════════════════════════════════════════════════════════════════════
step "步驟 4／4：開始 PPO 訓練"
# ════════════════════════════════════════════════════════════════════════
info "按 Ctrl-C 可安全中止訓練（最後一個 checkpoint 不會遺失）。"
info "TensorBoard：http://localhost:6006"
echo ""
cd "$RL_DIR"
python3 -u train_ppo.py "${TRAIN_ARGS[@]}" &
TRAIN_PID=$!
wait "$TRAIN_PID"
TRAIN_PID=""

info "訓練結束。"
