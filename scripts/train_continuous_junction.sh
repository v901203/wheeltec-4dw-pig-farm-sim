#!/usr/bin/env bash
# Run only after stopping the previous controller; reuse the isolated simulator.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 IGN_PARTITION=4wd_rl_junction GZ_PARTITION=4wd_rl_junction
CONTINUOUS_MODEL="${CONTINUOUS_BC_MODEL:-logs/rl_junction_continuous/bc/20261104_142847_119472/bc_model.zip}"
if [[ ! -f "$CONTINUOUS_MODEL" ]]; then
    echo "Missing converted BC: set CONTINUOUS_BC_MODEL to the new continuous bc_model.zip" >&2
    exit 1
fi
mkdir -p logs/rl_junction_continuous
/usr/bin/python3 -u rl_junction/continuous_train.py --mode ppo \
    --init-model "$CONTINUOUS_MODEL" --timesteps 5120 --eval-episodes 3 \
    --output logs/rl_junction_continuous/training "$@" \
    2>&1 | tee "logs/rl_junction_continuous/console_$(date +%Y%m%d_%H%M%S).log"
