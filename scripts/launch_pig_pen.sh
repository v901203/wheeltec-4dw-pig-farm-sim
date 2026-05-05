#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="$DIR/../worlds/pig_pen_8units(lv4).world"
LOG_DIR="$DIR/../logs"

mkdir -p "$LOG_DIR"

if [ -f "/opt/ros/humble/setup.bash" ]; then
		# shellcheck disable=SC1091
		# `setup.bash` may reference variables that are unset when `set -u` is enabled.
		set +u
		source /opt/ros/humble/setup.bash
		set -u
fi

echo "Starting Gazebo world: $WORLD_FILE (logs: $LOG_DIR/gz_sim.log)"
gz sim "$WORLD_FILE" -r &> "$LOG_DIR/gz_sim.log" &
GZ_PID=$!

sleep 3

# PX4 and XRCE are optional; keep them off by default so this world only shows the custom 4WD model.
if [ "${START_PX4:-0}" = "1" ]; then
	if command -v MicroXRCEAgent >/dev/null 2>&1; then
		echo "Starting MicroXRCEAgent (udp4 port 8888) -> $LOG_DIR/microxrce.log"
		MicroXRCEAgent udp4 -p 8888 &> "$LOG_DIR/microxrce.log" &
		XRCE_PID=$!
	else
		echo "MicroXRCEAgent not found in PATH; skipping agent start"
	fi
else
	echo "START_PX4 is not 1; skipping PX4 and MicroXRCEAgent startup"
fi

# Start PX4 SITL only when explicitly requested.
if [ "${START_PX4:-0}" = "1" ]; then
	PX4_DIR="/home/vito/src/PX4-Autopilot"
	PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
	if [ -x "$PX4_BUILD/bin/px4" ]; then
		echo "Starting PX4 SITL (logs: $LOG_DIR/px4.log)"
		export HEADLESS=1
		export PX4_SIM_MODEL=gz_rover_differential
		export PX4_GZ_WORLD=rover
		export PX4_GZ_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
		export PX4_GZ_MODELS="$PX4_DIR/Tools/simulation/gz/models"
		export PX4_GZ_PLUGINS="$PX4_BUILD/src/modules/simulation/gz_plugins"
		export PX4_GZ_SERVER_CONFIG="$PX4_DIR/src/modules/simulation/gz_bridge/server.config"
		export GZ_SIM_RESOURCE_PATH="$PX4_GZ_MODELS:$PX4_GZ_WORLDS"
		export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_GZ_PLUGINS"
		export GZ_SIM_SERVER_CONFIG_PATH="$PX4_GZ_SERVER_CONFIG"

		(cd "$PX4_BUILD" && "$PX4_BUILD/bin/px4" etc -s etc/init.d-posix/rcS -i 0 -d) &> "$LOG_DIR/px4.log" &
		PX4_PID=$!
	else
		echo "PX4 SITL binary not found at $PX4_BUILD/bin/px4; skipping PX4 start"
	fi
else
	echo "START_PX4 is not 1 — skipping PX4 startup"
fi

sleep 6

# Spawn a robot into the world using existing spawn script
SPAWN_SCRIPT="$DIR/spawn_robot.sh"
DEFAULT_URDF="$DIR/../turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf"
if [ -x "$SPAWN_SCRIPT" ] || [ -f "$SPAWN_SCRIPT" ]; then
	if [ -f "$DEFAULT_URDF" ]; then
		echo "Spawning robot from $DEFAULT_URDF (logs: $LOG_DIR/spawn.log)"
		bash "$SPAWN_SCRIPT" --file "$DEFAULT_URDF" --model wheeltec_mini --pos -7 0 0 --yaw 0 &> "$LOG_DIR/spawn.log" &
		SPAWN_PID=$!
	else
		echo "Default URDF not found ($DEFAULT_URDF). To spawn from topic, run: $SPAWN_SCRIPT --model NAME"
	fi
else
	echo "Spawn script not found: $SPAWN_SCRIPT"
fi

echo "Launched processes:" 
if [ -n "${GZ_PID:-}" ]; then echo " - gz sim PID $GZ_PID (log: $LOG_DIR/gz_sim.log)"; fi
if [ -n "${XRCE_PID:-}" ]; then echo " - MicroXRCEAgent PID $XRCE_PID (log: $LOG_DIR/microxrce.log)"; fi
if [ -n "${PX4_PID:-}" ]; then echo " - PX4 PID $PX4_PID (log: $LOG_DIR/px4.log)"; fi
if [ -n "${SPAWN_PID:-}" ]; then echo " - spawn PID $SPAWN_PID (log: $LOG_DIR/spawn.log)"; fi

echo "All logs: $LOG_DIR"

echo "When finished, kill these PIDs if needed: $GZ_PID ${XRCE_PID:-} ${PX4_PID:-} ${SPAWN_PID:-}"

exit 0
