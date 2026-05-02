#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="$DIR/../worlds/pig_pen_8units.world"

if [ -f "/opt/ros/humble/setup.bash" ]; then
	# shellcheck disable=SC1091
	# `setup.bash` may reference variables that are unset when `set -u` is enabled.
	set +u
	source /opt/ros/humble/setup.bash
	set -u
fi

exec gz sim "$WORLD_FILE" -r
