#!/bin/bash

# Launch script for Gazebo (Ignition / gz) headless simulation

set -e

# Extend resource path so models in current directory are found
export IGN_GAZEBO_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH}:$(pwd)"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:$(pwd)"

WORLD_FILE="MyModel.world"

if [ ! -f "$WORLD_FILE" ]; then
	echo "World file $WORLD_FILE not found in $(pwd)" >&2
	exit 1
fi

echo "Launching simulation using world: $WORLD_FILE"

if command -v gz >/dev/null 2>&1; then
	# Gazebo Garden (gz) style
	echo "Detected 'gz' binary. Starting with gz sim (render disabled)."
	# -r disables rendering, keeping server active headless
	gz sim -v 3 -r "$WORLD_FILE"
elif command -v ign >/dev/null 2>&1; then
	# Ignition Fortress style
	echo "Detected 'ign' binary. Starting with ign gazebo (render disabled)."
	# -r skips rendering; avoid -s which may exit early after load
	IGN_GUI=0 ign gazebo -v 3 -r "$WORLD_FILE"
else
	echo "Neither 'gz' nor 'ign' command found in PATH." >&2
	exit 1
fi

