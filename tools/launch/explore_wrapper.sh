#!/bin/bash
# explore_wrapper.sh
# Sources the local deps workspace (where explore_lite is built) and runs the command.

# Source ROS 2 base
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source local workspace
WORKSPACE_INSTALL="$(dirname "$0")/../../deps/install/setup.bash"
if [ -f "$WORKSPACE_INSTALL" ]; then
    source "$WORKSPACE_INSTALL"
else
    echo "WARNING: Local workspace setup.bash not found at $WORKSPACE_INSTALL"
fi

# Exec the command
# Filter out arguments that cause issues with ros2 run (like use_rviz:=True from GUI)
filtered_args=()
for arg in "$@"; do
    if [[ "$arg" != "use_rviz:="* ]]; then
        filtered_args+=("$arg")
    fi
done

# Exec the command with filtered arguments
exec "${filtered_args[@]}"
