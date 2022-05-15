#!/bin/bash

# Sources a ROS2 workspace provided by the config files
# Check for home-dir connection
if ! snapctl is-connected home; then
	echo "Please run \"snap connect ros2-foxy-rosbag:home :home\""
	return 1
fi

# Source the config management script
. "$SNAP/usr/bin/config-manager"

# Get the workspace path variable 
export CUSTOMWS=$(custom_workspace_path)

# Source it if not empty
if [ -z "$CUSTOMWS" ]; then
	echo "Sourcing custom ROS workspace: ${CUSTOMWS}"
	source "${CUSTOMWS}/install/setup.bash"
else
	echo "No custom workspace defined. Custom message types may be unavailable."
fi


# Run ros2 foxy command with the given arguments
$SNAP/opt/ros/foxy/bin/ros2 "$@"
