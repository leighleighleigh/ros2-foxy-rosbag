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
if ! [[ -z "$CUSTOMWS" ]]; then
	echo -e "\e[1mSourcing custom ROS workspace \e[33m${CUSTOMWS}\e[0m"
	# Redirect errors away from us
	source "${CUSTOMWS}/install/setup.bash" > /dev/null 2>&1
else
	echo -e "\e[1m\e[33mNo custom workspace defined. Custom message types may be unavailable.\e[0m"
fi

# Clear colours
echo -e '\e[0m'


# Run ros2 foxy command with the given arguments
$SNAP/opt/ros/foxy/bin/ros2 "$@"
