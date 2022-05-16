#!/bin/bash

# source the management script
. "$SNAP/usr/bin/config-manager"

handle_ws_config()
{
	# Check for home-dir connection
	if ! snapctl is-connected home; then
		echo "Please run \"snap connect ros2-foxy-rosbag:home :home\""
		return 1
	fi

	# Get the current workspace path
	custom_ws="$(custom_workspace_path)"

	# Validate 
	if ! [ -d "${custom_ws}" ]; then
		echo "\"$custom_ws\" folder does not exist?" 
	fi

	# run function from management script
	set_custom_workspace_path "$custom_ws"
}

handle_ws_config
