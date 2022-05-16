#!/bin/bash
# No workspace is sourced by default
DEFAULT_WS=""

set_custom_workspace_path()
{
	# Substitute any ~ to $SNAP_REAL_HOME
	wspath="$1"
	if [[ -n "$wspath" ]]; then
		wspath="${wspath/#\~/$SNAP_REAL_HOME}"
	fi
	snapctl set custom-workspace-path="$wspath"
}

custom_workspace_path()
{
	wsval="$(snapctl get custom-workspace-path)"
	
	# If no workspace defined
	if [[ -z "$wsval" ]]; then

		# Check for a 'nova_ws' workspace
		if [[ -d "$SNAP_REAL_HOME/nova_ws" ]]; then
			DEFAULT_WS="$SNAP_REAL_HOME/nova_ws"
			echo -e "\e[1m\e[33mAuto-detected ~/nova_ws\e[0m"
		fi

		wsval="$DEFAULT_WS"
		set_custom_workspace_path "$wsval"
	fi

	echo "$wsval"
}
