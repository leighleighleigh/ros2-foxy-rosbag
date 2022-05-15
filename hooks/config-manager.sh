#!/bin/sh
# No workspace is sourced by default
DEFAULT_WS=""

set_custom_workspace_path()
{
	snapctl set custom-workspace-path="$1"
}

custom_workspace_path()
{
	slpval="$(snapctl get custom-workspace-path)"
	if [ -z "$wsval" ]; then
		wsval="$DEFAULT_WS"
		set_custom_workspace_path $wsval
	fi
	echo "$wsval"
}
