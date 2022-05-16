# ros2-foxy-rosbag
A portable ROS2 Foxy toolkit, to provide the `ros2 bag` utility anywhere, plus extras.

```bash
sudo snap install ros2-foxy-rosbag

# Specify your custom colcon workspace, if needed (try to provide the full path, sometimes `sudo` stuffs it up)
snap set ros2-foxy-rosbag custom-workspace-path=~/nova_ws
snap get ros2-foxy-rosbag

# Use ros2 CLI tools
ros2-foxy-rosbag.ros2 topic list

# Record a bag file
ros2-foxy-rosbag.ros2 bag record -a
# Record an MCAP file
ros2-foxy-rosbag.ros2 bag record -a -s mcap

# Start a websocket server to use Foxglove easily
ros2-foxy-rosbag.rosbridge-server

```

## About
ROS2 Foxy brings lots of QoS-related support to the ROS2 platform. 
On hardware devices that do not support ROS2 Foxy (such as the Nvidia Jetson TX2), it can be difficult to capture bagfiles when QoS is in use.

This tool allows for the ROS2 Foxy 'rosbag2' packages to be used, instead - and also bundles support for the MCAP storage plugin.
Since the base 'ros2' command is exposed, it can be used as a generic ROS2 swiss-army-knife on any platform!

