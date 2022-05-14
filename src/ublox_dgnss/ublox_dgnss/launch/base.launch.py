""" Launch ublox_dgnss_node publishing high precision Lon/Lat messages"""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  """Generate launch description for ublox_dgnss components."""
  params = [{'CFG_USBOUTPROT_NMEA': False},
            {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
            {'CFG_NAVSPG_DYNMODEL': 2},
            {'CFG_NAVHPG_DGNSSMODE':3},
            {'CFG_NAVSPG_FIXMODE':3},
            {'CFG_MSGOUT_UBX_NAV_SIG_USB': 1},
            {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
            {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 5}]

  container1 = ComposableNodeContainer(
    node_name='gps_base_container',
    node_namespace='',
    package='rclcpp_components',
    node_executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_dgnss_node',
        node_plugin='ublox_dgnss::UbloxDGNSSNode',
        node_name='gps_base',
        parameters=params
      )
    ]
  )

  return launch.LaunchDescription([container1])
