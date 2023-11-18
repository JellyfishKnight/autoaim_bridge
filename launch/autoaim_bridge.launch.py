from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='autoaim_bridge',
            executable='autoaim_bridge_node',
            emulate_tty=True,
        ),
    ])
