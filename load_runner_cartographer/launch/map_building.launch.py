import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', PathJoinSubstitution([
                                                get_package_share_directory('load_runner_cartographer'),
                                                'config']),
            '-configuration_basename', 'map_building.lua'
        ],
        remappings=[
            ("scan_1", "front_scan"),
            ("scan_2", "rear_scan")
        ],
        parameters=[{
            "use_sim_time": True,
        }]
    )

    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        output='screen',
        arguments=[
            '--resolution', '0.05'
        ],
        parameters=[{
            "use_sim_time": True,
        }]
    )

    ld.add_action(cartographer_node)
    ld.add_action(cartographer_grid_node)
    return ld