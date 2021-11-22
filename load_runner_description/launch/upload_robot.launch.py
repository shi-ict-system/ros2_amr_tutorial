import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    robot_model = DeclareLaunchArgument("robot_model", default_value="load_runner_300lt")

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        get_package_share_directory('load_runner_description'),
                        'urdf/robot.urdf.xacro',
                    ]),
                    ' robot_model:=',
                    LaunchConfiguration('robot_model')
                ]),
            'use_sim_time': True
        }]
    )

    ld.add_action(robot_model)
    ld.add_action(rsp_node)

    return ld