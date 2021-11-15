import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    robot_model = DeclareLaunchArgument("robot_model", default_value="load_runner_300lt")

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '1'
    ign_gazebo_paths = '/usr/lib/x86_64-linux-gnu/ign-gazebo-5/plugins'
    model_path = get_package_share_directory('load_runner_description')
    model_path = model_path[:len(model_path) - len('load_runner_description')]
    ign_gazebo_paths += pathsep + model_path

    environ['IGN_FILE_PATH'] = ign_gazebo_paths

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_ign_gazebo'),
            '/launch/ign_gazebo.launch.py'
        ]),
        launch_arguments={
            'ign_args': '-r default.sdf -v 0'
        }.items(),
    )

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('load_runner_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'robot_model': LaunchConfiguration('robot_model')
        }.items()
    )

    spawn_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_model'),
            '-topic', 'robot_description'
        ]
    )

    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[ '/front_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                    '/rear_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    static_tf_front_lidar = Node(
        package='tf2_ros',
        name="static_tf_front_lidar",
        executable='static_transform_publisher',
        arguments=[ "0.0", "0.0", "0.0", "0.0", "0.0", "0.0",
                    "front_lidar",
                    [LaunchConfiguration('robot_model'), "/base_footprint/front_lidar"]
        ],
        output='screen'
    )

    static_tf_rear_lidar = Node(
        package='tf2_ros',
        name="static_tf_rear_lidar",
        executable='static_transform_publisher',
        arguments=[ "0.0", "0.0", "0.0", "0.0", "0.0", "0.0",
                    "rear_lidar",
                    [LaunchConfiguration('robot_model'), "/base_footprint/rear_lidar"]
        ],
        output='screen'
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    ld.add_action(robot_model)
    ld.add_action(ign_gazebo)
    ld.add_action(upload_robot)
    ld.add_action(spawn_node)
    ld.add_action(ign_bridge)
    ld.add_action(static_tf_front_lidar)
    ld.add_action(static_tf_rear_lidar)
    ld.add_action(jsp_node)

    return ld
