import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, Shutdown, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_model = DeclareLaunchArgument("robot_model", default_value="load_runner_300lt")
    world_name = DeclareLaunchArgument("world_name", default_value="simple_world.sdf")

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
            'ign_args': [
                            '-r -v0 ',
                            PathJoinSubstitution([
                                    get_package_share_directory('load_runner_gazebo'),
                                    'worlds',
                                    LaunchConfiguration('world_name')
                            ])
                        ]
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
        ],
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
        parameters=[{
            "source_list": ['joint_states'],
            "rate": 100.0,
            "use_sim_time": True
        }],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_lift_up_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
                'lift_up_controller'],
        output='screen'
    )

    load_lift_turn_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
                'lift_turn_controller'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
                'base_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_lift_up_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_lift_up_controller,
                on_exit=[load_lift_turn_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_lift_turn_controller,
                on_exit=[load_base_controller],
            )
        ),
        robot_model,
        world_name,
        upload_robot,
        ign_gazebo,
        spawn_node,
        jsp_node,
        ign_bridge,
        static_tf_front_lidar,
        static_tf_rear_lidar
    ])
