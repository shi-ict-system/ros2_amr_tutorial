import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    filter_map_yaml_file = LaunchConfiguration('filter_mask')
    bringup_dir = get_package_share_directory('load_runner_navigation2')


    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': filter_map_yaml_file
    }

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'filter_mask',
            default_value=os.path.join(bringup_dir, 'maps', 'speed_limit_1637303820.yaml'),
            description='Full path to map yaml file to load'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_speed_limit_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_speed_limit_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['filter_speed_limit_mask_server', 'costmap_filter_speed_limit_info_server']}])
    ])
