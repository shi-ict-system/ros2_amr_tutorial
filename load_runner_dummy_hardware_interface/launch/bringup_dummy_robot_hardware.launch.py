from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('load_runner_dummy_hardware_interface'),
            'urdf/dummy_robot.urdf.xacro',
        ])
    ])
    dummy_robot_description = {"robot_description": robot_description_content}

    dummy_robot_controllers = PathJoinSubstitution([
            FindPackageShare('load_runner_dummy_hardware_interface'),
            "config/controllers.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            dummy_robot_description,
            dummy_robot_controllers
            ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    ld.add_action(control_node)
    return ld