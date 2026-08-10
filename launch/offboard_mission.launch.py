from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('offboard_cpp'), 'config', 'mission_common.yaml']),
        ),
        Node(
            package='offboard_cpp',
            executable='offboard_mission_node',
            name='offboard_mission_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
