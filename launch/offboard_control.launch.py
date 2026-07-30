#!/usr/bin/env python3

import uuid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('offboard_cpp').find('offboard_cpp')
    default_param_file = PathJoinSubstitution([pkg_share, "config", "vertical_test.yaml"])

    launch_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "config_file", default_value=default_param_file,
            description="Reviewed offboard config; vertical_test.yaml is the first-flight envelope"),
        DeclareLaunchArgument("task_id", default_value="3"),
        DeclareLaunchArgument(
            "hover_height",
            default_value="0.5",
            description="Vertical-test hover height above the START position in metres"),
        DeclareLaunchArgument(
            "relative_takeoff_height",
            default_value="true",
            description="Interpret hover_height relative to the START position"),
        DeclareLaunchArgument(
            "hold_after_takeoff",
            default_value="true",
            description="Hold the vertical-test setpoint until operator takeover"),
        DeclareLaunchArgument(
            'enable_arm',
            default_value='false',
            description='Explicit arm/rearm permission; false is the production-safe default'),
        DeclareLaunchArgument('owner_id', default_value='flight-sequence'),
        DeclareLaunchArgument('lease_id', default_value=str(uuid.uuid4())),
        DeclareLaunchArgument("epoch", default_value=str(uuid.uuid4())),
        DeclareLaunchArgument(
            "mission_source_epoch", default_value="0",
            description="Nonzero uint32 epoch shared with mission_bridge; zero fails closed"),
    ]
    common_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    offboard_node = Node(
        package='offboard_cpp',
        executable='offboard_node',
        name='offboard_control_node',
        output='screen',
        parameters=[
            LaunchConfiguration("config_file"),
            {
                'use_sim_time': common_time,
                'takeoff_land.enable_arm': ParameterValue(
                    LaunchConfiguration('enable_arm'), value_type=bool),
                'safety.expected_owner': LaunchConfiguration('owner_id'),
                'safety.expected_lease': LaunchConfiguration('lease_id'),
                'safety.expected_epoch': LaunchConfiguration('epoch'),
            },
        ],
        emulate_tty=True,
    )

    flight_sequence_node = Node(
        package='offboard_cpp',
        executable='flight_sequence_node',
        name='flight_sequence_node',
        output='screen',
        parameters=[
            LaunchConfiguration("config_file"),
            {
                'use_sim_time': common_time,
                'mission.task_id': ParameterValue(
                    LaunchConfiguration('task_id'), value_type=int),
                'mission.takeoff_height': ParameterValue(
                    LaunchConfiguration('hover_height'), value_type=float),
                'mission.relative_takeoff_height': ParameterValue(
                    LaunchConfiguration('relative_takeoff_height'), value_type=bool),
                'mission.hold_after_takeoff': ParameterValue(
                    LaunchConfiguration('hold_after_takeoff'), value_type=bool),
                'mission.expected_source_epoch': ParameterValue(
                    LaunchConfiguration('mission_source_epoch'), value_type=int),
            },
        ],
        emulate_tty=True,
    )

    rc_operator_node = Node(
        package="offboard_cpp",
        executable="rc_operator_adapter_node",
        name="rc_operator_adapter_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file"), {"use_sim_time": common_time}],
        emulate_tty=True,
    )

    authority_node = Node(
        package='offboard_cpp',
        executable='offboard_authority_node',
        name='offboard_authority_node',
        output='screen',
        parameters=[
            LaunchConfiguration("config_file"),
            {
                'use_sim_time': common_time,
                'authority.owner': LaunchConfiguration('owner_id'),
                'authority.lease': LaunchConfiguration('lease_id'),
                'authority.epoch': LaunchConfiguration('epoch'),
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription(
        launch_args + [rc_operator_node, authority_node, flight_sequence_node, offboard_node])
