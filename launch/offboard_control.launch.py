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
    param_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'ctrl_param.yaml'
    ])

    launch_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('task_id', default_value='1'),
        DeclareLaunchArgument(
            'enable_arm',
            default_value='false',
            description='Explicit arm/rearm permission; false is the production-safe default'),
        DeclareLaunchArgument('owner_id', default_value='flight-sequence'),
        DeclareLaunchArgument('lease_id', default_value=str(uuid.uuid4())),
        DeclareLaunchArgument('epoch', default_value=str(uuid.uuid4())),
    ]
    common_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    offboard_node = Node(
        package='offboard_cpp',
        executable='offboard_node',
        name='offboard_control_node',
        output='screen',
        parameters=[
            param_file,
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
            param_file,
            {
                'use_sim_time': common_time,
                'mission.task_id': ParameterValue(
                    LaunchConfiguration('task_id'), value_type=int),
            },
        ],
        emulate_tty=True,
    )

    authority_node = Node(
        package='offboard_cpp',
        executable='offboard_authority_node',
        name='offboard_authority_node',
        output='screen',
        parameters=[
            param_file,
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
        launch_args + [offboard_node, flight_sequence_node, authority_node])
