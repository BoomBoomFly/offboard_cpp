#!/usr/bin/env python3

import uuid

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Foxy can resolve a substitution default to an empty config_file when
    # this launch is nested.  Resolve the package path while constructing the
    # description so every node receives the reviewed vertical-test envelope.
    default_param_file = get_package_share_directory('offboard_cpp') + '/config/vertical_test.yaml'

    launch_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("task_id", default_value="3"),
        DeclareLaunchArgument(
            "takeoff_height",
            default_value="1.0",
            description="Vertical takeoff height above the valid local NED position in metres"),
        DeclareLaunchArgument(
            "relative_takeoff_height",
            default_value="true",
            description="Interpret hover_height relative to the START position"),
        DeclareLaunchArgument(
            "hold_after_takeoff",
            default_value="true",
            description="Hold the vertical-test setpoint until operator takeover"),
        DeclareLaunchArgument(
            'auto_arm',
            default_value='false',
            description='Send one arm request after Offboard confirmation; false is the safe default'),
        DeclareLaunchArgument(
            'require_armed_before_offboard',
            default_value='true',
            description='Wait for a manual RC arm before sending the Offboard prestream'),
        DeclareLaunchArgument(
            'mission_auto_takeoff',
            default_value='false',
            description='Start the configured task after manual arm and confirmed Offboard'),
        DeclareLaunchArgument('offboard_warmup_seconds', default_value='2.0',
            description='Continuous Offboard signal duration before the mode request'),
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
            default_param_file,
            {
                'use_sim_time': common_time,
                'auto_arm': ParameterValue(
                    LaunchConfiguration('auto_arm'), value_type=bool),
                'safety.require_armed_before_offboard': ParameterValue(
                    LaunchConfiguration('require_armed_before_offboard'), value_type=bool),
                'offboard_warmup_seconds': ParameterValue(
                    LaunchConfiguration('offboard_warmup_seconds'), value_type=float),
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
            default_param_file,
            {
                'use_sim_time': common_time,
                'mission.task_id': ParameterValue(
                    LaunchConfiguration('task_id'), value_type=int),
                'mission.takeoff_height': ParameterValue(
                    LaunchConfiguration('takeoff_height'), value_type=float),
                'mission.auto_takeoff': ParameterValue(
                    LaunchConfiguration('mission_auto_takeoff'), value_type=bool),
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
        parameters=[default_param_file, {"use_sim_time": common_time}],
        emulate_tty=True,
    )

    authority_node = Node(
        package='offboard_cpp',
        executable='offboard_authority_node',
        name='offboard_authority_node',
        output='screen',
        parameters=[
            default_param_file,
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
        launch_args + [
            rc_operator_node, authority_node, flight_sequence_node, offboard_node])
