# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Launch the hospital simulation controllers in a single composed process.

All four C++ controllers are ``rclcpp_components``; loading them into one
``component_container`` keeps the simulation in a single process with low
inter-node overhead.

Run on its own (controllers use their compiled-in default parameters)::

    ros2 launch hospital_controller controller.launch.py

Or with an external parameter file::

    ros2 launch hospital_controller controller.launch.py params_file:=/path/to/params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    # An empty params_file means "use the controllers' compiled defaults".
    common_params = [params_file] if params_file else []

    container = ComposableNodeContainer(
        name='hospital_controller_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='hospital_controller',
                plugin='hospital_controller::DoorControllerNode',
                name='door_controller_node',
                parameters=common_params,
            ),
            ComposableNode(
                package='hospital_controller',
                plugin='hospital_controller::CtScannerControllerNode',
                name='ct_scanner_controller_node',
                parameters=common_params,
            ),
            ComposableNode(
                package='hospital_controller',
                plugin='hospital_controller::LabControllerNode',
                name='lab_controller_node',
                parameters=common_params,
            ),
            ComposableNode(
                package='hospital_controller',
                plugin='hospital_controller::OperatingRoomControllerNode',
                name='operating_room_controller_node',
                parameters=common_params,
            ),
        ],
    )
    return [container]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Optional YAML parameter file for the controllers.'),
        OpaqueFunction(function=_launch_setup),
    ])
