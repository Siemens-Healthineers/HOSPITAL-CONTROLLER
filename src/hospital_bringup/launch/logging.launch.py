# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Spawn the room-dwell + task activity logger.

The logger is a single ``hospital_robot::ActivityLoggerNode`` composable node.
It is robot-agnostic: it only subscribes to each robot's public pose and infers
room occupancy from geometry, so it works for the bundled simulation and for
third-party robots alike. Recording is toggled from the RViz panel's Start/Stop
Recording buttons (std_srvs/SetBool on ``/hospital/logging/set_recording``).

It reads its parameters from the ``activity_logger_node:`` section of the shared
params file (robot roster, room rectangles, clinical corridor polygon, log directory).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    common_params = [params_file] if params_file else []
    if params_file and os.path.isfile(params_file):
        with open(params_file, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        robot_ids = (
            data.get('robot_server_node', {})
            .get('ros__parameters', {})
            .get('robot_ids')
        )
        if robot_ids is not None:
            common_params.append({'robot_ids': robot_ids})

    container = ComposableNodeContainer(
        name='hospital_logging_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='hospital_robot',
                plugin='hospital_robot::ActivityLoggerNode',
                name='activity_logger_node',
                parameters=common_params,
            ),
        ],
    )
    return [container]


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory('hospital_bringup'),
        'config', 'hospital_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML parameter file providing the logger params.'),
        OpaqueFunction(function=_launch_setup),
    ])
