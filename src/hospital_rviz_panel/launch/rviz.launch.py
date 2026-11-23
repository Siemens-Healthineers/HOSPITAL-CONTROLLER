# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Launch RViz with the hospital control panel.

Starts ``rviz2`` preloaded with a config that docks the
``hospital_rviz_panel/HospitalControlPanel`` panel. The panel is a pure ROS 2
client, so it can run alongside other clients against the same controllers.

By default the panel uses its built-in device IDs (``ct_door``,
``angiography_door``, ``ct_scanner``), which match the controllers' defaults.
To override them, pass a params file with an ``rviz2:`` section::

    ros2 launch hospital_rviz_panel rviz.launch.py params_file:=/path/to/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _launch_setup(context, *args, **kwargs):
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    parameters = [params_file] if params_file else []
    if params_file and os.path.isfile(params_file):
        with open(params_file, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        robot_ids = (
            data.get('robot_server_node', {})
            .get('ros__parameters', {})
            .get('robot_ids')
        )
        if robot_ids is not None:
            parameters.append({'robot_ids': robot_ids})

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=parameters,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    panel_share = get_package_share_directory('hospital_rviz_panel')
    default_rviz = os.path.join(panel_share, 'rviz', 'hospital.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='RViz config file to load (with the hospital panel).'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Optional YAML params with an rviz2 section '
                        '(door_ids).'),
        OpaqueFunction(function=_launch_setup),
    ])
