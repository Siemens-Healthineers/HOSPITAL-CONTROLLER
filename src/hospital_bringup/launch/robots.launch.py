# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Spawn the fleet of hospital robots.

Each robot is an independent ``hospital_robot::RobotServerNode`` composable
node. They run concurrently: every robot hosts its own ExecuteTask action
server, publishes its own pose/TF, and serialises only its *own* tasks, so
different robots execute missions at the same time.

To add or remove a robot, edit ``robot_server_node.robot_ids`` in the params
file. Fleet offsets are applied along the x-axis by roster position.

``robot_id`` drives the derived interface names (shared with the RViz panel):

    action  /hospital/<robot_id>/ward/execute_task
    pose    /hospital/<robot_id>/pose
    frame   <robot_id>_base_link

Robots share the room waypoints / timings / driven-interface names from the
``robot_server_node:`` section of the params file.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def _shared_robot_params(params_file: str) -> tuple[dict, list[str]]:
    """Read common robot parameters and the canonical roster."""
    if not params_file or not os.path.isfile(params_file):
        return {}, ['robot_1']
    with open(params_file, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    section = data.get('robot_server_node', {})
    params = section.get('ros__parameters', {}) or {}
    robot_ids = params.pop('robot_ids', ['robot_1'])
    if not isinstance(robot_ids, list) or not robot_ids or not all(
            isinstance(robot_id, str) and robot_id for robot_id in robot_ids):
        raise ValueError(
            'robot_server_node.robot_ids must be a non-empty list of strings')
    if len(set(robot_ids)) != len(robot_ids):
        raise ValueError(
            'robot_server_node.robot_ids must not contain duplicates')
    return params, robot_ids


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    shared, robot_ids = _shared_robot_params(params_file)

    nodes = []
    for index, robot_id in enumerate(robot_ids):
        per_robot = dict(shared)
        per_robot.update({
            'robot_id': robot_id,
            'pose_offset_x': 0.2 * index,
            'pose_offset_y': 0.0,
            # Roster position selects the robot's color from the shared palette
            # so its 3D marker matches its row color in the RViz panel table.
            'robot_index': int(index),
        })
        nodes.append(
            ComposableNode(
                package='hospital_robot',
                plugin='hospital_robot::RobotServerNode',
                name=f'{robot_id}_server',
                parameters=[per_robot],
            )
        )

    container = ComposableNodeContainer(
        name='hospital_robots_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=nodes,
    )
    return [container]


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory('hospital_bringup'),
        'config', 'hospital_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML parameter file providing the shared robot params.'),
        OpaqueFunction(function=_launch_setup),
    ])
