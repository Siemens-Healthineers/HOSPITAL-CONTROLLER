# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Standalone launch for the robot simulation node.

Normally the robot fleet is started by ``hospital_bringup/robots.launch.py``.
This standalone launch is handy for running or debugging the robot on its own::

    ros2 launch hospital_robot robot.launch.py
    ros2 launch hospital_robot robot.launch.py params_file:=/path/to/params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    # An empty params_file means "use the node's compiled defaults".
    common_params = [params_file] if params_file else []

    container = ComposableNodeContainer(
        name='hospital_robot_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='hospital_robot',
                plugin='hospital_robot::RobotServerNode',
                name='robot_server_node',
                parameters=common_params,
            ),
        ],
    )
    return [container]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Optional YAML parameter file for the robot node.'),
        OpaqueFunction(function=_launch_setup),
    ])
