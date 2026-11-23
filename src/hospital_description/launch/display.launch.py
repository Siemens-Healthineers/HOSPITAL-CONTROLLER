# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Display the hospital description in RViz.

Runs robot_state_publisher with the xacro-expanded URDF, a
joint_state_publisher_gui (so the articulated joints added in Phase 2 get
sliders), and RViz2 with a preset config.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hospital_description')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(pkg_share, 'urdf', 'hospital.urdf.xacro'),
        description='Absolute path to the robot xacro/urdf file.',
    )

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(pkg_share, 'rviz', 'hospital.rviz'),
        description='Absolute path to the RViz config file.',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Start joint_state_publisher_gui (needed once joints are '
        'articulated in Phase 2). Requires ros-<distro>-joint-state-publisher-gui.',
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', LaunchConfiguration('model')]), value_type=str
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            gui_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )
