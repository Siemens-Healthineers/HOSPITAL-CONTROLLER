# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Top-level bringup for the hospital project.

Starts the simulation controllers (composed in one process), the robot fleet,
the activity logger and RViz with the hospital panel with a single command::

    ros2 launch hospital_bringup bringup.launch.py

Common overrides::

    ros2 launch hospital_bringup bringup.launch.py use_sim:=false       # no controllers
    ros2 launch hospital_bringup bringup.launch.py use_rviz:=false      # no RViz panel
    ros2 launch hospital_bringup bringup.launch.py params_file:=/path/to/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('hospital_bringup')
    default_params = os.path.join(bringup_share, 'config', 'hospital_params.yaml')

    sim_launch = os.path.join(
        get_package_share_directory('hospital_controller'), 'launch', 'controller.launch.py')

    rviz_launch = os.path.join(
        get_package_share_directory('hospital_rviz_panel'), 'launch', 'rviz.launch.py')

    viz_launch = os.path.join(
        get_package_share_directory('hospital_arena'), 'launch', 'viz_bridge.launch.py')

    robots_launch = os.path.join(
        get_package_share_directory('hospital_bringup'), 'launch', 'robots.launch.py')

    logging_launch = os.path.join(
        get_package_share_directory('hospital_bringup'), 'launch', 'logging.launch.py')

    params_file = LaunchConfiguration('params_file')
    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robots = LaunchConfiguration('use_robots')
    use_logging = LaunchConfiguration('use_logging')
    description_package = LaunchConfiguration('description_package')

    return LaunchDescription([
        DeclareLaunchArgument(
            'description_package', default_value='hospital_description',
            description='Package that provides urdf/hospital.urdf.xacro and the '
                        'meshes.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML parameter file shared by the sim and clients.'),
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='Start the C++ simulation controllers.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the hospital control panel.'),
        DeclareLaunchArgument(
            'use_robots', default_value='true',
            description='Start the fleet of robot task servers.'),
        DeclareLaunchArgument(
            'use_logging', default_value='true',
            description='Start the room-dwell + task activity logger.'),

        # Simulation controllers, composed into a single container process.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={'params_file': params_file}.items(),
            condition=IfCondition(use_sim),
        ),

        # Fleet of robot task servers. Each robot runs its own ExecuteTask
        # action server and executes missions concurrently with the others.
        # robot_server_node.robot_ids configures the fleet, logger, and panel.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robots_launch),
            launch_arguments={'params_file': params_file}.items(),
            condition=IfCondition(use_robots),
        ),

        # Room-dwell + task activity logger. Robot-agnostic: it infers room
        # occupancy from each robot's pose and records durations to a CSV when
        # recording is toggled from the RViz panel.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(logging_launch),
            launch_arguments={'params_file': params_file}.items(),
            condition=IfCondition(use_logging),
        ),

        # RViz with the custom control panel. A pure ROS 2 client that runs
        # against the same controllers. Reads its device IDs from the
        # ``rviz2:`` section of the same params file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            launch_arguments={'params_file': params_file}.items(),
            condition=IfCondition(use_rviz),
        ),

        # Marker bridge that renders the abstract device state as a 3D scene in
        # RViz. Started together with RViz so the panel and the scene share one
        # window. Reads its geometry params from the same params file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viz_launch),
            launch_arguments={
                'params_file': params_file,
                'description_package': description_package,
            }.items(),
            condition=IfCondition(use_rviz),
        ),
    ])
