# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Launch the hospital 3-D model bridge.

Starts ``robot_state_publisher`` with the xacro-expanded URDF from
the selected description package and ``joint_bridge_node``, which converts the abstract
device telemetry (door / CT table / CT scanner state) into a
``sensor_msgs/JointState`` on ``/joint_states``. Together they render a live,
controllable 3-D RViz scene (a RobotModel display) that mirrors the controllers.

Run on its own, then open RViz (e.g. ``hospital_rviz_panel/rviz.launch.py``) to
see the model, or include it from the top-level bringup::

    ros2 launch hospital_arena viz_bridge.launch.py
    ros2 launch hospital_arena viz_bridge.launch.py params_file:=/path/to/my.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    parameters = [params_file] if params_file else []

    model = LaunchConfiguration('model').perform(context)
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', model]), value_type=str
        )
    }

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='hospital_arena',
            executable='joint_bridge_node',
            name='joint_bridge_node',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_model = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration('description_package')),
        'urdf', 'hospital.urdf.xacro',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'description_package', default_value='hospital_description',
            description='Package that provides urdf/hospital.urdf.xacro and the '
                        'meshes.'),
        DeclareLaunchArgument(
            'model', default_value=default_model,
            description='Absolute path to the robot xacro/urdf file.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Optional YAML params with a joint_bridge_node section '
                        '(publish_rate).'),
        OpaqueFunction(function=_launch_setup),
    ])
