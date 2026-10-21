# Copyright 2026 Improvia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Real robot: hospital navigation stack without Gazebo and RViz.

Starts AMCL on the given map, Nav2 with the hospital behavior trees
(navigation.launch.py, config/nav2_params_robot.yaml), the route_manager, the
MissionManagerNode and optionally the foxglove_bridge.

Expects the robot platform (EZ-Wheel / EZ-Way) to provide the hardware side:
lidar on /laser_1/scan_filtered, /odom, TF odom -> base_link -> laser_1 and
a driver listening on /cmd_vel. The EZ-Way navigation ("automatic mode") must not
run at the same time: it cannot load the hospital BT plugins.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=arguments.items())


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('my_bot'), 'config', 'nav2_params_robot.yaml')
    return LaunchDescription([
        # No default on purpose: the robot must use a map of the real building.
        DeclareLaunchArgument('map', description='Map yaml of the real building for AMCL'),
        DeclareLaunchArgument(
            'routes_file',
            default_value=os.path.join(
                get_package_share_directory('my_bot'), 'config', 'routes.yaml'),
            description='Routes file of the route_manager (read and written)'),
        DeclareLaunchArgument(
            'timing_file',
            default_value=os.path.join(
                get_package_share_directory('hospital_robot_bringup'), 'config', 'timing.yaml'),
            description='Timing parameters (navigation.*)'),
        DeclareLaunchArgument(
            'foxglove', default_value='true', description='Start foxglove_bridge on port 8765'),

        _include('my_bot', 'localization_launch.py', {
            'map': LaunchConfiguration('map'),
            'params_file': params_file,
            'use_sim_time': 'false',
        }),
        _include('hospital_robot_bringup', 'navigation.launch.py', {
            'params_file': params_file,
            'timing_file': LaunchConfiguration('timing_file'),
            'use_sim_time': 'false',
        }),
        Node(
            package='my_bot', executable='RouteManagerServer.py', name='route_manager',
            output='screen', parameters=[{'routes_file': LaunchConfiguration('routes_file')}]),
        Node(
            package='my_bot', executable='MissionManagerNode.py', name='mission_manager',
            output='screen'),
        Node(
            package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
            output='log', parameters=[{'port': 8765}],
            condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
