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
Gazebo test of the hospital behavior tree.

Starts the hospital world (my_bot/launch_sim.launch.py), RViz, AMCL on the
hospital map, Nav2 with the hospital behavior tree (navigation.launch.py), the
route_manager and the MissionManagerNode.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Gazebo needs a few seconds before /clock, the lidar and /tf exist; earlier gives TF errors.
NAV_START_DELAY_S = 8.0


def _include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=arguments.items())


def generate_launch_description():
    my_bot = get_package_share_directory('my_bot')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='/ros2_ws/maps/ziekenhuis.yaml',
            description='Map for AMCL'),
        DeclareLaunchArgument(
            'timing_file',
            default_value=os.path.join(
                get_package_share_directory('hospital_robot_bringup'), 'config', 'timing.yaml'),
            description='Timing parameters (navigation.*)'),
        DeclareLaunchArgument(
            'routes_file',
            # Source tree, not install/: routes saved with route_draft/save must survive a build.
            default_value='/ros2_ws/src/navigatie_ezwheels/config/routes.yaml',
            description='Routes file of the route_manager (read and written)'),

        _include('my_bot', 'launch_sim.launch.py', {}),

        Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', os.path.join(my_bot, 'config', 'setup.rviz')],
            parameters=[{'use_sim_time': True}]),

        TimerAction(period=NAV_START_DELAY_S, actions=[
            _include('my_bot', 'localization_launch.py', {
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'true',
            }),
            _include('hospital_robot_bringup', 'navigation.launch.py', {
                'timing_file': LaunchConfiguration('timing_file'),
                'use_sim_time': 'true',
            }),
            Node(
                package='my_bot', executable='RouteManagerServer.py', name='route_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'routes_file': LaunchConfiguration('routes_file'),
                }]),
            Node(
                package='my_bot', executable='MissionManagerNode.py', name='mission_manager',
                output='screen', parameters=[{'use_sim_time': True}]),
        ]),
    ])
