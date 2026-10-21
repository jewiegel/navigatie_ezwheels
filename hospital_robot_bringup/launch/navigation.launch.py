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
Start Nav2 (my_bot/navigation_launch.py) with the hospital behavior tree.

Fills the hospital tree templates (NavigateThroughPoses and NavigateToPose) from
config/timing.yaml, adds the hospital BT plugins to plugin_lib_names and sets the
progress checker timeout.
"""

import os

from ament_index_python.packages import get_package_share_directory
from hospital_robot_bringup.bt_template import (
    load_navigation_timing, nav2_params_with_hospital_bt, render_bt, write_temp,
    write_temp_yaml)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml

HOSPITAL_BT_PLUGINS = [
    'hospital_robot_publish_blocked_bt_node',
    'hospital_robot_is_path_blocked_bt_node',
    'hospital_robot_get_next_waypoint_bt_node',
    'hospital_robot_pop_waypoint_bt_node',
]

# bt_navigator parameter -> template in hospital_robot_navigation/behavior_trees
HOSPITAL_TREES = {
    'default_nav_through_poses_bt_xml': 'hospital_navigate.xml.in',
    'default_nav_to_pose_bt_xml': 'hospital_navigate_to_pose.xml.in',
}


def _prepare_nav2(context):
    timing = load_navigation_timing(LaunchConfiguration('timing_file').perform(context))

    tree_dir = os.path.join(
        get_package_share_directory('hospital_robot_navigation'), 'behavior_trees')
    bt_files = {}
    for param, template in HOSPITAL_TREES.items():
        with open(os.path.join(tree_dir, template)) as f:
            bt_xml = render_bt(f.read(), timing)
        bt_files[param] = write_temp(bt_xml, template.replace('.xml.in', '_'), '.xml')

    with open(LaunchConfiguration('params_file').perform(context)) as f:
        base_params = yaml.safe_load(f)
    params = nav2_params_with_hospital_bt(
        base_params, bt_files, HOSPITAL_BT_PLUGINS, timing['progress_timeout_s'])
    params_file = write_temp_yaml(params, 'hospital_nav2_params_')

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('my_bot'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'timing_file',
            default_value=os.path.join(
                get_package_share_directory('hospital_robot_bringup'), 'config', 'timing.yaml'),
            description='Timing parameters (navigation.*)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('my_bot'), 'config', 'nav2_params.yaml'),
            description='Base Nav2 parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use the simulation clock'),
        OpaqueFunction(function=_prepare_nav2),
    ])
