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

"""Fill the hospital BT template and check it against the registered BT plugins."""

import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from hospital_robot_bringup.bt_template import (
    load_navigation_timing, nav2_params_with_hospital_bt, render_bt)
import pytest
import yaml

TEST_VALUES = {'progress_timeout_s': 12.5, 'blockade_wait_s': 7.5}
HOSPITAL_PLUGINS = [
    'hospital_robot_publish_blocked_bt_node',
    'hospital_robot_is_path_blocked_bt_node',
    'hospital_robot_get_next_waypoint_bt_node',
    'hospital_robot_pop_waypoint_bt_node',
]
GENERIC_ATTRIBUTES = {'name', 'ID'}
# Built-in BehaviorTree.CPP nodes are not in the plugin model; their ports used in the tree:
BUILTIN_PORTS = {'RetryUntilSuccessful': {'num_attempts'}}


TEMPLATES = ['hospital_navigate.xml.in', 'hospital_navigate_to_pose.xml.in']


def _template(name='hospital_navigate.xml.in'):
    path = os.path.join(
        get_package_share_directory('hospital_robot_navigation'), 'behavior_trees', name)
    with open(path) as f:
        return f.read()


NAV2_PARAM_FILES = ['nav2_params.yaml', 'nav2_params_robot.yaml']   # simulation, real robot


def _nav2_params(name='nav2_params.yaml'):
    path = os.path.join(get_package_share_directory('my_bot'), 'config', name)
    with open(path) as f:
        return yaml.safe_load(f)


@pytest.mark.parametrize('template', TEMPLATES)
def test_render_fills_all_placeholders(template):
    rendered = render_bt(_template(template), TEST_VALUES)
    assert '${' not in rendered
    root = ET.fromstring(rendered)
    assert root.find('.//Wait').get('wait_duration') == '7.5'
    # Never gives up: every retry loop is infinite.
    assert {n.get('num_attempts') for n in root.iter('RetryUntilSuccessful')} == {'-1'}


def test_render_fails_on_missing_key():
    with pytest.raises(KeyError, match='blockade_wait_s'):
        render_bt(_template(), {'progress_timeout_s': 1.0})


def test_render_fails_on_leftover_placeholder():
    with pytest.raises(ValueError, match='line 2'):
        render_bt('<root>\n<Wait wait_duration="${not a key}"/></root>', TEST_VALUES)


def test_timing_file_missing_key(tmp_path):
    timing = tmp_path / 'timing.yaml'
    timing.write_text('navigation:\n  progress_timeout_s: 1.0\n')
    with pytest.raises(KeyError, match='blockade_wait_s'):
        load_navigation_timing(str(timing))


@pytest.mark.parametrize('template', TEMPLATES)
def test_installed_timing_file_is_complete(template):
    timing = load_navigation_timing(os.path.join(
        get_package_share_directory('hospital_robot_bringup'), 'config', 'timing.yaml'))
    render_bt(_template(template), timing)


@pytest.mark.parametrize('param_file', NAV2_PARAM_FILES)
def test_nav2_params_get_tree_plugins_and_timeout(param_file):
    base = _nav2_params(param_file)
    trees = {
        'default_nav_through_poses_bt_xml': '/tmp/through.xml',
        'default_nav_to_pose_bt_xml': '/tmp/to_pose.xml',
    }
    params = nav2_params_with_hospital_bt(base, trees, HOSPITAL_PLUGINS, 12.5)
    bt = params['bt_navigator']['ros__parameters']
    base_libs = base['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert bt['default_nav_through_poses_bt_xml'] == '/tmp/through.xml'
    assert bt['default_nav_to_pose_bt_xml'] == '/tmp/to_pose.xml'
    assert bt['plugin_lib_names'] == base_libs + HOSPITAL_PLUGINS
    checker = params['controller_server']['ros__parameters']['progress_checker']
    assert checker['movement_time_allowance'] == 12.5
    assert base['bt_navigator']['ros__parameters'] is not bt   # base left untouched


@pytest.mark.parametrize('param_file', NAV2_PARAM_FILES)
@pytest.mark.parametrize('template', TEMPLATES)
def test_tree_uses_only_registered_nodes_and_ports(tmp_path, template, param_file):
    tree_file = tmp_path / 'tree.xml'
    tree_file.write_text(render_bt(_template(template), TEST_VALUES))
    libs = _nav2_params(param_file)['bt_navigator']['ros__parameters']['plugin_lib_names']
    checker = os.path.join(
        get_package_prefix('hospital_robot_navigation'),
        'lib', 'hospital_robot_navigation', 'check_bt_xml')

    result = subprocess.run(
        [checker, str(tree_file)] + libs + HOSPITAL_PLUGINS,
        capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr

    # Node model of all plugin node types: check every attribute is a declared port.
    model = ET.fromstring(result.stdout)
    ports = {
        node.get('ID'): {port.get('name') for port in node}
        for node in model.find('TreeNodesModel')}
    tree = ET.parse(tree_file).getroot().find('BehaviorTree')
    for element in tree.iter():
        if element is tree:
            continue
        known = ports.get(element.tag, BUILTIN_PORTS.get(element.tag, set()))
        unknown = set(element.attrib) - GENERIC_ATTRIBUTES - known
        assert not unknown, f'<{element.tag}> has unknown port(s): {sorted(unknown)}'
