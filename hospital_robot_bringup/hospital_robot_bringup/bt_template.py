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

"""Fill the hospital behavior tree template and the Nav2 parameters at launch."""

import copy
import re
import tempfile

import yaml

PLACEHOLDER = re.compile(r'\$\{(\w+)\}')
NAVIGATION_KEYS = ('progress_timeout_s', 'blockade_wait_s')


def load_navigation_timing(timing_file):
    """Return the 'navigation' section of timing.yaml; fail if a key is missing."""
    with open(timing_file) as f:
        data = yaml.safe_load(f) or {}
    navigation = data.get('navigation')
    if not isinstance(navigation, dict):
        raise KeyError(f"{timing_file}: missing section 'navigation'")
    missing = [key for key in NAVIGATION_KEYS if key not in navigation]
    if missing:
        raise KeyError(f"{timing_file}: missing key(s) in 'navigation': {', '.join(missing)}")
    return navigation


def render_bt(template_text, values):
    """Replace every ${key} with values[key]; fail on unknown keys or leftovers."""
    def substitute(match):
        key = match.group(1)
        if key not in values:
            raise KeyError(f"Behavior tree template uses '${{{key}}}' but no value was given")
        return str(values[key])

    rendered = PLACEHOLDER.sub(substitute, template_text)
    if '${' in rendered:
        line = next(n for n, text in enumerate(rendered.splitlines(), 1) if '${' in text)
        raise ValueError(f"Unfilled '${{' left in behavior tree at line {line}")
    return rendered


def nav2_params_with_hospital_bt(base_params, bt_xml_files, plugin_libs, progress_timeout_s):
    """
    Return a copy of the Nav2 parameter dict with the hospital trees and plugins.

    bt_xml_files maps bt_navigator parameters (e.g. default_nav_to_pose_bt_xml) to tree files.
    """
    params = copy.deepcopy(base_params)
    try:
        bt_navigator = params['bt_navigator']['ros__parameters']
        progress_checker = params['controller_server']['ros__parameters']['progress_checker']
    except KeyError as e:
        raise KeyError(f'Nav2 parameter file has no {e} section') from None

    bt_navigator.update(bt_xml_files)
    # Iron: plugin_lib_names replaces Nav2's default list, so append to the full list.
    libs = list(bt_navigator.get('plugin_lib_names', []))
    if not libs:
        raise KeyError('bt_navigator.plugin_lib_names is empty; Iron needs the full Nav2 list')
    bt_navigator['plugin_lib_names'] = libs + [lib for lib in plugin_libs if lib not in libs]
    progress_checker['movement_time_allowance'] = float(progress_timeout_s)
    return params


def write_temp(text, prefix, suffix):
    """Write text to a new temporary file and return its path."""
    with tempfile.NamedTemporaryFile(
            'w', prefix=prefix, suffix=suffix, delete=False) as f:
        f.write(text)
        return f.name


def write_temp_yaml(data, prefix):
    """Write a dict as YAML to a new temporary file and return its path."""
    return write_temp(yaml.safe_dump(data, sort_keys=False), prefix, '.yaml')
