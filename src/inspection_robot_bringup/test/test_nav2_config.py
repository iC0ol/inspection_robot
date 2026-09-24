# Copyright 2026 inspection_robot contributors
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

"""Validate the critical Nav2 configuration used by inspection_robot."""

import ast
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / 'config'


def load_yaml(filename):
    """Load a YAML configuration file from the package."""
    path = CONFIG_DIR / filename

    assert path.is_file(), f'Missing configuration file: {path}'

    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def assert_robot_footprint(costmap_params):
    """Check the frozen robot footprint dimensions."""
    footprint = costmap_params['footprint']

    if isinstance(footprint, str):
        footprint = ast.literal_eval(footprint)

    assert len(footprint) == 4

    xs = [point[0] for point in footprint]
    ys = [point[1] for point in footprint]

    assert max(xs) == pytest.approx(0.12)
    assert min(xs) == pytest.approx(-0.20)
    assert max(ys) == pytest.approx(0.12)
    assert min(ys) == pytest.approx(-0.12)

    assert costmap_params['footprint_padding'] == pytest.approx(0.01)


def test_amcl_frames():
    """AMCL must maintain the map-to-odom localization transform."""
    config = load_yaml('amcl.yaml')
    params = config['amcl']['ros__parameters']

    assert params['global_frame_id'] == 'map'
    assert params['odom_frame_id'] == 'odom'
    assert params['base_frame_id'] == 'base_footprint'
    assert params['tf_broadcast'] is True


def test_planner_server():
    """Planner Server must use the NavFn GridBased planner."""
    config = load_yaml('planner.yaml')
    params = config['planner_server']['ros__parameters']

    assert 'GridBased' in params['planner_plugins']

    grid_based = params['GridBased']

    assert grid_based['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert grid_based['use_astar'] is False
    assert grid_based['allow_unknown'] is True


def test_global_costmap():
    """Global Costmap must remain map based and non-rolling."""
    config = load_yaml('planner.yaml')

    params = (
        config['global_costmap']
        ['global_costmap']
        ['ros__parameters']
    )

    assert params['global_frame'] == 'map'
    assert params['robot_base_frame'] == 'base_footprint'
    assert params['rolling_window'] is False
    assert params['resolution'] == pytest.approx(0.05)

    assert params['plugins'] == [
        'static_layer',
        'obstacle_layer',
        'inflation_layer',
    ]

    assert (
        params['static_layer']['plugin']
        == 'nav2_costmap_2d::StaticLayer'
    )

    assert params['static_layer']['map_subscribe_transient_local'] is True

    assert (
        params['obstacle_layer']['plugin']
        == 'nav2_costmap_2d::ObstacleLayer'
    )

    assert (
        params['inflation_layer']['plugin']
        == 'nav2_costmap_2d::InflationLayer'
    )

    assert_robot_footprint(params)


def test_controller_server():
    """Controller Server must use DWB and EKF odometry."""
    config = load_yaml('controller.yaml')
    params = config['controller_server']['ros__parameters']

    assert params['odom_topic'] == '/odometry/filtered'
    assert 'FollowPath' in params['controller_plugins']

    follow_path = params['FollowPath']

    assert follow_path['plugin'] == 'dwb_core::DWBLocalPlanner'

    # Differential-drive robots cannot move sideways.
    assert follow_path['min_vel_y'] == pytest.approx(0.0)
    assert follow_path['max_vel_y'] == pytest.approx(0.0)

    # Keep the first validated M5 speed limits conservative.
    assert follow_path['max_vel_x'] == pytest.approx(0.20)
    assert follow_path['max_vel_theta'] == pytest.approx(0.8)


def test_local_costmap():
    """Local Costmap must remain a 3 m rolling window in odom."""
    config = load_yaml('controller.yaml')

    params = (
        config['local_costmap']
        ['local_costmap']
        ['ros__parameters']
    )

    assert params['global_frame'] == 'odom'
    assert params['robot_base_frame'] == 'base_footprint'

    assert params['rolling_window'] is True

    assert params['width'] == 3
    assert params['height'] == 3
    assert params['resolution'] == pytest.approx(0.05)

    assert params['plugins'] == [
        'obstacle_layer',
        'inflation_layer',
    ]

    assert (
        params['obstacle_layer']['plugin']
        == 'nav2_costmap_2d::ObstacleLayer'
    )

    assert (
        params['inflation_layer']['plugin']
        == 'nav2_costmap_2d::InflationLayer'
    )

    assert_robot_footprint(params)


def test_behavior_server():
    """Behavior Server must use the local odom coordinate frame."""
    config = load_yaml('behavior.yaml')
    params = config['behavior_server']['ros__parameters']

    assert params['global_frame'] == 'odom'
    assert params['robot_base_frame'] == 'base_footprint'

    plugins = params['behavior_plugins']

    assert 'spin' in plugins
    assert 'backup' in plugins
    assert 'wait' in plugins


def test_bt_navigator():
    """BT Navigator must expose the validated navigation interfaces."""
    config = load_yaml('bt_navigator.yaml')
    params = config['bt_navigator']['ros__parameters']

    assert params['global_frame'] == 'map'
    assert params['robot_base_frame'] == 'base_footprint'
    assert params['odom_topic'] == '/odometry/filtered'

    plugin_libs = params['plugin_lib_names']

    required_plugins = [
        'nav2_compute_path_to_pose_action_bt_node',
        'nav2_follow_path_action_bt_node',
        'nav2_clear_costmap_service_bt_node',
        'nav2_recovery_node_bt_node',
        'nav2_pipeline_sequence_bt_node',
        'nav2_round_robin_node_bt_node',
        'nav2_spin_action_bt_node',
        'nav2_wait_action_bt_node',
        'nav2_back_up_action_bt_node',
    ]

    for plugin in required_plugins:
        assert plugin in plugin_libs
