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

"""Tests for the mobile-base controller configuration."""

from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONTROLLERS_FILE = PACKAGE_DIR / "config" / "controllers.yaml"


def load_controller_config():
    """Load the ros2_control controller configuration."""
    with CONTROLLERS_FILE.open(encoding="utf-8") as file:
        return yaml.safe_load(file)


def test_required_controllers_are_configured():
    """Required mobile-base controllers must be declared."""
    config = load_controller_config()

    manager = config["controller_manager"]["ros__parameters"]

    assert (
        manager["joint_state_broadcaster"]["type"]
        == "joint_state_broadcaster/JointStateBroadcaster"
    )

    assert (
        manager["diff_drive_controller"]["type"]
        == "diff_drive_controller/DiffDriveController"
    )


def test_diff_drive_geometry_matches_robot_description():
    """Diff-drive geometry must match the current robot model."""
    config = load_controller_config()

    params = config["diff_drive_controller"]["ros__parameters"]

    assert params["left_wheel_names"] == ["left_wheel_joint"]
    assert params["right_wheel_names"] == ["right_wheel_joint"]

    assert params["wheel_radius"] == 0.05
    assert params["wheel_separation"] == 0.28
    assert params["wheels_per_side"] == 1


def test_diff_drive_odometry_policy():
    """Odometry configuration must match the planned EKF architecture."""
    config = load_controller_config()

    params = config["diff_drive_controller"]["ros__parameters"]

    assert params["odom_frame_id"] == "odom"
    assert params["base_frame_id"] == "base_footprint"

    assert params["position_feedback"] is True
    assert params["open_loop"] is False

    # robot_localization will become the sole publisher of
    # odom -> base_footprint in the next milestone.
    assert params["enable_odom_tf"] is False
