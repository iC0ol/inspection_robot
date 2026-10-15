
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

"""Integration test for navigation retry and mission completion."""

import unittest

import launch
import launch_testing.actions
import pytest
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    """Launch a mock Nav2 server and the mission manager."""
    mock_server = Node(
        package='inspection_robot_mission',
        executable='mock_nav2_server',
        name='mock_nav2_server',
        output='screen',
        parameters=[{
            'success_on_attempt': 2,
            'wait_for_cancel': False,
        }],
    )

    mission_manager = Node(
        package='inspection_robot_mission',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'waypoints_x': [0.0],
            'waypoints_y': [0.0],
            'waypoints_yaw': [0.0],
            'wait_seconds': 0.0,
            'max_navigation_retries': 1,
            'retry_delay_seconds': 0.2,
            'retry_exhausted_policy': 'FAIL_MISSION',
        }],
    )

    return (
        launch.LaunchDescription([
            mock_server,
            mission_manager,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'mock_server': mock_server,
            'mission_manager': mission_manager,
        },
    )


class TestMissionRetry(unittest.TestCase):
    """Check that navigation retries succeed end to end."""

    def test_retry_then_success(
        self,
        proc_output,
        proc_info,
        mock_server,
        mission_manager,
    ):
        """Verify the first goal fails and the next succeeds."""
        proc_output.assertWaitFor(
            'Goal 1 -> ABORTED',
            process=mock_server,
            timeout=30,
        )

        proc_output.assertWaitFor(
            'Goal 2 -> SUCCEEDED',
            process=mock_server,
            timeout=30,
        )

        proc_output.assertWaitFor(
            'P1 第 1 次导航失败',
            process=mission_manager,
            timeout=30,
        )

        proc_output.assertWaitFor(
            'Mission: SUCCEEDED',
            process=mission_manager,
            timeout=30,
        )

        proc_info.assertWaitForShutdown(
            process=mission_manager,
            timeout=10,
        )
