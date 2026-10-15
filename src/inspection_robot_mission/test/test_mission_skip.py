
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

"""Integration test for skipping an unreachable waypoint."""

import unittest

import launch
import launch_testing.actions
import pytest
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    """Launch a mission where P1 fails and P2 succeeds."""
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
            'waypoints_x': [0.0, 1.0],
            'waypoints_y': [0.0, 0.0],
            'waypoints_yaw': [0.0, 0.0],
            'wait_seconds': 0.0,
            'max_navigation_retries': 0,
            'retry_delay_seconds': 0.2,
            'retry_exhausted_policy': 'SKIP_WAYPOINT',
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


class TestMissionSkip(unittest.TestCase):
    """Verify waypoint skipping and partial mission completion."""

    def test_skip_then_success(
        self,
        proc_output,
        proc_info,
        mock_server,
        mission_manager,
    ):
        """Verify P1 is skipped and P2 completes successfully."""
        # 第一次导航失败
        proc_output.assertWaitFor(
            'Goal 1 -> ABORTED',
            process=mock_server,
            timeout=30,
        )

        # P1 应被跳过
        proc_output.assertWaitFor(
            'P1 重试耗尽，已跳过当前站点',
            process=mission_manager,
            timeout=30,
        )

        # Manager 自动派发 P2
        proc_output.assertWaitFor(
            '正在发送 P2 导航目标',
            process=mission_manager,
            timeout=30,
        )

        # 第二次导航成功
        proc_output.assertWaitFor(
            'Goal 2 -> SUCCEEDED',
            process=mock_server,
            timeout=30,
        )

        proc_output.assertWaitFor(
            'P2: SUCCEEDED',
            process=mission_manager,
            timeout=30,
        )

        # 存在跳过站点，最终不能报告全部成功
        proc_output.assertWaitFor(
            'Mission: COMPLETED_WITH_SKIPS',
            process=mission_manager,
            timeout=30,
        )

        # Manager 应当正常退出
        proc_info.assertWaitForShutdown(
            process=mission_manager,
            timeout=10,
        )

        # 检查服务器实际收到的 Goal 数量
        server_output = ''.join(
            event.text.decode('utf-8', errors='replace')
            for event in proc_output[mock_server]
        )

        self.assertEqual(
            server_output.count('次导航请求'),
            2,
        )

        self.assertNotIn(
            '收到第 3 次导航请求',
            server_output,
        )

        # 防止 Mission 被错误地标记为全部成功
        manager_output = ''.join(
            event.text.decode('utf-8', errors='replace')
            for event in proc_output[mission_manager]
        )

        self.assertNotIn(
            'Mission: SUCCEEDED',
            manager_output,
        )
