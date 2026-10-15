
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

"""Integration test for navigation retry exhaustion."""

import unittest

import launch
import launch_testing.actions
import pytest
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    """Launch a mock server that always aborts navigation."""
    mock_server = Node(
        package='inspection_robot_mission',
        executable='mock_nav2_server',
        name='mock_nav2_server',
        output='screen',
        parameters=[{
            'success_on_attempt': 0,
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
            'max_navigation_retries': 2,
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


class TestMissionRetryExhaustion(unittest.TestCase):
    """Verify that exhausted retries terminate the mission."""

    def test_retry_exhaustion(
        self,
        proc_output,
        proc_info,
        mock_server,
        mission_manager,
    ):
        """Verify three aborted goals and final mission failure."""
        # 第一次正常导航失败
        proc_output.assertWaitFor(
            'Goal 1 -> ABORTED',
            process=mock_server,
            timeout=30,
        )

        # 第一次额外重试失败
        proc_output.assertWaitFor(
            'Goal 2 -> ABORTED',
            process=mock_server,
            timeout=30,
        )

        # 第二次额外重试失败
        proc_output.assertWaitFor(
            'Goal 3 -> ABORTED',
            process=mock_server,
            timeout=30,
        )

        # Mission Manager 正确识别重试耗尽
        proc_output.assertWaitFor(
            'P1 导航重试耗尽，累计失败 3 次',
            process=mission_manager,
            timeout=30,
        )

        proc_output.assertWaitFor(
            'Mission: FAILED',
            process=mission_manager,
            timeout=30,
        )

        # 任务失败后，Manager 必须自动退出
        proc_info.assertWaitForShutdown(
            process=mission_manager,
            timeout=10,
        )

        # 读取 Mock Server 的全部已捕获输出
        server_output = ''.join(
            event.text.decode('utf-8', errors='replace')
            for event in proc_output[mock_server]
        )

        # 必须恰好收到三个导航请求
        self.assertEqual(
            server_output.count('次导航请求'),
            3,
        )

        # 不允许出现第四次导航
        self.assertNotIn(
            '收到第 4 次导航请求',
            server_output,
        )
