
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

"""Integration test for canceling an active inspection mission."""

import unittest

import os
import launch
import launch_testing.actions
import pytest
import rclpy
from launch_ros.actions import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    """Launch the mock server and an active mission manager."""
    mock_server = Node(
        package='inspection_robot_mission',
        executable='mock_nav2_server',
        name='mock_nav2_server',
        output='screen',
        parameters=[{
            'wait_for_cancel': True,
            'reject_cancel': False,
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


class TestMissionCancel(unittest.TestCase):
    """Verify cancellation through a real ROS2 service call."""

    def test_cancel_active_navigation(
        self,
        proc_output,
        proc_info,
        mock_server,
        mission_manager,
    ):
        """Cancel P1 and verify that P2 is never dispatched."""
        # 1. 确认 P1 已进入导航阶段
        proc_output.assertWaitFor(
            'P1: NAVIGATING',
            process=mission_manager,
            timeout=30,
        )

        # 2. 创建独立 ROS2 Context 和 Service Client
        context = Context()
        rclpy.init(
            context=context,
            domain_id=int(os.environ['ROS_DOMAIN_ID']),
        )

        node = rclpy.create_node(
            'mission_cancel_test_client',
            context=context,
        )

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        try:
            client = node.create_client(
                Trigger,
                '/mission/cancel',
            )

            self.assertTrue(
                client.wait_for_service(timeout_sec=15.0),
                '取消服务未就绪',
            )

            # 3. 发送取消请求
            future = client.call_async(Trigger.Request())

            executor.spin_until_future_complete(
                future,
                timeout_sec=15.0,
            )

            self.assertTrue(
                future.done(),
                '取消服务响应超时',
            )

            response = future.result()

            self.assertIsNotNone(response)
            self.assertTrue(
                response.success,
                response.message,
            )

            self.assertIn(
                '取消请求已受理',
                response.message,
            )

        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

        # 4. 验证 Mock Server 接受取消请求
        proc_output.assertWaitFor(
            'Mock Server 接受 Cancel 请求',
            process=mock_server,
            timeout=30,
        )

        # 5. 验证 Action 最终状态
        proc_output.assertWaitFor(
            'Goal 1 -> CANCELED',
            process=mock_server,
            timeout=30,
        )

        proc_output.assertWaitFor(
            'Nav2 Goal 已确认取消',
            process=mission_manager,
            timeout=30,
        )

        # 6. 验证 Mission 最终状态
        proc_output.assertWaitFor(
            'Mission: CANCELED',
            process=mission_manager,
            timeout=30,
        )

        # 7. 验证 Mission Manager 自动退出
        proc_info.assertWaitForShutdown(
            process=mission_manager,
            timeout=10,
        )

        # 8. 确认没有执行 P2
        manager_output = ''.join(
            event.text.decode('utf-8', errors='replace')
            for event in proc_output[mission_manager]
        )

        self.assertNotIn(
            '正在发送 P2 导航目标',
            manager_output,
        )
