
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

"""Integration test for rejected navigation cancellation."""

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
    """Launch a server that rejects cancel requests."""
    mock_server = Node(
        package='inspection_robot_mission',
        executable='mock_nav2_server',
        name='mock_nav2_server',
        output='screen',
        parameters=[{
            'wait_for_cancel': True,
            'reject_cancel': True,
            'finish_after_seconds': 20.0,
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


class TestMissionCancelRejected(unittest.TestCase):
    """Verify behavior when navigation cancellation is refused."""

    def test_cancel_rejected(
        self,
        proc_output,
        proc_info,
        mock_server,
        mission_manager,
    ):
        """Keep the mission pending until the goal actually ends."""
        # 1. 确认 P1 已进入导航阶段
        proc_output.assertWaitFor(
            'P1: NAVIGATING',
            process=mission_manager,
            timeout=30,
        )

        # 2. 创建测试客户端，连接 Domain 177
        context = Context()
        rclpy.init(
            context=context,
            domain_id=int(os.environ['ROS_DOMAIN_ID']),
        )

        node = rclpy.create_node(
            'mission_cancel_rejected_test_client',
            context=context,
        )

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        try:
            cancel_client = node.create_client(
                Trigger, '/mission/cancel'
            )

            status_client = node.create_client(
                Trigger, '/mission/status'
            )

            self.assertTrue(
                cancel_client.wait_for_service(
                    timeout_sec=10.0
                )
            )

            self.assertTrue(
                status_client.wait_for_service(
                    timeout_sec=10.0
                )
            )

            # 3. 主动请求取消
            cancel_future = cancel_client.call_async(
                Trigger.Request()
            )

            executor.spin_until_future_complete(
                cancel_future,
                timeout_sec=10.0,
            )

            self.assertTrue(cancel_future.done())
            cancel_response = cancel_future.result()

            self.assertIsNotNone(cancel_response)
            self.assertTrue(cancel_response.success)

            self.assertIn(
                '取消请求已受理',
                cancel_response.message,
            )

            # 4. 等待 Mock Server 明确拒绝取消
            proc_output.assertWaitFor(
                'Mock Server 拒绝 Cancel 请求',
                process=mock_server,
                timeout=15,
            )

            proc_output.assertWaitFor(
                'Nav2 未接受取消请求',
                process=mission_manager,
                timeout=15,
            )

            # 5. 在 Goal 尚未结束时查询 Mission 状态
            status_future = status_client.call_async(
                Trigger.Request()
            )

            executor.spin_until_future_complete(
                status_future,
                timeout_sec=10.0,
            )

            self.assertTrue(status_future.done())
            status_response = status_future.result()

            self.assertIsNotNone(status_response)
            self.assertTrue(status_response.success)

            self.assertIn(
                'mission=RUNNING',
                status_response.message,
            )

            self.assertIn(
                'phase=CANCEL_REQUESTED',
                status_response.message,
            )

            self.assertIn(
                'waypoint_status=NAVIGATING',
                status_response.message,
            )

        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

        # 6. Mock Goal 在拒绝取消后自行完成
        proc_output.assertWaitFor(
            'Goal 1 -> SUCCEEDED',
            process=mock_server,
            timeout=45,
        )

        # 7. Manager 不再执行后续站点
        proc_output.assertWaitFor(
            '取消请求后，Nav2 Goal 以其他状态结束: 4',
            process=mission_manager,
            timeout=30,
        )

        proc_output.assertWaitFor(
            'Mission: CANCELED',
            process=mission_manager,
            timeout=30,
        )

        proc_info.assertWaitForShutdown(
            process=mission_manager,
            timeout=10,
        )

        # 8. 确保取消后没有错误地执行 P2
        manager_output = ''.join(
            event.text.decode('utf-8', errors='replace')
            for event in proc_output[mission_manager]
        )

        self.assertNotIn(
            '正在发送 P2 导航目标',
            manager_output,
        )

        self.assertNotIn(
            'Mission: SUCCEEDED',
            manager_output,
        )

        self.assertNotIn(
            'Nav2 Goal 已确认取消',
            manager_output,
        )
