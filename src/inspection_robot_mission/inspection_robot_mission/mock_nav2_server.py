
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

"""Simulate Nav2 results, delayed goals, and cancellation."""

import math
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
)
from rclpy.node import Node


class MockNav2Server(Node):
    """Provide configurable fake navigation behavior."""

    def __init__(self):
        """Initialize the fake NavigateToPose server."""
        super().__init__('mock_nav2_server')

        self.declare_parameter('success_on_attempt', 2)
        self.declare_parameter('wait_for_cancel', False)
        self.declare_parameter('goal_accept_delay_seconds', 0.0)

        # 本次新增的两个参数
        self.declare_parameter('reject_cancel', False)
        self.declare_parameter('finish_after_seconds', 0.0)

        self.success_on_attempt = self.get_parameter(
            'success_on_attempt'
        ).value

        self.wait_for_cancel = self.get_parameter(
            'wait_for_cancel'
        ).value

        self.goal_accept_delay_seconds = self.get_parameter(
            'goal_accept_delay_seconds'
        ).value

        self.reject_cancel = self.get_parameter(
            'reject_cancel'
        ).value

        self.finish_after_seconds = self.get_parameter(
            'finish_after_seconds'
        ).value

        if (
            type(self.success_on_attempt) is not int
            or self.success_on_attempt < 0
        ):
            raise ValueError(
                'success_on_attempt 必须是非负整数'
            )

        if type(self.wait_for_cancel) is not bool:
            raise ValueError(
                'wait_for_cancel 必须是布尔值'
            )

        if type(self.reject_cancel) is not bool:
            raise ValueError(
                'reject_cancel 必须是布尔值'
            )

        for name, value in (
            ('goal_accept_delay_seconds',
             self.goal_accept_delay_seconds),
            ('finish_after_seconds',
             self.finish_after_seconds),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError(
                    f'{name} 必须是非负有限数'
                )

        self.attempt_count = 0

        self.action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            'Mock NavigateToPose Server 已启动'
        )

    def goal_callback(self, goal_request):
        """Optionally delay accepting a navigation goal."""
        delay = self.goal_accept_delay_seconds

        if delay > 0:
            self.get_logger().warn(
                f'收到 Goal 请求，延迟 {delay:.1f} 秒接受'
            )
            time.sleep(delay)

        self.get_logger().info('Mock Server 接受 Goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a navigation cancellation request."""
        if self.reject_cancel:
            self.get_logger().warn(
                'Mock Server 拒绝 Cancel 请求'
            )
            return CancelResponse.REJECT

        self.get_logger().warn(
            'Mock Server 接受 Cancel 请求'
        )
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute a controlled fake navigation request."""
        self.attempt_count += 1
        attempt = self.attempt_count

        self.get_logger().info(
            f'收到第 {attempt} 次导航请求'
        )

        if self.wait_for_cancel:
            self.get_logger().info(
                'Mock Goal 正在执行，等待取消或自动结束'
            )

            start_time = time.monotonic()

            while rclpy.ok() and goal_handle.is_active:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().warn(
                        f'Goal {attempt} -> CANCELED'
                    )
                    return NavigateToPose.Result()

                if self.finish_after_seconds > 0:
                    elapsed = time.monotonic() - start_time

                    if elapsed >= self.finish_after_seconds:
                        goal_handle.succeed()
                        self.get_logger().info(
                            f'Goal {attempt} -> SUCCEEDED'
                        )
                        return NavigateToPose.Result()

                time.sleep(0.1)

            if goal_handle.is_active:
                goal_handle.abort()

            return NavigateToPose.Result()

        if (
            self.success_on_attempt > 0
            and attempt >= self.success_on_attempt
        ):
            goal_handle.succeed()
            self.get_logger().info(
                f'Goal {attempt} -> SUCCEEDED'
            )
        else:
            goal_handle.abort()
            self.get_logger().warn(
                f'Goal {attempt} -> ABORTED'
            )

        return NavigateToPose.Result()


def main(args=None):
    """Run the mock server using concurrent callbacks."""
    rclpy.init(args=args)

    node = MockNav2Server()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.action_server.destroy()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
