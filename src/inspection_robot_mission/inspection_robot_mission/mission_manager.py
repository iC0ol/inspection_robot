
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

"""Execute a sequential multi-waypoint inspection mission."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from inspection_robot_mission.mission_model import (
    InspectionMission,
    MissionStatus,
    Waypoint,
    WaypointStatus,
)


class MissionManager(Node):
    """Schedule navigation and inspection for each waypoint."""

    def __init__(self):
        """Initialize the mission and Nav2 Action client."""
        super().__init__('mission_manager')

        self.declare_parameter('waypoints_x')
        self.declare_parameter('waypoints_y')
        self.declare_parameter('waypoints_yaw')
        self.declare_parameter('wait_seconds', 2.0)

        xs = self.get_parameter('waypoints_x').value
        ys = self.get_parameter('waypoints_y').value
        yaws = self.get_parameter('waypoints_yaw').value
        wait = self.get_parameter('wait_seconds').value

        if not all(isinstance(v, list) for v in (xs, ys, yaws)):
            raise ValueError('必须配置三个 Waypoint 坐标数组')

        if not xs or len(xs) != len(ys) or len(xs) != len(yaws):
            raise ValueError('三个坐标数组必须非空且长度相等')

        values = xs + ys + yaws
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError('Waypoint 坐标必须是有限数')

        if (
            not isinstance(wait, (int, float))
            or not math.isfinite(wait)
            or wait < 0
        ):
            raise ValueError('wait_seconds 必须是非负有限数')

        self.wait_seconds = float(wait)

        waypoints = [
            Waypoint(
                name=f'P{i + 1}',
                x=float(xs[i]),
                y=float(ys[i]),
                yaw=float(yaws[i]),
                wait_seconds=self.wait_seconds,
                action='wait',
            )
            for i in range(len(xs))
        ]

        # 第一阶段先关闭自动重试，验证顺序调度。
        self.mission = InspectionMission(
            mission_id='mission_001',
            waypoints=waypoints,
            max_navigation_retries=0,
        )

        self.client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )

        self.goal_handle = None
        self.navigation_active = False
        self.finished = False
        self.stopping = False
        self.feedback_count = 0
        self.wait_timer = None

        self.get_logger().info(
            f'已加载 {len(waypoints)} 个巡检点'
        )

    def start(self):
        """Start the mission and dispatch its first waypoint."""
        self.mission.start()
        self.get_logger().info('Mission: RUNNING')

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.fail_mission('Nav2 Action Server 不可用')
            return

        self.send_current_goal()

    def send_current_goal(self):
        """Send the navigation goal for the current waypoint."""
        if self.mission.state != MissionStatus.RUNNING:
            return

        index = self.mission.current_index
        waypoint = self.mission.waypoints[index]

        self.feedback_count = 0
        self.goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        goal.pose.pose.orientation.z = math.sin(
            waypoint.yaw / 2.0
        )
        goal.pose.pose.orientation.w = math.cos(
            waypoint.yaw / 2.0
        )

        self.get_logger().info(
            f'正在发送 {waypoint.name} 导航目标'
        )

        future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Process the navigation goal acceptance response."""
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.fail_mission(f'Goal 请求异常: {exc}')
            return

        if self.stopping or self.finished:
            if self.goal_handle.accepted:
                self.goal_handle.cancel_goal_async()
            return

        if not self.goal_handle.accepted:
            self.fail_mission('导航 Goal 被拒绝')
            return

        index = self.mission.current_index
        waypoint = self.mission.waypoints[index]

        self.mission.transition_waypoint(
            index, WaypointStatus.NAVIGATING
        )

        self.navigation_active = True
        self.get_logger().info(
            f'{waypoint.name}: NAVIGATING'
        )

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, message):
        """Print periodic feedback for the active waypoint."""
        if (
            self.stopping
            or self.finished
            or not self.navigation_active
        ):
            return

        self.feedback_count += 1

        if self.feedback_count % 20 == 0:
            index = self.mission.current_index
            waypoint = self.mission.waypoints[index]
            distance = message.feedback.distance_remaining

            self.get_logger().info(
                f'{waypoint.name} 剩余距离: {distance:.2f} m'
            )

    def result_callback(self, future):
        """Process the navigation result and start inspection."""
        self.navigation_active = False

        if self.stopping or self.finished:
            return

        try:
            response = future.result()
        except Exception as exc:
            self.fail_mission(f'获取导航 Result 失败: {exc}')
            return

        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail_mission(
                f'导航失败，状态码: {response.status}'
            )
            return

        index = self.mission.current_index
        waypoint = self.mission.waypoints[index]

        self.mission.transition_waypoint(
            index, WaypointStatus.INSPECTING
        )

        self.get_logger().info(
            f'{waypoint.name} 导航成功'
        )
        self.get_logger().info(
            f'{waypoint.name}: INSPECTING，'
            f'等待 {waypoint.wait_seconds:.1f} 秒'
        )

        if waypoint.wait_seconds == 0:
            self.complete_inspection()
        else:
            self.wait_timer = self.create_timer(
                waypoint.wait_seconds,
                self.complete_inspection,
            )

    def complete_inspection(self):
        """Finish inspection and schedule the next waypoint."""
        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.destroy_timer(self.wait_timer)
            self.wait_timer = None

        if self.stopping or self.finished:
            return

        index = self.mission.current_index
        waypoint = self.mission.waypoints[index]

        self.mission.transition_waypoint(
            index, WaypointStatus.SUCCEEDED
        )
        self.get_logger().info(
            f'{waypoint.name}: SUCCEEDED'
        )

        self.mission.advance_to_next_waypoint()

        # 所有巡检点都已完成。
        if self.mission.state == MissionStatus.SUCCEEDED:
            self.get_logger().info('Mission: SUCCEEDED')
            self.finished = True
            return

        # Mission 仍在运行，自动派发下一站。
        self.get_logger().info(
            f'准备执行第 {self.mission.current_index + 1} 个巡检点'
        )
        self.send_current_goal()

    def fail_mission(self, reason):
        """Stop mission progression after an execution failure."""
        if self.finished:
            return

        self.get_logger().error(reason)

        index = self.mission.current_index
        status = self.mission.waypoint_states[index]

        if status == WaypointStatus.NAVIGATING:
            self.mission.record_navigation_failure(index)

        elif status in (
            WaypointStatus.PENDING,
            WaypointStatus.INSPECTING,
        ):
            self.mission.transition_waypoint(
                index, WaypointStatus.FAILED
            )

        self.get_logger().error(
            f'Mission: {self.mission.state.value}'
        )
        self.finished = True

    def request_stop(self):
        """Attempt to cancel the currently active Nav2 goal."""
        self.stopping = True

        if self.goal_handle is None or not self.navigation_active:
            return

        future = self.goal_handle.cancel_goal_async()

        if rclpy.ok():
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=3.0
            )

        self.get_logger().warn(
            '已尝试取消导航，请确认机器人停止'
        )


def main(args=None):
    """Start the mission manager node."""
    rclpy.init(args=args)
    node = None

    try:
        node = MissionManager()
        node.start()

        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None and rclpy.ok():
            node.request_stop()

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
