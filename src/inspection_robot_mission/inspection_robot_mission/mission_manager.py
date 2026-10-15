
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
from functools import partial
from std_srvs.srv import Trigger

from inspection_robot_mission.mission_model import (
    InspectionMission,
    MissionStatus,
    Waypoint,
    WaypointStatus,
    RetryExhaustedPolicy,
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

        self.declare_parameter('max_navigation_retries', 2)
        self.declare_parameter('retry_delay_seconds', 2.0)

        self.declare_parameter(
            'retry_exhausted_policy',
            'FAIL_MISSION',
        )

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

        max_retries = self.get_parameter(
            'max_navigation_retries'
        ).value

        retry_delay = self.get_parameter(
            'retry_delay_seconds'
        ).value

        policy_name = self.get_parameter(
            'retry_exhausted_policy'
        ).value

        try:
            retry_policy = RetryExhaustedPolicy(policy_name)
        except (ValueError, TypeError) as exc:
            raise ValueError(
                'retry_exhausted_policy 必须为 '
                'FAIL_MISSION 或 SKIP_WAYPOINT'
            ) from exc

        if type(max_retries) is not int or max_retries < 0:
            raise ValueError('max_navigation_retries 必须是非负整数')

        if (
            isinstance(retry_delay, bool)
            or not isinstance(retry_delay, (int, float))
            or not math.isfinite(retry_delay)
            or retry_delay <= 0
        ):
            raise ValueError('retry_delay_seconds 必须是正数')

        self.retry_delay_seconds = float(retry_delay)

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
            max_navigation_retries=max_retries,
            retry_exhausted_policy=retry_policy,
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
        self.goal_serial = 0
        self.active_goal_serial = None
        self.wait_timer = None
        self.retry_timer = None

        self.cancel_service = self.create_service(
            Trigger,
            '/mission/cancel',
            self.cancel_service_callback,
        )

        self.get_logger().info(
            f'已加载 {len(waypoints)} 个巡检点'
        )

        self.status_service = self.create_service(
            Trigger,
            '/mission/status',
            self.status_service_callback,
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
        if self.active_goal_serial is not None:
            raise RuntimeError("上一个导航 Goal 尚未结束")

        if (
            self.stopping
            or self.finished
            or self.mission.state != MissionStatus.RUNNING
        ):
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

        # 为本次 Goal 分配独立编号
        self.goal_serial += 1
        serial = self.goal_serial
        self.active_goal_serial = serial

        self.get_logger().info(
            f'[Goal {serial}] 开始发送导航请求'
        )

        try:
            future = self.client.send_goal_async(
                goal,
                feedback_callback=partial(
                    self.feedback_callback, serial
                ),
            )
        except Exception as exc:
            self.active_goal_serial = None
            self.fail_mission(f'发送 Goal 失败: {exc}')
            return

        future.add_done_callback(
            partial(self.goal_response_callback, serial)
        )

    def goal_response_callback(self, serial, future):
        """Handle acceptance and pending cancellation of a goal."""
        if serial != self.active_goal_serial:
            return

        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.active_goal_serial = None
            self.fail_mission(f'Goal 请求异常: {exc}')
            return

        if not self.goal_handle.accepted:
            self.active_goal_serial = None

            if self.stopping:
                self.finalize_mission_cancel()
            else:
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
        result_future.add_done_callback(
            partial(self.result_callback, serial)
        )

        # 用户可能在 Goal 尚未被接受时就要求取消。
        if self.stopping:
            self.request_nav_cancel()

    def feedback_callback(self, serial, message):
        """Print periodic feedback for the active waypoint."""
        if serial != self.active_goal_serial:
            return
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
                f'[Goal {serial}] {waypoint.name} '
                f'剩余距离: {distance:.2f} m'
            )

    def result_callback(self, serial, future):
        """Handle navigation results and mission cancellation."""
        # 1. 过滤不属于当前 Goal 的旧回调
        if serial != self.active_goal_serial:
            return

        # 2. 当前 Goal 已经返回最终结果
        self.active_goal_serial = None
        self.navigation_active = False

        if self.finished:
            return

        # 3. 获取 Nav2 Action Result
        try:
            response = future.result()
        except Exception as exc:
            self.fail_mission(
                f'获取导航 Result 失败: {exc}'
            )
            return

        # 4. 优先处理用户主动取消任务的情况
        if self.stopping:
            if response.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(
                    'Nav2 Goal 已确认取消'
                )
            else:
                self.get_logger().warn(
                    '取消请求后，Nav2 Goal 以其他状态结束: '
                    f'{response.status}'
                )

            self.finalize_mission_cancel()
            return

        # 5. Nav2 导航失败：交给 Retry / Skip / Fail 策略
        if response.status == GoalStatus.STATUS_ABORTED:
            self.handle_navigation_abort()
            return

        # 6. 其他非成功状态：终止任务
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail_mission(
                '导航未成功且不进行重试，'
                f'状态码: {response.status}'
            )
            return

        # 7. 导航成功：获取当前巡检点
        index = self.mission.current_index
        waypoint = self.mission.waypoints[index]

        # 8. 进入巡检阶段
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

        # 9. 执行巡检等待
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
        if self.mission.state in (
            MissionStatus.SUCCEEDED,
            MissionStatus.COMPLETED_WITH_SKIPS,
        ):
            self.get_logger().info(
                f'Mission: {self.mission.state.value}'
            )
            self.finished = True
            return

        # Mission 仍在运行，自动派发下一站。
        self.get_logger().info(
            f'准备执行第 {self.mission.current_index + 1} 个巡检点'
        )
        self.send_current_goal()

    def fail_mission(self, reason):
        """Terminate the mission after an unrecoverable failure."""
        if self.finished:
            return

        self.get_logger().error(reason)

        index = self.mission.current_index
        status = self.mission.waypoint_states[index]

        if status in (
            WaypointStatus.PENDING,
            WaypointStatus.NAVIGATING,
            WaypointStatus.RETRY_PENDING,
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

    def handle_navigation_abort(self):
        """Handle navigation failure using the configured policy."""
        index = self.mission.current_index
        waypoint = self.mission.waypoints[index]

        should_retry = self.mission.record_navigation_failure(
            index
        )

        failures = self.mission.navigation_failures[index]

        # 情况 1：还有重试机会
        if should_retry:
            self.get_logger().warn(
                f'{waypoint.name} 第 {failures} 次导航失败，'
                f'{self.retry_delay_seconds:.1f} 秒后重试'
            )

            self.retry_timer = self.create_timer(
                self.retry_delay_seconds,
                self.retry_current_goal,
            )
            return

        # 情况 2：当前站点被跳过
        if (
            self.mission.waypoint_states[index]
            == WaypointStatus.SKIPPED
        ):
            self.get_logger().warn(
                f'{waypoint.name} 重试耗尽，已跳过当前站点'
            )

            # 如果跳过的是最后一个站点，任务结束。
            if (
                self.mission.state
                == MissionStatus.COMPLETED_WITH_SKIPS
            ):
                self.get_logger().info(
                    'Mission: COMPLETED_WITH_SKIPS'
                )
                self.finished = True
                return

            # 如果还有其他站点，继续导航。
            if self.mission.state == MissionStatus.RUNNING:
                self.get_logger().info(
                    '准备执行下一个巡检点'
                )
                self.send_current_goal()
                return

        # 情况 3：默认 Fail-fast，整个任务失败
        if self.mission.state == MissionStatus.FAILED:
            self.get_logger().error(
                f'{waypoint.name} 导航重试耗尽，'
                f'累计失败 {failures} 次'
            )
            self.get_logger().error('Mission: FAILED')
            self.finished = True
            return

        # 防止未预期的状态继续派发导航目标。
        self.get_logger().error(
            '导航失败处理出现未预期的 Mission 状态'
        )
        self.finished = True

    def retry_current_goal(self):
        """Dispatch a fresh goal after the retry delay."""
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None

        if self.stopping or self.finished:
            return

        if self.mission.state != MissionStatus.RUNNING:
            return

        index = self.mission.current_index
        status = self.mission.waypoint_states[index]

        if status != WaypointStatus.RETRY_PENDING:
            self.fail_mission('重试时 Waypoint 状态异常')
            return

        waypoint = self.mission.waypoints[index]

        self.get_logger().info(
            f'重新派发 {waypoint.name} 导航目标'
        )

        self.send_current_goal()

    def cancel_service_callback(self, request, response):
        """Accept a request to cancel the current mission."""
        if (
            self.finished
            or self.stopping
            or self.mission.state != MissionStatus.RUNNING
        ):
            response.success = False
            response.message = 'Mission 不在可取消状态'
            return response

        # 立即禁止继续派发新的导航 Goal。
        self.stopping = True
        self.get_logger().warn('收到 Mission 取消请求')

        # 停止等待巡检或等待重试的 Timer。
        for timer_name in ('wait_timer', 'retry_timer'):
            timer = getattr(self, timer_name)

            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
                setattr(self, timer_name, None)

        if self.active_goal_serial is None:
            # 当前没有尚未结束的导航 Goal。
            self.finalize_mission_cancel()
            response.message = 'Mission 已取消，无活动导航 Goal'

        elif self.goal_handle is None:
            # Goal 已发出，但尚未收到接受或拒绝响应。
            response.message = (
                '取消请求已受理，等待 Goal 接受响应'
            )

        else:
            # Goal 已被接受，异步请求 Nav2 取消。
            self.request_nav_cancel()
            response.message = (
                '取消请求已受理，等待 Nav2 最终 Result'
            )

        response.success = True
        return response

    def request_nav_cancel(self):
        """Send an asynchronous cancel request to Nav2."""
        if self.goal_handle is None:
            return

        if not self.goal_handle.accepted:
            return

        try:
            future = self.goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(
                f'发送 Nav2 Cancel 请求失败: {exc}'
            )
            return

        future.add_done_callback(
            self.cancel_response_callback
        )

        self.get_logger().warn('已发送 Nav2 Cancel 请求')

    def cancel_response_callback(self, future):
        """Inspect the Nav2 cancel request response."""
        try:
            cancel_response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'获取 Cancel 响应失败: {exc}'
            )
            return

        if cancel_response.goals_canceling:
            self.get_logger().info(
                'Nav2 已接受取消请求，等待最终 Result'
            )
        else:
            self.get_logger().warn(
                'Nav2 未接受取消请求，或 Goal 已结束；'
                '继续等待最终 Result'
            )

    def finalize_mission_cancel(self):
        """Finalize cancellation after navigation is no longer active."""
        if self.finished:
            return

        if self.active_goal_serial is not None:
            self.get_logger().error(
                '仍有未结束的导航 Goal，不能确认任务取消'
            )
            return

        self.mission.cancel()

        self.get_logger().warn(
            f'Mission: {self.mission.state.value}'
        )
        self.finished = True

    def status_service_callback(self, request, response):
        """Return a snapshot of the current mission status."""
        mission = self.mission
        index = mission.current_index
        total = len(mission.waypoints)

        # 判断 Mission Manager 当前的执行阶段
        if self.finished:
            phase = 'TERMINAL'
        elif self.stopping:
            phase = 'CANCEL_REQUESTED'
        elif mission.state == MissionStatus.RUNNING:
            phase = 'ACTIVE'
        else:
            phase = 'IDLE'

        # 获取当前站点的信息
        if index < total:
            waypoint_name = mission.waypoints[index].name
            waypoint_status = mission.waypoint_states[index].value
        else:
            waypoint_name = 'NONE'
            waypoint_status = 'NONE'

        # 获取当前 Action Goal 的编号
        if self.active_goal_serial is None:
            active_goal = 'NONE'
        else:
            active_goal = str(self.active_goal_serial)

        response.success = True
        response.message = (
            f'mission={mission.state.value}; '
            f'phase={phase}; '
            f'waypoint={waypoint_name}; '
            f'waypoint_status={waypoint_status}; '
            f'active_goal={active_goal}; '
            f'current_index={index}; '
            f'total_waypoints={total}'
        )

        return response


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
