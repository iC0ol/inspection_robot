#!/usr/bin/env python3

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

"""
Planner-to-controller diagnostic tool.

This script bypasses BT Navigator and directly:
1. requests a path from ComputePathToPose;
2. sends that path to FollowPath.

It is intended for component-level Nav2 debugging.
Normal navigation should use NavigateToPose through BT Navigator.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import FollowPath


class PlanAndFollowNode(Node):

    def __init__(self):
        super().__init__('plan_and_follow')

        # =========================
        # Goal parameters
        # =========================
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', -1.0)
        self.declare_parameter('goal_yaw', 0.0)

        self.goal_x = (
            self.get_parameter('goal_x')
            .get_parameter_value()
            .double_value
        )

        self.goal_y = (
            self.get_parameter('goal_y')
            .get_parameter_value()
            .double_value
        )

        self.goal_yaw = (
            self.get_parameter('goal_yaw')
            .get_parameter_value()
            .double_value
        )

        # =========================
        # Nav2 Action Clients
        # =========================
        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose',
        )

        self.controller_client = ActionClient(
            self,
            FollowPath,
            '/follow_path',
        )

        self.feedback_count = 0
        self.follow_goal_handle = None


def main(args=None):
    rclpy.init(args=args)

    node = PlanAndFollowNode()

    try:
        # ============================================================
        # Step 1: wait for Planner Server
        # ============================================================
        node.get_logger().info(
            'Waiting for /compute_path_to_pose...'
        )

        if not node.planner_client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(
                'Planner action server is not available.'
            )
            return

        # ============================================================
        # Step 2: construct planning goal
        # ============================================================
        planner_goal = ComputePathToPose.Goal()

        planner_goal.goal.header.frame_id = 'map'
        planner_goal.goal.header.stamp = (
            node.get_clock().now().to_msg()
        )

        planner_goal.goal.pose.position.x = node.goal_x
        planner_goal.goal.pose.position.y = node.goal_y
        planner_goal.goal.pose.position.z = 0.0

        half_yaw = node.goal_yaw / 2.0

        planner_goal.goal.pose.orientation.z = math.sin(half_yaw)
        planner_goal.goal.pose.orientation.w = math.cos(half_yaw)

        # Let Planner obtain the current robot pose from TF.
        planner_goal.use_start = False
        planner_goal.planner_id = 'GridBased'

        node.get_logger().info(
            f'Requesting path to '
            f'x={node.goal_x:.2f}, '
            f'y={node.goal_y:.2f}, '
            f'yaw={node.goal_yaw:.2f}'
        )

        # ============================================================
        # Step 3: send ComputePathToPose
        # ============================================================
        send_plan_future = (
            node.planner_client.send_goal_async(planner_goal)
        )

        rclpy.spin_until_future_complete(
            node,
            send_plan_future,
        )

        planner_goal_handle = send_plan_future.result()

        if planner_goal_handle is None:
            node.get_logger().error(
                'Failed to obtain planner goal handle.'
            )
            return

        if not planner_goal_handle.accepted:
            node.get_logger().error(
                'Planning goal was rejected.'
            )
            return

        node.get_logger().info(
            'Planning goal accepted.'
        )

        plan_result_future = (
            planner_goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            node,
            plan_result_future,
        )

        plan_result_response = plan_result_future.result()

        if plan_result_response is None:
            node.get_logger().error(
                'Planner returned no result.'
            )
            return

        if (
            plan_result_response.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            node.get_logger().error(
                f'Planning failed with status '
                f'{plan_result_response.status}.'
            )
            return

        path = plan_result_response.result.path

        if len(path.poses) == 0:
            node.get_logger().error(
                'Planner returned an empty path.'
            )
            return

        node.get_logger().info(
            f'Planner succeeded: '
            f'{len(path.poses)} poses in path.'
        )

        # ============================================================
        # Step 4: wait for Controller Server
        # ============================================================
        node.get_logger().info(
            'Waiting for /follow_path...'
        )

        if not node.controller_client.wait_for_server(
            timeout_sec=10.0
        ):
            node.get_logger().error(
                'Controller action server is not available.'
            )
            return

        # ============================================================
        # Step 5: construct FollowPath goal
        # ============================================================
        follow_goal = FollowPath.Goal()

        follow_goal.path = path

        # Matches controller.yaml
        follow_goal.controller_id = 'FollowPath'
        follow_goal.goal_checker_id = 'general_goal_checker'

        def feedback_callback(feedback_msg):
            node.feedback_count += 1

            # Avoid printing at the full controller frequency.
            if node.feedback_count % 10 != 0:
                return

            feedback = feedback_msg.feedback

            node.get_logger().info(
                f'distance_to_goal='
                f'{feedback.distance_to_goal:.3f} m, '
                f'speed={feedback.speed:.3f} m/s'
            )

        # ============================================================
        # Step 6: send FollowPath
        # ============================================================
        node.get_logger().info(
            'Sending path to Controller Server.'
        )

        send_follow_future = (
            node.controller_client.send_goal_async(
                follow_goal,
                feedback_callback=feedback_callback,
            )
        )

        rclpy.spin_until_future_complete(
            node,
            send_follow_future,
        )

        node.follow_goal_handle = send_follow_future.result()

        if node.follow_goal_handle is None:
            node.get_logger().error(
                'Failed to obtain controller goal handle.'
            )
            return

        if not node.follow_goal_handle.accepted:
            node.get_logger().error(
                'FollowPath goal was rejected.'
            )
            return

        node.get_logger().info(
            'FollowPath accepted. Robot should start moving.'
        )

        follow_result_future = (
            node.follow_goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            node,
            follow_result_future,
        )

        follow_result_response = follow_result_future.result()

        if follow_result_response is None:
            node.get_logger().error(
                'Controller returned no result.'
            )
            return

        if (
            follow_result_response.status
            == GoalStatus.STATUS_SUCCEEDED
        ):
            node.get_logger().info(
                'FollowPath succeeded.'
            )
        else:
            node.get_logger().error(
                f'FollowPath finished with status '
                f'{follow_result_response.status}.'
            )

    except KeyboardInterrupt:
        node.get_logger().warn(
            'Interrupted by user.'
        )

        # Try to cancel an active FollowPath goal.
        if (
            node.follow_goal_handle is not None
            and node.follow_goal_handle.accepted
        ):
            node.get_logger().warn(
                'Cancelling FollowPath goal...'
            )

            cancel_future = (
                node.follow_goal_handle.cancel_goal_async()
            )

            rclpy.spin_until_future_complete(
                node,
                cancel_future,
                timeout_sec=2.0,
            )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
