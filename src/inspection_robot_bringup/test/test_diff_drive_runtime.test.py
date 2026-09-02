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

"""Runtime integration test for the diff-drive control stack."""

from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the control stack while the integration test runs."""
    bringup_share = Path(
        get_package_share_directory("inspection_robot_bringup")
    )

    control_launch = bringup_share / "launch" / "control.launch.py"

    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(control_launch))
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDiffDriveRuntime(unittest.TestCase):
    """Verify diff-drive commands against mock hardware feedback."""

    def setUp(self):
        """Create the ROS interfaces used by the test."""
        rclpy.init()

        self.node = Node("test_diff_drive_runtime")
        self.latest_joint_state = None

        self.publisher = self.node.create_publisher(
            Twist,
            "/diff_drive_controller/cmd_vel_unstamped",
            10,
        )

        self.subscription = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

    def tearDown(self):
        """Destroy the test ROS node."""
        self.node.destroy_node()
        rclpy.shutdown()

    def _joint_state_callback(self, message):
        """Store the newest joint-state message."""
        self.latest_joint_state = message

    def _spin_until(self, predicate, timeout_sec):
        """Spin until a predicate succeeds or the timeout expires."""
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if predicate():
                return True

        return False

    def _wheel_velocities(self):
        """Return left and right wheel velocities when available."""
        message = self.latest_joint_state

        if message is None:
            return None

        if len(message.name) != len(message.velocity):
            return None

        velocities = dict(zip(message.name, message.velocity))

        if "left_wheel_joint" not in velocities:
            return None

        if "right_wheel_joint" not in velocities:
            return None

        return (
            velocities["left_wheel_joint"],
            velocities["right_wheel_joint"],
        )

    def _publish_stop(self):
        """Publish zero velocity several times before ending the test."""
        stop_command = Twist()

        for _ in range(5):
            self.publisher.publish(stop_command)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_forward_command_produces_expected_wheel_speed(self):
        """A 0.2 m/s command must produce about 4 rad/s per wheel."""
        matched = False
        last_velocities = None

        try:
            controller_ready = self._spin_until(
                lambda: self.publisher.get_subscription_count() > 0,
                timeout_sec=10.0,
            )

            self.assertTrue(
                controller_ready,
                "diff_drive_controller did not subscribe to cmd_vel.",
            )

            command = Twist()
            command.linear.x = 0.2

            deadline = time.monotonic() + 3.0

            while time.monotonic() < deadline:
                self.publisher.publish(command)
                rclpy.spin_once(self.node, timeout_sec=0.05)

                last_velocities = self._wheel_velocities()

                if last_velocities is None:
                    continue

                left_velocity, right_velocity = last_velocities

                if (
                    abs(left_velocity - 4.0) < 0.1
                    and abs(right_velocity - 4.0) < 0.1
                ):
                    matched = True
                    break

        finally:
            self._publish_stop()

        self.assertTrue(
            matched,
            (
                "Expected wheel velocities near 4.0 rad/s, "
                f"but last values were {last_velocities}."
            ),
        )
