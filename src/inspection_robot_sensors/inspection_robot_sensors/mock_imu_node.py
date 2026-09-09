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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState


class MockImuNode(Node):
    """Generate a minimal mock IMU measurement from wheel joint states."""

    def __init__(self):
        super().__init__('mock_imu_node')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.28)
        self.declare_parameter('left_wheel_name', 'left_wheel_joint')
        self.declare_parameter('right_wheel_name', 'right_wheel_joint')
        self.declare_parameter('imu_frame_id', 'imu_link')

        self.wheel_radius = (
            self.get_parameter('wheel_radius').get_parameter_value().double_value
        )
        self.wheel_separation = (
            self.get_parameter(
                'wheel_separation'
            ).get_parameter_value().double_value
        )
        self.left_wheel_name = (
            self.get_parameter(
                'left_wheel_name'
            ).get_parameter_value().string_value
        )
        self.right_wheel_name = (
            self.get_parameter(
                'right_wheel_name'
            ).get_parameter_value().string_value
        )
        self.imu_frame_id = (
            self.get_parameter(
                'imu_frame_id'
            ).get_parameter_value().string_value
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            'imu/data',
            10,
        )

        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
        )

    def joint_state_callback(self, message):
        """Convert wheel velocities into a mock yaw-rate measurement."""
        if len(message.name) != len(message.velocity):
            return

        velocities = dict(zip(message.name, message.velocity))

        if self.left_wheel_name not in velocities:
            return

        if self.right_wheel_name not in velocities:
            return

        left_velocity = velocities[self.left_wheel_name]
        right_velocity = velocities[self.right_wheel_name]

        yaw_rate = (
            self.wheel_radius
            * (right_velocity - left_velocity)
            / self.wheel_separation
        )

        imu_message = Imu()

        imu_message.header.stamp = message.header.stamp
        imu_message.header.frame_id = self.imu_frame_id

        imu_message.angular_velocity.z = yaw_rate

        # Orientation is not simulated by this minimal mock IMU.
        imu_message.orientation_covariance[0] = -1.0

        # Only yaw angular velocity is considered useful in this mock.
        imu_message.angular_velocity_covariance[0] = 1.0e6
        imu_message.angular_velocity_covariance[4] = 1.0e6
        imu_message.angular_velocity_covariance[8] = 0.01

        # Linear acceleration is not simulated.
        imu_message.linear_acceleration_covariance[0] = -1.0

        self.imu_publisher.publish(imu_message)


def main(args=None):
    """Run the mock IMU node."""
    rclpy.init(args=args)

    node = MockImuNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
