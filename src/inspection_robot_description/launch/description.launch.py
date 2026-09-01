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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch the minimal robot description and TF publishers."""
    default_model_path = PathJoinSubstitution(
        [
            FindPackageShare("inspection_robot_description"),
            "urdf",
            "inspection_robot.urdf.xacro",
        ]
    )

    model = LaunchConfiguration("model")
    publish_joints = LaunchConfiguration("publish_joints")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 在 Launch 运行阶段执行：
    # xacro <model路径>
    # Command 会把生成的完整 URDF XML 作为字符串返回。
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model_path,
                description="Absolute path to the robot Xacro file.",
            ),
            DeclareLaunchArgument(
                "publish_joints",
                default_value="true",
                description=(
                    "Start joint_state_publisher when no real joint-state "
                    "source is available."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock when true.",
            ),

            # 当前还没有编码器、Gazebo 或 ros2_control，
            # 因此暂时用它发布左右轮的默认关节位置。
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=IfCondition(publish_joints),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),

            # 读取完整 URDF，并结合 /joint_states 发布 TF。
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    robot_description,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
                output="screen",
            ),
        ]
    )
