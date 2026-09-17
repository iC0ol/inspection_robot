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
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch the inspection robot in Gazebo Classic."""
    robot_xacro = PathJoinSubstitution(
        [
            FindPackageShare("inspection_robot_description"),
            "urdf",
            "inspection_robot.urdf.xacro",
        ]
    )

    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("inspection_robot_bringup"),
            "config",
            "controllers.yaml",
        ]
    )

    gazebo_params_file = PathJoinSubstitution(
        [
            FindPackageShare("inspection_robot_bringup"),
            "config",
            "gazebo.yaml",
        ]
    )

    ekf_file = PathJoinSubstitution(
        [
            FindPackageShare("inspection_robot_bringup"),
            "config",
            "ekf.yaml",
        ]
    )

    world_file = PathJoinSubstitution(
        [
            FindPackageShare("inspection_robot_bringup"),
            "worlds",
            "inspection_room.world",
        ]
    )

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                robot_xacro,
                " ",
                "use_gazebo:=true",
                " ",
                "controllers_file:=",
                controllers_file,
            ]
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world": world_file,
            "params_file": gazebo_params_file,
        }.items(),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "inspection_robot",
            "-z",
            "0.01",
        ],
        output="screen",
    )

    mock_imu_node = Node(
        package="inspection_robot_sensors",
        executable="mock_imu_node",
        name="mock_imu_node",
        parameters=[
            {"use_sim_time": True},
        ],
        output="screen",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[
            ekf_file,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
            ],
        )
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_robot,
            start_controllers_after_spawn,
            mock_imu_node,
            ekf_node,
        ]
    )
