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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the complete inspection robot navigation simulation."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock',
    )

    bringup_share = FindPackageShare('inspection_robot_bringup')

    gazebo_launch = PathJoinSubstitution(
        [
            bringup_share,
            'launch',
            'gazebo.launch.py',
        ]
    )

    navigation_launch = PathJoinSubstitution(
        [
            bringup_share,
            'launch',
            'navigation.launch.py',
        ]
    )

    rviz_config = PathJoinSubstitution(
        [
            bringup_share,
            'rviz',
            'navigation.rviz',
        ]
    )

    # ============================================================
    # Gazebo + robot + ros2_control + EKF
    # ============================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
    )

    # ============================================================
    # Nav2
    #
    # Delay it slightly so that Gazebo, /clock, TF, controllers,
    # LaserScan and EKF have time to start first.
    # ============================================================
    navigation = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',

                    # Fixed spawn pose in inspection_room_v1 map
                    'set_initial_pose': 'true',
                    'initial_pose_x': '-0.311',
                    'initial_pose_y': '-0.093',
                    'initial_pose_yaw': '1.417',
                }.items(),
            )
        ],
    )

    # ============================================================
    # RViz
    # ============================================================
    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d',
                    rviz_config,
                ],
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            gazebo,
            navigation,
            rviz,
        ]
    )
