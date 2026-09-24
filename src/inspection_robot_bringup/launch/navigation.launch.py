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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch localization and Nav2 navigation servers."""
    # ============================================================
    # Launch arguments
    # ============================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_file = LaunchConfiguration('map')

    set_initial_pose = LaunchConfiguration('set_initial_pose')

    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock',
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure and activate Nav2 nodes',
    )

    default_map = PathJoinSubstitution(
        [
            FindPackageShare('inspection_robot_bringup'),
            'maps',
            'inspection_room_v1.yaml',
        ]
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file',
    )

    declare_set_initial_pose = DeclareLaunchArgument(
        'set_initial_pose',
        default_value='false',
        description='Initialize AMCL from launch parameters',
    )

    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial robot X pose in map frame',
    )

    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial robot Y pose in map frame',
    )

    declare_initial_pose_yaw = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial robot yaw in map frame',
    )

    # ============================================================
    # Parameter files
    # ============================================================
    amcl_config = PathJoinSubstitution(
        [
            FindPackageShare('inspection_robot_bringup'),
            'config',
            'amcl.yaml',
        ]
    )

    planner_config = PathJoinSubstitution(
        [
            FindPackageShare('inspection_robot_bringup'),
            'config',
            'planner.yaml',
        ]
    )

    controller_config = PathJoinSubstitution(
        [
            FindPackageShare('inspection_robot_bringup'),
            'config',
            'controller.yaml',
        ]
    )

    behavior_config = PathJoinSubstitution(
        [
            FindPackageShare('inspection_robot_bringup'),
            'config',
            'behavior.yaml',
        ]
    )

    bt_config = PathJoinSubstitution(
        [
            FindPackageShare('inspection_robot_bringup'),
            'config',
            'bt_navigator.yaml',
        ]
    )

    # ============================================================
    # Localization
    # ============================================================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_file,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {
                'use_sim_time': use_sim_time,

                'set_initial_pose': ParameterValue(
                    set_initial_pose,
                    value_type=bool,
                ),

                'initial_pose.x': ParameterValue(
                    initial_pose_x,
                    value_type=float,
                ),

                'initial_pose.y': ParameterValue(
                    initial_pose_y,
                    value_type=float,
                ),

                'initial_pose.yaw': ParameterValue(
                    initial_pose_yaw,
                    value_type=float,
                ),
            },
        ],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'map_server',
                    'amcl',
                ],
            }
        ],
    )

    # ============================================================
    # Planner
    # ============================================================
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            planner_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ============================================================
    # Controller
    # ============================================================
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            controller_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            (
                'cmd_vel',
                '/diff_drive_controller/cmd_vel_unstamped',
            ),
        ],
    )

    # ============================================================
    # Recovery / Behaviors
    # ============================================================
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            behavior_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            (
                'cmd_vel',
                '/diff_drive_controller/cmd_vel_unstamped',
            ),
        ],
    )

    # ============================================================
    # BT Navigator
    # ============================================================
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            bt_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_autostart,
            declare_map,

            declare_set_initial_pose,
            declare_initial_pose_x,
            declare_initial_pose_y,
            declare_initial_pose_yaw,

            map_server,
            amcl,
            lifecycle_manager_localization,

            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager_navigation,
        ]
    )
