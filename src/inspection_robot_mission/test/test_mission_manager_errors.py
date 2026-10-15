
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

"""Test Mission Manager behavior when Action callbacks fail."""

from types import MethodType
from types import SimpleNamespace
from unittest.mock import Mock

from inspection_robot_mission.mission_manager import MissionManager
from inspection_robot_mission.mission_model import (
    InspectionMission,
    MissionStatus,
    Waypoint,
    WaypointStatus,
)


def make_fake_manager(navigating=False, with_timers=False):
    """Create a minimal manager for callback-level testing."""
    mission = InspectionMission(
        mission_id='test_mission',
        waypoints=[Waypoint('P1', 0.0, 0.0, 0.0)],
    )
    mission.start()

    if navigating:
        mission.transition_waypoint(
            0, WaypointStatus.NAVIGATING
        )

    logger = Mock()

    manager = SimpleNamespace(
        mission=mission,
        finished=False,
        stopping=False,
        active_goal_serial=1,
        navigation_active=navigating,
        wait_timer=Mock() if with_timers else None,
        retry_timer=Mock() if with_timers else None,
        destroy_timer=Mock(),
        get_logger=Mock(return_value=logger),
    )

    # 将真实的 fail_mission 方法绑定到测试替身上。
    manager.fail_mission = MethodType(
        MissionManager.fail_mission,
        manager,
    )

    return manager, logger


def assert_unknown_navigation_warning(logger):
    """Check that unknown navigation state is reported."""
    messages = [
        call.args[0]
        for call in logger.error.call_args_list
    ]

    assert any(
        '无法确认 Nav2 Goal 已停止' in message
        for message in messages
    )


def test_goal_response_exception():
    """A failed Goal response must report uncertain navigation."""
    manager, logger = make_fake_manager()

    future = Mock()
    future.result.side_effect = RuntimeError(
        'Goal response communication error'
    )

    MissionManager.goal_response_callback(
        manager, 1, future
    )

    assert manager.finished is True
    assert manager.active_goal_serial is None
    assert manager.mission.state == MissionStatus.FAILED
    assert (
        manager.mission.waypoint_states[0]
        == WaypointStatus.FAILED
    )

    assert_unknown_navigation_warning(logger)


def test_result_exception():
    """A failed Result response must not imply safe stopping."""
    manager, logger = make_fake_manager(navigating=True)

    future = Mock()
    future.result.side_effect = RuntimeError(
        'Navigation result communication error'
    )

    MissionManager.result_callback(
        manager, 1, future
    )

    assert manager.finished is True
    assert manager.active_goal_serial is None
    assert manager.navigation_active is False
    assert manager.mission.state == MissionStatus.FAILED
    assert (
        manager.mission.waypoint_states[0]
        == WaypointStatus.FAILED
    )

    assert_unknown_navigation_warning(logger)


def test_fail_mission_cleans_timers():
    """Mission failure must release pending local timers."""
    manager, logger = make_fake_manager(
        with_timers=True
    )

    # 此处模拟没有派发 Goal 的失败情况。
    manager.active_goal_serial = None

    wait_timer = manager.wait_timer
    retry_timer = manager.retry_timer

    manager.fail_mission('Nav2 Action Server 不可用')

    assert manager.finished is True
    assert manager.mission.state == MissionStatus.FAILED

    assert manager.wait_timer is None
    assert manager.retry_timer is None

    wait_timer.cancel.assert_called_once()
    retry_timer.cancel.assert_called_once()

    assert manager.destroy_timer.call_count == 2

    # 未发送 Goal，不应该误报导航状态未知。
    messages = [
        call.args[0]
        for call in logger.error.call_args_list
    ]

    assert not any(
        '无法确认 Nav2 Goal 已停止' in message
        for message in messages
    )
