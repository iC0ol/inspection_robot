
"""Unit tests for inspection mission data models."""

import pytest

from inspection_robot_mission.mission_model import (
    InspectionMission,
    MissionStatus,
    Waypoint,
    WaypointStatus,
    RetryExhaustedPolicy,
)


def make_mission():
    """Create a sample mission for unit tests."""
    points = [
        Waypoint("P1", 0.0, 0.0, 0.0),
        Waypoint("P2", 1.0, 0.0, 0.0),
        Waypoint("P3", 1.0, 1.0, 0.0),
    ]

    return InspectionMission("mission_001", points)


def test_mission_initial_state():
    """A new mission must start with pending waypoints."""
    mission = make_mission()

    assert mission.state == MissionStatus.IDLE
    assert mission.current_index == 0
    assert len(mission.waypoints) == 3

    assert mission.waypoint_states == [
        WaypointStatus.PENDING,
        WaypointStatus.PENDING,
        WaypointStatus.PENDING,
    ]


def test_mission_start():
    """Starting a mission must change its state to RUNNING."""
    mission = make_mission()

    mission.start()

    assert mission.state == MissionStatus.RUNNING


def test_valid_waypoint_transition():
    """A waypoint must support the normal transition path."""
    mission = make_mission()
    mission.start()

    mission.transition_waypoint(
        0, WaypointStatus.NAVIGATING
    )
    mission.transition_waypoint(
        0, WaypointStatus.INSPECTING
    )
    mission.transition_waypoint(
        0, WaypointStatus.SUCCEEDED
    )

    assert (
        mission.waypoint_states[0]
        == WaypointStatus.SUCCEEDED
    )


def test_invalid_waypoint_transition():
    """An invalid transition must not alter waypoint state."""
    mission = make_mission()
    mission.start()

    with pytest.raises(ValueError, match="非法状态转换"):
        mission.transition_waypoint(
            0, WaypointStatus.SUCCEEDED
        )

    assert (
        mission.waypoint_states[0]
        == WaypointStatus.PENDING
    )


def test_transition_requires_running():
    """Waypoint transitions must require a running mission."""
    mission = make_mission()

    with pytest.raises(ValueError, match="RUNNING"):
        mission.transition_waypoint(
            0, WaypointStatus.NAVIGATING
        )


def test_empty_mission_rejected():
    """A mission without waypoints must be rejected."""
    with pytest.raises(ValueError):
        InspectionMission("empty_mission", [])


def test_mission_states_are_independent():
    """Different missions must not share runtime states."""
    mission_a = make_mission()
    mission_b = make_mission()

    mission_a.start()
    mission_a.transition_waypoint(
        0, WaypointStatus.NAVIGATING
    )

    assert (
        mission_a.waypoint_states[0]
        == WaypointStatus.NAVIGATING
    )

    assert (
        mission_b.waypoint_states[0]
        == WaypointStatus.PENDING
    )


def test_cannot_advance_before_success():
    """An unfinished waypoint must block mission progress."""
    mission = make_mission()
    mission.start()

    with pytest.raises(ValueError, match="未成功"):
        mission.advance_to_next_waypoint()

    assert mission.current_index == 0
    assert mission.state == MissionStatus.RUNNING


def test_advance_after_waypoint_success():
    """A successful waypoint must allow moving to the next."""
    mission = make_mission()
    mission.start()

    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)
    mission.transition_waypoint(0, WaypointStatus.INSPECTING)
    mission.transition_waypoint(0, WaypointStatus.SUCCEEDED)

    mission.advance_to_next_waypoint()

    assert mission.current_index == 1
    assert mission.state == MissionStatus.RUNNING
    assert mission.waypoint_states[1] == WaypointStatus.PENDING


def test_mission_completes_after_last_waypoint():
    """Completing every waypoint must finish the mission."""
    mission = make_mission()
    mission.start()

    for index in range(len(mission.waypoints)):
        mission.transition_waypoint(
            index, WaypointStatus.NAVIGATING
        )
        mission.transition_waypoint(
            index, WaypointStatus.INSPECTING
        )
        mission.transition_waypoint(
            index, WaypointStatus.SUCCEEDED
        )
        mission.advance_to_next_waypoint()

    assert mission.current_index == 3
    assert mission.state == MissionStatus.SUCCEEDED


def test_cannot_modify_future_waypoint():
    """A future waypoint must remain untouched."""
    mission = make_mission()
    mission.start()

    with pytest.raises(ValueError, match="当前站点"):
        mission.transition_waypoint(
            2, WaypointStatus.NAVIGATING
        )

    assert mission.waypoint_states[2] == WaypointStatus.PENDING


def test_navigation_failure_stops_mission():
    """A navigation failure must stop mission execution."""
    mission = make_mission()
    mission.start()

    # P1 成功
    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)
    mission.transition_waypoint(0, WaypointStatus.INSPECTING)
    mission.transition_waypoint(0, WaypointStatus.SUCCEEDED)
    mission.advance_to_next_waypoint()

    # P2 导航失败
    mission.transition_waypoint(1, WaypointStatus.NAVIGATING)
    mission.transition_waypoint(1, WaypointStatus.FAILED)

    assert mission.state == MissionStatus.FAILED
    assert mission.current_index == 1

    assert mission.waypoint_states == [
        WaypointStatus.SUCCEEDED,
        WaypointStatus.FAILED,
        WaypointStatus.PENDING,
    ]

    # 失败后不允许继续推进
    with pytest.raises(ValueError, match="RUNNING"):
        mission.advance_to_next_waypoint()


def test_inspection_failure_stops_mission():
    """An inspection failure must fail the entire mission."""
    mission = make_mission()
    mission.start()

    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)
    mission.transition_waypoint(0, WaypointStatus.INSPECTING)
    mission.transition_waypoint(0, WaypointStatus.FAILED)

    assert mission.state == MissionStatus.FAILED
    assert mission.current_index == 0
    assert mission.waypoint_states[0] == WaypointStatus.FAILED


def test_goal_dispatch_failure():
    """A goal dispatch failure must stop the mission."""
    mission = make_mission()
    mission.start()

    # 模拟 Nav2 在开始导航前拒绝目标
    mission.transition_waypoint(0, WaypointStatus.FAILED)

    assert mission.state == MissionStatus.FAILED
    assert mission.current_index == 0

    assert mission.waypoint_states == [
        WaypointStatus.FAILED,
        WaypointStatus.PENDING,
        WaypointStatus.PENDING,
    ]


def test_navigation_retry_keeps_mission_running():
    """A retryable failure must not fail the mission."""
    mission = make_mission()
    mission.start()
    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)

    should_retry = mission.record_navigation_failure(0)

    assert should_retry is True
    assert mission.navigation_failures[0] == 1
    assert mission.current_index == 0
    assert mission.state == MissionStatus.RUNNING
    assert (
        mission.waypoint_states[0]
        == WaypointStatus.RETRY_PENDING
    )


def test_navigation_retry_exhaustion():
    """Exhausting the retry budget must fail the mission."""
    mission = make_mission()
    mission.start()
    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)

    # 两次失败后仍可重试
    for _ in range(2):
        assert mission.record_navigation_failure(0) is True
        mission.transition_waypoint(
            0, WaypointStatus.NAVIGATING
        )

    # 第三次失败，重试次数耗尽
    assert mission.record_navigation_failure(0) is False

    assert mission.navigation_failures[0] == 3
    assert mission.state == MissionStatus.FAILED
    assert mission.current_index == 0
    assert mission.waypoint_states[0] == WaypointStatus.FAILED
    assert mission.waypoint_states[1] == WaypointStatus.PENDING


def test_retry_requires_navigating():
    """Only an active navigation state can record failure."""
    mission = make_mission()
    mission.start()

    with pytest.raises(ValueError, match="NAVIGATING"):
        mission.record_navigation_failure(0)

    assert mission.navigation_failures[0] == 0
    assert mission.waypoint_states[0] == WaypointStatus.PENDING


def test_zero_retries_fail_immediately():
    """A zero retry budget must preserve fail-fast behavior."""
    mission = InspectionMission(
        "mission_no_retry",
        [Waypoint("P1", 0.0, 0.0, 0.0)],
        max_navigation_retries=0,
    )

    mission.start()
    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)

    should_retry = mission.record_navigation_failure(0)

    assert should_retry is False
    assert mission.navigation_failures[0] == 1
    assert mission.state == MissionStatus.FAILED
    assert mission.waypoint_states[0] == WaypointStatus.FAILED


def test_skip_waypoint_after_retry_exhaustion():
    """Exhausted retries may skip the current waypoint."""
    mission = InspectionMission(
        "mission_skip",
        [
            Waypoint("P1", 0.0, 0.0, 0.0),
            Waypoint("P2", 1.0, 0.0, 0.0),
        ],
        max_navigation_retries=1,
        retry_exhausted_policy=RetryExhaustedPolicy.SKIP_WAYPOINT,
    )

    mission.start()
    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)

    # 第一次失败，允许再尝试一次
    assert mission.record_navigation_failure(0) is True
    assert mission.waypoint_states[0] == WaypointStatus.RETRY_PENDING

    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)

    # 第二次失败，重试预算耗尽，跳过 P1
    assert mission.record_navigation_failure(0) is False

    assert mission.navigation_failures[0] == 2
    assert mission.waypoint_states[0] == WaypointStatus.SKIPPED
    assert mission.current_index == 1
    assert mission.state == MissionStatus.RUNNING
    assert mission.waypoint_states[1] == WaypointStatus.PENDING


def test_mission_completes_after_skipped_waypoint():
    """Completing after a skip must not report full success."""
    mission = InspectionMission(
        "mission_partial",
        [
            Waypoint("P1", 0.0, 0.0, 0.0),
            Waypoint("P2", 1.0, 0.0, 0.0),
        ],
        max_navigation_retries=0,
        retry_exhausted_policy=RetryExhaustedPolicy.SKIP_WAYPOINT,
    )

    mission.start()

    # P1 导航失败并被跳过
    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)
    assert mission.record_navigation_failure(0) is False

    # P2 正常完成
    mission.transition_waypoint(1, WaypointStatus.NAVIGATING)
    mission.transition_waypoint(1, WaypointStatus.INSPECTING)
    mission.transition_waypoint(1, WaypointStatus.SUCCEEDED)
    mission.advance_to_next_waypoint()

    assert mission.current_index == 2
    assert mission.state == MissionStatus.COMPLETED_WITH_SKIPS
    assert mission.waypoint_states == [
        WaypointStatus.SKIPPED,
        WaypointStatus.SUCCEEDED,
    ]


def test_skip_last_waypoint_finishes_mission():
    """Skipping the final waypoint must finish the mission."""
    mission = InspectionMission(
        "mission_last_skip",
        [Waypoint("P1", 0.0, 0.0, 0.0)],
        max_navigation_retries=0,
        retry_exhausted_policy=RetryExhaustedPolicy.SKIP_WAYPOINT,
    )

    mission.start()
    mission.transition_waypoint(0, WaypointStatus.NAVIGATING)

    assert mission.record_navigation_failure(0) is False

    assert mission.current_index == 1
    assert mission.waypoint_states[0] == WaypointStatus.SKIPPED
    assert mission.state == MissionStatus.COMPLETED_WITH_SKIPS
