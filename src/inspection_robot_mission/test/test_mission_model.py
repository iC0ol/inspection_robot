
"""Unit tests for inspection mission data models."""

import pytest

from inspection_robot_mission.mission_model import (
    InspectionMission,
    MissionStatus,
    Waypoint,
    WaypointStatus,
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
