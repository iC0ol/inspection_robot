
"""Data models and state transitions for inspection missions."""

from dataclasses import dataclass, field
from enum import Enum


class MissionStatus(Enum):
    """Represent the lifecycle status of a mission."""

    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class WaypointStatus(Enum):
    """Represent the execution status of a waypoint."""

    PENDING = "PENDING"
    NAVIGATING = "NAVIGATING"
    INSPECTING = "INSPECTING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    SKIPPED = "SKIPPED"


ALLOWED_TRANSITIONS = {
    WaypointStatus.PENDING: {
        WaypointStatus.NAVIGATING,
    },
    WaypointStatus.NAVIGATING: {
        WaypointStatus.INSPECTING,
        WaypointStatus.FAILED,
    },
    WaypointStatus.INSPECTING: {
        WaypointStatus.SUCCEEDED,
        WaypointStatus.FAILED,
    },
    WaypointStatus.SUCCEEDED: set(),
    WaypointStatus.FAILED: set(),
    WaypointStatus.SKIPPED: set(),
}


@dataclass(frozen=True)
class Waypoint:
    """Describe a waypoint and its configured inspection action."""

    name: str
    x: float
    y: float
    yaw: float
    wait_seconds: float = 0.0
    action: str = "wait"


@dataclass
class InspectionMission:
    """Maintain mission data and waypoint execution states."""

    mission_id: str
    waypoints: list[Waypoint]

    state: MissionStatus = MissionStatus.IDLE
    current_index: int = 0

    waypoint_states: list[WaypointStatus] = field(
        init=False
    )

    def __post_init__(self):
        """Validate and initialize mission runtime data."""
        if not self.waypoints:
            raise ValueError("Mission 至少需要一个站点")

        self.waypoint_states = [
            WaypointStatus.PENDING
            for _ in self.waypoints
        ]

    def start(self):
        """Start a mission from the IDLE state."""
        if self.state != MissionStatus.IDLE:
            raise ValueError("只有 IDLE 状态才能开始任务")

        self.state = MissionStatus.RUNNING

    def transition_waypoint(self, index, new_status):
        """Validate and apply a waypoint state transition."""
        if self.state != MissionStatus.RUNNING:
            raise ValueError("任务不在 RUNNING 状态")

        if not 0 <= index < len(self.waypoints):
            raise IndexError("Waypoint 索引超出范围")

        if not isinstance(new_status, WaypointStatus):
            raise TypeError("new_status 必须为 WaypointStatus")

        old_status = self.waypoint_states[index]
        allowed = ALLOWED_TRANSITIONS[old_status]

        if new_status not in allowed:
            raise ValueError(
                f"非法状态转换: "
                f"{old_status.value} -> {new_status.value}"
            )

        self.waypoint_states[index] = new_status
