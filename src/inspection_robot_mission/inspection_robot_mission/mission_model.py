
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
    COMPLETED_WITH_SKIPS = "COMPLETED_WITH_SKIPS"


class WaypointStatus(Enum):
    """Represent the execution status of a waypoint."""

    PENDING = "PENDING"
    NAVIGATING = "NAVIGATING"
    INSPECTING = "INSPECTING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    SKIPPED = "SKIPPED"
    RETRY_PENDING = "RETRY_PENDING"


class RetryExhaustedPolicy(Enum):
    """Define the action taken after navigation retries run out."""

    FAIL_MISSION = "FAIL_MISSION"
    SKIP_WAYPOINT = "SKIP_WAYPOINT"


ALLOWED_TRANSITIONS = {
    WaypointStatus.PENDING: {
        WaypointStatus.NAVIGATING,
        WaypointStatus.FAILED,
    },
    WaypointStatus.NAVIGATING: {
        WaypointStatus.INSPECTING,
        WaypointStatus.FAILED,
    },
    WaypointStatus.INSPECTING: {
        WaypointStatus.SUCCEEDED,
        WaypointStatus.FAILED,
    },
    WaypointStatus.RETRY_PENDING: {
        WaypointStatus.NAVIGATING,
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

    max_navigation_retries: int = 2

    retry_exhausted_policy: RetryExhaustedPolicy = (
        RetryExhaustedPolicy.FAIL_MISSION
    )

    waypoint_states: list[WaypointStatus] = field(
        init=False
    )

    navigation_failures: list[int] = field(init=False)

    def __post_init__(self):
        """Validate and initialize mission runtime data."""
        if not self.waypoints:
            raise ValueError("Mission 至少需要一个站点")

        if not isinstance(
            self.retry_exhausted_policy, RetryExhaustedPolicy
        ):
            raise TypeError(
                "retry_exhausted_policy 必须为 RetryExhaustedPolicy"
            )

        self.waypoint_states = [
            WaypointStatus.PENDING
            for _ in self.waypoints
        ]

        if (
            type(self.max_navigation_retries) is not int
            or self.max_navigation_retries < 0
        ):
            raise ValueError("max_navigation_retries 必须是非负整数")

        self.navigation_failures = [
            0 for _ in self.waypoints
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

        if index != self.current_index:
            raise ValueError("只能修改当前站点状态")

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

        if new_status == WaypointStatus.FAILED:
            self.state = MissionStatus.FAILED

    def advance_to_next_waypoint(self):
        """Advance after completing the current waypoint."""
        if self.state != MissionStatus.RUNNING:
            raise ValueError("任务不在 RUNNING 状态")

        if self.current_index >= len(self.waypoints):
            raise ValueError("所有站点已处理完毕")

        current_status = self.waypoint_states[self.current_index]

        if current_status != WaypointStatus.SUCCEEDED:
            raise ValueError("当前站点未成功，不能推进")

        self.current_index += 1

        if self.current_index == len(self.waypoints):
            if WaypointStatus.SKIPPED in self.waypoint_states:
                self.state = MissionStatus.COMPLETED_WITH_SKIPS
            else:
                self.state = MissionStatus.SUCCEEDED

    def record_navigation_failure(self, index):
        """Record a failed navigation attempt and decide retry."""
        if self.state != MissionStatus.RUNNING:
            raise ValueError("任务不在 RUNNING 状态")

        if not 0 <= index < len(self.waypoints):
            raise IndexError("Waypoint 索引超出范围")

        if index != self.current_index:
            raise ValueError("只能处理当前站点")

        if self.waypoint_states[index] != WaypointStatus.NAVIGATING:
            raise ValueError("站点必须处于 NAVIGATING 状态")

        # 记录当前站点的一次导航失败
        self.navigation_failures[index] += 1

        failures = self.navigation_failures[index]

        # 仍然有剩余重试机会
        if failures <= self.max_navigation_retries:
            self.waypoint_states[index] = (
                WaypointStatus.RETRY_PENDING
            )
            return True

        # 超过重试预算时，允许根据策略跳过当前站点
        if (
            self.retry_exhausted_policy
            == RetryExhaustedPolicy.SKIP_WAYPOINT
        ):
            self.waypoint_states[index] = WaypointStatus.SKIPPED
            self.current_index += 1

            if self.current_index == len(self.waypoints):
                self.state = MissionStatus.COMPLETED_WITH_SKIPS

            return False

        # 默认策略：结束整个任务
        self.transition_waypoint(
            index, WaypointStatus.FAILED
        )
        return False

        # 超过重试预算，进入最终失败
        self.transition_waypoint(
            index, WaypointStatus.FAILED
        )
        return False
