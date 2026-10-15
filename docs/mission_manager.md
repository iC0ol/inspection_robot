# Mission Manager (M6)

## 1. Overview

`inspection_robot_mission` is the ROS 2 Humble mission-management package for the Inspection Robot project. It adds **sequential, waypoint-based inspection orchestration** on top of the existing M5 Nav2 navigation stack.

The current M6 implementation supports:

- Sequential dispatch of Nav2 `NavigateToPose` goals.
- Per-waypoint inspection waiting after successful navigation.
- Bounded navigation retries after an `ABORTED` result.
- Configurable handling of exhausted retries: fail the mission or skip the waypoint.
- Mission cancellation, including goals still awaiting acceptance and cancellation rejected by the server.
- Read-only mission status queries while the Mission Manager node is running.
- A configurable mock Nav2 Action Server and automated tests.

**Scope:** This is a simulation-validated, one-shot mission runner. It is not a physical emergency-stop controller and does not yet perform sensor-based inspection or persist results.

## 2. Package Structure

```text
src/inspection_robot_mission/
├── inspection_robot_mission/
│   ├── mission_model.py         # Mission and waypoint data/state transitions
│   ├── mission_manager.py       # ROS 2 node, Nav2 ActionClient, timers and services
│   └── mock_nav2_server.py      # Controllable NavigateToPose ActionServer for tests
├── test/                        # Unit, integration and exception-injection tests
├── package.xml
└── setup.py
```

The main entry points defined in `setup.py` are:

```text
mission_manager = inspection_robot_mission.mission_manager:main
mock_nav2_server = inspection_robot_mission.mock_nav2_server:main
```

## 3. Architecture and Communication

```text
          Operator / service client
              |             |
       /mission/status   /mission/cancel
              |             |
              v             v
      +---------------------------+
      |       Mission Manager     |
      |  - sequential dispatch    |
      |  - retry / skip / cancel  |
      |  - inspection wait timer  |
      +-------------+-------------+
                    |
          NavigateToPose (ActionClient)
                    |
                    v
         Nav2 bt_navigator OR
         mock_nav2_server (tests)
                    |
       Goal response / feedback / result
                    |
                    v
            InspectionMission
          (mission_model state)
```

`MissionManager` coordinates the mission; it does not implement path planning, localization, obstacle avoidance or motion control. Those are provided by the M5 Nav2 stack when using the real simulation.

`mission_model.py` owns the business state. `mission_manager.py` owns ROS communication and scheduling. `mock_nav2_server.py` provides controllable Action outcomes without launching Gazebo.

## 4. Mission and Waypoint States

Mission statuses in the model:

| MissionStatus | Meaning |
| --- | --- |
| `IDLE` | Mission has not started |
| `RUNNING` | Mission is being executed |
| `SUCCEEDED` | Every waypoint completed successfully |
| `FAILED` | Mission stopped due to failure |
| `CANCELED` | Mission canceled after cancellation handling |
| `COMPLETED_WITH_SKIPS` | Mission ended with at least one skipped waypoint |
| `PAUSED` | Reserved in the model; pause/resume workflow is not implemented |

Waypoint statuses:

| WaypointStatus | Meaning |
| --- | --- |
| `PENDING` | Not started |
| `NAVIGATING` | Current navigation attempt is active |
| `RETRY_PENDING` | Waiting before another attempt |
| `INSPECTING` | Navigation completed; inspection wait is active |
| `SUCCEEDED` | Waypoint completed |
| `FAILED` | Waypoint failed |
| `SKIPPED` | Waypoint bypassed under the configured policy |
| `CANCELED` | Waypoint canceled |

Normal sequence:

```text
PENDING -> NAVIGATING -> INSPECTING -> SUCCEEDED
                 |
                 +-- ABORTED -> RETRY_PENDING -> NAVIGATING
                 |
                 +-- retries exhausted -> FAILED or SKIPPED
```

Mission cancellation is handled separately. When a cancel request arrives during active navigation, the status service may temporarily report `mission=RUNNING; phase=CANCEL_REQUESTED` until the Action result is resolved.

## 5. Retry and Skip Policies

`max_navigation_retries` means **extra attempts after the initial attempt**, not the total number of attempts.

For example, `max_navigation_retries:=2` allows up to three attempts for one waypoint: the initial attempt plus two retries.

On a navigation `ABORTED` result:

1. If retries remain, transition the waypoint to `RETRY_PENDING`, start a delay timer, then dispatch a new goal.
2. If retries are exhausted and `retry_exhausted_policy=FAIL_MISSION`, transition the mission to `FAILED`.
3. If retries are exhausted and `retry_exhausted_policy=SKIP_WAYPOINT`, skip the waypoint and continue. If the mission reaches the end with one or more skips, its final status is `COMPLETED_WITH_SKIPS`.

Other unsuccessful Action status codes are not automatically treated as retryable `ABORTED` results.

## 6. ROS 2 Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Send navigation goals and receive results |
| `/mission/cancel` | `std_srvs/srv/Trigger` | Request cancellation of the current mission |
| `/mission/status` | `std_srvs/srv/Trigger` | Get the current mission status snapshot |

Cancel a running mission:

```bash
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
```

Read the mission status:

```bash
ros2 service call /mission/status std_srvs/srv/Trigger '{}'
```

The status response is currently a semicolon-separated string containing these fields:

```text
mission=RUNNING; phase=ACTIVE; waypoint=P1;
waypoint_status=NAVIGATING; active_goal=1;
current_index=0; total_waypoints=2
```

The status `phase` is one of `IDLE`, `ACTIVE`, `CANCEL_REQUESTED`, or `TERMINAL`. These phases describe the Mission Manager lifecycle and are distinct from `MissionStatus`.

**Important:** A successful `/mission/cancel` service response means that the cancellation request was accepted **by the Mission Manager**. It does **not** prove that Nav2 has stopped moving. The manager waits for the navigation goal's final result when possible.

The current one-shot Mission Manager destroys its ROS node after reaching a terminal state, so `/mission/status` is no longer available once the process exits. Persisted terminal status is future work.

## 7. Build and Run with Gazebo + Nav2

Build the workspace:

```bash
cd ~/inspection_robot_ws
colcon build --symlink-install
source install/setup.bash
```

Terminal A: launch the existing M5 integrated navigation simulation:

```bash
source ~/inspection_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch inspection_robot_bringup simulation_navigation.launch.py
```

Wait until Nav2 is active and localization is ready.

Terminal B: start a two-waypoint mission (**example coordinates only; replace with reachable poses in your saved map**):

```bash
source ~/inspection_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 run inspection_robot_mission mission_manager --ros-args \
  -p use_sim_time:=true \
  -p waypoints_x:='[0.0, 1.0]' \
  -p waypoints_y:='[0.0, 0.0]' \
  -p waypoints_yaw:='[0.0, 0.0]' \
  -p wait_seconds:=2.0 \
  -p max_navigation_retries:=2 \
  -p retry_delay_seconds:=2.0 \
  -p retry_exhausted_policy:=FAIL_MISSION
```

Use the **same `ROS_DOMAIN_ID`** in all terminals for the same robot system. `use_sim_time:=true` is appropriate when the goal timestamps must use Gazebo's simulated clock; for the standalone mock examples below, use `false`.

The waypoint arrays must have the same, non-zero length. Coordinates and yaw values are floating-point numbers in the `map` frame; yaw is in radians. The current implementation uses a shared `wait_seconds` value for all waypoints and implements the `wait` inspection action only.

## 8. Run with the Mock Nav2 Server

Mock tests do not require Gazebo or the full Nav2 stack. Do not run the mock server alongside the real Nav2 server in the same ROS Domain because both expose `/navigate_to_pose`.

Terminal A — mock the first attempt as `ABORTED`, then return success:

```bash
source ~/inspection_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 run inspection_robot_mission mock_nav2_server --ros-args \
  -p success_on_attempt:=2
```

Terminal B — launch the Mission Manager:

```bash
source ~/inspection_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 run inspection_robot_mission mission_manager --ros-args \
  -p use_sim_time:=false \
  -p waypoints_x:='[0.0]' \
  -p waypoints_y:='[0.0]' \
  -p waypoints_yaw:='[0.0]' \
  -p wait_seconds:=0.0 \
  -p max_navigation_retries:=1 \
  -p retry_delay_seconds:=0.2
```

Expected progression:

```text
Goal 1 -> ABORTED
Goal 2 -> SUCCEEDED
Mission: SUCCEEDED
```

Useful Mock Server parameters:

| Parameter | Example | Behavior |
| --- | --- | --- |
| `success_on_attempt` | `2` | First attempt aborts; second and subsequent attempts succeed (`0`: always abort) |
| `wait_for_cancel` | `true` | Keep a goal active until cancellation or configured automatic finish |
| `reject_cancel` | `true` | Reject cancellation requests |
| `finish_after_seconds` | `20.0` | When waiting for cancel, succeed after the specified delay |
| `goal_accept_delay_seconds` | `2.0` | Delay the Goal acceptance response |

## 9. Testing

The package contains unit tests for mission state transitions, `launch_testing` integration tests for real ROS 2 Action/Service communications, and exception-injection tests for unexpected Action Future failures.

```bash
cd ~/inspection_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select inspection_robot_mission \
  --event-handlers console_direct+
colcon test-result --verbose
```

Individual integration tests can be run via pytest, for example:

```bash
python3 -m pytest -v -s \
  src/inspection_robot_mission/test/test_mission_cancel.py
```

Integration test scenarios include:

- Retry: first attempt fails, subsequent attempt succeeds.
- Retry exhaustion: the mission transitions to `FAILED`.
- Skip: one waypoint is skipped and another succeeds.
- Cancel: active navigation ends with `CANCELED`.
- Cancel rejected: Mission Manager retains `CANCEL_REQUESTED` until the goal resolves.
- Action callback error injection and local timer cleanup.

The pytest suite uses `domain_coordinator` to reserve an available `ROS_DOMAIN_ID` for the test session. Do not hard-code domain IDs inside individual launch tests. The current session-scoped approach assumes integration tests are executed sequentially inside one pytest session.

## 10. Current Limitations and Safety Notes

- Only sequential, one-shot missions are supported; there is no persistent task queue or automatic patrol loop.
- The inspection action is currently a timed wait, not camera capture, visual inference or sensor recording.
- The status service uses `Trigger` and a formatted text response rather than a structured custom ROS interface.
- After terminal mission completion, the Mission Manager process exits and its services disappear.
- Cancel request acceptance is not proof of immediate physical stopping. If Nav2 refuses cancellation, the manager waits for the goal to finish rather than dispatching the next waypoint.
- If an Action response/result is unavailable, navigation status can remain unknown; reporting `FAILED` does not imply the robot is safely stopped.
- Physical robot deployment needs independently validated motion-stop behavior, watchdogs, and fault handling.

## 11. Related Documentation

- [`README.md`](../README.md) — overall project overview and quick start.
- [`architecture.md`](architecture.md) — M5 navigation architecture and coordinate frames.
