# Inspection Robot

A ROS 2 based autonomous mobile robot project for navigation and inspection tasks.

The project is developed incrementally from simulation toward a low-cost physical differential-drive robot. M5 provides autonomous navigation in Gazebo Classic using ROS 2 Humble and Nav2; M6 adds sequential waypoint inspection missions, retry/skip policies, cancellation, and status reporting.

## Current Status

Milestone M6: Inspection Mission Management (built on M5 autonomous navigation)

Implemented and validated:

* Differential-drive robot simulation in Gazebo Classic
* `ros2_control` based wheel control
* Robot state publishing and TF tree
* EKF-based odometry fusion
* Static map loading
* AMCL localization
* Global and local costmaps
* Static, obstacle, and inflation costmap layers
* NavFn global planning
* DWB local trajectory control
* Behavior Server
* BT Navigator
* `NavigateToPose`
* Dynamic obstacle avoidance
* Global path replanning
* Lifecycle-managed Nav2 startup
* Automatic AMCL initialization for simulation
* One-command simulation navigation startup
* Automated configuration and runtime tests
* Sequential multi-waypoint mission execution using `NavigateToPose`
* Configurable bounded retries (`FAIL_MISSION` / `SKIP_WAYPOINT`)
* Mission cancellation and live status services
* Standalone mock Nav2 server and automated mission tests

## Environment

Tested with:

* Ubuntu 22.04
* ROS 2 Humble
* Gazebo Classic 11
* Nav2

## System Architecture

The main navigation data flow is:

```text
                         Saved Map
                            |
                            v
                       Map Server
                            |
                            v
LaserScan ----------->    AMCL
                            |
                        map -> odom
                            |
                            v
                       TF Tree
                            |
                            v
                 Global / Local Costmap
                       |         |
                       v         v
                  NavFn       DWB
                 Planner    Controller
                       \       /
                        \     /
                         v   v
                     BT Navigator
                          |
                          v
                    NavigateToPose
                          |
                          v
                      cmd_vel
                          |
                          v
               diff_drive_controller
                          |
                          v
                         Robot
```

Odometry is provided through:

```text
Wheel Odometry + IMU
         |
         v
        EKF
         |
         v
odom -> base_footprint
```

The complete TF chain is:

```text
map
 └── odom
      └── base_footprint
           └── base_link
                └── sensor frames
```

More details are available in [`docs/architecture.md`](docs/architecture.md).
For the mission controller, see [`docs/mission_manager.md`](docs/mission_manager.md).

## Quick Start

Build the workspace:

```bash
cd ~/inspection_robot_ws
colcon build --symlink-install
source install/setup.bash
```

Launch the complete simulation and navigation system:

```bash
ros2 launch inspection_robot_bringup simulation_navigation.launch.py
```

The launch file starts:

* Gazebo
* robot model
* controllers
* EKF
* Map Server
* AMCL
* Planner Server
* Controller Server
* Behavior Server
* BT Navigator
* lifecycle managers
* RViz

After startup, use **2D Goal Pose** in RViz to send a navigation goal.

The simulation uses a predefined AMCL initial pose corresponding to the current inspection-room map.

### Run a waypoint inspection mission (M6)

Start the integrated simulation above and wait until Nav2 localization and navigation are ready. In a second terminal, use the **same `ROS_DOMAIN_ID`**:

```bash
cd ~/inspection_robot_ws
source install/setup.bash
# Use the same ROS Domain as Nav2; change 73 if your Nav2 uses another domain.
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

**The coordinates above are examples, not guaranteed reachable poses.**
Replace them with valid map-frame coordinates in your map.
`max_navigation_retries` counts additional attempts after the first attempt.

While the mission runs, query or request cancellation from another terminal in the same ROS Domain:

```bash
ros2 service call /mission/status std_srvs/srv/Trigger '{}'
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
```

The manager exits after the mission reaches a terminal state; the status service then disappears.
A successful cancel-service response acknowledges a **request**, not a confirmed physical stop.
For a Gazebo-free mock example, see [Mission Manager documentation](docs/mission_manager.md).

## Navigation Components

### Localization

AMCL estimates the robot pose in the saved map and publishes:

```text
map -> odom
```

The EKF publishes:

```text
odom -> base_footprint
```

Together they provide the global robot pose used by Nav2.

### Global Planning

The project currently uses:

```text
nav2_navfn_planner/NavfnPlanner
```

The planner operates on the global costmap and produces the global path.

### Local Control

The local controller uses:

```text
dwb_core::DWBLocalPlanner
```

DWB evaluates candidate robot trajectories while considering:

* global path tracking
* target direction
* obstacle distance
* robot motion limits

### Costmaps

Global Costmap:

* `map` frame
* static map
* laser obstacle layer
* inflation layer
* non-rolling

Local Costmap:

* `odom` frame
* 3 m × 3 m rolling window
* laser obstacle layer
* inflation layer

### Behavior Tree Navigation

`bt_navigator` coordinates high-level navigation using:

* path computation
* path following
* costmap clearing
* spin
* backup
* wait
* recovery behaviors

The navigation pipeline supports periodic replanning when the environment changes.

## Project Structure

Important files:

```text
inspection_robot_bringup/
├── launch/
│   ├── gazebo.launch.py
│   ├── navigation.launch.py
│   └── simulation_navigation.launch.py
│
├── config/
│   ├── controllers.yaml
│   ├── ekf.yaml
│   ├── amcl.yaml
│   ├── planner.yaml
│   ├── controller.yaml
│   ├── behavior.yaml
│   └── bt_navigator.yaml
│
├── maps/
│   └── inspection_room_v1.yaml
│
├── rviz/
│   └── navigation.rviz
│
├── scripts/
│   └── plan_and_follow.py
│
└── test/
    ├── test_controllers_config.py
    ├── test_diff_drive_runtime.test.py
    └── test_nav2_config.py
```

`plan_and_follow.py` is retained as a component-level diagnostic tool that bypasses BT Navigator and directly tests the Planner-to-Controller pipeline.

### Mission package (M6)

```text
src/inspection_robot_mission/
├── inspection_robot_mission/
│   ├── mission_model.py
│   ├── mission_manager.py
│   └── mock_nav2_server.py
├── test/
│   ├── conftest.py
│   ├── test_mission_model.py
│   ├── test_mission_manager_errors.py
│   ├── test_mission_retry.py
│   ├── test_mission_retry_exhaustion.py
│   ├── test_mission_skip.py
│   ├── test_mission_cancel.py
│   └── test_mission_cancel_rejected.py
├── package.xml
└── setup.py
```

## Testing

Run the project tests with:

```bash
cd ~/inspection_robot_ws

colcon test \
  --packages-select inspection_robot_bringup inspection_robot_sensors inspection_robot_mission

colcon test-result --verbose
```

The test suite currently checks areas including:

* controller configuration
* differential-drive runtime behavior
* AMCL frame configuration
* global and local costmap configuration
* robot footprint
* NavFn planner configuration
* DWB controller configuration
* Behavior Server configuration
* BT Navigator configuration
* mission state-machine transitions and retry/skip policies
* mocked Action integration: retry, exhaustion, skip and cancellation
* rejected cancellation and exception-injection handling

## Known Limitations

The current implementation is primarily validated in simulation.

Current limitations include:

* AMCL simulation initialization is tied to the current saved map and spawn pose
* the simulated IMU is currently generated from wheel motion rather than a physically independent IMU sensor model
* dynamic obstacle handling currently relies on Nav2 costmaps and replanning
* hardware deployment has not yet been completed
* M6 currently provides one-shot mission execution, not a persistent scheduler
* inspection is currently a timed wait, not sensor data acquisition
* mission history and terminal status are not persisted after node exit
* Mission `FAILED` / `CANCELED` must not be treated as proof of physical stopping

These limitations will be addressed in later milestones.

## Roadmap

Planned future work includes:

* navigation system robustness improvements
* mission configuration files and persistent history
* periodic patrol scheduling and pause/resume
* real sensor inspection actions and result storage
* hardware abstraction and real robot deployment
* physical IMU and sensor integration
* fault handling and diagnostics
* visualization and inspection-result interfaces

## License

Apache License 2.0