# Inspection Robot

A ROS 2 based autonomous mobile robot project for navigation and inspection tasks.

The project is developed incrementally from simulation toward a low-cost physical differential-drive robot. The current version focuses on a complete autonomous navigation pipeline in Gazebo Classic using ROS 2 Humble and Nav2.

## Current Status

Milestone M5: Autonomous Navigation

Implemented and validated:

* Differential-drive robot simulation in Gazebo Classic
* `ros2\_control` based wheel control
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
                       \\       /
                        \\     /
                         v   v
                     BT Navigator
                          |
                          v
                    NavigateToPose
                          |
                          v
                      cmd\_vel
                          |
                          v
               diff\_drive\_controller
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
odom -> base\_footprint
```

The complete TF chain is:

```text
map
 └── odom
      └── base\_footprint
           └── base\_link
                └── sensor frames
```

More details are available in [`docs/architecture.md`](docs/architecture.md).

## Quick Start

Build the workspace:

```bash
cd \~/inspection\_robot\_ws
colcon build --symlink-install
source install/setup.bash
```

Launch the complete simulation and navigation system:

```bash
ros2 launch inspection\_robot\_bringup simulation\_navigation.launch.py
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

## Navigation Components

### Localization

AMCL estimates the robot pose in the saved map and publishes:

```text
map -> odom
```

The EKF publishes:

```text
odom -> base\_footprint
```

Together they provide the global robot pose used by Nav2.

### Global Planning

The project currently uses:

```text
nav2\_navfn\_planner/NavfnPlanner
```

The planner operates on the global costmap and produces the global path.

### Local Control

The local controller uses:

```text
dwb\_core::DWBLocalPlanner
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

`bt\_navigator` coordinates high-level navigation using:

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
inspection\_robot\_bringup/
├── launch/
│   ├── gazebo.launch.py
│   ├── navigation.launch.py
│   └── simulation\_navigation.launch.py
│
├── config/
│   ├── controllers.yaml
│   ├── ekf.yaml
│   ├── amcl.yaml
│   ├── planner.yaml
│   ├── controller.yaml
│   ├── behavior.yaml
│   └── bt\_navigator.yaml
│
├── maps/
│   └── inspection\_room\_v1.yaml
│
├── rviz/
│   └── navigation.rviz
│
├── scripts/
│   └── plan\_and\_follow.py
│
└── test/
    ├── test\_controllers\_config.py
    ├── test\_diff\_drive\_runtime.test.py
    └── test\_nav2\_config.py
```

`plan\_and\_follow.py` is retained as a component-level diagnostic tool that bypasses BT Navigator and directly tests the Planner-to-Controller pipeline.

## Testing

Run the project tests with:

```bash
cd \~/inspection\_robot\_ws

colcon test \\
  --packages-select inspection\_robot\_bringup inspection\_robot\_sensors

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

## Known Limitations

The current implementation is primarily validated in simulation.

Current limitations include:

* AMCL simulation initialization is tied to the current saved map and spawn pose
* the simulated IMU is currently generated from wheel motion rather than a physically independent IMU sensor model
* dynamic obstacle handling currently relies on Nav2 costmaps and replanning
* hardware deployment has not yet been completed

These limitations will be addressed in later milestones.

## Roadmap

Planned future work includes:

* navigation system robustness improvements
* waypoint-based inspection missions
* autonomous patrol logic
* inspection task management
* hardware abstraction and real robot deployment
* physical IMU and sensor integration
* fault handling and diagnostics
* visualization and inspection-result interfaces

## License

Apache License 2.0