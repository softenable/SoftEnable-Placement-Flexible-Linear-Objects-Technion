# Displacement Planner - ROS 2

A ROS 2 service for planning flexible object displacement trajectories. This package provides both a ROS 2 service interface and standalone capabilities for planning the manipulation of flexible linear objects. The original implementation can be found at: [SoftEnable-Placement-Flexible-Linear-Objects-Technion](https://github.com/softenable/SoftEnable-Placement-Flexible-Linear-Objects-Technion)

## Overview

The displacement planner implements a three-phase algorithm for flexible object manipulation:

1. **Placement Phase**: Initial object placement on the surface
2. **Rolling Phase**: Rolling the object to change orientation  
3. **Final Placement**: Precise positioning of the object

The planner uses Pinocchio for robot kinematics and physics-based models for flexible object behaviour.

## Installation and Usage

### Option 1: Local Installation

#### Prerequisites

- ROS 2 (Humble or newer)
- Python 3.8+
- Required Python packages: numpy, scipy, matplotlib, pinocchio

#### Building the Package

```bash
# Clone the repository
git clone <repository-url>
cd softenable_placement_flexible_linear_object_ocado

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select displacement_planner

# Source the workspace
source install/setup.bash
```

#### Running Locally

**Start the Service:**

```bash
# Using launch file (recommended)
ros2 launch displacement_planner displacement_planner_launch.py

# Or directly
ros2 run displacement_planner planner
```

**Test the Service:**

```bash
# Run example client
ros2 run displacement_planner example_client

# Or call service manually
ros2 service call /placement/plan_trajectory displacement_flexible_linear_objects_msgs/srv/GetObjectPlacementTrajectory "{
  target_length: 0.25,
  distance_from_tip: 0.05,
  roll_direction: 'left',
  start_pose: {
    position: {x: 0.4, y: 0.2, z: 0.3},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  target_pose: {
    position: {x: 0.5, y: 0.2, z: 0.1}, 
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  start_state: {
    name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'],
    position: [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0]
  }
}"
```

### Option 2: Docker Installation

#### Building the Docker Image

```bash
# Navigate to the project root
cd softenable_placement_flexible_linear_object_ocado

# Build the Docker image
./build.sh
```

#### Running with Docker

**Start the Service:**

```bash
# Start the displacement planner service
./run.sh ros2 launch displacement_planner displacement_planner_launch.py
```

**Test the Service (in another terminal):**

```bash
# Run example client
./run.sh ros2 run displacement_planner example_client

# Or call service manually
./run.sh ros2 service call /placement/plan_trajectory displacement_flexible_linear_objects_msgs/srv/GetObjectPlacementTrajectory "{
  target_length: 0.25,
  distance_from_tip: 0.05,
  roll_direction: 'left',
  start_pose: {
    position: {x: 0.4, y: 0.2, z: 0.3},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  target_pose: {
    position: {x: 0.5, y: 0.2, z: 0.1}, 
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  start_state: {
    name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'],
    position: [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0]
  }
}"
```

**Interactive Docker Session:**

```bash
# Start an interactive session for development
./run.sh bash

# Inside the container, you can run:
colcon build --packages-select displacement_planner
source install/setup.bash
ros2 launch displacement_planner displacement_planner_launch.py
```

### Standalone Usage

The displacement algorithm can also be used standalone without ROS:

```bash
# Local usage
python3 -m displacement_planner.displacement.main /path/to/robot.urdf

# Docker usage
./run.sh python3 -m displacement_planner.displacement.main /ws/src/displacement_planner/config/softenable.urdf

# Or use the console script
ros2 run displacement_planner displacement_calculator /path/to/robot.urdf
```

## Configuration

### Service Parameters

The service can be configured via ROS parameters in `config/displacement_planner.yaml`:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `urdf_file_path` | string | `$(find displacement_planner)/config/softenable.urdf` | Robot URDF file path |
| `object_length` | double | 0.25 | Default object length (metres) |
| `friction_coefficient_object_tray` | double | 0.5 | Friction between object and tray |
| `friction_coefficient_tray_table` | double | 0.1 | Friction between tray and table |
| `table_height` | double | 0.0 | Table height relative to world frame |
| `alpha` | double | 0.6 | Planning algorithm parameter |
| `tip_link` | string | `dlr_gripper_center` | Robot end-effector link name |

### Service Interface

**Service Name**: `/placement/plan_trajectory`

**Service Type**: `displacement_flexible_linear_objects_msgs/srv/GetObjectPlacementTrajectory`

#### Request Fields

- `target_length` (float32): Length of the flexible object in metres
- `distance_from_tip` (float32): Distance from robot tip to object attachment point
- `roll_direction` (string): Rolling direction ("left" or "right")
- `start_state` (sensor_msgs/JointState): Initial robot joint configuration
- `start_pose` (geometry_msgs/Pose): Initial 3D pose of the object
- `target_pose` (geometry_msgs/Pose): Target 3D pose for object placement

#### Response Fields

- `joint_trajectory` (trajectory_msgs/JointTrajectory): Planned robot joint trajectory
- `cartesian_trajectory` (trajectory_msgs/MultiDOFJointTrajectory): End-effector cartesian trajectory

## Visualisation

The service publishes trajectory visualisation markers to `/visualization/trajectory_marker` for RViz:

- **Red spheres**: Sub-problem 1 (placement phase)
- **Green spheres**: Sub-problem 2 (rolling phase)  
- **Blue spheres**: Sub-problem 3 (final placement)

## Docker Environment Details

The Docker setup includes:

- **Base Image**: ROS 2 Humble with development tools
- **Dependencies**: All required packages including Pinocchio, NumPy, SciPy
- **Workspace**: Mounted to `/ws` inside the container
- **Display**: X11 forwarding enabled for GUI applications
- **Networking**: Host networking for ROS communication

### Docker Volumes

The `run.sh` script automatically mounts:
- `./src:/ws/src/` - Source code
- `./configs:/ws/configs/` - Configuration files
- X11 display for GUI applications

## Algorithm Details

The displacement planner uses an A* search algorithm with elastic rod physics to plan feasible trajectories for flexible object manipulation. The algorithm considers:

- Object elasticity and bending constraints
- Friction between object, tray, and table
- Robot kinematic constraints using Pinocchio
- Collision avoidance
- Smooth trajectory generation

## Dependencies

- ROS 2 (Humble or newer)
- Pinocchio (for robot kinematics)
- displacement_flexible_linear_objects_msgs (service definitions)
- Standard ROS 2 message packages (geometry_msgs, trajectory_msgs, etc.)
- Python packages: numpy, scipy, matplotlib

## Troubleshooting

### Common Issues

1. **Service not starting**: Check that all dependencies are installed and the URDF file path is correct
2. **Docker display issues**: Ensure X11 forwarding is properly configured with `xhost +local:docker`
3. **Planning failures**: Verify that start and target poses are physically reachable and collision-free

### Debug Mode

For debugging, increase logging verbosity:

```bash
# Local
ros2 launch displacement_planner displacement_planner_launch.py --ros-args --log-level DEBUG

# Docker
./run.sh ros2 launch displacement_planner displacement_planner_launch.py --ros-args --log-level DEBUG
```
