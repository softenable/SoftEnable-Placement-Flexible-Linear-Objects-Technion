# SoftEnable Placement - Flexible Linear Objects 

A Python library for planning the placement of flexible linear objects using robotic manipulation. This project implements a three-phase approach based on Euler's Elastica theory for deformable object handling, specifically designed for applications in food processing, textile handling, and cable management.

## Quick Start

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Run basic scenarios:**
   ```bash
   python run_planner.py --visualize
   ```

3. **Benchmark IK performance:**
   ```bash
   python benchmark_ik_performance.py
   ```

## Project Structure

```
├── src/                           # Main source code
│   ├── __init__.py               # Python package initialization
│   ├── main.py                   # PlacementPlanner class and entry point
│   ├── displacement.py           # A* path planning for three sub-problems
│   ├── node_class.py             # Node, Target, and Bound class definitions
│   └── utils.py                  # IK solvers and utility functions
├── tests/                        # Comprehensive test suite
│   ├── test_ik_solvers.py        # IK solver validation and comparison
│   └── test_ik_comprehensive.py  # Comprehensive IK testing
├── urdf/                        # URDF files for robot models
│   ├── softenable_iiwa.urdf     # KUKA iiwa14 robot model file
│   └── softenable.urdf           # Complete SoftEnable setup
├── benchmark_ik_performance.py   # IK solver performance benchmarking
├── run_planner.py               # Command-line interface for planning
├── requirements.txt             # Python dependencies
├── LICENSE                      # Project license
├── .gitignore                   # Git ignore patterns
└── README.md                    # This file
```

## Usage

### Command Line Interface

The main planning interface is available through `run_planner.py`:

```bash
python run_planner.py [OPTIONS]

Options:
  --urdf PATH          Path to robot URDF file (default: softenable.urdf)
  --direction CHOICE   Movement direction: 'right' or 'left' (default: right)
  --length FLOAT       Object length in meters (default: 0.3)
  --ik-solver CHOICE   IK solver: 'legacy', 'fast', or 'robust' (default: fast)
  --visualize         Show path visualization
  --benchmark         Run IK solver benchmark
  --profile           Enable performance profiling (default: enabled)
  --help              Show help message
```

Alternative entry points:
- `python src/main.py` - Direct access to PlacementPlanner class

### IK Solver Options

The project includes three IK solvers with different performance characteristics:

- **`legacy`** - Original implementation
- **`fast`** - Optimized solver with joint limits and warm starting
- **`robust`** - Multiple-attempt solver for difficult poses

### Programmatic Usage

```python
from src.main import PlacementPlanner

# Create planner with fast IK solver
planner = PlacementPlanner("path/to/robot.urdf", ik_solver='fast')

# Define scenario
start_pose = {
    'position': [0.4, 0.2, 0.09],
    'orientation': np.pi/2
}

target_poses = [{
    'position': [0.5, 0.2, 0.0],
    'orientation': 0
}]

object_params = {'L': 0.3}  # 30cm object
env_params = {
    'mu_fric': 0.1,      # Friction coefficient
    'h_table': 0.0,      # Table height
    'alpha': 0.6         # Stiffness parameter
}

# Plan placement with profiling
success, paths, execution_time = planner.plan_placement(
    start_pose, target_poses, object_params, env_params, "right", profile=True
)

if success:
    print(f"Planning completed in {execution_time:.3f}s")
    
    # Get detailed performance statistics
    stats = planner.get_performance_stats()
    print(f"IK solver made {stats['ik_stats']['total_solves']} calls")
    print(f"Average IK time: {stats['ik_stats']['avg_solve_time']:.6f}s")
    
    planner.visualize_paths(paths, object_params, "right")
```

### Performance Benchmarking

Comprehensive benchmarking tools are provided:

```bash
# Benchmark IK solvers standalone
python benchmark_ik_performance.py

# Benchmark via main interface
python src/main.py --benchmark
```


## Dependencies

- Python 3.8+
- NumPy ≥1.20.0
- Matplotlib ≥3.5.0
- SciPy ≥1.8.0
- Pinocchio ≥2.7.0 (robot dynamics and kinematics)
- Plotly ≥5.0.0 (advanced visualization)
- Lark (parsing)
- Pytest (testing framework)

Install all dependencies with:
```bash
pip install -r requirements.txt
```

**Note:** Pinocchio installation may require additional system dependencies. See the [Pinocchio documentation](https://stack-of-tasks.github.io/pinocchio/download.html) for platform-specific installation instructions.

## Testing

The project includes a comprehensive test suite to validate functionality and performance:

### Running Tests

```bash
# Run all tests
python -m pytest tests/

# Run specific test modules
python -m pytest tests/test_ik_solvers.py -v
python -m pytest tests/test_ik_comprehensive.py -v

# Run IK performance benchmark
python benchmark_ik_performance.py
```
