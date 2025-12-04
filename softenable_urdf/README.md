# SoftEnable URDF

This folder has the SoftEnable URDF along with relevant STLs. For convience a script is provided for visualize it using pinocchio.

## Installation

To install the dependencies create a virtual environment and do the following:

Inside `softenable_urdf` folder run:

```bash
python3 -m venv .venv --prompt "technion_urdf"
source .venv/bin/activate
pip install -r requirements.txt
```

The frame for placement planning is called: `placement_link`.

## Visualizing Joint Trajectories

The `visualize_urdf.py` script can load and animate joint trajectories from text files.

### Basic Usage

**Static visualization (neutral pose):**
```bash
python3 visualize_urdf.py
```

**Animate a trajectory:**
```bash
python3 visualize_urdf.py --trajectory joint_trajectory.txt
```

If you don't have a joint trajectory folder run the placement planner first:

```bash
cd code
python3 main.py
```

**Customize animation speed:**
```bash
python3 visualize_urdf.py --trajectory trajectory.txt --fps 60
```

### Trajectory File Format

The trajectory file should be a text file where:
- Each row represents one timestep
- Each row contains joint values (space or comma separated)
- For the 7-DOF IIWA robot, each row should have 7 values (one per joint)
- Lines starting with `#` are treated as comments

Example `trajectory.txt`:
```
# Joint positions: j1 j2 j3 j4 j5 j6 j7
0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.1 0.0 0.0 0.0 0.0 0.0 0.0
0.2 0.1 0.0 0.0 0.0 0.0 0.0
...
```

### Command Line Options

- `--trajectory, -t PATH`: Path to the trajectory text file
- `--fps FPS`: Animation frames per second (default: 30)