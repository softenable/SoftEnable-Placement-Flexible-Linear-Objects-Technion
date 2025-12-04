import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import os
import time
import numpy as np
import argparse

def load_trajectory(filename):
    """
    Load joint trajectory from a text file.
    Expected format: Each row contains joint positions for one timestep.
    For a robot with 7 joints, each row should have 7 values (space or comma separated).
    The file can also include a free-flyer base (7 extra values: x, y, z, qx, qy, qz, qw).
    
    Returns:
        numpy array of shape (n_timesteps, n_joints)
    """
    try:
        data = np.loadtxt(filename)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        print(f"Loaded trajectory with shape: {data.shape}")
        print(f"  {data.shape[0]} timesteps, {data.shape[1]} values per timestep")
        return data
    except Exception as e:
        print(f"Error loading trajectory file: {e}")
        return None

def animate_trajectory(viz, model, data, trajectory, fps=30, show_frame=True):
    """
    Animate the robot following the joint trajectory.
    
    Args:
        viz: MeshcatVisualizer instance
        model: Pinocchio model
        data: Pinocchio data
        trajectory: numpy array of shape (n_timesteps, n_values)
        fps: frames per second for animation
        show_frame: whether to show the placement_link frame
    """
    n_timesteps = trajectory.shape[0]
    dt = 1.0 / fps
    
    print(f"\nAnimating trajectory with {n_timesteps} timesteps at {fps} fps")
    print("Press Ctrl+C to stop the animation")
    
    try:
        # Get configuration indices for the joints (do this once outside the loop)
        joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4",
                       "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
        joint_ids = [model.getJointId(name) for name in joint_names]
        joint_q_indices = [model.joints[jid].idx_q for jid in joint_ids]
        
        print('Starting animation in 5 seconds...')
        time.sleep(5)
        for i in range(n_timesteps):
            q = trajectory[i, :]
            q_full = pin.neutral(model)
            
            # Set the joint values at their configuration indices
            for j, q_idx in enumerate(joint_q_indices):
                q_full[q_idx] = q[j]
            
            viz.display(q_full)
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("\n\nAnimation stopped by user")

# Get the directory of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Parse command line arguments
parser = argparse.ArgumentParser(description='Visualize URDF and animate joint trajectory')
parser.add_argument('--trajectory', '-t', type=str, help='Path to trajectory file (txt)')
parser.add_argument('--fps', type=int, default=30, help='Frames per second for animation (default: 30)')
parser.add_argument('--no-frame', action='store_true', help='Don\'t show placement_link frame marker')
args = parser.parse_args()

# Find URDF file in the same folder
urdf_files = [f for f in os.listdir(current_dir) if f.endswith('.urdf')]

if not urdf_files:
    print("No URDF file found in the current directory")
    exit(1)

urdf_path = os.path.join(current_dir, urdf_files[0])
print(f"Loading URDF: {urdf_path}")

# Load the URDF model with mesh directory
mesh_dir = current_dir
model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
data = model.createData()

# Load the visual model with mesh directory
visual_model = pin.buildGeomFromUrdf(model, urdf_path, pin.GeometryType.VISUAL, mesh_dir)
viz = MeshcatVisualizer(model, collision_model=None, visual_model=visual_model)

# Initialize the viewer
viz.initViewer()
viz.loadViewerModel()

print(f"\nViewer ready at: {viz.viewer.url()}")

# If trajectory file provided, animate it
if args.trajectory:
    trajectory = load_trajectory(args.trajectory)
    if trajectory is not None:
        # Display initial configuration
        q_init = trajectory[0, :]
        if len(q_init) < model.nq:
            q_full = pin.neutral(model)
            q_full[-len(q_init):] = q_init
            q_init = q_full
        elif len(q_init) > model.nq:
            q_init = q_init[:model.nq]
        
        viz.display(q_init)
        print("\nDisplaying initial configuration. Animation will start in 2 seconds...")
        time.sleep(2)
        
        animate_trajectory(viz, model, data, trajectory, fps=args.fps, show_frame=not args.no_frame)
else:
    # No trajectory file, just display neutral configuration
    q = pin.neutral(model)
    viz.display(q)
    
    # Add frame marker for placement_link
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    try:
        frame_id = model.getFrameId("placement_link")
        viz.viewer[f"frames/placement_link"].set_object(
            pin.visualize.meshcat_visualizer.meshcat.geometry.triad(0.1)
        )
        viz.viewer[f"frames/placement_link"].set_transform(data.oMf[frame_id].homogeneous)
    except:
        print("Warning: placement_link frame not found")
    
    print("\nNo trajectory file provided. Displaying neutral configuration.")
    print("Usage: python visualize_urdf.py --trajectory <trajectory_file.txt> [--fps 30]")
    print("\nPress Ctrl+C to exit.")
    
    # Keep the script running
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting...")