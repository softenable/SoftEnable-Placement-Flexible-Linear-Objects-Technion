import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import os
import time

# Get the directory of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))

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

# Set neutral configuration
q = pin.neutral(model)
viz.display(q)

# Add frame marker for placement_link
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

frame_id = model.getFrameId("placement_link")
viz.viewer[f"frames/placement_link"].set_object(
    pin.visualize.meshcat_visualizer.meshcat.geometry.triad(0.1)
)
viz.viewer[f"frames/placement_link"].set_transform(data.oMf[frame_id].homogeneous)

print("\nVisualization is ready. Press Ctrl+C to exit.")
print(f"Open the viewer at: {viz.viewer.url()}")

# Keep the script running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")