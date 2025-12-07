import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import os
import time
import meshcat.geometry as g
import matplotlib.pyplot as plt
import utils as hf


def plot_static_robot_config(q, X0, Y0, Z0):
    """
    Display a static robot configuration q using Meshcat.
    The function does NOT block the rest of the program.
    """

    # ---------------------------------------------------------------
    # 1. Locate URDF file
    # ---------------------------------------------------------------
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_files = [f for f in os.listdir(current_dir) if f.endswith(".urdf")]

    if not urdf_files:
        print("ERROR: No URDF file found in this directory.")
        return

    urdf_path = os.path.join(current_dir, urdf_files[0])
    print(f"Loading URDF: {urdf_path}")

    # ---------------------------------------------------------------
    # 2. Load robot model
    # ---------------------------------------------------------------
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()

    q = np.asarray(q, dtype=float)
    if len(q) != model.nq:
        raise ValueError(f"Wrong q dimension: got {len(q)}, expected {model.nq}")

    # ---------------------------------------------------------------
    # 3. Load visual model
    # ---------------------------------------------------------------
    mesh_dir = [current_dir]
    visual_model = pin.buildGeomFromUrdf(
        model, urdf_path, pin.GeometryType.VISUAL, mesh_dir
    )

    viz = MeshcatVisualizer(model, visual_model=visual_model)
    viz.initViewer()
    viz.loadViewerModel()

    # ---------------------------------------------------------------
    # 4. Display robot
    # ---------------------------------------------------------------
    viz.display(q)

    # Create a small sphere geometry (e.g. radius = 0.01 m)
    sphere_radius = 0.01
    sphere_geom = g.Sphere(sphere_radius)

    # Black material
    black_material = g.MeshLambertMaterial(color=0x000000)

    # Attach the sphere to a path in Meshcat, e.g. "target_point"
    viz.viewer["target_point"].set_object(sphere_geom, black_material)

    # Build a 4x4 transform with translation (x, y, z)
    T = np.eye(4)
    T[:3, 3] = np.array([X0, Y0, Z0], dtype=float)

    viz.viewer["target_point"].set_transform(T)

    # ---------------------------------------------------------------
    # 5. Show placement_link frame
    # ---------------------------------------------------------------
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    placement_id = model.getFrameId("placement_link")

    viz.viewer["frames/placement_link"].set_object(g.triad(0.1))
    viz.viewer["frames/placement_link"].set_transform(
        data.oMf[placement_id].homogeneous
    )

    print("\nMeshcat visualization ready.")
    print(f"Open viewer at: {viz.viewer.url()}")
    print("Program continues running...\n")


def animate_robot_and_cable_meshcat(urdf_path, robot_joint_pose, x, y, z, dt=0.03):
    """
    Visualize the robot (URDF) together with the flexible object (phantom)
    in Meshcat along a pre–computed trajectory, and show a moving
    coordinate frame attached to 'placement_link'.

    Parameters
    ----------
    urdf_path : str
        Path to the URDF file of the robot.
    robot_joint_pose : list of array-like
        List of joint configurations q_i, one per time step.
        Each element must be a vector of size model.nq.
    x, y, z : np.ndarray
        Arrays of shape (N_frames, N_points) describing the cable shape
        at each time step. For frame i, the cable points are:
            X = x[i, :], Y = y[i, :], Z = z[i, :].
    dt : float, optional
        Time (in seconds) to wait between frames in the animation.
    """

    # ------------------------------------------------------------------
    # 1. Build Pinocchio model and visual model from the URDF
    # ------------------------------------------------------------------
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()

    # Visual geometry model (depends on how meshes are referenced in URDF)
    mesh_dir = os.path.dirname(os.path.abspath(urdf_path))
    visual_model = pin.buildGeomFromUrdf(
        model,
        urdf_path,
        pin.GeometryType.VISUAL,
        [mesh_dir],
    )

    viz = MeshcatVisualizer(model, None, visual_model)
    viz.initViewer()
    viz.loadViewerModel()

    # ------------------------------------------------------------------
    # 2. Show the robot in a neutral configuration first
    # ------------------------------------------------------------------
    q0 = pin.neutral(model)
    viz.display(q0)

    # ------------------------------------------------------------------
    # 3. Create an object in Meshcat for the cable (polyline)
    # ------------------------------------------------------------------
    num_pts = x.shape[1]
    pts0 = np.zeros((3, num_pts))
    viz.viewer["cable"].set_object(
        g.Line(g.PointsGeometry(pts0))
    )

    # ------------------------------------------------------------------
    # 4. Create a coordinate frame attached to 'placement_link'
    #    (3 colored axes). We will update its transform every frame.
    # ------------------------------------------------------------------
    axis_len = 0.1  # length of each axis in meters

    # X-axis: from (0,0,0) to (axis_len, 0, 0)
    pts_x = np.array([
        [0.0, axis_len],
        [0.0, 0.0],
        [0.0, 0.0],
    ])
    viz.viewer["frames/placement_link/x"].set_object(
        g.Line(g.PointsGeometry(pts_x))
    )

    # Y-axis: from (0,0,0) to (0, axis_len, 0)
    pts_y = np.array([
        [0.0, 0.0],
        [0.0, axis_len],
        [0.0, 0.0],
    ])
    viz.viewer["frames/placement_link/y"].set_object(
        g.Line(g.PointsGeometry(pts_y))
    )

    # Z-axis: from (0,0,0) to (0, 0, axis_len)
    pts_z = np.array([
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, axis_len],
    ])
    viz.viewer["frames/placement_link/z"].set_object(
        g.Line(g.PointsGeometry(pts_z))
    )

    # Get the frame index of 'placement_link' once (we'll reuse it)
    placement_id = model.getFrameId("placement_link")

    # ------------------------------------------------------------------
    # 5. Main animation loop: robot + cable + moving frame
    # ------------------------------------------------------------------
    n_frames = x.shape[0]

    for i in range(n_frames):
        # ---- Robot configuration for this frame ----
        q = np.asarray(robot_joint_pose[i], dtype=float)
        viz.display(q)

        # ---- Update forward kinematics and frame placements ----
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        # SE3 pose of placement_link in world
        oMf = data.oMf[placement_id]
        T = oMf.homogeneous  # 4x4 homogeneous transform

        # Apply the same transform to all three axis-objects
        viz.viewer["frames/placement_link/x"].set_transform(T)
        viz.viewer["frames/placement_link/y"].set_transform(T)
        viz.viewer["frames/placement_link/z"].set_transform(T)

        # ---- Cable shape for this frame ----
        pts = np.vstack([x[i, :], y[i, :], z[i, :]])
        viz.viewer["cable"].set_object(
            g.Line(g.PointsGeometry(pts))
        )

        time.sleep(dt)

    print("Animation finished.")


def plot_graph(path, direction):
    L = path[0].L
    if direction == "right":
        a = 1
    elif direction == "left":
        a = -1
        phi_0_T2 = np.pi
    for n in path:
        plt.figure('1')
        [xs, zs, phi_s] = hf.get_shape(n.X0 + a * n.l, n.Z0, n.phi_0, n.mu, n.L_gal, n.S0,
                                       np.linspace(0, L - n.l))
        plt.plot(xs, zs)
        plt.grid()
        plt.axis('equal')
        plt.title("shapes")
    plt.show()
    index = 0
    for n in path:
        plt.figure('2')
        plt.plot(index, n.phi_L * 180 / np.pi, 'ro')
        plt.grid()
        plt.title("phi_L")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('3')
        plt.plot(index, n.jointsPose[0], 'ro')
        plt.grid()
        plt.title("joint 1")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('2')
        plt.plot(index, n.jointsPose[1], 'ro')
        plt.grid()
        plt.title("joint 2")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('4')
        plt.plot(index, n.jointsPose[2], 'ro')
        plt.grid()
        plt.title("joint 3")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('5')
        plt.plot(index, n.jointsPose[3], 'ro')
        plt.grid()
        plt.title("joint 4")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('6')
        plt.plot(index, n.jointsPose[4], 'ro')
        plt.grid()
        plt.title("joint 5")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('7')
        plt.plot(index, n.jointsPose[5], 'ro')
        plt.grid()
        plt.title("joint 6")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('8')
        plt.plot(index, n.jointsPose[6], 'ro')
        plt.grid()
        plt.title("joint 7")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('8')
        plt.plot(index, n.XL, 'ro')
        plt.grid()
        plt.title(" XL")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('8')
        plt.plot(index, n.YL, 'ro')
        plt.grid()
        plt.title("YL")
        index = index + 1
    plt.show()
    index = 0
    for n in path:
        plt.figure('8')
        plt.plot(index, n.ZL, 'ro')
        plt.grid()
        plt.title("ZL")
        index = index + 1
    plt.show()
