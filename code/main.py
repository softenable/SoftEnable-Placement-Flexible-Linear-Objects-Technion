import time
import numpy as np
import utils as hf
import node_class
import placement as dis
import visualize_urdf as vis
import os

current_dir = os.path.dirname(os.path.abspath(__file__))

# Find URDF file in the same folder
urdf_files = [f for f in os.listdir(current_dir) if f.endswith('.urdf')]

if not urdf_files:
    print("No URDF file found in the current directory")
    exit(1)

urdf = os.path.join(current_dir, urdf_files[0])
print(f"Loading URDF: {urdf}")

direction = "left"
L = 0.3
mu_fric = 0.1
h_table = 0
alpha = 0.6  # parameter scaling
if direction == "right":
    C = 1 / 4
    phi_0_T2 = 0
elif direction == "left":
    C = 3 / 4
    phi_0_T2 = np.pi

# define start and target for sub-problem 1
X0_S1 = 0.8
Y0_S1 = 0.4
Z0_S1 = h_table + 0.3 * L
theta_S1 = np.pi/2
X0_T1 = 0.6
Y0_T1 = 0.4
Z0_T1 = h_table
theta_T1 = np.pi/2.5
L_gal_S = 2 * L
target_1 = node_class.Target(X0_T1, Y0_T1, Z0_T1, np.pi / 2, theta_T1, 1, C * L_gal_S, L_gal_S, 0, L, direction)
start_1 = node_class.Node(target_1, X0_S1, Y0_S1, Z0_S1, np.pi / 2, theta_S1, 1, C * L_gal_S, L_gal_S, 0, [], alpha, L,
                          direction)

mu_T2 = 0
L_gal_T2 = 3 * L
target_2 = node_class.Target(target_1.X0, target_1.Y0, target_1.Z0, phi_0_T2, target_1.theta, mu_T2, C * L_gal_T2,
                             L_gal_T2, 0, L, direction)

# define target for sub-problem 3
mu_T3 = 0
l_T3 = 0.5 * L
L_gal_T3 = 3.5 * (L - l_T3)
target_3 = node_class.Target(target_2.X0, target_2.Y0, target_2.Z0, phi_0_T2, target_2.theta, mu_T3, C * L_gal_T3,
                             L_gal_T3, l_T3, L, direction)
# find feasible q_start
q_start = hf.find_feasible_q_start(urdf, start_1, target_1, target_2, target_3)

if len(q_start) > 2:
    # plot initial arm configuration
    # vis.plot_static_robot_config(q_start, start_1.XL, start_1.YL, start_1.ZL)
    # get full path
    t_start = time.time()

    path_1 = dis.placement(start_1, target_1, L, mu_fric, direction, h_table, alpha, urdf, q_start, 1)
    print("path 1 done")
    if path_1:
        path_2 = dis.placement(path_1[-1], target_2, L, mu_fric, direction, h_table, alpha, urdf, path_1[-1].jointsPose,
                               2)
        print("path 2 done")
        if path_2:
            path_3 = dis.placement(path_2[-1], target_3, L, mu_fric, direction, h_table, alpha, urdf,
                                   path_2[-1].jointsPose, 3)
            print("path 3 done")
            if path_3:

                t_end = time.time()
                ExecutionTime1_2_3 = t_end - t_start
                print("Execution Time =", ExecutionTime1_2_3)

                path = path_1 + path_2 + path_3
                P, R = hf.get_gripper_path(path)
                robot_joint_pose = hf.path_to_robot_joint_pose(path)
                x, y, z = hf.get_shapes(path, L, direction, h_table)
                #
                vis.animate_robot_and_cable_meshcat(
                    urdf,
                    robot_joint_pose,
                    x,
                    y,
                    z,
                    dt=0.03,
                )
                vis.plot_graph(path, direction)

else:
    print("there is no feasible joints path")
