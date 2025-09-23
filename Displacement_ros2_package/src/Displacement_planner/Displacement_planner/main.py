import time
import numpy as np
import matplotlib.pyplot as plt
import utils as hf
import node_class
import displacement as dis
import pinocchio as pin
import sys

# define LDO parameters, environment parameters and direction displacement
urdf = "/home/itay/pythonProject/elastica_scripts/iiwa14_with_ft_and_dlr_gripper/iiwa14_with_ft_and_dlr_gripper.urdf"# path of the URDF
urdf_filename = (
        urdf
        if len(sys.argv) < 2
        else sys.argv[1]
    )
model = pin.buildModelFromUrdf(urdf_filename)
q_start = pin.neutral(model) # q_start should be input parameter

direction = "right"
L = 0.3
mu_fric = 0.1
h_table = 0.0
alpha = 0.6
if direction == "right":
    C = 1/4
    phi_0_T2 = 0
elif direction == "left":
    C = 3/4
    phi_0_T2 = np.pi

# define start and target for sub-problem 1
X0_S1 = 0.4
Y0_S1 = 0.2
Z0_S1 = h_table + 0.3*L
theta_S1 = np.pi/2
X0_T1 = 0.5
Y0_T1 = 0.2
Z0_T1 = h_table
theta_T1 = 0
L_gal_S = 2*L
target_1 = node_class.Target(X0_T1, Y0_T1, Z0_T1, np.pi/2, theta_T1, 1, C*L_gal_S, L_gal_S, 0, L, direction)
start_1 = node_class.Node(target_1, X0_S1, Y0_S1, Z0_S1, np.pi/2, theta_S1, 1, C*L_gal_S, L_gal_S, 0, [], alpha, L, direction)

# define target for sub-problem 2
mu_T2 = 0
L_gal_T2 = 3.5*L
target_2 = node_class.Target(target_1.X0, target_1.Y0, target_1.Z0, phi_0_T2, target_1.theta, mu_T2, C*L_gal_T2, L_gal_T2, 0, L, direction)

# define target for sub-problem 3
mu_T3 = 0
l_T3 = 0.5*L
L_gal_T3 = 4.5*(L - l_T3)
target_3 = node_class.Target(target_2.X0, target_2.Y0, target_2.Z0, phi_0_T2, target_2.theta, mu_T3, C*L_gal_T3, L_gal_T3, l_T3, L, direction)

# get full path
t_start = time.time()

path_1 = dis.displacement(start_1, target_1, L, mu_fric, direction, h_table, alpha, urdf, q_start, 1)
path_2 = dis.displacement(target_1, target_2, L, mu_fric, direction, h_table, alpha, urdf, q_start,2)
path_3 = dis.displacement(target_2, target_3, L, mu_fric, direction, h_table, alpha, urdf, q_start,3)
path = [path_1,path_2,path_3]
if path:
    path_1, path_2, path_3 = path

    # check displacement
    skip = 8
    num_t = 8
    t = np.linspace(0, 1, num_t)
    s = np.linspace(0, 1, num_t)

    x1, y1, z1, phi_l_1, theta_1, l1, q1 = hf.get_smooth_path_displacement(path_1, skip, t, L, direction, urdf, path_1[0].jointsPose)
    x2, y2, z2, phi_l_2, theta_2, l2, q2 = hf.get_smooth_path_displacement(path_2, skip, t, L, direction, urdf, path_2[0].jointsPose)
    x3, y3, z3, phi_l_3, theta_3, l3, q3 = hf.get_smooth_path_displacement(path_3, skip, t, L, direction, urdf, path_3[0].jointsPose)

    t_end = time.time()
    ExecutionTime1_2_3 = t_end - t_start
    print("Execution Time =", ExecutionTime1_2_3)

    P1, R1 = hf.get_gripper_path(x1[:, -1], y1[:, -1], z1[:, -1], phi_l_1, theta_1)
    P2, R2 = hf.get_gripper_path(x2[:, -1], y2[:, -1], z2[:, -1], phi_l_2, theta_2)
    P3, R3 = hf.get_gripper_path(x3[:, -1], y3[:, -1], z3[:, -1], phi_l_3, theta_3)

    # make x,y z to np.array
    x = np.array(x1.tolist() + x2.tolist() + x3.tolist())
    y = np.array(y1.tolist() + y2.tolist() + y3.tolist())
    z = np.array(z1.tolist() + z2.tolist() + z3.tolist())

    # check by graphs
    path = path_1 + path_2 + path_3
    gripper_P = np.array(P1.tolist() + P2.tolist() + P3.tolist())
    gripper_R = R1 + R2 + R3
    robot_joint_pose = q1 + q2 + q3
    if direction == "right":
        a = 1
    elif direction == "left":
        a = -1
        phi_0_T2 = np.pi

    for n in path:
        plt.figure('1')
        [xs, zs, phi_s] = hf.get_shape(n.X0 + a*n.l, n.Z0, n.phi_0, n.mu, n.L_gal, n.S0, np.linspace(0, L-n.l))
        plt.plot(xs, zs)
        plt.grid()
    plt.show()







