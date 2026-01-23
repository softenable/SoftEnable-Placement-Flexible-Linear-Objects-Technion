# This script is the whole process of displacement

import numpy as np
import utils as hf
import node_class


def placement(start, target, L, mu_fric, direction, h_table, alpha, urdf, q_start, mode, tip_link):
    # define the LDO and environment parameters
    gamma = np.arctan(mu_fric)

    Bound = node_class.Bound(L=L, direction=direction, gamma=gamma, x_max=2, x_min=-2, y_max=2, y_min=-2, z_max=4*L,
                             z_min=h_table, theta_max=np.pi, theta_min=-np.pi, dx=0.01, dy=0.01, dz=0.01*L,
                             d_phi_0=2*np.pi/180, d_theta=2*np.pi/180, dmu=0.01, dL_gal=0.01*L, dl=0.008*L,
                             alpha=alpha, L_gal_max=10*L, phi_L_min=-180*np.pi/180, phi_L_max=180*np.pi/180)
    Bound.urdf = urdf
    # get path for sub-problem 1
    if mode == 1:
        path_1 = A_star_displacement(start, target, Bound, mode, q_start, tip_link)
        if not path_1:
            print("path 1 not valid")
            return False
        else:
            return path_1
    elif mode == 2:
        # get path for sub-problem 2
        start_2 = node_class.Node(target, start.X0, start.Y0, start.Z0, start.phi_0, start.theta,
                                start.mu, start.S0, start.L_gal, start.l, [], alpha, L, direction)
        path_2 = A_star_displacement(start_2, target, Bound, mode, q_start, tip_link)
        if not path_2:
            print("path 2 not valid")
            return False
        else:
            path_2.pop(0)
            return path_2
    elif mode == 3:
        # get path for sub-problem 3
        start_3 = node_class.Node(target, start.X0, start.Y0, start.Z0, start.phi_0, start.theta,
                                start.mu, start.S0, start.L_gal, start.l, [], alpha, L, direction)
        path_3 = A_star_displacement(start_3, target, Bound, mode, q_start, tip_link)
        if not path_3:
            print("path 3 not valid")
            return False
        else:
            path_3.pop(0)
            return path_3

def A_star_displacement(start, target, Bound, mode, q_start, tip_link):
    PATH = []
    flag = 0
    CHECK = hf.check_node_disp(start, Bound, mode, q_start, tip_link) and hf.check_node_disp(target, Bound, mode, q_start, tip_link)
    # start navigation
    Open = [start]
    Close = []
    if CHECK:
        while len(Open) > 0:
            x_best = Open[-1]
            Close.append(x_best)
            Open.pop(-1)
            if hf.check_reach_target(x_best, target, Bound, mode, q_start):
                x_best = node_class.Node(target, target.X0, target.Y0, target.Z0, target.phi_0, target.theta,
                                         x_best.mu, x_best.S0, x_best.L_gal, target.l, x_best, Bound.alpha,
                                         Bound.L, Bound.displacment_diraction)
                if hf.check_node_disp(x_best, Bound, mode, q_start, tip_link):
                    flag = 1
                    break
                else:
                    Close.append(x_best)
            Open, Close = hf.get_star(x_best, Open, Close, Bound, target, mode, q_start, tip_link) # define this function
    else:
        print("there is no valid path for sub-problem", mode)
    if flag:
        path = x_best
        while path:
            if path.jointsPose is not None:
                PATH.append(path)
            path = path.father
        PATH.reverse()
        return PATH
    else:
        print("there is no valid path for sub-problem", mode)
