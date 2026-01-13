from scipy import special
import node_class
import pinocchio as pin
import numpy as np
import sys
from numpy.linalg import norm, solve
import copy


def get_shape(X0, Y0, PHI0, mu, L_gal, s0, s):
    """
    What the function do
    ____________________
    The function give us the elastica shape of the cable

    Parameters
    ----------
    X0 - initial X
    Y0 - initial Y
    PHI0 - initial phi
    mu - modul elliptical related to the maximum tangent direction relative to elastica axis
    L_gal - the full period length of the elastica
    s0 - phase parameter of s
    s - arclength , the cable length between 0 to L

    Returns
    -------
    elastica shape defind by [x,y,phi]
    """
    # elastica parameters:
    k = np.sqrt((1 - mu) / 2)
    l = (4 * special.ellipk(k ** 2) / L_gal) ** 2
    a = np.sqrt(2 * (1 - mu) / l)
    # epsilon:
    (sn, cn, dn, am) = special.ellipj(np.sqrt(l) * (s0 + s), k ** 2)
    jaceps = special.ellipeinc(am, k ** 2)
    (sn1, cn1, dn1, am1) = special.ellipj(np.sqrt(l) * s0, k ** 2)
    jaceps0 = special.ellipeinc(am1, k ** 2)
    # elastic shape before trans:
    x_tilda = (2 / np.sqrt(l)) * (jaceps - jaceps0) - s
    y_tilda = a * (cn - cn1)
    phi_tilda = -2 * np.arcsin(k * sn) + 2 * np.arcsin(k * sn1)
    Psi = PHI0 + 2 * np.arcsin(k * sn1)
    # shape after trans:
    x = X0 + np.cos(Psi) * x_tilda - np.sin(Psi) * y_tilda
    y = Y0 + np.sin(Psi) * x_tilda + np.cos(Psi) * y_tilda
    phi = PHI0 + phi_tilda
    return x, y, phi

def get_dist(node, father, alpha):
    """
    :param node: node on the search graph
    :param father: the previous node
    :param alpha:  wight vector
    :return: norm
    """
    dist = np.sqrt(
        alpha * (node.phi_0 - father.phi_0) ** 2 +
        alpha * (node.theta - father.theta) ** 2 +
        (node.X0 - father.X0) ** 2 +
        (node.Y0 - father.Y0) ** 2 +
        (node.Z0 - father.Z0) ** 2 +
        (node.mu - father.mu) ** 2 +
        (node.L_gal - father.L_gal) ** 2 +
        (node.l - father.l) ** 2
    )
    return dist

def check_reach_target(node, target, bound, mode, q_start):
    """
    :return: check if the shape is in the target position
    """
    flag = 0
    if np.abs(node.X0 - target.X0) <= bound.dx and np.abs(node.Y0 - target.Y0) <= bound.dy and \
            np.abs(node.Z0 - target.Z0) <= bound.dz and np.abs(node.phi_0 - target.phi_0) <= bound.d_phi_0 and \
            np.abs(node.theta - target.theta) <= bound.d_theta and np.abs(node.l - target.l) <= bound.dl:
         flag = 1
    return flag

def check_in_list(neighbor, List, bound):
    """
    What the function do
    ____________________
    The function checking if a node is in the list

    Returns
    -------
    if True: the index
    if False: bool
    """
    if not List:
        return False
    for n in List:
        flag = np.abs(neighbor.X0 - n.X0) <= bound.dx / 2 and np.abs(neighbor.Y0 - n.Y0) <= bound.dy / 2 and \
               np.abs(neighbor.Z0 - n.Z0) <= bound.dz / 2 and np.abs(neighbor.phi_0 - n.phi_0) <= bound.d_phi_0 / 2 and \
               np.abs(neighbor.theta - n.theta) <= bound.d_theta / 2 and np.abs(neighbor.mu - n.mu) <= bound.dmu / 2 and \
               np.abs(neighbor.L_gal - n.L_gal) <= bound.dL_gal / 2 and np.abs(neighbor.l - n.l) <= bound.dl / 2
        if flag:
            return List.index(n)
    return False

def where_to_put_neighbor(neighbor, open_list):
    """
    What the function do
    ____________________
    Sorting the array with a new node

    Returns
    -------
    Sorting list
    """
    if not open_list: return 0
    index_a = 0
    index_b = len(open_list)
    mid = 0
    while index_a < index_b:
        mid = round((index_a + index_b) / 2)
        if mid > (index_a + index_b) / 2:
            mid = mid - 1
        if open_list[mid].F >= neighbor.F:
            index_a = mid + 1
        else:
            index_b = mid - 1
    return index_a

def get_neighbor(neighbor, open_list, close_list, bound, mode, q_start):
    """
    What the function do
    ____________________
    The function decides whether the current neighbor is acceptable
    and put it in the right place in the array

    Parameters
    ----------
    neighbor
    open_list
    close_list
    bound
    mode
    q_start
    T_wb
    Returns
    -------
    Sorted open and close lists
    """
    if check_node_disp(neighbor, bound, mode, q_start):
        if not check_in_list(neighbor, close_list, bound):
            index = check_in_list(neighbor, open_list, bound)
            if index:
                if open_list[index].F >= neighbor.F:
                    open_list.remove(open_list[index])
                    index = where_to_put_neighbor(neighbor, open_list)
                    open_list.insert(index, neighbor)
            else:
                index = where_to_put_neighbor(neighbor, open_list)
                open_list.insert(index, neighbor)
    else:
        close_list.append(neighbor)
    return open_list, close_list

def get_star(x_best, open_list, close_list, bound, target, mode, q_start):
    """
    What the function do
    ____________________

    open all the neighbors of the current node

    Parameters
    ----------
    open_list
    close_list
    x_best
    target
    bound
    mode - indicate related sub-problem
    q_start - the joints vector of the robot in the start configuration
    T_wb

    Returns
    -------
    Sorted open and close lists
    """
    if bound.displacment_diraction == "right":
        C = 1 / 4
    elif bound.displacment_diraction == "left":
        C = 3 / 4
    if mode == 1:
        # target, X0, Y0, Z0, phi_0, theta, mu, S0, L_gal, l, father, alpha, L, direction
        n1 = node_class.Node(target, x_best.X0 + bound.dx, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n2 = node_class.Node(target, x_best.X0 - bound.dx, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n3 = node_class.Node(target, x_best.X0, x_best.Y0 + bound.dy, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n4 = node_class.Node(target, x_best.X0, x_best.Y0 - bound.dy, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n5 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0 + bound.dz, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n6 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0 - bound.dz, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n7 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta + bound.d_theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n8 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta - bound.d_theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        neighbors = [n1, n2, n3, n4, n5, n6, n7, n8]
    elif mode == 2:
        n1 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0 + bound.d_phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n2 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0 - bound.d_phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n3 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu + bound.dmu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n4 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu - bound.dmu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n5 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal + bound.dL_gal), x_best.L_gal + bound.dL_gal,
                             x_best.l, x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n6 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal - bound.dL_gal), x_best.L_gal - bound.dL_gal,
                             x_best.l, x_best, bound.alpha, bound.L, bound.displacment_diraction)
        neighbors = [n1, n2, n3, n4, n5, n6]
    elif mode == 3:
        n1 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu + bound.dmu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n2 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu - bound.dmu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n3 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal + bound.dL_gal), x_best.L_gal + bound.dL_gal,
                             x_best.l, x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n4 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal - bound.dL_gal), x_best.L_gal - bound.dL_gal,
                             x_best.l, x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n5 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal - bound.dl), x_best.L_gal - bound.dl,
                             x_best.l + bound.dl, x_best, bound.alpha, bound.L, bound.displacment_diraction)
        n6 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal + bound.dl), x_best.L_gal + bound.dl,
                             x_best.l - bound.dl, x_best, bound.alpha, bound.L, bound.displacment_diraction)
        neighbors = [n1, n2, n3, n4, n5, n6]
    for n in neighbors:
        open_list, close_list = get_neighbor(n, open_list, close_list, bound, mode, q_start)
    return open_list, close_list

def check_node_disp(node, Bound, mode, q_start):
    """
     What the function do
    ____________________
    For each mode the function check if the node is the C space

    Parameters
    ----------
    node
    Bound
    mode
    q_start
    T_wb
    Returns
    -------
    bool: Ture/False
    """
    if mode == 1:
        check_x = Bound.x_max >= node.X0 >= Bound.x_min
        check_y = Bound.y_max >= node.Y0 >= Bound.y_min
        check_z = Bound.z_max >= node.Z0 >= Bound.z_min
        check_theta = Bound.theta_max >= node.theta >= Bound.theta_min
        check = check_x and check_y and check_z and check_theta
    elif mode == 2:
        check_phi = Bound.max_phi_0 + 0.0001 >= node.phi_0 >= Bound.min_phi_0 - 0.0001
        check_mu = Bound.max_mu >= node.mu >= Bound.min_mu
        check_L_gal = Bound.L_gal_max >= node.L_gal >= 1.86 * (Bound.L - node.l)  # stability criteria
        if Bound.displacment_diraction == 'right':  # friction cone check
            check_fric = np.pi / 2 + Bound.gamma >= node.phi_0 + np.arccos(node.mu) >= np.pi / 2 - Bound.gamma
        elif Bound.displacment_diraction == 'left':
            check_fric = np.pi / 2 + Bound.gamma >= node.phi_0 - np.arccos(node.mu) >= np.pi / 2 - Bound.gamma
        check_phi_L = Bound.phi_L_min <= node.phi_L <= Bound.phi_L_max
        check = check_phi and check_mu and check_L_gal and check_fric and check_phi_L
    elif mode == 3:
        check_mu = Bound.max_mu >= node.mu >= Bound.min_mu
        check_L_gal = Bound.L_gal_max >= node.L_gal >= 1.86 * (Bound.L - node.l)  # stability criteria
        check_l = Bound.min_l <= node.l <= Bound.max_l
        check_fric = np.pi / 2 + Bound.gamma >= np.arccos(node.mu) >= np.pi / 2 - Bound.gamma
        check_phi_L = Bound.phi_L_min <= node.phi_L <= Bound.phi_L_max
        check = check_mu and check_L_gal and check_l and check_fric and check_phi_L
    else:
        print("somthing strainge!")
        check = 0
    if check:
        check, q = IKFromURDF(Bound.urdf, node, q_start)
        node.jointsPose = q
    return check

def get_smooth_path_displacement(path, skip, t, L, direction, urdf, q_start):
    """
    What the function do
    ____________________
    after planning the path the function smooth it,
    in order for the robot movement to be smoother.

    Parameters
    ----------
    path
    skip
    t
    L
    direction
    urdf

    Returns
    -------
    smooth path
    """
    q_temp = q_start

    parameters = []
    for i in range(0, len(path), skip):
        parameters.append([
            path[i].mu, path[i].S0, path[i].L_gal, path[i].l,
            path[i].phi_0, path[i].theta, path[i].X0, path[i].Y0, path[i].Z0
        ])

    # ensure last node is included if skip > 1
    if i != len(path) - 1:
        parameters.append([
            path[-1].mu, path[-1].S0, path[-1].L_gal, path[-1].l,
            path[-1].phi_0, path[-1].theta, path[-1].X0, path[-1].Y0, path[-1].Z0
        ])

    xs, ys, zs = [], [], []
    phi_l_s, theta_s, ls = [], [], []
    q = []
    nodes = []  # instead of Node = [[]]

    # direction sign for elastica parameterization
    C = 1 if direction == "right" else -1

    parameters = np.array(parameters)

    for i in range(1, len(parameters)):
        for tau in t:
            line = parameters[i - 1] + tau * (parameters[i] - parameters[i - 1])

            xt, zt, phi_t = get_shape(
                C * line[3],  # l (with sign)
                line[8],  # Z0
                line[4],  # phi_0
                line[0],  # mu
                line[2],  # L_gal
                line[1],  # S0
                np.linspace(0, L - line[3], 50)
            )

            R = np.array([
                [np.cos(line[5]), -np.sin(line[5])],
                [np.sin(line[5]), np.cos(line[5])]
            ])

            XY = np.dot(R, np.vstack([xt, np.zeros_like(xt)]))

            xs.append(XY[0, :] + line[6])
            ys.append(XY[1, :] + line[7])
            zs.append(zt)
            phi_l_s.append(phi_t[-1])
            theta_s.append(line[5])
            ls.append(line[3])

            # choose father only if there is a previous node
            father = nodes[-1] if len(nodes) > 0 else None

            node = node_class.JointNode(
                XL=xs[-1][-1],
                YL=ys[-1][-1],
                ZL=zs[-1][-1],
                phi_L=phi_l_s[-1],
                theta=line[5],
                father=father
            )

            ok, q_temp = IKFromURDF(urdf, node, q_start)

            if not ok:
                # if IK fails here, you can choose to skip this configuration
                # or break; for now we just skip adding it
                continue

            q.append(q_temp.copy().tolist())
            node.jointsPose = q_temp.copy()
            nodes.append(node)

    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)
    phi_l_s = np.array(phi_l_s)
    theta_s = np.array(theta_s)
    ls = np.array(ls)

    return xs, ys, zs, phi_l_s, theta_s, ls, q

import numpy as np
import node_class

def get_smooth_path_displacement_nodes(path, skip, t, L, direction, urdf, q_start):
    """
    Smooth a planned path by interpolating between nodes and solving IK at each
    interpolated step, returning a *path-like list of nodes* (same style as `path`).

    Parameters
    ----------
    path : list[node_class.Node]
        Original A* path.
    skip : int
        Take every `skip` nodes from the original path as interpolation keyframes.
    t : array-like
        Interpolation coefficients between consecutive keyframes, e.g. np.linspace(0,1,5).
    L : float
        Cable length.
    direction : str
        "right" or "left"
    urdf : str
        URDF path.
    q_start : array-like
        Initial joint config (seed) for IK.

    Returns
    -------
    path_smooth : list[node_class.Node]
        Smoothed path where each node has jointsPose and father pointer.
        Only nodes for which IK succeeded are included.
    """

    # ---------- collect keyframes ----------
    key_params = []
    for i in range(0, len(path), skip):
        key_params.append([
            path[i].mu, path[i].S0, path[i].L_gal, path[i].l,
            path[i].phi_0, path[i].theta, path[i].X0, path[i].Y0, path[i].Z0
        ])

    if len(path) > 0 and (len(key_params) == 0 or (path[-1].mu != key_params[-1][0] or path[-1].X0 != key_params[-1][6])):
        key_params.append([
            path[-1].mu, path[-1].S0, path[-1].L_gal, path[-1].l,
            path[-1].phi_0, path[-1].theta, path[-1].X0, path[-1].Y0, path[-1].Z0
        ])

    key_params = np.asarray(key_params, dtype=float)

    # direction sign for elastica parameterization
    C = 1 if direction == "right" else -1

    # ---------- build smoothed path ----------
    path_smooth = []
    q_temp = np.asarray(q_start, dtype=float)

    # We need the same "target" object that your Node constructor expects.
    # In your code, Node(...) expects `target` to compute heuristic/distances.
    # For a smoothed path used ONLY for visualization/IK playback, we can pass
    # the original last node as a dummy target (not used later).
    dummy_target = path[-1] if len(path) > 0 else None

    for i in range(1, len(key_params)):
        p0 = key_params[i - 1]
        p1 = key_params[i]

        for tau in t:
            line = p0 + tau * (p1 - p0)

            mu, S0, L_gal, l = line[0], line[1], line[2], line[3]
            phi_0, theta = line[4], line[5]
            X0, Y0, Z0 = line[6], line[7], line[8]

            # ---- compute cable end (XL,YL,ZL,phi_L) using your existing get_shape
            xt, zt, phi_t = get_shape(
                C * l,
                Z0,
                phi_0,
                mu,
                L_gal,
                S0,
                np.linspace(0, L - l, 50)
            )

            R = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta),  np.cos(theta)]])
            XY = R @ np.vstack([xt, np.zeros_like(xt)])

            XL = float(XY[0, -1] + X0)
            YL = float(XY[1, -1] + Y0)
            ZL = float(zt[-1])
            phi_L = float(phi_t[-1])

            # ---- father pointer
            father = path_smooth[-1] if len(path_smooth) > 0 else None

            # ---- create a "path Node" (same class as A* path)
            # IMPORTANT: your Node constructor computes XL/YL/ZL internally too.
            # We pass l, mu, etc. so it stays consistent with your system.
            node = node_class.Node(
                dummy_target,
                X0, Y0, Z0,
                phi_0, theta,
                mu, S0, L_gal, l,
                father,
                alpha=getattr(path[0], "_alpha", 0.0) if len(path) > 0 else 0.0,
                L=L,
                direction=direction
            )

            # If you want to force the computed end values (optional):
            node.XL, node.YL, node.ZL, node.phi_L = XL, YL, ZL, phi_L

            # ---- solve IK relative to this node
            ok, q_temp = IKFromURDF(urdf, node, q_temp)
            if not ok:
                # Skip this sample if IK failed
                continue

            node.jointsPose = np.asarray(q_temp, dtype=float).copy()
            path_smooth.append(node)

    return path_smooth


def get_gripper_path(path):

    R = []
    P = []
    Rx = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, -1, 0]
    ])
    for n in path:
        Ryt = np.array([
            [np.cos(np.pi/2 - n.theta), 0, np.sin(np.pi/2 - n.theta)],
            [0, 1, 0],
            [-np.sin(np.pi/2 - n.theta), 0, np.cos(np.pi/2 - n.theta)]
        ])
        Rxyt = np.dot(Rx, Ryt)
        Rxtt = np.array([
            [1, 0, 0],
            [0, np.cos(n.phi_L - np.pi/2), -np.sin(n.phi_L - np.pi/2)],
            [0, np.sin(n.phi_L - np.pi/2), np.cos(n.phi_L - np.pi/2)]
        ])
        P.append(np.array([n.XL, n.YL, n.ZL]))
        Ri = np.dot(Rxyt, Rxtt)
        R.append(Ri)

    return P, R

def IKFromURDF(urdf: str, node, q_start):
    """
    Inverse kinematics that aligns the frame 'placement_link'
    with a desired SE3 pose computed from the node parameters.

    The desired pose is defined in the WORLD frame, and we
    directly control the placement_link frame (not the last joint).

    Parameters
    ----------
    urdf : str
        Path to URDF file.
    node : object
        Node with attributes XL, YL, ZL, phi_L, theta, and possibly father.
    q_start : array-like
        Initial guess for joint configuration.

    Returns
    -------
    (success, q) : (bool, np.ndarray or None)
        - success = True if IK converged within limits
        - q is the found configuration (or None if failure)
    """
    # -----------------------------------------------------------
    # 1. Load model from URDF
    # -----------------------------------------------------------
    urdf_filename = urdf if len(sys.argv) < 2 else sys.argv[1]
    model = pin.buildModelFromUrdf(urdf_filename)
    data = model.createData()

    # Frame id of placement_link
    placement_id = model.getFrameId("placement_link")

    # -----------------------------------------------------------
    # 2. Desired pose oMdes of placement_link in WORLD frame
    # -----------------------------------------------------------
    # Here we assume that (XL, YL, ZL, phi_L, theta) are already
    # expressed in the WORLD frame. If they are in another frame,
    # you must convert them accordingly.
    P, R = get_gripper_path([node])

    oMdes = pin.SE3(R[0], np.array(P[0]))   # WORLD -> desired placement_link

    # -----------------------------------------------------------
    # 3. Choose initial guess q
    # -----------------------------------------------------------
    if getattr(node, "father", None) is not None:
        father_q = getattr(node.father, "jointsPose", None)
        if father_q is not None and len(father_q) == model.nq:
            q = np.asarray(father_q, dtype=float)
        else:
            q = np.asarray(q_start, dtype=float)
    else:
        q = np.asarray(q_start, dtype=float)

    # -----------------------------------------------------------
    # 4. Iterative IK on placement_link frame
    # -----------------------------------------------------------
    eps = 1e-3
    IT_MAX = 1000
    DT = 1e-1
    damp = 1e-12

    success = False

    for it in range(IT_MAX):
        # Forward kinematics and frame placements
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        # Current pose of placement_link in WORLD frame
        oMf = data.oMf[placement_id]

        # Pose error in the local frame of placement_link
        # iMd = current_pose^-1 * desired_pose
        iMd = oMf.actInv(oMdes)
        err = pin.log(iMd).vector  # 6D error (orientation + position)

        if norm(err) < eps:
            success = True
            break

        # 6xN frame Jacobian in LOCAL frame
        J6 = pin.computeFrameJacobian(model, data, q, placement_id,
                                      pin.ReferenceFrame.LOCAL)

        # Map it with Jlog6 (same idea as your joint-based IK)
        J = -pin.Jlog6(iMd.inverse()) @ J6

        # Damped least-squares step
        v = -J.T @ solve(J @ J.T + damp * np.eye(6), err)

        # Integrate configuration
        q = pin.integrate(model, q, v * DT)

    # -----------------------------------------------------------
    # 5. Return result (with joint limit check if you want)
    # -----------------------------------------------------------
    if success:
        # Example: check only first 7 (KUKA iiwa) are inside limits
        if model.lowerPositionLimit.shape[0] >= 7:
            if not (np.all(q[:7] >= model.lowerPositionLimit[:7]) and
                    np.all(q[:7] <= model.upperPositionLimit[:7])):
                return False, None
        return True, q
    else:
        return False, None

def find_feasible_q_start(urdf: str, start, target_1, target_2, target_3):
    model = pin.buildModelFromUrdf(urdf)
    max_iter = 5000
    i = 0
    while True:
        if i >= max_iter:
            return None
        check, qs = IKFromURDF(urdf, start, sample_random_q(model.lowerPositionLimit, model.upperPositionLimit))
        if check:
            check, q1 = IKFromURDF(urdf, target_1, qs)
            if check:
                check, q2 = IKFromURDF(urdf, target_2, q1)
                if check:
                    check, q3 = IKFromURDF(urdf, target_3, q2)
                    if check:
                        return qs
        i += 1

def sample_random_q(joint_min, joint_max):
    joint_min = np.asarray(joint_min, dtype=float)
    joint_max = np.asarray(joint_max, dtype=float)
    return joint_min + (joint_max - joint_min) * np.random.rand(joint_min.size)

def path_to_robot_joint_pose(path):
    """
    Convert a path (list of Node objects) to a list of joint configurations.

    Parameters
    ----------
    path : list of node_class.Node
        Output path from displacement.A_star_displacement / displacement.displacement.
        Each node is expected to have 'jointsPose' filled by the IK.

    Returns
    -------
    robot_joint_pose : list of np.ndarray
        List of joint vectors q_i, one per node along the path.
    """
    robot_joint_pose = []

    for i, node in enumerate(path):
        if getattr(node, "jointsPose", None) is None or len(node.jointsPose) == 0:
            raise ValueError(f"Node {i} has empty jointsPose – IK was not stored.")

        q = np.asarray(node.jointsPose, dtype=float)
        robot_joint_pose.append(q)

    return robot_joint_pose

def get_path_shapes(path, L, direction, num_points=50):
    """
    Compute the elastica shape (x, y, z) of the phantom for each node in the path,
    without interpolation and without IK.

    Parameters
    ----------
    path : list of Node
        Full path from A* (each node has mu, S0, L_gal, l, phi_0, theta, X0, Y0, Z0).
    L : float
        phantom length.
    direction : str
        'right' or 'left' – used to set the sign in the elastica parameterization.
    num_points : int, optional
        Number of discretization points along the cable for each shape (default: 50).

    Returns
    -------
    x, y, z : np.ndarray
        Arrays of shape (N_nodes, num_points), where each row is one phantom shape.
    """

    xs = []
    ys = []
    zs = []
    # sign for elastica parameterization (same as in get_smooth_path_displacement)
    C = 1 if direction == "right" else -1

    for node in path:
        # arclength samples along the free part of the cable (0..L - l)
        s_samples = np.linspace(0, L - node.l, num_points)

        # elastica in local X-Z plane (like in get_smooth_path_displacement)
        xt, zt, phi_t = get_shape(
            C * node.l,   # signed l
            node.Z0,       # Z0
            node.phi_0,       # phi_0
            node.mu,       # mu
            node.L_gal,       # L_gal
            node.S0,       # S0
            s_samples
        )

        # rotate around Z by theta to get global X-Y
        R = np.array([
            [np.cos(node.theta), -np.sin(node.theta)],
            [np.sin(node.theta),  np.cos(node.theta)]
        ])

        XY = np.dot(R, np.vstack([xt, np.zeros_like(xt)]))

        # translate by (X0, Y0)
        xs.append(XY[0, :] + node.X0)
        ys.append(XY[1, :] + node.Y0)
        zs.append(zt)

    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)

    return xs, ys, zs

def moving_average_1d(x, w):
    """1D moving-average filter (edge-padded)."""
    x = np.asarray(x, dtype=float)
    if w <= 1:
        return x.copy()
    pad = w // 2
    x_pad = np.pad(x, (pad, pad), mode="edge")
    kernel = np.ones(w) / w
    return np.convolve(x_pad, kernel, mode="valid")


def moving_average_matrix(Q, w):
    """
    Apply moving average to a matrix Q of shape (N, nq),
    filtering each joint dimension independently.
    """
    Q = np.asarray(Q, dtype=float)
    N, nq = Q.shape
    out = np.zeros_like(Q)
    for j in range(nq):
        out[:, j] = moving_average_1d(Q[:, j], w)
    return out


def filter_path(path, urdf, window_size=5):
    """
    Smooth the Cartesian, orientation, and JOINT trajectories
    inside a list of Node objects.

    Parameters
    ----------
    path : list of Node
        Each node must contain: X0, Y0, Z0, phi_0, theta, jointsPose.
    window_size : int
        Odd size of moving-average window (5, 7, ...).
    urdf : urdf file
        robot urdf
    Returns
    -------
    path_filt : list of Node
        New filtered path (deep copy).
    """

    # ---------- Extract arrays from nodes ----------
    X = np.array([n.X0 for n in path])
    Y = np.array([n.Y0 for n in path])
    Z = np.array([n.Z0 for n in path])
    phi = np.array([n.phi_0 for n in path])
    theta = np.array([n.theta for n in path])

    # Joint matrix (N x nq)
    Q = np.array([np.asarray(n.jointsPose, dtype=float) for n in path])

    # ---------- Filter Cartesian ----------
    Xf = moving_average_1d(X, window_size)
    Yf = moving_average_1d(Y, window_size)
    Zf = moving_average_1d(Z, window_size)
    phi_f = moving_average_1d(phi, window_size)
    theta_f = moving_average_1d(theta, window_size)

    # ---------- Filter joint trajectory ----------
    Qf = moving_average_matrix(Q, window_size)

    # ---------- Build new filtered path ----------
    path_filt = []
    for i, node in enumerate(path):
        new_node = copy.deepcopy(node)
        new_node.X0 = Xf[i]
        new_node.Y0 = Yf[i]
        new_node.Z0 = Zf[i]
        new_node.phi_0 = phi_f[i]
        new_node.theta = theta_f[i]
        new_node.jointsPose = None
        new_node.father = []
        check_IK, q = IKFromURDF(urdf, new_node,  Qf[i])
        if check_IK:
            new_node.jointsPose = q
            path_filt.append(new_node)
        else:
            path_filt = False
    return path_filt
