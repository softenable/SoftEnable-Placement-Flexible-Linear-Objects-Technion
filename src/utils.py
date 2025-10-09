from scipy import special
try:
    import node_class
except ImportError:
    # Allow utils to be imported without node_class for testing
    node_class = None
import pinocchio as pin
import numpy as np
import sys
import time
from numpy.linalg import norm, solve


class IKSolver:
    """
    Improved IK solver using Pinocchio with joint limits and performance optimization.
    """
    
    def __init__(self, urdf_path, joint_limits_margin=0.0):
        """
        Initialize the IK solver with robot model.
        
        Args:
            urdf_path (str): Path to URDF file
            joint_limits_margin (float): Margin to expand joint limits by (in radians/meters)
                                        Use 0.0 for strict limits, positive values for more permissive
        """
        self.urdf_path = urdf_path
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # Use correct end-effector frame (iiwa_link_ee) not last joint
        try:
            self.ee_frame_id = self.model.getFrameId("iiwa_link_ee")
        except Exception:
            # Fallback to last joint if frame not found
            self.ee_frame_id = None
            self.joint_id = self.model.njoints - 1
            print("Warning: iiwa_link_ee frame not found, using last joint")
        
        # Cache for performance
        self.last_q = None
        self.last_target = None
        
        # Performance tracking
        self.solve_times = []
        self.solve_counts = 0
        self.total_iterations = 0
        
        # Extract joint limits from URDF with optional margin
        self.q_min = self.model.lowerPositionLimit.copy()
        self.q_max = self.model.upperPositionLimit.copy()
        
        if joint_limits_margin > 0:
            self.q_min -= joint_limits_margin
            self.q_max += joint_limits_margin
        
        print(f"IK Solver initialized for {self.model.name}")
        print(f"DOF: {self.model.nq}")
        print(f"Joint limits: min={self.q_min.T}, max={self.q_max.T}")
        if joint_limits_margin > 0:
            print(f"Joint limits margin: {joint_limits_margin:.3f}")
        
        if self.ee_frame_id is not None:
            print(f"Using end-effector frame: iiwa_link_ee (ID: {self.ee_frame_id})")
        else:
            print(f"Using joint {self.joint_id} as end-effector")
    
    def get_end_effector_pose(self, q):
        """Get end-effector pose for given configuration."""
        # Create fresh data to avoid state corruption
        data = self.model.createData()
        pin.forwardKinematics(self.model, data, q)
        
        if self.ee_frame_id is not None:
            pin.updateFramePlacements(self.model, data)
            return data.oMf[self.ee_frame_id]
        else:
            return data.oMi[self.joint_id]
    
    def get_end_effector_jacobian(self, q):
        """Get end-effector Jacobian for given configuration."""
        # Ensure kinematics are updated for this configuration
        pin.forwardKinematics(self.model, self.data, q)
        if self.ee_frame_id is not None:
            pin.updateFramePlacements(self.model, self.data)
            return pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        else:
            return pin.computeJointJacobian(self.model, self.data, q, self.joint_id)
    
    def get_default_configuration(self):
        """Get a good default configuration for the robot."""
        # Use the specified initial position that works well for this robot
        default_q = np.array([0.52359878, 1.04719755, 0.26179939, -0.52359878, 
                             -1.04719755, 0.34906585, 0.17453293])
        return default_q
    
    def solve_ik_fast(self, target_pose, q_init=None, max_iter=1000, tolerance=1e-4):
        """
        Fast IK solver with joint limits using Pinocchio.
        
        Args:
            target_pose: SE3 transformation for end-effector
            q_init: Initial joint configuration
            max_iter: Maximum iterations
            tolerance: Convergence tolerance
            
        Returns:
            tuple: (success, joint_config, iterations, solve_time)
        """
        start_time = time.perf_counter()
        
        # Use previous solution as warm start if available
        if q_init is None:
            if self.last_q is not None:
                q = self.last_q.copy()
            else:
                q = self.get_default_configuration()
        else:
            q = q_init.copy()
        
        # Clamp initial configuration to joint limits
        q = np.clip(q, self.q_min, self.q_max)
        
        # Solver parameters - improved damping strategy
        dt = 0.3  # More conservative initial step size
        damp = 1e-2  # Higher initial damping for stability
        
        prev_error = float('inf')
        stagnation_count = 0
        
        for i in range(max_iter):
            # Current end-effector pose
            current_pose = self.get_end_effector_pose(q)
            
            # Use mixed error: Cartesian position + rotation error
            position_error = target_pose.translation - current_pose.translation
            
            # Rotation error using SE3 log
            error_se3 = target_pose.actInv(current_pose)
            rotation_error = pin.log(error_se3).vector[3:]  # Only rotation part
            
            # Combined error vector [position, rotation]
            error_vector = np.concatenate([position_error, rotation_error])
            
            # Check convergence
            error_norm = norm(error_vector)
            if error_norm < tolerance:
                solve_time = time.perf_counter() - start_time
                self._update_stats(solve_time, i + 1, True)
                self.last_q = q.copy()
                return True, q, i + 1, solve_time
            
            # Adaptive damping based on error progress
            if error_norm >= prev_error:
                stagnation_count += 1
                damp = min(damp * 2.0, 1.0)  # Increase damping but cap it
                dt = max(dt * 0.7, 0.01)   # Reduce step size but keep minimum
            else:
                stagnation_count = 0
                damp = max(damp * 0.8, 1e-6)  # Reduce damping
                dt = min(dt * 1.2, 1.0)       # Increase step size but cap it
            
            # Exit if stagnating too long
            if stagnation_count > 15:
                break
            
            # Compute Jacobian in correct frame
            J = self.get_end_effector_jacobian(q)
            
            # Improved damped least squares with better conditioning
            JJt = J.dot(J.T)
            cond_num = np.linalg.cond(JJt)
            
            # Adaptive damping based on conditioning
            if cond_num > 1e8:
                adaptive_damp = max(damp, 1e-1)
            elif cond_num > 1e6:
                adaptive_damp = max(damp, 1e-2)
            else:
                adaptive_damp = damp
            
            try:
                JJt_reg = JJt + adaptive_damp * np.eye(6)
                dv = solve(JJt_reg, error_vector)
                dq = J.T.dot(dv)
            except np.linalg.LinAlgError:
                # Fallback to pseudoinverse with high damping
                dq = np.linalg.pinv(J, rcond=1e-3).dot(error_vector)
                dt *= 0.5  # Reduce step size for safety
            
            # Adaptive step size based on joint velocity magnitude
            dq_norm = norm(dq)
            if dq_norm > 2.0:  # If step is too large
                dt = min(dt, 1.0 / dq_norm)  # Scale down to reasonable magnitude
            
            # Apply update with joint limits
            q_new = q + dt * dq
            q_new = np.clip(q_new, self.q_min, self.q_max)
            
            # Check for joint limit saturation and reduce step size more aggressively
            if np.allclose(q_new, q, atol=1e-8):
                dt *= 0.2  # More aggressive reduction
                if dt < 1e-5:  # Lower threshold
                    break
            elif np.any(np.abs(q_new - self.q_max) < 1e-6) or np.any(np.abs(q_new - self.q_min) < 1e-6):
                # Near joint limits, be more careful
                dt *= 0.7
            
            q = q_new
            prev_error = error_norm
        
        solve_time = time.perf_counter() - start_time
        self._update_stats(solve_time, max_iter, False)
        return False, q, max_iter, solve_time
    
    def solve_ik_robust(self, target_pose, q_init=None, max_iter=1000, tolerance=1e-5):
        """
        Robust IK solver with better convergence for difficult poses.
        
        Args:
            target_pose: SE3 transformation for end-effector
            q_init: Initial joint configuration
            max_iter: Maximum iterations
            tolerance: Convergence tolerance
            
        Returns:
            tuple: (success, joint_config, iterations, solve_time)
        """
        start_time = time.perf_counter()
        
        # Try multiple initial guesses if first fails
        initial_guesses = []
        if q_init is not None:
            initial_guesses.append(q_init.copy())
        
        # Add the good default configuration first
        initial_guesses.append(self.get_default_configuration())
        
        # Add neutral pose as backup
        initial_guesses.append(pin.neutral(self.model))
        
        # Add random configurations within joint limits
        for _ in range(3):
            q_rand = pin.randomConfiguration(self.model)
            initial_guesses.append(q_rand)
        
        best_error = float('inf')
        best_q = None
        best_iterations = 0
        
        for guess_idx, q_start in enumerate(initial_guesses):
            success, q_result, iterations, _ = self._solve_ik_single_attempt(
                target_pose, q_start, max_iter, tolerance
            )
            
            if success:
                solve_time = time.perf_counter() - start_time
                self._update_stats(solve_time, iterations, True)
                self.last_q = q_result.copy()
                return True, q_result, iterations, solve_time
            
            # Track best attempt even if unsuccessful
            current_pose = self.get_end_effector_pose(q_result)
            error_se3 = target_pose.actInv(current_pose)
            error_norm = norm(pin.log(error_se3).vector)
            
            if error_norm < best_error:
                best_error = error_norm
                best_q = q_result.copy()
                best_iterations = iterations
        
        solve_time = time.perf_counter() - start_time
        self._update_stats(solve_time, best_iterations, False)
        return False, best_q, best_iterations, solve_time
    
    def _solve_ik_single_attempt(self, target_pose, q_init, max_iter, tolerance):
        """Single IK solving attempt with given initial configuration."""
        q = np.clip(q_init.copy(), self.q_min, self.q_max)
        
        # More conservative parameters for robust solving
        dt = 0.3
        damp = 1e-2  # Start with higher damping
        
        prev_error = float('inf')
        stagnation_count = 0
        
        for i in range(max_iter):
            # Current end-effector pose
            current_pose = self.get_end_effector_pose(q)
            
            # Error computation in correct frame
            error_se3 = target_pose.actInv(current_pose)
            error_vector = pin.log(error_se3).vector
            error_norm = norm(error_vector)
            
            # Check convergence
            if error_norm < tolerance:
                return True, q, i + 1, 0
            
            # More aggressive adaptive strategy
            if error_norm >= prev_error:
                stagnation_count += 1
                damp = min(damp * 1.5, 0.1)  # Increase damping more aggressively
                dt = max(dt * 0.6, 0.001)    # Reduce step size more aggressively
            else:
                stagnation_count = 0
                damp = max(damp * 0.9, 1e-6)
                dt = min(dt * 1.1, 0.5)
            
            # Exit if stagnating too long
            if stagnation_count > 20:
                break
            
            # Jacobian computation
            J = self.get_end_effector_jacobian(q)
            
            # Better regularization strategy
            JJt = J.dot(J.T)
            cond_num = np.linalg.cond(JJt)
            
            # Condition-based damping
            if cond_num > 1e10:
                reg_damp = 0.1
            elif cond_num > 1e8:
                reg_damp = max(damp, 1e-1)
            elif cond_num > 1e6:
                reg_damp = max(damp, 1e-2)
            else:
                reg_damp = damp
            
            try:
                JJt_reg = JJt + reg_damp * np.eye(6)
                dv = solve(JJt_reg, error_vector)
                dq = J.T.dot(dv)
            except np.linalg.LinAlgError:
                # More conservative fallback
                dq = np.linalg.pinv(J, rcond=1e-2).dot(error_vector)
                dt *= 0.5
            
            # Line search with joint limits - improved version
            best_q = q
            best_error = error_norm
            
            for alpha_step in [1.0, 0.7, 0.4, 0.1, 0.05]:
                q_candidate = q + alpha_step * dt * dq
                q_candidate = np.clip(q_candidate, self.q_min, self.q_max)
                
                # Check if this step improves the solution
                test_pose = self.get_end_effector_pose(q_candidate)
                test_error_se3 = target_pose.actInv(test_pose)
                test_error_norm = norm(pin.log(test_error_se3).vector)
                
                if test_error_norm < best_error:
                    best_q = q_candidate
                    best_error = test_error_norm
                    break
            
            # Use best step found
            if best_error < error_norm:
                q = best_q
            else:
                # No improvement found, reduce step size significantly
                dt *= 0.3
                if dt < 1e-6:
                    break
            
            prev_error = error_norm
        
        return False, q, max_iter, 0
    
    def _update_stats(self, solve_time, iterations, success):
        """Update performance statistics."""
        self.solve_times.append(solve_time)
        self.solve_counts += 1
        self.total_iterations += iterations
        
        if len(self.solve_times) > 1000:  # Keep only last 1000 solves
            self.solve_times = self.solve_times[-1000:]
    
    def get_performance_stats(self):
        """Get performance statistics."""
        if not self.solve_times:
            return {}
        
        return {
            'total_solves': self.solve_counts,
            'avg_solve_time': np.mean(self.solve_times),
            'min_solve_time': np.min(self.solve_times),
            'max_solve_time': np.max(self.solve_times),
            'avg_iterations': self.total_iterations / self.solve_counts if self.solve_counts > 0 else 0,
            'total_time': np.sum(self.solve_times)
        }
    
    def reset_stats(self):
        """Reset performance statistics."""
        self.solve_times = []
        self.solve_counts = 0
        self.total_iterations = 0


# Global solver instances
_ik_solvers = {}


def get_ik_solver(urdf_path, joint_limits_margin=0.0):
    """Get or create IK solver for given URDF with specified joint limits margin."""
    cache_key = f"{urdf_path}_{joint_limits_margin}"
    if cache_key not in _ik_solvers:
        _ik_solvers[cache_key] = IKSolver(urdf_path, joint_limits_margin)
    return _ik_solvers[cache_key]


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
        alpha * (node.phi_0 - father.phi_0)**2 +
        alpha * (node.theta - father.theta)**2 +
        (node.X0 - father.X0)**2 +
        (node.Y0 - father.Y0)**2 +
        (node.Z0 - father.Z0)**2 +
        (node.mu - father.mu)**2 +
        (node.L_gal - father.L_gal)**2 +
        (node.l - father.l)**2
    )
    return dist


def check_reach_target(node, target, bound):
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
        flag = np.abs(neighbor.X0 - n.X0) <= bound.dx/2 and np.abs(neighbor.Y0 - n.Y0) <= bound.dy/2 and \
            np.abs(neighbor.Z0 - n.Z0) <= bound.dz/2 and np.abs(neighbor.phi_0 - n.phi_0) <= bound.d_phi_0/2 and \
            np.abs(neighbor.theta - n.theta) <= bound.d_theta/2 and np.abs(neighbor.mu - n.mu) <= bound.dmu/2 and \
            np.abs(neighbor.L_gal - n.L_gal) <= bound.dL_gal/2 and np.abs(neighbor.l - n.l) <= bound.dl/2
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


def get_neighbor(neighbor, open_list, close_list, bound, mode, q_start, ik_solver='fast'):
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
    ik_solver: 'legacy', 'fast', or 'robust'

    Returns
    -------
    Sorted open and close lists
    """
    if check_node_disp(neighbor, bound, mode, q_start, ik_solver):
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
    return open_list, close_list


def get_star(x_best, open_list, close_list, bound, target, mode, q_start, ik_solver='fast'):
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
    ik_solver: 'legacy', 'fast', or 'robust'

    Returns
    -------
    Sorted open and close lists
    """
    if bound.displacment_diraction == "right":
        C = 1 / 4
    elif bound.displacment_diraction == "left":
        C = 3 / 4
    if mode == 1:
        n1 = node_class.Node(target, x_best.X0 + bound.dx, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n2 = node_class.Node(target, x_best.X0 - bound.dx, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n3 = node_class.Node(target, x_best.X0, x_best.Y0 + bound.dy, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n4 = node_class.Node(target, x_best.X0, x_best.Y0 - bound.dy, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n5 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0 + bound.dz, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n6 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0 - bound.dz, x_best.phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n7 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta + bound.d_theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n8 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta - bound.d_theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        neighbors = [n1, n2, n3, n4, n5, n6, n7, n8]
    elif mode == 2:
        n1 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0 + bound.d_phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n2 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0 - bound.d_phi_0,
                             x_best.theta, x_best.mu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n3 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu + bound.dmu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n4 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu - bound.dmu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n5 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C*(x_best.L_gal + bound.dL_gal), x_best.L_gal + bound.dL_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n6 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C*(x_best.L_gal - bound.dL_gal), x_best.L_gal - bound.dL_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        neighbors = [n1, n2, n3, n4, n5, n6]
    elif mode == 3:
        n1 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu + bound.dmu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n2 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu - bound.dmu, x_best.S0, x_best.L_gal, x_best.l,
                             x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n3 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal + bound.dL_gal), x_best.L_gal + bound.dL_gal,
                             x_best.l, x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n4 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal - bound.dL_gal), x_best.L_gal - bound.dL_gal,
                             x_best.l, x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n5 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal - bound.dl), x_best.L_gal - bound.dl,
                             x_best.l + bound.dl, x_best, bound.L, bound.alpha, bound.displacment_diraction)
        n6 = node_class.Node(target, x_best.X0, x_best.Y0, x_best.Z0, x_best.phi_0,
                             x_best.theta, x_best.mu, C * (x_best.L_gal + bound.dl), x_best.L_gal + bound.dl,
                             x_best.l - bound.dl, x_best, bound.L, bound.alpha, bound.displacment_diraction)
        neighbors = [n1, n2, n3, n4, n5, n6]
    for n in neighbors:
        open_list, close_list = get_neighbor(n, open_list, close_list, bound, mode, q_start, ik_solver)
    return open_list, close_list


def check_node_disp(node, Bound, mode, q_start, ik_solver='fast'):
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
    ik_solver: 'legacy', 'fast', or 'robust'

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
        check_phi = Bound.max_phi_0 >= node.phi_0 >= Bound.min_phi_0
        check_mu = Bound.max_mu >= node.mu >= Bound.min_mu
        check_L_gal = Bound.L_gal_max >= node.L_gal >= 1.86 * (Bound.L - node.l)  # stability criteria
        if Bound.displacment_diraction == 'right':  # friction cone check
            check_fric = np.pi / 2 + Bound.gamma >= node.phi_0 + np.arccos(node.mu) >= np.pi / 2 - Bound.gamma
        elif Bound.displacment_diraction == 'left':
            check_fric = np.pi / 2 + Bound.gamma >= node.phi_0 - np.arccos(node.mu) >= np.pi / 2 - Bound.gamma
        check = check_phi and check_mu and check_L_gal and check_fric
    elif mode == 3:
        check_mu = Bound.max_mu >= node.mu >= Bound.min_mu
        check_L_gal = Bound.L_gal_max >= node.L_gal >= 1.86 * (Bound.L - node.l)  # stability criteria
        check_l = Bound.min_l <= node.l <= Bound.max_l
        check_fric = np.pi / 2 + Bound.gamma >= np.arccos(node.mu) >= np.pi / 2 - Bound.gamma
        check = check_mu and check_L_gal and check_l and check_fric
    else:
        print("somthing straing!")
        check = 0
    if check:
        check, q = IKFromURDF(Bound.urdf, node, q_start, ik_solver)
        node.jointsPose = q
    return check


def get_smooth_path_displacement(path, skip, t, L, direction, urdf, q_start, ik_solver='fast'):
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
    q_start
    ik_solver: 'legacy', 'fast', or 'robust'

    Returns
    -------
    smooth path
    """
    parameters = []
    for i in range(0, len(path), skip):
        parameters.append([
            path[i].mu, path[i].S0, path[i].L_gal, path[i].l,
            path[i].phi_0, path[i].theta, path[i].X0, path[i].Y0, path[i].Z0
        ])

    if i != len(path) - 1:
        parameters.append([
            path[-1].mu, path[-1].S0, path[-1].L_gal, path[-1].l,
            path[-1].phi_0, path[-1].theta, path[-1].X0, path[-1].Y0, path[-1].Z0
        ])

    xs = []
    ys = []
    zs = []
    phi_l_s = []
    theta_s = []
    ls = []
    q = []
    Node = [[]]

    if direction == 'right':
        C = 1
    else:
        C = -1

    parameters = np.array(parameters)

    for i in range(1, len(parameters)):
        for j in range(len(t)):
            line = parameters[i - 1] + t[j] * (parameters[i] - parameters[i - 1])
            xt, zt, phi_t = get_shape(C * line[3], line[8], line[4], line[0], line[2], line[1], np.linspace(0, L - line[3], 50))

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
            node = node_class.JointNode(XL=xs[-1][-1], YL=ys[-1][-1], ZL=zs[-1][-1], phi_L=phi_l_s[-1],
                                        theta=line[5], father=Node[-1])
            check, q_temp = IKFromURDF(urdf, node, q_start, ik_solver)
            q.append(q_temp.tolist())
            node.jointsPose = q_temp
            Node.append(node)
    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)
    phi_l_s = np.array(phi_l_s)
    theta_s = np.array(theta_s)
    ls = np.array(ls)

    return xs, ys, zs, phi_l_s, theta_s, ls, q


def get_gripper_path(x, y, z, phi_L, theta):
    """
    What the function do
    ____________________

    Parameters
    ----------
    x
    y
    z
    phi_L
    theta

    Returns
    -------

    """
    R = []
    P = np.column_stack((x, y, z))
    for i in range(len(phi_L)):
        Rz = np.array([
            [np.cos(theta[i]), -np.sin(theta[i]), 0],
            [np.sin(theta[i]), np.cos(theta[i]), 0],
            [0, 0, 1]
        ])

        Ry = np.array([
            [np.cos(phi_L[i] + np.pi / 2), 0, -np.sin(phi_L[i] + np.pi / 2)],
            [0, 1, 0],
            [np.sin(phi_L[i] + np.pi / 2), 0, np.cos(phi_L[i] + np.pi / 2)]
        ])

        Ri = np.dot(Rz, Ry)
        R.append(Ri)

    return P, R


def IKFromURDF_new(urdf_path, node, q_start=None, solver_type='fast', profile=True, planning_mode=False):
    """
    New improved IK solver using Pinocchio with joint limits and performance optimization.
    
    Args:
        urdf_path: Path to URDF file
        node: Node with target pose information (XL, YL, ZL, phi_L, theta)
        q_start: Initial joint configuration
        solver_type: 'fast' or 'robust'
        profile: Whether to track performance
        planning_mode: If True, use more permissive joint limits for planning
        
    Returns:
        tuple: (success, joint_config) or (success, joint_config, solve_time, iterations) if profile=True
    """
    start_time = time.perf_counter() if profile else None
    
    # Get solver instance with appropriate joint limits
    margin = 3.0 if planning_mode else 0.0  # Very permissive margin for planning
    solver = get_ik_solver(urdf_path, margin)
    
    # Create target pose directly from node information
    target_pose = pin.SE3()
    
    # Ensure coordinates are scalars, not arrays
    x_coord = float(node.XL) if hasattr(node.XL, '__iter__') and not isinstance(node.XL, str) else node.XL
    y_coord = float(node.YL) if hasattr(node.YL, '__iter__') and not isinstance(node.YL, str) else node.YL  
    z_coord = float(node.ZL) if hasattr(node.ZL, '__iter__') and not isinstance(node.ZL, str) else node.ZL
    
    # Handle the case where coordinates might be arrays - take the first element
    if hasattr(x_coord, '__iter__') and not isinstance(x_coord, str):
        x_coord = x_coord.flat[0] if hasattr(x_coord, 'flat') else x_coord[0]
    if hasattr(y_coord, '__iter__') and not isinstance(y_coord, str):
        y_coord = y_coord.flat[0] if hasattr(y_coord, 'flat') else y_coord[0]
    if hasattr(z_coord, '__iter__') and not isinstance(z_coord, str):
        z_coord = z_coord.flat[0] if hasattr(z_coord, 'flat') else z_coord[0]
    
    target_pose.translation = np.array([float(x_coord), float(y_coord), float(z_coord)])
    
    # Handle orientation - simplified for better reliability
    # Use identity rotation if phi_L and theta are small (near zero)
    if abs(node.phi_L) < 1e-6 and abs(node.theta) < 1e-6:
        target_pose.rotation = np.eye(3)
    else:
        # Use get_gripper_path for non-trivial orientations
        P, R = get_gripper_path(node.XL, node.YL, node.ZL, [node.phi_L], [node.theta])
        target_pose.rotation = R[0]
    
    # Get initial configuration
    if q_start is None:
        if hasattr(node, 'father') and node.father and hasattr(node.father, 'jointsPose'):
            q_init = np.array(node.father.jointsPose)
        else:
            q_init = solver.get_default_configuration()
    else:
        q_init = np.array(q_start)
    
    # Ensure q_init has correct size
    if len(q_init) < solver.model.nq:
        q_full = np.zeros(solver.model.nq)
        q_full[:len(q_init)] = q_init
        q_init = q_full
    
    # Solve IK
    if solver_type == 'fast':
        success, q_result, iterations, solve_time = solver.solve_ik_fast(target_pose, q_init)
    elif solver_type == 'robust':
        success, q_result, iterations, solve_time = solver.solve_ik_robust(target_pose, q_init)
    else:
        raise ValueError(f"Unknown solver type: {solver_type}")
    
    if profile:
        total_time = time.perf_counter() - start_time
        return success, q_result, solve_time, iterations, total_time
    else:
        return success, q_result


def IKFromURDF(urdf: str, node, q_start, solver_type='legacy'):
    """
    Original IK solver with option to use new implementation.
    
    Args:
        urdf: Path to URDF file
        node: Node with target pose information
        q_start: Initial joint configuration
        solver_type: 'legacy', 'fast', or 'robust'
        
    Returns:
        list: [success, joint_config] for legacy compatibility
    """
    if solver_type == 'legacy':
        return _IKFromURDF_legacy(urdf, node, q_start)
    else:
        # Use planning mode (more permissive joint limits) for path planning
        success, q_result = IKFromURDF_new(urdf, node, q_start, solver_type, profile=False, planning_mode=True)
        return [success, q_result]


def _IKFromURDF_legacy(urdf: str, node, q_start):
    """
    Original IK implementation (legacy).
    
    This is the original slow implementation kept for comparison.
    """
    # Use the provided URDF path
    urdf_filename = urdf

    # Load the urdf model
    model = pin.buildModelFromUrdf(urdf_filename)
    # print("model name: " + model.name)

    # Create data required by the algorithms
    data = model.createData()
    
    # Use correct end-effector frame (iiwa_link_ee)
    try:
        EE_FRAME_ID = model.getFrameId("iiwa_link_ee")
        use_frame = True
    except Exception:
        # Fallback to last joint if frame not found
        JOINT_ID = model.njoints - 1
        use_frame = False
        print("Warning: iiwa_link_ee frame not found in legacy solver, using last joint")
    # Sample a start configuration
    P, R = get_gripper_path(node.XL, node.YL, node.ZL, [node.phi_L], [node.theta])
    oMdes = pin.SE3(R[0], np.array(P[0]))
    if node.father:
        q = node.father.jointsPose
    else:
        q = q_start # need to get from the robot joint the initial position
    # print("q: %s" % q.T)

    eps = 1e-4
    IT_MAX = 1000
    DT = 1e-1
    damp = 1e-12
    i = 0

    while True:
        pin.forwardKinematics(model, data, q)
        
        if use_frame:
            pin.updateFramePlacements(model, data)
            current_pose = data.oMf[EE_FRAME_ID]
        else:
            current_pose = data.oMi[JOINT_ID]
            
        iMd = current_pose.actInv(oMdes)
        err = pin.log(iMd).vector  # in joint frame
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
            
        if use_frame:
            J = pin.computeFrameJacobian(model, data, q, EE_FRAME_ID)
        else:
            J = pin.computeJointJacobian(model, data, q, JOINT_ID)  # in joint frame
            
        J = -np.dot(pin.Jlog6(iMd.inverse()), J)
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pin.integrate(model, q, v * DT)
        i += 1

    if success:
        return [True, q]
    else:
        return [False, None]


def profile_ik_solvers(urdf_path, test_poses, q_start=None, num_runs=5):
    """
    Profile and compare different IK solvers.
    
    Args:
        urdf_path: Path to URDF file
        test_poses: List of target poses to test
        q_start: Initial joint configuration
        num_runs: Number of runs for each solver
        
    Returns:
        dict: Performance comparison results
    """
    print("Profiling IK Solvers...")
    print("=" * 50)
    
    # Create test node for each pose
    test_nodes = []
    for pose in test_poses:
        node = type('TestNode', (), {
            'XL': pose[0], 'YL': pose[1], 'ZL': pose[2],
            'phi_L': pose[3], 'theta': pose[4],
            'father': None
        })()
        test_nodes.append(node)
    
    results = {
        'legacy': {'times': [], 'success_rate': 0, 'total_time': 0},
        'fast': {'times': [], 'success_rate': 0, 'total_time': 0},
        'robust': {'times': [], 'success_rate': 0, 'total_time': 0}
    }
    
    # Test each solver
    for solver_name in ['legacy', 'fast', 'robust']:
        print(f"\nTesting {solver_name} solver...")
        
        successes = 0
        total_time = 0
        
        for run in range(num_runs):
            run_start = time.perf_counter()
            
            for i, node in enumerate(test_nodes):
                if solver_name == 'legacy':
                    result = _IKFromURDF_legacy(urdf_path, node, q_start)
                    success = result[0] if isinstance(result, list) else bool(result)
                else:
                    success, _, solve_time, iterations, _ = IKFromURDF_new(
                        urdf_path, node, q_start, solver_name, profile=True
                    )
                
                if success:
                    successes += 1
            
            run_time = time.perf_counter() - run_start
            results[solver_name]['times'].append(run_time)
            total_time += run_time
        
        results[solver_name]['success_rate'] = successes / (num_runs * len(test_nodes))
        results[solver_name]['total_time'] = total_time
        results[solver_name]['avg_time'] = np.mean(results[solver_name]['times'])
        results[solver_name]['std_time'] = np.std(results[solver_name]['times'])
        
        print(f"  Success rate: {results[solver_name]['success_rate']:.2%}")
        print(f"  Average time: {results[solver_name]['avg_time']:.4f}s")
        print(f"  Std dev time: {results[solver_name]['std_time']:.4f}s")
    
    # Print comparison
    print("\n" + "=" * 50)
    print("PERFORMANCE COMPARISON")
    print("=" * 50)
    
    legacy_time = results['legacy']['avg_time']
    for solver_name in ['fast', 'robust']:
        speedup = legacy_time / results[solver_name]['avg_time']
        print(f"{solver_name.capitalize()} solver is {speedup:.1f}x faster than legacy")
    
    # Get detailed stats from new solvers
    solver = get_ik_solver(urdf_path)
    if solver.solve_counts > 0:
        stats = solver.get_performance_stats()
        print(f"\nDetailed stats (new solvers):")
        print(f"  Total IK solves: {stats['total_solves']}")
        print(f"  Average solve time: {stats['avg_solve_time']:.6f}s")
        print(f"  Average iterations: {stats['avg_iterations']:.1f}")
    
    return results


