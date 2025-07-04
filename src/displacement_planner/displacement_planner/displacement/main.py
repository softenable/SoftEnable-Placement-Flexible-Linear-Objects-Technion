"""
Displacement Planning Module

This module provides the main entry point for flexible object displacement planning.
It contains the core algorithm and utilities for planning trajectories that manipulate
flexible linear objects through three phases: placement, rolling, and final positioning.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import pinocchio as pin
import sys
from typing import Optional, Tuple, List, Dict, Any

# Import local modules with correct path
import displacement_planner.utils as hf
import displacement_planner.node_class as node_class
import displacement_planner.displacement as dis


class DisplacementCalculator:
    """
    Main class for displacement planning calculations.
    
    This class encapsulates the displacement planning algorithm and provides
    a clean interface for computing trajectories for flexible object manipulation.
    """
    
    def __init__(self, 
                 urdf_path: str,
                 object_length: float = 0.25,
                 table_height: float = 0.0,
                 friction_coefficient: float = 0.1,
                 alpha: float = 0.6):
        """
        Initialise the displacement calculator.
        
        Args:
            urdf_path: Path to the robot URDF file
            object_length: Length of the flexible object in meters
            table_height: Height of the table surface
            friction_coefficient: Friction coefficient for planning
            alpha: Planning algorithm parameter
        """
        self.urdf_path = urdf_path
        self.object_length = object_length
        self.table_height = table_height
        self.mu_fric = friction_coefficient
        self.alpha = alpha
        
        # Load robot model
        try:
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
        except Exception as e:
            raise RuntimeError(f"Failed to load robot model from {urdf_path}: {e}")
    
    def plan_displacement(self,
                         start_pose: List[float],
                         target_pose: List[float],
                         q_start: Optional[np.ndarray] = None,
                         direction: str = "right") -> Optional[Dict[str, Any]]:
        """
        Plan displacement trajectory for flexible object.
        
        Args:
            start_pose: [x, y, z] starting position of object
            target_pose: [x, y, z] target position of object  
            q_start: Initial joint configuration (uses neutral if None)
            direction: Rolling direction ("right" or "left")
            
        Returns:
            Dictionary containing trajectory data or None if planning failed
        """
        if q_start is None:
            q_start = pin.neutral(self.model)
        
        # Validate direction
        if direction not in ["right", "left"]:
            raise ValueError(f"Invalid direction '{direction}'. Use 'right' or 'left'")
        
        # Direction-dependent constants
        if direction == "right":
            C = 1/4
            phi_0_T2 = 0
        else:  # left
            C = 3/4
            phi_0_T2 = np.pi
        
        # Create target nodes for the three sub-problems
        start_1, target_1, target_2, target_3 = self._create_targets(
            start_pose, target_pose, direction, C, phi_0_T2
        )
        
        # Execute displacement planning
        t_start = time.time()
        path = dis.displacement(
            start_1, target_1, target_2, target_3, 
            self.object_length, self.mu_fric, direction, 
            self.table_height, self.alpha, self.urdf_path, q_start
        )
        
        if not path:
            return None
            
        # Generate smooth trajectories
        trajectory_data = self._generate_trajectories(path, direction, q_start)
        
        planning_time = time.time() - t_start
        trajectory_data['planning_time'] = planning_time
        
        return trajectory_data
    
    def _create_targets(self, start_pose: List[float], target_pose: List[float], 
                       direction: str, C: float, phi_0_T2: float) -> Tuple:
        """Create target nodes for displacement sub-problems."""
        X0_S1, Y0_S1, Z0_S1 = start_pose
        X0_T1, Y0_T1, Z0_T1 = target_pose
        
        # Ensure target is at table level
        Z0_T1 = max(Z0_T1, self.table_height)
        
        theta_S1 = np.pi/2
        theta_T1 = 0
        L_gal_S = 2 * self.object_length
        
        # Sub-problem 1: Initial placement
        target_1 = node_class.Target(
            X0_T1, Y0_T1, Z0_T1, np.pi/2, theta_T1, 1, 
            C * L_gal_S, L_gal_S, 0, self.object_length, direction
        )
        
        start_1 = node_class.Node(
            target_1, X0_S1, Y0_S1, Z0_S1, np.pi/2, theta_S1, 1, 
            C * L_gal_S, L_gal_S, 0, [], self.alpha, self.object_length, direction
        )
        
        # Sub-problem 2: Rolling phase
        mu_T2 = 0
        L_gal_T2 = 2 * self.object_length
        target_2 = node_class.Target(
            target_1.X0, target_1.Y0, target_1.Z0, phi_0_T2, target_1.theta, 
            mu_T2, C * L_gal_T2, L_gal_T2, 0, self.object_length, direction
        )
        
        # Sub-problem 3: Final placement
        mu_T3 = 0
        l_T3 = 0.5 * self.object_length
        L_gal_T3 = 4 * (self.object_length - l_T3)
        target_3 = node_class.Target(
            target_2.X0, target_2.Y0, target_2.Z0, phi_0_T2, target_2.theta,
            mu_T3, C * L_gal_T3, L_gal_T3, l_T3, self.object_length, direction
        )
        
        return start_1, target_1, target_2, target_3
    
    def _generate_trajectories(self, path: List, direction: str, q_start: np.ndarray) -> Dict[str, Any]:
        """Generate smooth trajectories from displacement path."""
        path_1, path_2, path_3 = path
        
        # Trajectory generation parameters
        skip = 8
        num_t = 8
        t = np.linspace(0, 1, num_t)
        
        # Generate smooth paths for each sub-problem
        x1, y1, z1, phi_l_1, theta_1, l1, q1 = hf.get_smooth_path_displacement(
            path_1, skip, t, self.object_length, direction, self.urdf_path, path_1[0].jointsPose
        )
        x2, y2, z2, phi_l_2, theta_2, l2, q2 = hf.get_smooth_path_displacement(
            path_2, skip, t, self.object_length, direction, self.urdf_path, path_2[0].jointsPose
        )
        x3, y3, z3, phi_l_3, theta_3, l3, q3 = hf.get_smooth_path_displacement(
            path_3, skip, t, self.object_length, direction, self.urdf_path, path_3[0].jointsPose
        )
        
        # Get gripper paths
        P1, R1 = hf.get_gripper_path(x1[:, -1], y1[:, -1], z1[:, -1], phi_l_1, theta_1)
        P2, R2 = hf.get_gripper_path(x2[:, -1], y2[:, -1], z2[:, -1], phi_l_2, theta_2)
        P3, R3 = hf.get_gripper_path(x3[:, -1], y3[:, -1], z3[:, -1], phi_l_3, theta_3)
        
        # Combine results
        return {
            'paths': [path_1, path_2, path_3],
            'object_positions': {
                'x': [x1, x2, x3],
                'y': [y1, y2, y3], 
                'z': [z1, z2, z3]
            },
            'object_orientations': {
                'phi': [phi_l_1, phi_l_2, phi_l_3],
                'theta': [theta_1, theta_2, theta_3]
            },
            'gripper_positions': [P1, P2, P3],
            'gripper_orientations': [R1, R2, R3],
            'joint_trajectories': [q1, q2, q3],
            'combined_joint_trajectory': q1 + q2 + q3,
            'combined_gripper_positions': np.array(P1.tolist() + P2.tolist() + P3.tolist()),
        }
    
    def visualize_trajectory(self, trajectory_data: Dict[str, Any], show_plot: bool = True) -> None:
        """Visualize the planned trajectory."""
        if not trajectory_data:
            print("No trajectory data to visualize")
            return
            
        paths = trajectory_data['paths']
        direction = "right" if len(paths) > 0 else "left"  # Default fallback
        
        plt.figure('Displacement Trajectory')
        plt.title('Flexible Object Displacement Planning')
        
        a = 1 if direction == "right" else -1
        
        colours = ['red', 'green', 'blue']
        labels = ['Phase 1: Placement', 'Phase 2: Rolling', 'Phase 3: Final']
        
        for phase_idx, (path, colour, label) in enumerate(zip(paths, colours, labels)):
            for node in path:
                xs, zs, phi_s = hf.get_shape(
                    node.X0 + a * node.l, node.Z0, node.phi_0, 
                    node.mu, node.L_gal, node.S0, 
                    np.linspace(0, self.object_length - node.l)
                )
                plt.plot(xs, zs, color=colour, alpha=0.7, 
                        label=label if node == path[0] else "")
        
        plt.xlabel('X Position (m)')
        plt.ylabel('Z Position (m)')
        plt.legend()
        plt.grid(True)
        
        if show_plot:
            plt.show()


def main_calculation(urdf_path: str, 
                    q_start: Optional[np.ndarray] = None,
                    object_length: float = 0.25, 
                    direction: str = "right", 
                    mu_fric: float = 0.1, 
                    h_table: float = 0.0, 
                    start_pose: List[float] = [0.4, 0.2, 0.3],
                    target_pose: Optional[List[float]] = None) -> Optional[Dict[str, Any]]:
    """
    Legacy function for backward compatibility.
    
    Args:
        urdf_path: Path to robot URDF file
        q_start: Initial joint configuration
        object_length: Length of flexible object
        direction: Rolling direction
        mu_fric: Friction coefficient
        h_table: Table height
        start_pose: Starting position [x, y, z]
        target_pose: Target position [x, y, z] (defaults to start_pose at table level)
        
    Returns:
        Trajectory data dictionary or None if planning failed
    """
    if target_pose is None:
        target_pose = [start_pose[0], start_pose[1], h_table]
    
    calculator = DisplacementCalculator(
        urdf_path=urdf_path,
        object_length=object_length,
        table_height=h_table,
        friction_coefficient=mu_fric
    )
    
    return calculator.plan_displacement(
        start_pose=start_pose,
        target_pose=target_pose,
        q_start=q_start,
        direction=direction
    )


# Alias for backward compatibility
mainCalculation = main_calculation


def main():
    """Example usage of the displacement calculator."""
    if len(sys.argv) < 2:
        print("Usage: python main.py <urdf_path>")
        print("Example: python main.py /path/to/robot.urdf")
        return
    
    urdf_path = sys.argv[1]
    
    print("=== Displacement Planning Example ===")
    print(f"Robot URDF: {urdf_path}")
    
    # Example parameters
    start_pose = [0.4, 0.2, 0.3]
    target_pose = [0.5, 0.2, 0.0]
    
    try:
        calculator = DisplacementCalculator(urdf_path)
        
        print(f"Planning displacement from {start_pose} to {target_pose}")
        result = calculator.plan_displacement(
            start_pose=start_pose,
            target_pose=target_pose,
            direction="right"
        )
        
        if result:
            print(f"Planning successful in {result['planning_time']:.2f} seconds")
            print(f"Generated {len(result['combined_joint_trajectory'])} trajectory points")
            
            # Visualize result
            calculator.visualize_trajectory(result, show_plot=True)
        else:
            print("Planning failed")
            
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()







