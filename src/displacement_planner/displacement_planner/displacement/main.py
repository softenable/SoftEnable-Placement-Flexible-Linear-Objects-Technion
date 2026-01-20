"""
Displacement Planning Module

This module provides the main entry point for flexible object displacement planning.
It contains the core algorithm and utilities for planning trajectories that manipulate
flexible linear objects through three phases: placement, rolling, and final positioning.
"""

import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
import argparse

import displacement_planner.displacement.displacement as dis
import displacement_planner.displacement.node_class as node_class

# Import local modules with correct path
import displacement_planner.displacement.utils as hf
import displacement_planner.displacement.visualize_urdf as vis
from displacement_planner.displacement.logger import Logger

class DisplacementCalculator:
    """
    Main class for displacement planning calculations.

    This class encapsulates the displacement planning algorithm and provides
    a clean interface for computing trajectories for flexible object manipulation.
    """

    def __init__(
        self,
        urdf_path: str,
        object_length: float = 0.3,
        table_height: float = 0.0,
        friction_coefficient: float = 0.1,
        alpha: float = 0.6,
        logger: Optional[Any] = None,
        tip_link: str = "dlr_gripper_center",
    ):
        """
        Initialise the displacement calculator.

        Args:
            urdf_path: Path to the robot URDF file
            object_length: Length of the flexible object in meters
            table_height: Height of the table surface
            friction_coefficient: Friction coefficient for planning
            alpha: Planning algorithm parameter
            tip_link: Name of the robot tip link for kinematics 
            logger: Logger instance for logging messages
        """
        self.urdf_path = urdf_path
        self.object_length = object_length
        self.table_height = table_height
        self.mu_fric = friction_coefficient
        self.alpha = alpha
        self.logger = logger
        self.tip_link = tip_link

        # Load robot model
        try:
            self.model = pin.buildModelFromUrdf(self.urdf_path)
            self.data = self.model.createData()
        except Exception as e:
            raise RuntimeError(f"Failed to load robot model from {urdf_path}: {e}")
        
        self.logger.info(f"Loaded robot model from {urdf_path} with {self.model.nq} DOF")
        if self.model.existFrame(self.tip_link):
            self.tip_frame_id = self.model.getFrameId(self.tip_link)
            self.logger.info(f"Using tip link '{self.tip_link}' with frame ID {self.tip_frame_id}")
        else:
            # If tip link not found, use end-effector
            self.tip_frame_id = self.model.nframes - 1
            self.logger.warn(f"Tip link '{self.tip_link}' not found, using frame ID {self.tip_frame_id}")


    def get_end_effector_pose(self, q: np.ndarray) -> pin.SE3:
        """Get end-effector pose for given joint configuration."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.tip_frame_id]
    
    def plan_displacement(
        self,
        start_pose: List[float],
        target_pose: List[float],
        q_start: Optional[np.ndarray] = None,
        direction: str = "left",
        object_length: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        """
        Plan displacement trajectory for flexible object.

        Args:
            start_pose: [x, y, z] starting position of object
            target_pose: [x, y, z] target position of object
            q_start: Initial joint configuration (uses neutral if None)
            direction: Rolling direction ("right" or "left")
            object_length: Length of the flexible object (defaults to class attribute)

        Returns:
            Dictionary containing trajectory data or None if planning failed
        """

        # Validate direction
        if direction not in ["right", "left"]:
            raise ValueError(f"Invalid direction '{direction}'. Use 'right' or 'left'")
        
        if object_length is not None:
            self.object_length = object_length

        # Direction-dependent constants
        if direction == "right":
            C = 1 / 4
            phi_0_T2 = 0
        else:  # left
            C = 3 / 4
            phi_0_T2 = np.pi
            
        self.logger.info(f"Planning displacement: start {start_pose}, target {target_pose}, direction {direction}, object length {self.object_length:.3f}m, friction {self.mu_fric:.2f}, table height {self.table_height:.3f}m, alpha {self.alpha:.2f}, C {C:.2f}, phi_0_T2 {phi_0_T2:.2f}rad, q_start {'provided' if q_start is not None else 'neutral'}")

        # Create target nodes for the three sub-problems
        self.logger.info("Creating target nodes for displacement sub-problems...")
        start_1, target_1, target_2, target_3 = self._create_targets(start_pose, target_pose, direction, C, phi_0_T2)
        t_start = time.time()
        if q_start is None:
            self.logger.info("Finding feasible starting joint configuration...")
            q_start = hf.find_feasible_q_start(self.urdf_path, start_1, target_1, target_2, target_3)
            # q_start = pin.neutral(self.model)

        self.logger.info("Planning sub-problem 1...")
        path_1 = dis.placement(start_1, target_1, self.object_length, self.mu_fric, direction, self.table_height, self.alpha, self.urdf_path, q_start, 1)

        if not path_1:
            self.logger.error("Path for sub-problem 1 not found")
            return []
        self.logger.info("Sub-problem 1 completed, planning sub-problem 2...")

        path_2 = dis.placement(path_1[-1], target_2, self.object_length, self.mu_fric, direction, self.table_height, self.alpha, self.urdf_path, path_1[-1].jointsPose, 2)

        if not path_2:
            self.logger.error("Path for sub-problem 2 not found")
            return []
        self.logger.info("Sub-problem 2 completed, planning sub-problem 3...")

        path_3 = dis.placement(path_2[-1], target_3, self.object_length, self.mu_fric, direction, self.table_height, self.alpha, self.urdf_path, path_2[-1].jointsPose, 3)

        if not path_3:
            self.logger.error("Path for sub-problem 3 not found")
            return []
        self.logger.info("Sub-problem 3 completed...")
        
        ### smoothness trajectory ###
        # path_filt = hf.filter_path(path, urdf, window_size=10)
        path_1 = hf.get_smooth_path_displacement_nodes(path_1, skip=5, t=np.linspace(0, 1, 10),
                                                            L=self.object_length, direction=direction, urdf=self.urdf_path,
                                                            q_start=path_1[0].jointsPose)
        path_2 = hf.get_smooth_path_displacement_nodes(path_2, skip=5, t=np.linspace(0, 1, 10),
                                                            L=self.object_length, direction=direction, urdf=self.urdf_path,
                                                            q_start=path_1[-1].jointsPose)
        path_3 = hf.get_smooth_path_displacement_nodes(path_3, skip=5, t=np.linspace(0, 1, 10),
                                                            L=self.object_length, direction=direction, urdf=self.urdf_path,
                                                            q_start=path_2[-1].jointsPose)

        return {
            "path": path_1 + path_2 + path_3,
            "path_1": path_1,
            "path_2": path_2,
            "path_3": path_3,
            "combined_joint_trajectory": hf.path_to_robot_joint_pose(path_1 + path_2 + path_3),
            "planning_time": time.time() - t_start
            }


    def _create_targets(
        self, start_pose: List[float], target_pose: List[float], direction: str, C: float, phi_0_T2: float
    ) -> Tuple:
        """Create target nodes for displacement sub-problems."""
        X0_S1, Y0_S1, Z0_S1 = start_pose
        X0_T1, Y0_T1, Z0_T1 = target_pose

        # Ensure target is at table level
        # Z0_T1 = max(Z0_T1, self.table_height)

        theta_S1 = np.pi / 2
        theta_T1 = np.pi / 2.5
        L_gal_S = 2 * self.object_length

        # Sub-problem 1: Initial placement
        target_1 = node_class.Target(
            X0_T1, Y0_T1, Z0_T1, np.pi / 2, theta_T1, 1, C * L_gal_S, L_gal_S, 0, self.object_length, direction
        )

        start_1 = node_class.Node(
            target_1,
            X0_S1,
            Y0_S1,
            Z0_S1,
            np.pi / 2,
            theta_S1,
            1,
            C * L_gal_S,
            L_gal_S,
            0,
            [],
            self.alpha,
            self.object_length,
            direction,
        )

        # Sub-problem 2: Rolling phase
        mu_T2 = 0
        L_gal_T2 = 3 * self.object_length
        target_2 = node_class.Target(
            target_1.X0,
            target_1.Y0,
            target_1.Z0,
            phi_0_T2,
            target_1.theta,
            mu_T2,
            C * L_gal_T2,
            L_gal_T2,
            0,
            self.object_length,
            direction,
        )

        # Sub-problem 3: Final placement
        mu_T3 = 0
        l_T3 = 0.5 * self.object_length
        L_gal_T3 = 3.5 * (self.object_length - l_T3)
        target_3 = node_class.Target(
            target_2.X0,
            target_2.Y0,
            target_2.Z0,
            phi_0_T2,
            target_2.theta,
            mu_T3,
            C * L_gal_T3,
            L_gal_T3,
            l_T3,
            self.object_length,
            direction,
        )

        return start_1, target_1, target_2, target_3

    def visualize_trajectory(self, path, show_plot: bool = True, direction: str = "right") -> None:
        """Visualize the planned trajectory."""
        P, R = hf.get_gripper_path(path)
        robot_joint_pose = hf.path_to_robot_joint_pose(path)
        x, y, z = hf.get_path_shapes(path, self.object_length, direction)
        vis.animate_robot_and_cable_meshcat(self.urdf_path, robot_joint_pose, x, y, z, dt=0.03)


def main_calculation(
    urdf_path: str,
    q_start: Optional[np.ndarray] = None,
    object_length: float = 0.25,
    direction: str = "right",
    mu_fric: float = 0.1,
    h_table: float = 0.0,
    start_pose: List[float] = [0.4, 0.2, 0.3],
    target_pose: Optional[List[float]] = None,
) -> Optional[Dict[str, Any]]:
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
        urdf_path=urdf_path, object_length=object_length, table_height=h_table, friction_coefficient=mu_fric
    )

    return calculator.plan_displacement(
        start_pose=start_pose, target_pose=target_pose, q_start=q_start, direction=direction
    )


# Alias for backward compatibility
mainCalculation = main_calculation


def main():
    """Example usage of the displacement calculator."""
    parser = argparse.ArgumentParser(
        description="Flexible Object Displacement Planning",
    )
    parser.add_argument(
        "--urdf-path",
        type=str,
        help="Path to the robot URDF file",
        default="/usr/share/iiwa-description/urdf/softenable.urdf"
    )
    parser.add_argument(
        "--direction",
        type=str,
        help="Path to the robot URDF file",
        default="left"
    )
    args = parser.parse_args()
    urdf_path = args.urdf_path

    print("=== Displacement Planning Example ===")
    print(f"Robot URDF: {urdf_path}")

    # Example parameters
    logger = Logger()

    calculator = DisplacementCalculator(urdf_path, logger=logger)

    # start_pose = [0.8, 0.4, 0.3]
    # target_pose = [0.6, 0.4, ]
    start_pose = [0.8, 0.4, calculator.table_height + 0.3 * calculator.object_length]
    target_pose = [0.6, 0.4, calculator.table_height]

    print(f"Planning displacement from {start_pose} to {target_pose}")
    result = calculator.plan_displacement(start_pose=start_pose, target_pose=target_pose, direction=args.direction)

    if result:
        print(f"Planning successful in {result['planning_time']:.2f} seconds")
        print(f"Generated {len(result['combined_joint_trajectory'])} trajectory points")

        # Visualize result
        calculator.visualize_trajectory(result["path"], show_plot=True, direction=args.direction)
        vis.plot_graphs(result["path_1"], result["path_2"], result["path_3"], args.direction)
    
    else:
        print("Planning failed")



if __name__ == "__main__":
    main()
