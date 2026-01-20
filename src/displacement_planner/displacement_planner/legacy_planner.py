import os
import sys
import time
from typing import Any, List, Tuple

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from displacement_flexible_linear_objects_msgs.srv import GetObjectPlacementTrajectory
from geometry_msgs.msg import Point, Transform, Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.time import Duration
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)
from visualization_msgs.msg import Marker

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from displacement_planner.displacement import displacement as dis
from displacement_planner.displacement import node_class
from displacement_planner.displacement import utils as hf


class DisplacementPlanner(Node):  # type: ignore
    """
    ROS 2 service node for planning flexible object displacement trajectories.

    This service takes object poses and robot start configuration and plans a trajectory
    for displacing a flexible linear object from start to target position.
    """

    def __init__(self) -> None:
        super().__init__("displacement_planner")

        # Create service
        self.service = self.create_service(
            GetObjectPlacementTrajectory, "placement/plan_trajectory", self.plan_trajectory
        )

        # Declare parameters
        self._declare_parameters()

        # Get parameters
        self._load_parameters()

        # Initialise robot model and kinematics
        self._setup_robot_model()

        # Setup visualization
        self.marker_pub = self.create_publisher(Marker, "visualization/trajectory_marker", 10)

        self.get_logger().info("Displacement planner service started")

    def _declare_parameters(self) -> None:
        """Declare ROS parameters for the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "urdf_file_path",
                    "/usr/share/iiwa-description/urdf/iiwa14_with_ft_and_dlr_gripper.urdf",
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING, description="The path to urdf file of the robot."
                    ),
                ),
                (
                    "object_length",
                    0.3,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE, description="Length of the flexible object in meters."
                    ),
                ),
                (
                    "friction_coefficient_object_tray",
                    0.5,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE, description="Friction coefficient between object and tray."
                    ),
                ),
                (
                    "friction_coefficient_tray_table",
                    0.1,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE, description="Friction coefficient between tray and table."
                    ),
                ),
                (
                    "table_height",
                    0.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE, description="Height of the table relative to world frame."
                    ),
                ),
                (
                    "alpha",
                    0.6,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE, description="Alpha parameter for displacement planning."
                    ),
                ),
                (
                    "tip_link",
                    "dlr_gripper_center",
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING, description="Name of the robot tip link for kinematics."
                    ),
                ),
            ],
        )

    def _load_parameters(self) -> None:
        """Load parameters from ROS parameter server."""
        urdf_path_param = self.get_parameter("urdf_file_path").get_parameter_value().string_value

        self.get_logger().info(f"Raw URDF path parameter: '{urdf_path_param}'")

        # Resolve package-relative paths
        if urdf_path_param.startswith("$(find "):
            # Extract package name from $(find package_name)/path format
            import re

            match = re.match(r"\$\(find\s+([\w-]+)\)/(.*)", urdf_path_param)
            if match:
                package_name = match.group(1)
                relative_path = match.group(2)
                self.get_logger().info(
                    f"Resolving package path: package='{package_name}', relative_path='{relative_path}'"
                )
                try:
                    package_share_dir = get_package_share_directory(package_name)
                    self.urdf_path = os.path.join(package_share_dir, relative_path)
                    self.get_logger().info(f"Resolved URDF path: {self.urdf_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to resolve package path {urdf_path_param}: {e}")
                    self.get_logger().info("Falling back to literal path interpretation")
                    self.urdf_path = urdf_path_param
            else:
                self.get_logger().warn(f"Invalid $(find ...) syntax in URDF path: {urdf_path_param}")
                self.urdf_path = urdf_path_param
        else:
            # Use path as-is
            self.urdf_path = urdf_path_param
            self.get_logger().info(f"Using URDF path as-is: {self.urdf_path}")

        # Verify the URDF file exists
        if not os.path.exists(self.urdf_path):
            self.get_logger().error(f"URDF file does not exist: {self.urdf_path}")
        else:
            self.get_logger().info(f"URDF file found: {self.urdf_path}")

        self.object_length = self.get_parameter("object_length").get_parameter_value().double_value
        self.mu_1 = self.get_parameter("friction_coefficient_object_tray").get_parameter_value().double_value
        self.mu_2 = self.get_parameter("friction_coefficient_tray_table").get_parameter_value().double_value
        self.table_height = self.get_parameter("table_height").get_parameter_value().double_value
        self.alpha = self.get_parameter("alpha").get_parameter_value().double_value
        self.tip_link = self.get_parameter("tip_link").get_parameter_value().string_value

        # Derived parameters
        self.mu_fric = np.min([self.mu_1, self.mu_2])

        self.get_logger().info(f"Loaded parameters:")
        self.get_logger().info(f"  URDF path: {self.urdf_path}")
        self.get_logger().info(f"  Object length: {self.object_length}")
        self.get_logger().info(f"  Friction coefficients: μ1={self.mu_1}, μ2={self.mu_2}")
        self.get_logger().info(f"  Table height: {self.table_height}")
        self.get_logger().info(f"  Alpha: {self.alpha}")

    def _setup_robot_model(self) -> None:
        """Initialise robot kinematic model using Pinocchio directly."""
        try:
            # Load robot model directly with Pinocchio
            self.model = pin.buildModelFromUrdf(self.urdf_path)
            self.data = self.model.createData()

            # Find the tip link frame ID
            if self.model.existFrame(self.tip_link):
                self.tip_frame_id = self.model.getFrameId(self.tip_link)
            else:
                # If tip link not found, use end-effector
                self.tip_frame_id = self.model.nframes - 1
                self.get_logger().warn(f"Tip link '{self.tip_link}' not found, using frame ID {self.tip_frame_id}")

            self.get_logger().info(f"Robot model loaded successfully:")
            self.get_logger().info(f"  Model name: {self.model.name}")
            self.get_logger().info(f"  DOF: {self.model.nq}")
            self.get_logger().info(f"  Tip frame ID: {self.tip_frame_id}")

        except Exception as e:
            self.get_logger().error(f"Failed to initialise robot model: {str(e)}")
            raise

    def _get_end_effector_pose(self, q: np.ndarray) -> pin.SE3:
        """Get end-effector pose for given joint configuration."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.tip_frame_id]

    def _extract_request_data(
        self, request: GetObjectPlacementTrajectory.Request
    ) -> Tuple[np.ndarray, List[float], List[float], str, float, float]:
        """Extract and validate data from service request."""
        try:
            # Extract start configuration
            q_start = np.array(list(request.start_state.position))

            # Pad joint configuration if needed to match model DOF
            if len(q_start) < self.model.nq:
                self.get_logger().info(
                    f"Padding joint configuration from {len(q_start)} to {self.model.nq} DOF (adding zeros for gripper joints)"
                )
                q_start = np.pad(q_start, (0, self.model.nq - len(q_start)), mode="constant", constant_values=0.0)
            elif len(q_start) > self.model.nq:
                self.get_logger().warn(
                    f"Joint configuration has {len(q_start)} DOF but model has {self.model.nq}, truncating"
                )
                q_start = q_start[: self.model.nq]

            # Extract object properties
            object_length = request.target_length
            distance_from_tip = request.distance_from_tip

            # Extract poses
            start_pose = request.start_pose
            target_pose = request.target_pose

            # Extract direction (now a string)
            roll_direction = request.roll_direction  # 'left' or 'right'

            # Extract positions
            start_pos = [start_pose.position.x, start_pose.position.y, start_pose.position.z]

            target_pos = [target_pose.position.x, target_pose.position.y, target_pose.position.z]

            self.get_logger().info(f"Request data extracted:")
            self.get_logger().info(f"  Object length: {object_length}")
            self.get_logger().info(f"  Distance from tip: {distance_from_tip}")
            self.get_logger().info(f"  Start position: {start_pos}")
            self.get_logger().info(f"  Target position: {target_pos}")
            self.get_logger().info(f"  Roll direction: {roll_direction}")
            self.get_logger().info(f"  Joint configuration: {len(q_start)} DOF")

            return q_start, start_pos, target_pos, roll_direction, object_length, distance_from_tip

        except Exception as e:
            self.get_logger().error(f"Failed to extract request data: {str(e)}")
            raise

    def _create_displacement_targets(
        self, start_pos: List[float], target_pos: List[float], direction: str, object_length: float
    ) -> Tuple[node_class.Node, node_class.Target, node_class.Target, node_class.Target]:
        """Create target nodes for the three displacement sub-problems."""
        # Direction-dependent constants
        if direction == "right":
            C = 1 / 4
            phi_0_T2 = 0
        elif direction == "left":
            C = 3 / 4
            phi_0_T2 = np.pi
        else:
            raise ValueError(f"Invalid direction: {direction}")

        # Sub-problem 1: Initial placement
        X0_S1, Y0_S1, Z0_S1 = start_pos
        Z0_S1 = max(Z0_S1, self.table_height + 0.3 * object_length)  # Ensure above table
        theta_S1 = np.pi / 2

        X0_T1, Y0_T1, Z0_T1 = target_pos
        Z0_T1 = max(Z0_T1, self.table_height)  # Ensure on table
        theta_T1 = 0

        L_gal_S = 2 * object_length

        target_1 = node_class.Target(
            X0_T1, Y0_T1, Z0_T1, np.pi / 2, theta_T1, 1, C * L_gal_S, L_gal_S, 0, object_length, direction
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
            object_length,
            direction,
        )

        # Sub-problem 2: Rolling phase
        mu_T2 = 0
        L_gal_T2 = 2 * object_length
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
            object_length,
            direction,
        )

        # Sub-problem 3: Final placement
        mu_T3 = 0
        l_T3 = 0.5 * object_length
        L_gal_T3 = 4 * (object_length - l_T3)
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
            object_length,
            direction,
        )

        return start_1, target_1, target_2, target_3

    def _generate_trajectory(
        self, path: List[List[node_class.Node]], direction: str, object_length: float
    ) -> Tuple[List[List[float]], List[List[List[float]]]]:
        """Generate smooth joint trajectory from displacement path."""
        if not path or len(path) != 3:
            raise ValueError("Invalid path structure - expected 3 sub-paths")

        path_1, path_2, path_3 = path

        # Trajectory generation parameters
        skip = 8
        num_t = 8
        t = np.linspace(0, 1, num_t)

        try:
            # Generate smooth paths for each sub-problem
            x1, y1, z1, phi_l_1, theta_1, l1, q1 = hf.get_smooth_path_displacement(
                path_1, skip, t, object_length, direction, self.urdf_path, path_1[0].jointsPose
            )
            x2, y2, z2, phi_l_2, theta_2, l2, q2 = hf.get_smooth_path_displacement(
                path_2, skip, t, object_length, direction, self.urdf_path, path_2[0].jointsPose
            )
            x3, y3, z3, phi_l_3, theta_3, l3, q3 = hf.get_smooth_path_displacement(
                path_3, skip, t, object_length, direction, self.urdf_path, path_3[0].jointsPose
            )

            # Get gripper paths for visualization
            P1, R1 = hf.get_gripper_path(x1[:, -1], y1[:, -1], z1[:, -1], phi_l_1, theta_1)
            P2, R2 = hf.get_gripper_path(x2[:, -1], y2[:, -1], z2[:, -1], phi_l_2, theta_2)
            P3, R3 = hf.get_gripper_path(x3[:, -1], y3[:, -1], z3[:, -1], phi_l_3, theta_3)

            # Combine trajectories
            gripper_paths = [P1.tolist(), P2.tolist(), P3.tolist()]
            joint_trajectory = q1 + q2 + q3

            return joint_trajectory, gripper_paths

        except Exception as e:
            self.get_logger().error(f"Failed to generate trajectory: {str(e)}")
            raise

    def _generate_cartesian_trajectory(self, joint_trajectory: List[List[float]], dt: float) -> MultiDOFJointTrajectory:
        """Generate cartesian trajectory from joint trajectory."""

        cartesian_traj = MultiDOFJointTrajectory()
        cartesian_traj.header.stamp = self.get_clock().now().to_msg()
        cartesian_traj.header.frame_id = "world"
        cartesian_traj.joint_names = ["end_effector"]

        for i, q in enumerate(joint_trajectory):
            try:
                # Get end-effector pose for this joint configuration
                pose = self._get_end_effector_pose(np.array(q))

                # Create trajectory point
                point = MultiDOFJointTrajectoryPoint()

                # Convert SE3 to Transform
                transform = Transform()
                transform.translation.x = pose.translation[0]
                transform.translation.y = pose.translation[1]
                transform.translation.z = pose.translation[2]

                # Convert rotation matrix to quaternion
                from scipy.spatial.transform import Rotation as R

                rot = R.from_matrix(pose.rotation)
                quat = rot.as_quat()  # [x, y, z, w]
                transform.rotation.x = quat[0]
                transform.rotation.y = quat[1]
                transform.rotation.z = quat[2]
                transform.rotation.w = quat[3]

                point.transforms = [transform]
                point.time_from_start = Duration(seconds=i * dt).to_msg()

                cartesian_traj.points.append(point)

            except Exception as e:
                self.get_logger().warn(f"Failed to compute cartesian point {i}: {str(e)}")
                continue

        return cartesian_traj

    def plan_trajectory(self, request: Any, response: Any) -> Any:
        """
        Main service callback for planning displacement trajectory.

        Args:
            request: Service request containing object poses and robot state
            response: Service response to populate with joint trajectory

        Returns:
            Service response with planned trajectory
        """
        start_time = time.time()
        self.get_logger().info("Displacement planning service called")

        try:
            # Extract and validate request data
            q_start, start_pos, target_pos, direction, object_length, distance_from_tip = self._extract_request_data(
                request
            )

            # Create displacement targets for the three sub-problems
            start_1, target_1, target_2, target_3 = self._create_displacement_targets(
                start_pos, target_pos, direction, object_length
            )

            # Plan displacement path
            self.get_logger().info("Starting displacement planning algorithm...")
            # TODO this will now be 1 target at a time I think
            path = dis.displacement(
                start_1,
                target_1,
                target_2,
                target_3,
                object_length,
                self.mu_fric,
                direction,
                self.table_height,
                self.alpha,
                self.urdf_path,
                q_start,
            )

            if not path:
                self.get_logger().error("Displacement planning failed - no valid path found")
                # Return empty trajectories on failure
                response.joint_trajectory.joint_names = [
                    "joint1",
                    "joint2",
                    "joint3",
                    "joint4",
                    "joint5",
                    "joint6",
                    "joint7",
                ]
                response.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
                response.joint_trajectory.header.frame_id = "world"
                response.cartesian_trajectory.header.stamp = self.get_clock().now().to_msg()
                response.cartesian_trajectory.header.frame_id = "world"
                return response

            self.get_logger().info(f"Displacement planning completed with {len(path)} sub-paths")

            # Generate smooth joint trajectory
            joint_trajectory, gripper_paths = self._generate_trajectory(path, direction, object_length)

            # Populate joint trajectory response
            response.joint_trajectory.joint_names = [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
                "joint7",
            ]
            response.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
            response.joint_trajectory.header.frame_id = "world"

            # Add trajectory points with timing
            dt = 0.1  # 100ms between points
            for i, joint_config in enumerate(joint_trajectory):
                point = JointTrajectoryPoint()
                point.positions = joint_config
                point.time_from_start = Duration(seconds=i * dt).to_msg()
                response.joint_trajectory.points.append(point)

            # Generate cartesian trajectory (end-effector poses)
            response.cartesian_trajectory = self._generate_cartesian_trajectory(joint_trajectory, dt)

            # Publish visualization markers
            self.publish_trajectory_marker(gripper_paths)

            planning_time = time.time() - start_time
            self.get_logger().info(f"Planning completed successfully in {planning_time:.2f} seconds")
            self.get_logger().info(f"Generated trajectory with {len(joint_trajectory)} points")

        except Exception as e:
            self.get_logger().error(f"Planning failed with error: {str(e)}")
            # Return empty trajectories on error
            response.joint_trajectory.joint_names = [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
                "joint7",
            ]
            response.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
            response.joint_trajectory.header.frame_id = "world"
            response.cartesian_trajectory.header.stamp = self.get_clock().now().to_msg()
            response.cartesian_trajectory.header.frame_id = "world"

        return response

    def publish_trajectory_marker(self, gripper_paths: List[List[List[float]]]) -> None:
        """
        Publish trajectory visualization markers for RViz.

        Args:
            gripper_paths: List of gripper positions for each sub-trajectory
        """
        colours = [
            (1.0, 0.0, 0.0),  # Red - Sub-problem 1
            (0.0, 1.0, 0.0),  # Green - Sub-problem 2
            (0.0, 0.0, 1.0),  # Blue - Sub-problem 3
        ]

        for idx, path_points in enumerate(gripper_paths):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "displacement_trajectory"
            marker.id = idx
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD

            # Marker properties
            marker.scale.x = 0.025
            marker.scale.y = 0.025
            marker.scale.z = 0.025

            # Colour for this sub-trajectory
            r, g, b = colours[idx % len(colours)]
            marker.color.a = 1.0
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            # Add points
            for point_coords in path_points:
                pt = Point()
                pt.x = point_coords[0]
                pt.y = point_coords[1]
                pt.z = point_coords[2]
                marker.points.append(pt)

            self.marker_pub.publish(marker)
            self.get_logger().info(
                f"Published trajectory visualization for sub-problem {idx + 1} " f"with {len(marker.points)} points"
            )


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DisplacementPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
