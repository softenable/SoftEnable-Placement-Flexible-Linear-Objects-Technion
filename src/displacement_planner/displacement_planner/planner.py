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
# from displacement_planner.displacement.node_class import Node, Target, JointNode
from displacement_planner.displacement import utils as hf
from displacement_planner.displacement.main import DisplacementCalculator


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
        
        self.displacement_calculator = DisplacementCalculator(
            urdf_path=self.urdf_path,
            object_length=self.object_length,
            table_height=self.table_height,
            friction_coefficient=np.min([self.mu_1, self.mu_2]),
            alpha=self.alpha,
            tip_link=self.tip_link,
            logger = self.get_logger(),
        )
        
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
                    "/usr/share/iiwa-description/urdf/softenable.urdf",
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
                (
                    "dt",
                    0.1,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE, description="Time step between trajectory points in seconds."
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
        self.dt = self.get_parameter("dt").get_parameter_value().double_value

        # Derived parameters
        self.mu_fric = np.min([self.mu_1, self.mu_2])

        self.get_logger().info(f"Loaded parameters:")
        self.get_logger().info(f"  URDF path: {self.urdf_path}")
        self.get_logger().info(f"  Object length: {self.object_length}")
        self.get_logger().info(f"  Friction coefficients: μ1={self.mu_1}, μ2={self.mu_2}")
        self.get_logger().info(f"  Table height: {self.table_height}")
        self.get_logger().info(f"  Alpha: {self.alpha}")
        self.get_logger().info(f"  Tip link: {self.tip_link}")
        self.get_logger().info(f"  Time step (dt): {self.dt}")
 
    def _extract_request_data(
        self, request: GetObjectPlacementTrajectory.Request
    ) -> Tuple[np.ndarray, List[float], List[float], str, float, float]:
        """Extract and validate data from service request."""
        try:
            # Extract start configuration
            q_start = np.array(list(request.start_state.position))

            # Pad joint configuration if needed to match model DOF
            if len(q_start) < self.displacement_calculator.model.nq:
                self.get_logger().info(
                    f"Padding joint configuration from {len(q_start)} to {self.displacement_calculator.model.nq} DOF (adding zeros for gripper joints)"
                )
                q_start = np.pad(q_start, (0, self.displacement_calculator.model.nq - len(q_start)), mode="constant", constant_values=0.0)
            elif len(q_start) > self.displacement_calculator.model.nq:
                self.get_logger().warn(
                    f"Joint configuration has {len(q_start)} DOF but model has {self.displacement_calculator.model.nq}, truncating"
                )
                q_start = q_start[: self.displacement_calculator.model.nq]

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

    def _generate_cartesian_trajectory(self, joint_trajectory: List[List[float]], dt: float) -> MultiDOFJointTrajectory:
        """Generate cartesian trajectory from joint trajectory."""

        cartesian_traj = MultiDOFJointTrajectory()
        cartesian_traj.header.stamp = self.get_clock().now().to_msg()
        cartesian_traj.header.frame_id = "world"
        cartesian_traj.joint_names = ["end_effector"]

        for i, q in enumerate(joint_trajectory):
            try:
                # Get end-effector pose for this joint configuration
                pose = self.displacement_calculator.get_end_effector_pose(np.array(q))

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

    def create_trajectory_msg(self, path: Any) -> JointTrajectory:
        """Convert joint trajectory list to JointTrajectory message."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "world"

        dt = 0.1  # 100ms between points
        for i, joint_config in enumerate(path):
                    point = JointTrajectoryPoint()
                    # Convert numpy array to list of floats for ROS2 message
                    point.positions = [float(x) for x in joint_config]
                    point.time_from_start = Duration(seconds=i * dt).to_msg()
                    traj_msg.points.append(point)

        return traj_msg
    
    def publish_cartesian_trajectory_marker(self, cartesian_trajectory: MultiDOFJointTrajectory) -> None:
        """
        Publish trajectory visualization markers for RViz.

        Args:
            gripper_paths: List of gripper positions for each sub-trajectory
        """

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "displacement_trajectory"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        # Marker properties
        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025

        # Colour for this sub-trajectory
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        for point in cartesian_trajectory.points:
            pt = Point()
            pt.x = point.transforms[0].translation.x
            pt.y = point.transforms[0].translation.y
            pt.z = point.transforms[0].translation.z
            marker.points.append(pt)

        self.marker_pub.publish(marker)
        self.get_logger().info(
            f"Published trajectory visualization with {len(marker.points)} points"
        )
    
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
            self.get_logger().info("Extracting request data...")
            q_start, start_pos, target_pos, direction, object_length, distance_from_tip = self._extract_request_data(
                request
            )
            
            self.get_logger().info(f"Starting displacement planning...")
            result = self.displacement_calculator.plan_displacement(
                start_pose = start_pos,
                target_pose = target_pos,
                # q_start = q_start, # TODO uncomment
                direction = direction,
                object_length = object_length
            )      
          
            if not result:               
                self.get_logger().error("Displacement planning failed - no valid path found")
                # Return empty trajectories on failure
                response.joint_trajectory = self.create_trajectory_msg([])
                response.cartesian_trajectory.header.stamp = self.get_clock().now().to_msg()
                response.cartesian_trajectory.header.frame_id = "world"
                return response
            
            self.get_logger().info(f"Planning successful in {result['planning_time']:.2f} seconds with {len(result['path'])} points")
            self.get_logger().info(f"Generated {len(result['combined_joint_trajectory'])} trajectory points")
            
            # Populate joint trajectory response
            response.joint_trajectory = self.create_trajectory_msg(result["combined_joint_trajectory"])

            # Generate cartesian trajectory (end-effector poses)
            response.cartesian_trajectory = self._generate_cartesian_trajectory(result["combined_joint_trajectory"], self.dt)

            # Publish visualization markers
            self.publish_cartesian_trajectory_marker(response.cartesian_trajectory)

            planning_time = time.time() - start_time
            self.get_logger().info(f"Planning completed successfully in {planning_time:.2f} seconds")
            self.get_logger().info(f"Generated trajectory with {len(response.joint_trajectory.points)} points")

        except Exception as e:
            self.get_logger().error(f"Planning failed with error: {str(e)}")
            # Return empty trajectories on error
            response.joint_trajectory = self.create_trajectory_msg([])
            response.cartesian_trajectory.header.stamp = self.get_clock().now().to_msg()
            response.cartesian_trajectory.header.frame_id = "world"

        return response

def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DisplacementPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
