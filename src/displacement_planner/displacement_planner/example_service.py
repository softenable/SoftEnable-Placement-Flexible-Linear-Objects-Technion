# © Copyright 2024, Ocado Group PLC. All the rights reserved.
import rclpy
from rclpy.node import Node
from displacement_flexible_linear_objects_msgs.srv import GetObjectPlacementTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.time import Duration
from typing import Any
from scipy.spatial.transform import Rotation as R


class ExampleService(Node):  # type: ignore
    def __init__(self) -> None:
        super().__init__("latent_space_planner")
        self.service = self.create_service(GetObjectPlacementTrajectory, "plan_trajectory", self.plan_trajectory)

        # It is good practice to use ROS parameters to store variables that will not change during runtime but may change between different runs of the demonstration.
        # For example, when running the demonstrator the robot URDF file path will not change but over time it may be revised.

        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "urdf_file_path",
                    "some/path/in/the/shared/volume.urdf",
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING, description="The path to urdf file of the robot."
                    ),
                ),
                (
                    "friction_coefficients",
                    [0.1, 0.2],
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE_ARRAY, description="The coefficient of friction between the table and the tray followed by the coefficient of friction between the tray and the object."
                    ),
                ),
                (
                    "table_height",
                    -0.1,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="The height of the robot relative to the world frame.",
                    ),
                ),
            ],
        )

        self.urdf_path = self.get_parameter("urdf_file_path").get_parameter_value().string_value
        self.friction_coefficients = self.get_parameter("friction_coefficients").get_parameter_value().double_array_value
        self.table_height = self.get_parameter("table_height").get_parameter_value().double_value

        self.get_logger().info("URDF file path: %s" % self.urdf_path)
        self.get_logger().info("Friction coefficients: %s" % self.friction_coefficients)
        self.get_logger().info("Table height: %s" % self.table_height)

    def plan_trajectory(self, request: Any, response: Any) -> Any:
        self.get_logger().info("Service called")

        # Log the request data
        self.get_logger().info("Received object start pose joint position: %s" % request.object_start_pose)
        self.get_logger().info("Received stab pose (where the tip of the fish meets the tray): %s" % request.stab_pose)
        self.get_logger().info("Received direction, left to right: %s" % request.roll_left_to_right)

        # Trigger the planner
        self.get_logger().info("Starting planner")
        
        #####################################################################
        # TODO: use the request data for calculation and return the trajectory response
        
        
        
        #######################################################################
        
        
        self.get_logger().warn("Implement your own calculations here!")

        # Example using 3rd party library scipy to convert the pose to a rotation matrix
        r = R.from_quat([request.object_start_pose.orientation.x, request.object_start_pose.orientation.y, request.object_start_pose.orientation.z, request.object_start_pose.orientation.w])
        rotation_matrix = r.as_matrix()
        self.get_logger().info("Example of using a package to test dependencies. Rotation matrix: %s" % rotation_matrix)

        response = GetObjectPlacementTrajectory.Response()
        response.joint_trajectory.points = [JointTrajectoryPoint(positions=[0, 0, 0, 0, 0, 0, 0], time_from_start=Duration(seconds=0.0).to_msg()),
                                            JointTrajectoryPoint(positions=[0.1, 0, 0, 0, 0, 0, 0], time_from_start=Duration(seconds=0.5).to_msg())]
        response.joint_trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        response.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
        response.joint_trajectory.header.frame_id = "world"
        self.get_logger().info("Planner finished")
        return response


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = ExampleService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
