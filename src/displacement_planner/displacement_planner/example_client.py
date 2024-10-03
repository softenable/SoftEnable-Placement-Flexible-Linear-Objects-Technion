# © Copyright 2024, Ocado Group PLC. All the rights reserved.
"""It is good practice to include an example client that can easily be used to test any services that have been created."""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from displacement_flexible_linear_objects_msgs.srv import GetObjectPlacementTrajectory
from geometry_msgs.msg import Pose
from typing import Any


class ExampleClient(Node):  # type: ignore
    def __init__(self) -> None:
        super().__init__("example_client")
        self.client = self.create_client(GetObjectPlacementTrajectory, "plan_trajectory")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

    def call_service(self, request: Any) -> None:
        self.get_logger().info("Calling service")
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.callback)

    def callback(self, future: Any) -> None:
        try:
            response = future.result()
            self.get_logger().info("Trajectory planned successfully")
            # TODO: Process the trajectory response here
            self.get_logger().info("Received joint names: %s" % response.joint_trajectory.joint_names)
            self.get_logger().info("Received trajectory points: %s" % response.joint_trajectory.points)
        except Exception as e:
            self.get_logger().error("Failed to plan trajectory: %s" % str(e))


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    example_client = ExampleClient()
    request = GetObjectPlacementTrajectory.Request()
    request.object_start_pose = Pose()
    request.object_start_pose.position.x = 0.4
    request.object_start_pose.position.y = 0.4
    request.object_start_pose.position.z = 0.4
    request.stab_pose = Pose()
    request.stab_pose.position.x = 0.5
    request.stab_pose.position.y = 0.5
    request.stab_pose.position.z = 0.4
    request.roll_left_to_right.data = True

    example_client.call_service(request)
    rclpy.spin(example_client)
    example_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
