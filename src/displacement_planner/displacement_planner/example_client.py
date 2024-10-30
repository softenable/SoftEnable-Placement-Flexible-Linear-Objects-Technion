# © Copyright 2024, Ocado Group PLC. All the rights reserved.
"""It is good practice to include an example client that can easily be used to test any services that have been created."""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from displacement_flexible_linear_objects_msgs.srv import GetObjectPlacementTrajectory
from geometry_msgs.msg import Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from typing import Any
import time

class ExampleClient(Node):  # type: ignore
    def __init__(self) -> None:
        super().__init__("example_client")
        self.client = self.create_client(GetObjectPlacementTrajectory, "plan_trajectory")
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

    def call_service(self) -> None:
        self.get_logger().info("Calling service")
        
        # deadlock due to threading, change in own thread
        # success = False
        # for _ in range(10):
        #     try:
        #         p_wrist = self.tf_buffer.lookup_transform("map", "finger", self.get_clock().now(), timeout=rclpy.duration.Duration(seconds=5))
        #         success = True
        #     except TransformException as ex:
        #         self.get_logger().warn(
        #             f'Could not transform msg.header.frame_id to world: {ex}')
        #     time.sleep(0.1)
        # if not success:
        #     print("TF ERROR")
        #     return
        
        # HACK TODO change this to tf 
        start_pose = [0.637, -0.085, 0.728]
        
        request = GetObjectPlacementTrajectory.Request()
        request.object_start_pose = Pose()
        request.object_start_pose.position.x = start_pose[0]
        request.object_start_pose.position.y = start_pose[1]
        request.object_start_pose.position.z = start_pose[2]
        
        # request.stab_pose = Pose()
        # request.stab_pose.position.x = 0.5
        # request.stab_pose.position.y = 0.5
        # request.stab_pose.position.z = 0.4
        
        request.roll_left_to_right.data = True
            
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


    example_client.call_service()
    rclpy.spin(example_client)
    example_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
