"""
Enhanced example client for testing the displacement planner service.

This client demonstrates how to call the displacement planning service
with the new message interface and provides comprehensive testing capabilities.
"""
import rclpy
from rclpy.node import Node
from displacement_flexible_linear_objects_msgs.srv import GetObjectPlacementTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from typing import Any
import numpy as np


class DisplacementPlannerClient(Node):  # type: ignore
    """Client for testing displacement planning service with new interface."""
    
    def __init__(self) -> None:
        super().__init__("displacement_planner_client")
        
        # Create service client
        self.client = self.create_client(
            GetObjectPlacementTrajectory, 
            "placement/plan_trajectory"
        )
        
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for displacement planner service...")
        
        self.get_logger().info("Connected to displacement planner service")

    def create_test_request(self, 
                           object_length: float = 0.25,
                           distance_from_tip: float = 0.05,
                           roll_direction: str = "left",
                           start_pos: list = [0.4, 0.2, 0.3],
                           target_pos: list = [0.5, 0.2, 0.1]) -> GetObjectPlacementTrajectory.Request:
        """Create a test service request with the new message format."""
        request = GetObjectPlacementTrajectory.Request()
        
        # Object properties
        request.target_length = object_length
        request.distance_from_tip = distance_from_tip
        request.roll_direction = roll_direction  # String: 'left' or 'right'
        
        # Object start pose
        request.start_pose = Pose()
        request.start_pose.position = Point(x=start_pos[0], y=start_pos[1], z=start_pos[2])
        request.start_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Target pose 
        request.target_pose = Pose()
        request.target_pose.position = Point(x=target_pos[0], y=target_pos[1], z=target_pos[2])
        request.target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Robot start configuration
        request.start_state = JointState()
        request.start_state.name = [
            "joint1", "joint2", "joint3", "joint4", 
            "joint5", "joint6", "joint7"
        ]
        # Use a reasonable starting configuration
        request.start_state.position = [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0]
        
        return request

    def call_planning_service(self, request: GetObjectPlacementTrajectory.Request) -> None:
        """Call the planning service asynchronously."""
        self.get_logger().info("Sending displacement planning request...")
        self.get_logger().info(f"  Object length: {request.target_length:.3f}m")
        self.get_logger().info(f"  Distance from tip: {request.distance_from_tip:.3f}m") 
        self.get_logger().info(f"  Roll direction: {request.roll_direction}")
        self.get_logger().info(f"  Start position: [{request.start_pose.position.x:.2f}, "
                             f"{request.start_pose.position.y:.2f}, "
                             f"{request.start_pose.position.z:.2f}]")
        self.get_logger().info(f"  Target position: [{request.target_pose.position.x:.2f}, "
                             f"{request.target_pose.position.y:.2f}, "
                             f"{request.target_pose.position.z:.2f}]")
        
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.planning_callback)

    def planning_callback(self, future: Any) -> None:
        """Handle planning service response."""
        try:
            response = future.result()
            
            # Check joint trajectory
            joint_traj = response.joint_trajectory
            cartesian_traj = response.cartesian_trajectory
            
            if joint_traj.points:
                self.get_logger().info("Displacement planning successful!")
                self.get_logger().info(f"   Joint trajectory:")
                self.get_logger().info(f"     - Joint names: {joint_traj.joint_names}")
                self.get_logger().info(f"     - Number of points: {len(joint_traj.points)}")
                if joint_traj.points:
                    duration = joint_traj.points[-1].time_from_start.sec + \
                              joint_traj.points[-1].time_from_start.nanosec * 1e-9
                    self.get_logger().info(f"     - Duration: {duration:.2f}s")
                
                # Check cartesian trajectory
                if cartesian_traj.points:
                    self.get_logger().info(f"   Cartesian trajectory:")
                    self.get_logger().info(f"     - Joint names: {cartesian_traj.joint_names}")
                    self.get_logger().info(f"     - Number of points: {len(cartesian_traj.points)}")
                
                # Log sample configurations
                if len(joint_traj.points) > 0:
                    first_config = joint_traj.points[0].positions
                    last_config = joint_traj.points[-1].positions
                    self.get_logger().info(f"   Sample configurations:")
                    self.get_logger().info(f"     - Start: {[f'{q:.3f}' for q in first_config[:3]]}...")
                    self.get_logger().info(f"     - End:   {[f'{q:.3f}' for q in last_config[:3]]}...")
                    
            else:
                self.get_logger().error("Displacement planning failed - empty trajectory returned")
                
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def test_multiple_scenarios(self) -> None:
        """Test multiple displacement scenarios."""
        test_cases = [
            {
                "name": "Short object, left roll",
                "params": {
                    "object_length": 0.20,
                    "roll_direction": "left",
                    "start_pos": [0.3, 0.1, 0.4],
                    "target_pos": [0.4, 0.1, 0.05]
                }
            },
            {
                "name": "Medium object, right roll", 
                "params": {
                    "object_length": 0.30,
                    "roll_direction": "right",
                    "start_pos": [0.5, -0.1, 0.3],
                    "target_pos": [0.6, -0.1, 0.05]
                }
            },
            {
                "name": "Long object, left roll",
                "params": {
                    "object_length": 0.40,
                    "roll_direction": "left", 
                    "start_pos": [0.4, 0.2, 0.5],
                    "target_pos": [0.3, 0.2, 0.05]
                }
            }
        ]
        
        for i, test_case in enumerate(test_cases):
            self.get_logger().info(f"\n📋 Test {i+1}: {test_case['name']}")
            request = self.create_test_request(**test_case['params'])
            self.call_planning_service(request)
            
            # Wait for response (simplified for demo)
            rclpy.spin_once(self, timeout_sec=10.0)


def main(args: Any = None) -> None:
    """Main function to test the displacement planner."""
    rclpy.init(args=args)
    
    try:
        # Create client
        client = DisplacementPlannerClient()
        
        # Run basic test
        print("=== Displacement Planner Client Test ===")
        request = client.create_test_request()
        client.call_planning_service(request)
        
        # Spin until response received
        rclpy.spin_once(client, timeout_sec=30.0)
        
        # Optionally run multiple test scenarios
        # client.test_multiple_scenarios()
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        if 'client' in locals():
            client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
