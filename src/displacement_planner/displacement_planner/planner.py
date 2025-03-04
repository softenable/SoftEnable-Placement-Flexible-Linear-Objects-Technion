# © Copyright 2024, Ocado Group PLC. All the rights reserved.
import sys
import os
import rclpy
from rclpy.node import Node
from displacement_flexible_linear_objects_msgs.srv import GetObjectPlacementTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.time import Duration
from typing import Any
from scipy.spatial.transform import Rotation as R
import pinocchio as pin
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from robot_tools.devices.robots import IIWA, IIWAParameters


from robot_tools.devices.robots import IIWA, IIWAParameters
from robot_tools.kinematics.basic_serial_manipulator_kinematics import (
    BasicSerialManipulatorKinematics,
)

from robot_tools.kinematics.kinematics_solver_parameters import (
    KinematicsSolverParameters,
)
from robot_tools.kinematics.basic_serial_manipulator_kinematics import (
    BasicSerialManipulatorKinematics,
)

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from displacement_planner.displacement import displacement as dis
from displacement_planner.displacement import node_class
from displacement_planner.displacement import utils as hf

class DisplacementPlanner(Node):  # type: ignore
    def __init__(self) -> None:
        super().__init__("displacement_planner")
        self.service = self.create_service(GetObjectPlacementTrajectory, "placement/plan_trajectory", self.plan_trajectory)

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
                    "friction_coefficients",
                    [0.5, 0.1],
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE_ARRAY, description="The coefficient of friction between the table and the tray followed by the coefficient of friction between the tray and the object."
                    ),
                ),
                (
                    "table_height",
                    0.0,
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

        # self.model = pin.buildModelFromUrdf(self.urdf_path)

        tip_link = "dlr_gripper_center"
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        

        # self.robot_parameters = IIWAParameters(urdf_path=self.urdf_path, tip_link=tip_link, config_path="/ws/src/iiwa_default.yaml")
        # self.robot = IIWA(parameters=self.robot_parameters)

        # self.model = self.robot.model
        
        self.marker_pub = self.create_publisher(Marker, 'visualization/trajectory_marker', 10)

        self.get_logger().info("FINISHED STARTING SERVICE")
        
    def plan_trajectory(self, request: Any, response: Any) -> Any:
        self.get_logger().info("Service called")

        # object_length = request.target_length
        object_length = 0.3
        # distance_from_tip = request.distance_from_tip
        # direction = request.roll_direction
        direction = "left"
        # q_start = request.start_state.position
        q_start = np.array(list(request.start_state.position))
        # q_start = pin.neutral(self.model)
        # print(q_start, type(q_start))
        # print(pin.neutral(self.model), type(pin.neutral(self.model)))
        # Log the request data
        self.get_logger().info("Received object start pose: %s" % request.start_pose)
        self.get_logger().info("Received object placement pose: %s" % request.target_pose)
        self.get_logger().info("Received direction: %s" % direction)
        self.get_logger().info("Received direction, object length: %s" % object_length)

        # Trigger the planner
        self.get_logger().info("Starting planner")
        direction = "left" 
        object_length = 0.3
        mu_1 = 0.5 # the friction coefficient between the phantom and the tray
        mu_2 = 0.1 # the friction coefficient between the tray and the table
        mu_fric = np.min([mu_1, mu_2])
        h_table = 0.0
        alpha = 0.6
        if direction == "right":
            C = 1/4
            phi_0_T2 = 0
        elif direction == "left":
            C = 3/4
            phi_0_T2 = np.pi

        # define start and target for sub-problem 1
        # X0_S1 = request.start_pose.position.x
        # Y0_S1 = request.start_pose.position.y
        # Z0_S1 = h_table + 0.3*object_length
        # theta_S1 = np.pi/2
        # X0_T1 = request.target_pose.position.x
        # Y0_T1 = request.target_pose.position.y
        # Z0_T1 = h_table
        # start_pose = [0.4, 0.2, h_table]
        # X0_S1 = start_pose[0]
        # Y0_S1 = start_pose[1]
        # Z0_S1 = start_pose[2]
        # theta_S1 = np.pi/2
        
        # X0_T1 = start_pose[0]
        # Y0_T1 = start_pose[1]
        # Z0_T1 = h_table
        # X0_S1 = 0.4
        # Y0_S1 = 0.2
        # Z0_S1 = h_table + 0.3*object_length
        # theta_S1 = np.pi/2
        # X0_T1 = 0.5
        # Y0_T1 = 0.2
        # Z0_T1 = h_table

        # h_table = 0.1
        alpha = 0.6
        if direction == "right":
            C = 1/4
            phi_0_T2 = 0
        elif direction == "left":
            C = 3/4
            phi_0_T2 = np.pi

        urdf_file_path = "/usr/share/iiwa-description/urdf/iiwa14_with_ft_and_dlr_gripper.urdf"
        tip_link = "dlr_gripper_center"

        # robot_parameters = IIWAParameters(urdf_path=urdf_path, tip_link=tip_link, config_path="/ws/src/iiwa_default.yaml", base_frame="world", reference_frame = pin.ReferenceFrame.LOCAL)
        # robot = IIWA(parameters=robot_parameters)


        # model: pin.Model = pin.buildModelFromUrdf(urdf_file_path)
        kinematic_parameters: KinematicsSolverParameters = (
            KinematicsSolverParameters(tip_link=tip_link)
        )
        kinematic_parameters.reference_frame = pin.ReferenceFrame.WORLD
        kinematic_parameters.base_frame = "world"

        kin = BasicSerialManipulatorKinematics(
            urdf_file_path, kinematic_parameters
        )
        kin.obtain_reduced_model(tip_link=tip_link)

        self.model = kin.model

        # data: pin.Data = self.model.createData()
        pose_actual: pin.SE3 =kin.forward_kinematics(q_start)

        start_pose = pose_actual.translation
        # start_pose = [0.4, 0.2, h_table]
        # # start_pose = [0.3374537663565831, 3.343451349141077e-12, h_table]
        # # define start and target for sub-problem 1
        direction = "right" 
        L = 0.3 
        mu_1 = 0.5 # the friction coefficient between the phantom and the tray
        mu_2 = 0.1 # the friction coefficient between the tray and the table
        mu_fric = np.min([mu_1, mu_2])
        h_table = 0.0
        alpha = 0.6
        if direction == "right":
            C = 1/4
            phi_0_T2 = 0
        elif direction == "left":
            C = 3/4
            phi_0_T2 = np.pi

        # define start and target for sub-problem 1
        X0_S1 = 0.4
        Y0_S1 = -0.2
        Z0_S1 = h_table + 0.3*L
        theta_S1 = np.pi/2
        X0_T1 = 0.5
        Y0_T1 = -0.2
        Z0_T1 = h_table
        theta_T1 = 0
        L_gal_S = 2*L
        object_length = L
        target_1 = node_class.Target(X0_T1, Y0_T1, Z0_T1, np.pi/2, theta_T1, 1, C*L_gal_S, L_gal_S, 0, object_length, direction)
        start_1 = node_class.Node(target_1, X0_S1, Y0_S1, Z0_S1, np.pi/2, theta_S1, 1, C*L_gal_S, L_gal_S, 0, [], alpha, object_length, direction)

        # define target for sub-problem 2
        mu_T2 = 0
        L_gal_T2 = 2*object_length
        target_2 = node_class.Target(target_1.X0, target_1.Y0, target_1.Z0, phi_0_T2, target_1.theta, mu_T2, C*L_gal_T2, L_gal_T2, 0, object_length, direction)

        # define target for sub-problem 3
        mu_T3 = 0
        l_T3 = 0.5*object_length
        L_gal_T3 = 4*(object_length - l_T3)
        target_3 = node_class.Target(target_2.X0, target_2.Y0, target_2.Z0, phi_0_T2, target_2.theta,
                                     mu_T3, C*L_gal_T3, L_gal_T3, l_T3, object_length, direction)
        

        self.get_logger().info("Starting displacement, with urdf: %s" % self.urdf_path)
        path = dis.displacement(start_1, target_1, target_2, target_3, object_length, mu_fric, direction, h_table, alpha, self.urdf_path, q_start, self.model)
        self.get_logger().info("Displacement path: %s" % path[0])

        # Converts the path[0] array full of Node objects to trajectory_msgs/JointTrajectory message without any extra comments
        print("HIHI")
        skip = 8
        num_t = 8
        t = np.linspace(0, 1, num_t)
        s = np.linspace(0, 1, num_t)


        # print(path[0][0])
        print(path[0][0].jointsPose)
        # path_1 = path[0]
        # path_1, path_2 = path
        # path_1, path_2, path_3 = path
        path_z = path[0] + path[1] + path[2]
        x1, y1, z1, phi_l_1, theta_1, l1, q1 = hf.get_smooth_path_displacement(path_z, skip, t, object_length, direction, self.urdf_path, path_z[0].jointsPose, self.model)
        # x1, y1, z1, phi_l_1, theta_1, l1, q1 = hf.get_smooth_path_displacement(path_1, skip, t, object_length, direction, self.urdf_path, path_1[0].jointsPose, self.model)
        # x2, y2, z2, phi_l_2, theta_2, l2, q2 = hf.get_smooth_path_displacement(path_2, skip, t, object_length, direction, self.urdf_path, path_2[0].jointsPose, self.model)
        # x3, y3, z3, phi_l_3, theta_3, l3, q3 = hf.get_smooth_path_displacement(path_3, skip, t, object_length, direction, self.urdf_path, path_3[0].jointsPose, self.model)

        
        P1, R1 = hf.get_gripper_path(x1[:, -1], y1[:, -1], z1[:, -1], phi_l_1, theta_1)
        # P2, R2 = hf.get_gripper_path(x2[:, -1], y2[:, -1], z2[:, -1], phi_l_2, theta_2)
        # P3, R3 = hf.get_gripper_path(x3[:, -1], y3[:, -1], z3[:, -1], phi_l_3, theta_3)

        # gripper_P = [P1.tolist(), P2.tolist()]#, P3.tolist()]
        gripper_P = [P1.tolist()]#, P3.tolist()]
        # gripper_P = [P1.tolist(), P2.tolist(), P3.tolist()]
        robot_joint_pose = q1
        # robot_joint_pose = q2 + q3
        # robot_joint_pose = q1 + q2 + q3
        self.publish_trajectory_marker(gripper_P)

        response.joint_trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        response.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
        response.joint_trajectory.header.frame_id = "world"

        response.joint_trajectory.points = [
            JointTrajectoryPoint(positions=node, time_from_start=Duration(seconds=0).to_msg())
            for i, node in enumerate(robot_joint_pose)
        ]
        # self.get_logger().info("The trajectory is: %s" % response.joint_trajectory)

        self.get_logger().info("Planner finished")
        return response

    def publish_trajectory_marker(self, path):
        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0)   # Blue
        ]

        for idx, target_path in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectory_trace"
            marker.id = idx
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD

            marker.scale.x = 0.025
            marker.scale.y = 0.025
            marker.scale.z = 0.025

            (r, g, b) = colors[idx % len(colors)]
            marker.color.a = 1.0
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            for point in target_path:
                # print(point)
                pt = Point()
                pt.x = point[0]
                pt.y = point[1]
                pt.z = point[2]
                marker.points.append(pt)

            self.marker_pub.publish(marker)
            self.get_logger().info(f"Published path {idx} with {len(marker.points)} points.")

def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DisplacementPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
