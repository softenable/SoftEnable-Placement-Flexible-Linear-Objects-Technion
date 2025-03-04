from robot_controller_interfaces.srv import CancelGoal, Init, Status

from robot_controller_interfaces.action import (
    MoveJointTrajectory,
    MoveSingleArm,
    MoveToJointPosition,
)
from geometry_msgs.msg import Point, Pose, Quaternion

from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

import matplotlib.pyplot as plt
from rclpy.time import Duration

import rclpy
from rclpy.node import Node
import pinocchio as pin
import time
from displacement_planner import utils as hf
from displacement_planner import node_class
from displacement_planner import displacement as dis
from robot_tools.devices.robots import IIWA, IIWAParameters

import numpy as np

from visualization_msgs.msg import Marker


from robot_tools.kinematics.kinematics_solver_parameters import (
    KinematicsSolverParameters,
)
from robot_tools.kinematics.basic_serial_manipulator_kinematics import (
    BasicSerialManipulatorKinematics,
)

# Initializes the robot by imitating the following call:
#  ros2 service call /robot/init robot_controller_interfaces/srv/Init "{controller: 'cartesian_impedance'}"
def init_robot():
    rclpy.init()
    node = Node("init_robot")
    client = node.create_client(Init, "/robot/init")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("service not available, waiting again...")
    request = Init.Request()
    request.controller = "joint_position"
    request.policy = "rollout_joint_trajectory"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info("Result: %r" % future.result())
    else:
        node.get_logger().info("Service call failed %r" % future.exception())
    node.destroy_node()
    rclpy.shutdown()

def init_robot_cartesian():
    rclpy.init()
    node = Node("init_robot")
    client = node.create_client(Init, "/robot/init")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("service not available, waiting again...")
    request = Init.Request()
    request.controller = "joint_impedance"
    request.policy = "reach_pose"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info("Result: %r" % future.result())
    else:
        node.get_logger().info("Service call failed %r" % future.exception())
    node.destroy_node()
    rclpy.shutdown()

def move_robot_to_init_Pose_action():
    rclpy.init()
    node = Node("move_robot_to_init_pose_action")

    client = ActionClient(node, MoveSingleArm, "/robot/move_to")
    while not client.wait_for_server(timeout_sec=1.0):
        node.get_logger().info("service not available, waiting again...")
    request = MoveSingleArm.Goal()
    request.pose = Pose()
    request.pose.position.x = 0.4
    request.pose.position.y = 0.2
    request.pose.position.z = 0.0 + 0.3 * 0.3
    request.pose.orientation = Quaternion()
    request.pose.orientation.x = 1.0
    request.pose.orientation.y = 0.0
    request.pose.orientation.z = 0.0
    request.pose.orientation.w = 0.0
    future = client.send_goal_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info("Result: %r" % future.result())
    else:
        node.get_logger().info("Service call failed %r" % future.exception())
    node.destroy_node()
    rclpy.shutdown()
    

    

def calculate_path_technion():
    #####################################################################
    # TODO: use the request data for calculation and return the trajectory response

    # define LDO parameters, environment parameters and direction displacement
    direction = "right" 
    urdf_path = "/usr/share/iiwa-description/urdf/iiwa14_with_ft_and_dlr_gripper.urdf"
    tip_link = "dlr_gripper_center"

    
    robot_parameters = IIWAParameters(urdf_path=urdf_path, tip_link=tip_link, config_path="/ws/src/iiwa_default.yaml", base_frame="world", reference_frame = pin.ReferenceFrame.LOCAL)
    robot = IIWA(parameters=robot_parameters)

    model = robot.model
    
    # model = pin.buildModelFromUrdf(urdf_path)
    q_start = robot.get_joint_positions() # q_start should be input parameter
    
    # print([frame.name for frame in model.frames if frame.type == pin.FrameType.JOINT])
    # model = pin.buildModelFromUrdf(urdf_path)
    # q_start = pin.neutral(model) # q_start should be input parameter
    # 0.0 0.7 0.0 -0.7 0.0 1.21 0.0.
    # q_start = np.array([0.0, 0.9, 0.0, -0.7, 0.0, 1.21, 0.0])
    # q_start = np.array([0.37461716, 1.11586231,  0.22107805, -2.10410042, -2.636115, 1.2118165, -0.94555805])
    print([frame.name for frame in model.frames if frame.type == pin.FrameType.JOINT])
    # exit()
    L = 0.3
    mu_1 = 0.5 # the friction coefficient between the phantom and the tray
    mu_2 = 0.1 # the friction coefficient between the tray and the table
    # mu_fric = np.min([mu_1, mu_2])
    mu_fric = 0.1
    h_table = 0.1
    alpha = 0.6
    if direction == "right":
        C = 1/4
        phi_0_T2 = 0
    elif direction == "left":
        C = 3/4
        phi_0_T2 = np.pi

    # define start and target for sub-problem 1
    X0_S1 = 0.4
    Y0_S1 = 0.2
    Z0_S1 = h_table + 0.3*L
    theta_S1 = np.pi/2
    X0_T1 = 0.5
    Y0_T1 = 0.2
    Z0_T1 = h_table
    # start_pose = [0.4, 0.2, h_table]
    # # start_pose = [0.3374537663565831, 3.343451349141077e-12, h_table]
    # # define start and target for sub-problem 1
    # X0_S1 = start_pose[0]
    # Y0_S1 = start_pose[1]
    # Z0_S1 = start_pose[2]
    # theta_S1 = np.pi/2
    
    # X0_T1 = start_pose[0]
    # Y0_T1 = start_pose[1]
    # Z0_T1 = h_table
    theta_T1 = 0
    L_gal_S = 2*L
    target_1 = node_class.Target(X0_T1, Y0_T1, Z0_T1, np.pi/2, theta_T1, 1, C*L_gal_S, L_gal_S, 0, L, direction)
    start_1 = node_class.Node(target_1, X0_S1, Y0_S1, Z0_S1, np.pi/2, theta_S1, 1, C*L_gal_S, L_gal_S, 0, [], alpha, L, direction)

    # define target for sub-problem 2
    mu_T2 = 0
    L_gal_T2 = 2*L
    target_2 = node_class.Target(target_1.X0, target_1.Y0, target_1.Z0, phi_0_T2, target_1.theta, mu_T2, C*L_gal_T2, L_gal_T2, 0, L, direction)

    # define target for sub-problem 3
    mu_T3 = 0
    l_T3 = 0.5*L
    L_gal_T3 = 4*(L - l_T3)
    target_3 = node_class.Target(target_2.X0, target_2.Y0, target_2.Z0, phi_0_T2, target_2.theta, mu_T3, C*L_gal_T3, L_gal_T3, l_T3, L, direction)

    # get full path
    t_start = time.time()
    path = dis.displacement(start_1, target_1, target_2, target_3, L, mu_fric, direction, h_table, alpha, urdf_path, q_start, model)
    path_1, path_2, path_3 = path

    # check displacement
    skip = 8
    num_t = 8
    t = np.linspace(0, 1, num_t)
    s = np.linspace(0, 1, num_t)

    x1, y1, z1, phi_l_1, theta_1, l1, q1 = hf.get_smooth_path_displacement(path_1, skip, t, L, direction, urdf_path, path_1[0].jointsPose, model)
    x2, y2, z2, phi_l_2, theta_2, l2, q2 = hf.get_smooth_path_displacement(path_2, skip, t, L, direction, urdf_path, path_2[0].jointsPose, model)
    x3, y3, z3, phi_l_3, theta_3, l3, q3 = hf.get_smooth_path_displacement(path_3, skip, t, L, direction, urdf_path, path_3[0].jointsPose, model)

    t_end = time.time()
    ExecutionTime1_2_3 = t_end - t_start
    print("Execution Time =", ExecutionTime1_2_3)

    P1, R1 = hf.get_gripper_path(x1[:, -1], y1[:, -1], z1[:, -1], phi_l_1, theta_1)
    P2, R2 = hf.get_gripper_path(x2[:, -1], y2[:, -1], z2[:, -1], phi_l_2, theta_2)
    P3, R3 = hf.get_gripper_path(x3[:, -1], y3[:, -1], z3[:, -1], phi_l_3, theta_3)

    # make x,y z to np.array
    x = np.array(x1.tolist() + x2.tolist() + x3.tolist())
    y = np.array(y1.tolist() + y2.tolist() + y3.tolist())
    z = np.array(z1.tolist() + z2.tolist() + z3.tolist())

    # check by graphs
    path2 = path_1 + path_2 + path_3
    gripper_P = np.array(P1.tolist() + P2.tolist() + P3.tolist())
    gripper_R = R1 + R2 + R3
    robot_joint_pose = q1 + q2 + q3
    if direction == "right":
        a = 1
    elif direction == "left":
        a = -1
        phi_0_T2 = np.pi

    for n in path2:
        plt.figure('1')
        [xs, zs, phi_s] = hf.get_shape(n.X0 + a*n.l, n.Z0, n.phi_0, n.mu, n.L_gal, n.S0, np.linspace(0, L-n.l))
        plt.plot(xs, zs)
        plt.grid()
    plt.show()
    # Writes the plot to a file
    plt.savefig('/ws/src/displacement.png')
    plt.close()
    # t_end = time.time()
    # ExecutionTime1_2_3 = t_end - t_start
    # print("Execution Time =", ExecutionTime1_2_3)
    
    robot_joint_pose = q1 + q2 + q3
    return robot_joint_pose, path, gripper_P

# Calls the MoveJointTrajectory action server with the calculated path
def send_move_trajectory_action(q):
    rclpy.init()
    node = Node("send_move_trajectory_action")
    client = ActionClient(node, MoveJointTrajectory, "/robot/follow_joint_trajectory")
    while not client.wait_for_server(timeout_sec=1.0):
        node.get_logger().info("service not available, waiting again...")
    request = MoveJointTrajectory.Goal()
    request.joint_trajectory = JointTrajectory()
    print("SOMETHING")
    print(q[0])

    request.joint_trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    request.joint_trajectory.header.stamp = node.get_clock().now().to_msg()
    request.joint_trajectory.header.frame_id = "world"

    request.joint_trajectory.points = [
        JointTrajectoryPoint(positions=node[:7], time_from_start=Duration(seconds=0).to_msg())
        for i, node in enumerate(q)
    ]
    print(request.joint_trajectory.points[0])
    future = client.send_goal_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info("Result: %r" % future.result())
    else:
        node.get_logger().info("Service call failed %r" % future.exception())
    node.destroy_node()
    rclpy.shutdown()

    # response.joint_trajectory.points = [
    #     JointTrajectoryPoint(positions=node.jointsPose, time_from_start=Duration(seconds=i * 0.1).to_msg())
    #     for i, node in enumerate(path[0])
    # ]
    # response.joint_trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    # response.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
    # response.joint_trajectory.header.frame_id = "world"
    # self.get_logger().info("The trajectory is: %s" % response.joint_trajectory)

# def publish_trajectory_marker(self, path):
#         # Create a line strip marker
#         marker = Marker()
#         marker.header.frame_id = "world"  # Must match your RViz fixed frame or TF
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "trajectory_trace"
#         marker.id = 0
#         # Use SPHERE_LIST to show spheres at each point
#         marker.type = Marker.SPHERE_LIST
#         marker.action = Marker.ADD
#         # Set sphere size (x, y, z dimensions all represent the diameter)
#         marker.scale.x = 0.025
#         marker.scale.y = 0.025
#         marker.scale.z = 0.025
#         # No expiration
#         marker.lifetime.sec = 0
#         marker.lifetime.nanosec = 0

#         # Set color (blue in this example)
#         marker.color.a = 1.0
#         marker.color.r = 1.0
#         marker.color.g = 0.0
#         marker.color.b = 0.0

#         for point in path[0]:
#             pt = Point()
#             pt.x = point.X0  # if len(point.positions) > 0 else 0.0
#             pt.y = point.Y0  # if len(point.positions) > 1 else 0.0
#             pt.z = point.Z0  # if len(point.positions) > 2 else 0.0
#             marker.points.append(pt)

#         for point in path[1]:
#             pt = Point()
#             pt.x = point.X0  # if len(point.positions) > 0 else 0.0
#             pt.y = point.Y0  # if len(point.positions) > 1 else 0.0
#             pt.z = point.Z0  # if len(point.positions) > 2 else 0.0
#             marker.points.append(pt)

#         for point in path[2]:
#             pt = Point()
#             pt.x = point.X0  # if len(point.positions) > 0 else 0.0
#             pt.y = point.Y0  # if len(point.positions) > 1 else 0.0
#             pt.z = point.Z0  # if len(point.positions) > 2 else 0.0
#             marker.points.append(pt)

#         self.marker_pub.publish(marker)
#         self.get_logger().info("Published trajectory marker with {} points".format(len(marker.points)))

    # Publishes the path as a marker to the visualization/trajectory_marker topic
def publish_trajectory_marker(path):
    rclpy.init()
    node = Node("publish_trajectory_marker")
    publisher = node.create_publisher(Marker, "visualization/trajectory_marker", 10)
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = "trajectory_trace"
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.025
    marker.scale.y = 0.025
    marker.scale.z = 0.025
    marker.lifetime.sec = 0
    marker.lifetime.nanosec = 0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    for point in path[0]:
        pt = Point()
        pt.x = point.X0
        pt.y = point.Y0
        pt.z = point.Z0
        marker.points.append(pt)

    for point in path[1]:
        pt = Point()
        pt.x = point.X0
        pt.y = point.Y0
        pt.z = point.Z0
        marker.points.append(pt)

    for point in path[2]:
        pt = Point()
        pt.x = point.X0
        pt.y = point.Y0
        pt.z = point.Z0
        marker.points.append(pt)

    publisher.publish(marker)
    node.get_logger().info("Published trajectory marker with {} points".format(len(marker.points)))
    node.destroy_node()
    rclpy.shutdown()
def publish_trajectory_marker2(path, gripper_P):
    rclpy.init()
    node = Node("publish_trajectory_marker")
    publisher = node.create_publisher(Marker, "visualization/trajectory_marker", 10)
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = "trajectory_trace"
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.025
    marker.scale.y = 0.025
    marker.scale.z = 0.025
    marker.lifetime.sec = 0
    marker.lifetime.nanosec = 0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    # A = list(gripper_P[0])
    # B = list(gripper_P[1])
    # C = list(gripper_P[2])
    for point in gripper_P:
        # print(A)
        # print(A[i])
        pt = Point()
        pt.x = point[0]
        pt.y = point[1]
        pt.z = point[2]
        marker.points.append(pt)

    # for point in path[1]:
    #     pt = Point()
    #     pt.x = point.X0
    #     pt.y = point.Y0
    #     pt.z = point.Z0
    #     marker.points.append(pt)

    # for point in path[2]:
    #     pt = Point()
    #     pt.x = point.X0
    #     pt.y = point.Y0
    #     pt.z = point.Z0
    #     marker.points.append(pt)

    publisher.publish(marker)
    node.get_logger().info("Published trajectory marker with {} points".format(len(marker.points)))
    node.destroy_node()
    rclpy.shutdown()



def main(args=None) -> None:  # type: ignore
    # Initializes the robot
    # init_robot_cartesian()
    # move_robot_to_init_Pose_action()
    # time.sleep(5)
    init_robot()
    time.sleep(5)

    q, path, gripper_P = calculate_path_technion()
    publish_trajectory_marker2(path, gripper_P)
    # publish_trajectory_marker(path, gripper_P)
    send_move_trajectory_action(q)


if __name__ == "__main__":
    main()
