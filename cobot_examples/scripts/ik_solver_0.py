#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState
from std_msgs.msg import Float64MultiArray
import math

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_solver')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_angles', 10)
        self.display_pub = self.create_publisher(DisplayRobotState, '/display_robot_state', 10)
        self.fake_controller_ready = False
        self.create_timer(0.0, self.delayed_init)

    def delayed_init(self):
        self.check_controller_timer = self.create_timer(0.0, self.check_controller)

    def check_controller(self):
        try:
            node_list = self.get_node_names()
            if 'fake_arm_controller' in node_list:
                self.fake_controller_ready = True
                self.get_logger().info("Fake controller is ready")
                self.check_controller_timer.cancel()
            else:
                self.get_logger().warn("Waiting for fake controller...")
        except Exception as e:
            self.get_logger().error(f"Controller check failed: {str(e)}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def solve_ik(self, x, y, z, roll, pitch, yaw):
        if not self.fake_controller_ready:
            self.get_logger().error("Fake controller not ready!")
            return

        while not self.ik_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn('Waiting for IK service...')

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        q = self.euler_to_quaternion(roll, pitch, yaw)
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]

        robot_state = RobotState()
        robot_state.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "virtual_joint"]
        robot_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        ik_request = PositionIKRequest()
        ik_request.group_name = "arm"
        ik_request.robot_state = robot_state
        ik_request.pose_stamped = pose_stamped
        ik_request.timeout.sec = 0

        future = self.ik_client.call_async(GetPositionIK.Request(ik_request=ik_request))
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            if len(result.solution.joint_state.position) > 0:
                joint_angles = result.solution.joint_state.position
                self.get_logger().info(f"Solved Joint Angles: {joint_angles}")
                self.send_joint_angles(joint_angles)
            else:
                self.get_logger().error("Empty IK solution!")
        else:
            self.get_logger().error("IK service call failed!")

    def send_joint_angles(self, joint_angles):
        # Gửi đến fake controller
        joint_msg = Float64MultiArray()
        joint_msg.data = list(joint_angles)
        self.joint_pub.publish(joint_msg)

        # Hiển thị trong Rviz
        robot_state = RobotState()
        robot_state.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "virtual_joint"]
        robot_state.joint_state.position = joint_angles
        display_msg = DisplayRobotState()
        display_msg.state = robot_state
        self.display_pub.publish(display_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    while rclpy.ok() and not node.fake_controller_ready:
        rclpy.spin_once(node)

    # Định nghĩa giá trị đầu vào trực tiếp trong code
    try:
        x_mm = 250.0  # Giá trị x (mm)
        y_mm = 0.0   # Giá trị y (mm)
        z_mm = 150.0 # Giá trị z (mm)
        roll_deg = 180.0  # Giá trị roll (độ)
        pitch_deg = 0.0 # Giá trị pitch (độ)
        yaw_deg = 90.0  # Giá trị yaw (độ)

        # Chuyển đổi từ mm sang m và từ độ sang radian
        x = x_mm / 1000.0
        y = y_mm / 1000.0
        z = z_mm / 1000.0
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        # Gọi hàm solve_ik với giá trị đã chuyển đổi
        node.solve_ik(x, y, z, roll, pitch, yaw)
    except ValueError:
        node.get_logger().error("Giá trị nhập không hợp lệ. Vui lòng kiểm tra các giá trị đã định nghĩa.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()