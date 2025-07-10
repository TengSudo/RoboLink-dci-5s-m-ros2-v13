#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState, Constraints, JointConstraint
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
                
                # Gửi giá trị khớp đến MoveIt để lập kế hoạch
                self.plan_and_execute(joint_angles)
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

    def plan_and_execute(self, joint_angles):
        # Tạo Action Client để giao tiếp với MoveIt
        action_client = ActionClient(self, MoveGroup, '/move_action')

        # Đợi kết nối đến MoveIt Action Server
        if not action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Không thể kết nối đến MoveIt Action Server!")
            return

        # Tạo Goal Message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"

        # Tạo ràng buộc JointConstraint cho từng khớp
        constraints = Constraints()
        for i, joint_name in enumerate(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "virtual_joint"]):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_angles[i]
            joint_constraint.tolerance_above = 0.0
            joint_constraint.tolerance_below = 0.0
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        # Gán ràng buộc vào goal
        goal_msg.request.goal_constraints.append(constraints)

        # Gửi goal đến MoveIt
        self.get_logger().info("Gửi goal đến MoveIt để lập kế hoạch...")
        future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        # Kiểm tra kết quả
        if future.result() is not None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal bị từ chối bởi server")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            if result_future.result() is not None:
                result = result_future.result().result
                # Kiểm tra error code (cách này có thể khác tùy phiên bản MoveIt2)
                if hasattr(result, 'error_code'):
                    if result.error_code.val == 1:  # SUCCESS = 1
                        self.get_logger().info("Thực thi quỹ đạo thành công!")
                    else:
                        self.get_logger().error(f"Lỗi khi thực thi quỹ đạo: {result.error_code.val}")
                else:
                    self.get_logger().error("Kết quả không chứa error_code")
            else:
                self.get_logger().error("Không nhận được kết quả từ Action Server")
        else:
            self.get_logger().error("Không nhận được phản hồi từ MoveIt Action Server!")

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    while rclpy.ok() and not node.fake_controller_ready:
        rclpy.spin_once(node)

    # Định nghĩa giá trị đầu vào trực tiếp trong code
    try:
        x_mm = 250.0  # Giá trị x (mm)
        y_mm = 10.0   # Giá trị y (mm)
        z_mm = 150.0 # Giá trị z (mm)
        roll_deg = 180.0  # Giá trị roll (độ)
        pitch_deg = 3.0 # Giá trị pitch (độ)
        yaw_deg = 120.0  # Giá trị yaw (độ)

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