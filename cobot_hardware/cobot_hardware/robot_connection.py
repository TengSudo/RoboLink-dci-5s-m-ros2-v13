import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from cri_lib import CRIController
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class RobotConnection(Node):
    def __init__(self):
        super().__init__('robot_connection')
        
        self.get_logger().info("Subscriber for /gui_joint_states created")

        # Tạo publisher cho từng loại trạng thái
        self.mode_publisher = self.create_publisher(String, 'robot/mode', 10)                               #RobotMode
        self.kinematics_state_publisher = self.create_publisher(String, 'robot/kinematics_state', 10)       #KinematicsState
        self.overrides_publisher = self.create_publisher(String, 'robot/overrides', 10)                     #Overrides
        self.dout_publisher = self.create_publisher(String, 'robot/dout', 10)                               #DigitalOut
        self.referencing_state_publisher = self.create_publisher(String, 'robot/referencing_state', 10)     #ReferencingState 
        self.error_states_publisher = self.create_publisher(String, 'robot/error_states', 10)               #ErrorStates
        self.position_robot_publisher = self.create_publisher(String, 'robot/position_robot', 10)           #PositionRobot
        self.cart_speed_mm_per_s_publisher = self.create_publisher(String, 'robot/cart_speed', 10)          #CartSpeedMMPerS
        self.actual_joint_pub = self.create_publisher(JointState, 'robot/actual_joint_states', 10)          #JointState

        # Kết nối với robot qua CRIController
        # self.connect_to_robot()
        self.receive_data_from_robot()

    def connect_to_robot(self):
        """Kết nối với robot qua CRI Protocol."""
        host = '192.168.3.11'  # IP của robot
        port = 3920  # Cổng robot đang lắng nghe

        # Khởi tạo CRIController và kết nối đến robot
        self.controller = CRIController()
        if self.controller.connect(host, port):  # Kết nối với robot
            self.get_logger().info(f"Connected to robot at {host}:{port}")
            # Bắt đầu nhận dữ liệu từ robot
            self.receive_data_from_robot()
        else:
            self.get_logger().error(f"Failed to connect to robot at {host}:{port}")
            
    def receive_data_from_robot(self):
        """Nhận dữ liệu trạng thái từ robot và phát lên topic."""
        while True:
            try:
                # Nhận và cập nhật trạng thái robot từ CRIController
                robot_state = self.controller.robot_state  # Lấy toàn bộ trạng thái robot
                
                if robot_state:

                    # Tạo và phát trạng thái chế độ hoạt động
                    mode_msg = self.create_mode_msg(robot_state)
                    self.mode_publisher.publish(mode_msg)

                    # Tạo và phát trạng thái KinematicsState
                    kinematics_state_msg = self.create_kinematics_state_msg(robot_state)
                    self.kinematics_state_publisher.publish(kinematics_state_msg)

                    # Publish giá trị thực từ robot
                    actual_msg = JointState()
                    actual_msg.header.stamp = self.get_clock().now().to_msg()
                    actual_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
                    actual_msg.position = [
                        math.radians(robot_state.joints_current.A1),
                        math.radians(robot_state.joints_current.A2),
                        math.radians(robot_state.joints_current.A3),
                        math.radians(robot_state.joints_current.A4),
                        math.radians(robot_state.joints_current.A5),
                    ]
                    self.actual_joint_pub.publish(actual_msg)

                    # Tạo và phát trạng thái Overrides
                    overrides_msg = self.create_overrides_msg(robot_state)
                    self.overrides_publisher.publish(overrides_msg)

                    # Tạo và phát trạng thái DigitalOut
                    dout_msg = self.create_dout_msg(robot_state)
                    self.dout_publisher.publish(dout_msg)

                    # Tạo và phát trạng thái ReferencingState
                    referencing_state_msg = self.create_referencing_state_msg(robot_state)
                    self.referencing_state_publisher.publish(referencing_state_msg)

                    # Tạo và phát trạng thái ErrorStates
                    error_states_msg = self.create_error_states_msg(robot_state)
                    self.error_states_publisher.publish(error_states_msg)

                    # Tạo và phát trạng thái PositionRobot
                    position_robot_msg = self.create_position_robot_msg(robot_state)
                    self.position_robot_publisher.publish(position_robot_msg)

                    # Tạo và phát trạng thái CartSpeedMMPerS
                    cart_speed_msg = self.create_cart_speed_mm_per_s_msg(robot_state)
                    self.cart_speed_mm_per_s_publisher.publish(cart_speed_msg)

                    # Ghi log thông báo đã phát thành công
                    # self.get_logger().info(f"Published joint states and robot state.")

            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")

    def create_mode_msg(self, robot_state):
        """Tạo thông báo trạng thái chế độ hoạt động từ robot_state."""
        mode_msg = String()
        mode_msg.data = f"Mode: {robot_state.mode}"
        return mode_msg
    
    def create_cart_speed_mm_per_s_msg(self, robot_state):
        """Tạo thông báo tốc độ Cartesian từ robot_state."""
        cart_speed_msg = String()
        cart_speed_msg.data = f"CartSpeedMMPerS: {robot_state.cart_speed_mm_per_s}"
        return cart_speed_msg
    
    def create_position_robot_msg(self, robot_state):
        """Tạo thông báo vị trí robot từ robot_state."""
        position_robot_msg = String()
        position_robot_msg.data = f"PositionRobot: {robot_state.position_robot}"
        return position_robot_msg
    
    def create_error_states_msg(self, robot_state):
        """Tạo thông báo trạng thái lỗi từ robot_state."""
        error_states_msg = String()
        error_states_msg.data = f"ErrorStates: {robot_state.error_states}"
        return error_states_msg
    
    def create_kinematics_state_msg(self, robot_state):
        """Tạo thông báo trạng thái Kinematics từ robot_state."""
        kinematics_state_msg = String()
        kinematics_state_msg.data = f"KinematicsState: {robot_state.kinematics_state}"
        return kinematics_state_msg
        """Tạo thông báo trạng thái chế độ hoạt động từ robot_state."""
        operation_mode_msg = String()
        operation_mode_msg.data = f"OperationMode: {robot_state.operation_mode}"
        return operation_mode_msg
    
    def create_referencing_state_msg(self, robot_state):
        """Tạo thông báo trạng thái Referencing từ robot_state."""
        referencing_state_msg = String()
        referencing_state_msg.data = f"ReferencingState: {robot_state.referencing_state}"
        return referencing_state_msg
    
    def create_dout_msg(self, robot_state):
        """Tạo thông báo trạng thái DigitalOut từ robot_state."""
        dout_msg = String()
        dout_msg.data = f"DOUT: {robot_state.dout}"
        return dout_msg
    
    def create_overrides_msg(self, robot_state):
        """Tạo thông báo trạng thái Overrides từ robot_state."""
        overrides_msg = String()
        overrides_msg.data = f"Overrides: {robot_state.override}"
        return overrides_msg

def main(args=None):
    rclpy.init(args=args)

    node = RobotConnection()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
