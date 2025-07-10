import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import degrees
from cri_lib import CRIController 

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/gui_joint_states',
            self.listener_callback,
            10
        )
        # Khởi tạo CRIController
        self.controller = CRIController()
        self.controller.connect("192.168.3.11")  # Kết nối đến iRC IP
        self.controller.set_active_control(True)
        self.controller.enable()
        self.get_logger().info("Controller initialized and motors enabled.")
        
        # Lưu trữ giá trị joint trước đó
        self.previous_joint_positions = [0.0] * 5  # Giả sử có 5 joint
        self.threshold = 0.01  # Ngưỡng thay đổi (độ) để gửi lệnh mới
        self.joint_count = 5  # Số lượng joint cần kiểm soát

    def listener_callback(self, msg):
        current_joint_positions = []
        for i, name in enumerate(msg.name):
            if i < len(msg.position): 
                pos_in_degrees = degrees(msg.position[i])
                current_joint_positions.append(pos_in_degrees)
                self.get_logger().info(f'Joint:{name} {pos_in_degrees:.2f}')
        
        # Kiểm tra nếu đủ số lượng giá trị joint cần thiết
        if len(current_joint_positions) >= self.joint_count:
            # So sánh với giá trị trước đó
            significant_change = False
            for i in range(self.joint_count):  # Sửa lỗi ở đây - dùng integer thay vì float
                if abs(current_joint_positions[i] - self.previous_joint_positions[i]) >= self.threshold:
                    significant_change = True
                    break
            
            if significant_change:
                joint_1, joint_2, joint_3, joint_4, joint_5 = current_joint_positions[:self.joint_count]
                self.controller.move_joints(
                    joint_1, joint_2, joint_3, joint_4, joint_5,
                    0.0, 0.0, 0.0, 0.0, 30.0,
                    wait_move_finished=False, move_finished_timeout=10000
                )
                # Cập nhật giá trị trước đó
                self.previous_joint_positions = current_joint_positions[:self.joint_count]
                self.get_logger().info("Sent new joint positions to controller.")

    def destroy_node(self):
        # Tắt động cơ và ngắt kết nối khi node bị hủy
        self.controller.disable()
        self.controller.close()
        self.get_logger().info("Controller closed and motors disabled.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()