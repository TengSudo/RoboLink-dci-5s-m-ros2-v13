import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

def main():
    rclpy.init()
    node = rclpy.create_node('moveit2_python_client')

    # Tạo Action Client để giao tiếp với MoveGroup
    action_client = ActionClient(node, MoveGroup, '/move_action')

    # Đợi kết nối đến MoveIt 2 Action Server
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('Không thể kết nối đến MoveIt 2 Action Server!')
        return

    # Tạo một goal với joint values của "zero"
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = "arm"

    # Định nghĩa joint values cho vị trí "zero" (thay số bằng giá trị thực tế của robot)
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4","joint_5", "joint_6"]
    joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Tạo constraints cho các joints
    constraints = Constraints()
    for name, value in zip(joint_names, joint_values):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = name
        joint_constraint.position = value
        joint_constraint.tolerance_above = 0.01
        joint_constraint.tolerance_below = 0.01
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)

    goal_msg.request.goal_constraints.append(constraints)

    # Gửi goal và đợi kết quả
    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Robot đã di chuyển đến "zero" thành công!')
    else:
        node.get_logger().error('Lỗi khi di chuyển robot!')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()