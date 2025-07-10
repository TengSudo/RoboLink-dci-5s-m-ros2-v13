import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import math

def main():
    rclpy.init()
    node = rclpy.create_node('moveit2_python_client')

    # Tạo Action Client để giao tiếp với MoveGroup
    action_client = ActionClient(node, MoveGroup, '/move_action')

    # Đợi kết nối đến MoveIt 2 Action Server
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('Không thể kết nối đến MoveIt 2 Action Server!')
        return

    # Tạo một goal
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = "arm"

    # Nhập tọa độ Descartes (x, y, z) và góc Euler (roll, pitch, yaw) từ người dùng
    print("Nhập tọa độ Descartes (m) và góc Euler (độ):")
    pose = Pose()
    while True:
        try:
            pose.position.x = float(input("Nhập x (m): "))
            pose.position.y = float(input("Nhập y (m): "))
            pose.position.z = float(input("Nhập z (m): "))
            roll_deg = float(input("Nhập roll (độ): "))
            pitch_deg = float(input("Nhập pitch (độ): "))
            yaw_deg = float(input("Nhập yaw (độ): "))
            break
        except ValueError:
            print("Vui lòng nhập các số hợp lệ!")

    # Chuyển đổi góc Euler (độ) sang radian và sau đó sang quaternion
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

    # Gán quaternion vào pose
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    # Thêm pose vào goal
    goal_msg.request.goal_constraints.append(create_pose_constraint(pose))

    # Gửi goal và đợi kết quả
    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Robot đã di chuyển đến vị trí Descartes được chỉ định thành công!')
    else:
        node.get_logger().error('Lỗi khi di chuyển robot!')

    node.destroy_node()
    rclpy.shutdown()

def create_pose_constraint(pose):
    from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
    from shape_msgs.msg import SolidPrimitive

    constraints = Constraints()

    # Ràng buộc vị trí
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "base_link"  # Thay bằng frame tham chiếu phù hợp
    position_constraint.link_name = "virtual_link"  # Thay bằng tên link đầu cuối (end-effector)
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0

    # Định nghĩa vùng mục tiêu (bounding volume)
    bounding_volume = BoundingVolume()
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [0.01]  # Bán kính 1cm
    bounding_volume.primitives.append(primitive)
    bounding_volume.primitive_poses.append(pose)
    position_constraint.constraint_region = bounding_volume
    position_constraint.weight = 1.0
    constraints.position_constraints.append(position_constraint)

    # Ràng buộc hướng
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "base_link"  # Thay bằng frame tham chiếu phù hợp
    orientation_constraint.link_name = "virtual_link"  # Thay bằng tên link đầu cuối
    orientation_constraint.orientation = pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.01
    orientation_constraint.absolute_y_axis_tolerance = 0.01
    orientation_constraint.absolute_z_axis_tolerance = 0.01
    orientation_constraint.weight = 1.0
    constraints.orientation_constraints.append(orientation_constraint)

    return constraints

if __name__ == '__main__':
    main()