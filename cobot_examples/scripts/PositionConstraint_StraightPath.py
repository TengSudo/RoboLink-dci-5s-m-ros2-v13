import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotTrajectory
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

    # Nhập các điểm cho đường thẳng (ít nhất 2 điểm)
    waypoints = []
    print("Nhập các điểm cho đường thẳng (tối thiểu 2 điểm).")
    while len(waypoints) < 2:
        print(f"Nhập điểm thứ {len(waypoints) + 1} (tọa độ Descartes và góc Euler):")
        pose = Pose()
        try:
            pose.position.x = float(input("Nhập x (m): "))
            pose.position.y = float(input("Nhập y (m): "))
            pose.position.z = float(input("Nhập z (m): "))
            roll_deg = float(input("Nhập roll (độ): "))
            pitch_deg = float(input("Nhập pitch (độ): "))
            yaw_deg = float(input("Nhập yaw (độ): "))

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

            waypoints.append(pose)
        except ValueError:
            print("Vui lòng nhập các số hợp lệ!")

        if len(waypoints) >= 2:
            print("Có muốn thêm điểm khác không? (y/n)")
            if input().lower() != 'y':
                break

    # Tạo kế hoạch đường đi Descartes
    plan = RobotTrajectory()
    try:
        # Gọi compute_cartesian_path để lập kế hoạch đường thẳng
        from moveit_msgs.srv import GetCartesianPath
        cartesian_path_client = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        if not cartesian_path_client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error('Không thể kết nối đến dịch vụ compute_cartesian_path!')
            return

        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"  # Thay bằng frame tham chiếu phù hợp
        request.link_name = "virtual_link"  # Thay bằng tên link đầu cuối
        request.group_name = "arm"
        request.waypoints = waypoints
        request.max_step = 0.01  # Bước tối đa giữa các điểm (m)
        request.jump_threshold = 0.0  # Ngưỡng nhảy (0 để tắt kiểm tra)
        request.avoid_collisions = True

        future = cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None and future.result().fraction > 0.0:
            plan = future.result().solution
            goal_msg.request.trajectory = plan
            node.get_logger().info(f'Đã lập kế hoạch đường thẳng ({future.result().fraction*100:.2f}% hoàn thành)')

            # Gửi kế hoạch đến MoveIt Action Server
            future = action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(node, future)

            if future.result() is not None:
                node.get_logger().info('Robot đã thực hiện đường thẳng thành công!')
            else:
                node.get_logger().error('Lỗi khi thực hiện đường thẳng!')
        else:
            node.get_logger().error('Không thể lập kế hoạch đường thẳng!')
    except Exception as e:
        node.get_logger().error(f'Lỗi khi lập kế hoạch: {str(e)}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()