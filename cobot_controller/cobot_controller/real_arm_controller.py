#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from sensor_msgs.msg import JointState
from cri_lib import CRIController
from math import degrees, radians, isclose
from rclpy.executors import Future

class RealArmController(Node):
    def __init__(self):
        super().__init__('real_arm_controller')

class RealArmController(Node):
    def __init__(self):
        super().__init__('real_arm_controller')

        # Khởi tạo controller
        self.controller = CRIController()
        host = '192.168.3.11'
        port = 3920    
        self.controller.connect(host=host, port=port)
        self.controller.set_active_control(True)
        self.controller.enable()

        self.get_logger().info("✅ CRIController connected and motors enabled.")

        # Khởi tạo action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_callback
        )

        # Thêm subscriber để lắng nghe joint_states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Biến lưu trữ trạng thái khớp hiện tại
        self.current_joint_positions = [0.0] * 5
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']  # Điều chỉnh theo tên khớp thực tế
        self.last_goal_handle = None

        self.get_logger().info("✅ Real Arm Controller (action server) is ready.")
        self.get_logger().info("✅ Subscribing to /joint_states topic.")

    def joint_state_callback(self, msg: JointState):
        """Callback để cập nhật vị trí khớp hiện tại."""
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                index = self.joint_names.index(name)
                self.current_joint_positions[index] = pos

    async def wait_for_movement_completion(self, target_positions: list, tolerance: float = 0.2, timeout: float = 15.0):
        """Chờ cho đến khi các khớp đạt đến vị trí mục tiêu với dung sai cho phép."""
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        check_count = 0
        min_check_count = 1  # Yêu cầu kiểm tra ít nhất 3 lần trước khi xác nhận hoàn thành
        
        while True:
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - start_time > timeout:
                self.get_logger().warn(f"❗ Timeout while waiting for movement completion. Target: {target_positions}")
                return False
            
            # Kiểm tra vị trí các khớp
            all_reached = True
            for current, target in zip(self.current_joint_positions, target_positions):
                if not isclose(current, target, abs_tol=tolerance):
                    all_reached = False
                    break
            
            if all_reached:
                check_count += 1
                if check_count >= min_check_count:
                    self.get_logger().info(f"✅ All joints reached target positions: {[round(p,4) for p in target_positions]}")
                    return True
            else:
                check_count = 0  # Reset nếu có khớp chưa đạt
            
            # Đợi 0.2s trước khi kiểm tra lại
            future = Future()
            self.create_timer(0.1, lambda: future.set_result(True))
            await future

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("📥 Received trajectory goal")
        self.last_goal_handle = goal_handle
        trajectory: JointTrajectory = goal_handle.request.trajectory

        try:
            # Thêm delay ban đầu để đảm bảo robot sẵn sàng
            future = Future()
            self.create_timer(0.5, lambda: future.set_result(True))
            await future

            for i, point in enumerate(trajectory.points):
                positions = point.positions
                if len(positions) < 5:
                    self.get_logger().warn(f"❗ Trajectory point {i} does not have enough joint values.")
                    continue

                target_positions = positions[:5]
                joint_angles_deg = [degrees(pos) for pos in target_positions]
                
                # Gửi lệnh di chuyển
                self.controller.move_joints(
                    *joint_angles_deg[:5],
                    0.0, 0.0, 0.0, 0.0, 30.0,
                    wait_move_finished=False,
                    move_finished_timeout=10000
                )
                self.get_logger().info(f"🚀 Sending point {i+1}/{len(trajectory.points)}: {[round(j,2) for j in joint_angles_deg]} deg")

                # Chờ hoàn thành di chuyển
                if not await self.wait_for_movement_completion(target_positions):
                    self.get_logger().error(f"❌ Failed to reach target at point {i+1}")
                    goal_handle.abort()
                    return FollowJointTrajectory.Result()

            goal_handle.succeed()
            self.get_logger().info("🎯 Trajectory execution completed successfully")
            return FollowJointTrajectory.Result()

        except Exception as e:
            self.get_logger().error(f"🔥 Exception in execute_callback: {str(e)}")
            goal_handle.abort()
            return FollowJointTrajectory.Result()


    def destroy_node(self):
        if self.last_goal_handle is not None and self.last_goal_handle.is_active:
            self.last_goal_handle.abort()
        self.controller.disable()
        self.controller.close()
        self.get_logger().info("🛑 CRIController disconnected and motors disabled.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()