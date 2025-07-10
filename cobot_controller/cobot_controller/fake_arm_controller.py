import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.action import ActionServer
import time

class FakeArmController(Node):
    def __init__(self):
        super().__init__('fake_arm_controller')
        
        self._joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'virtual_joint']
        self._current_positions = [0.0] * len(self._joint_names)  # Bao gồm virtual_joint

        # Publisher cho joint states
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber cho joint angles từ IK solver
        self._joint_angles_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_angles',
            self.joint_angles_callback,
            10
        )

        # Action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_callback
        )

        self.create_timer(0.1, self.publish_joint_states)
        self.get_logger().info('🚀 FakeArmController ready')

    def joint_angles_callback(self, msg):
        if len(msg.data) == len(self._joint_names):
            self._current_positions = msg.data
            self.get_logger().info(f"Updated joints: {self._current_positions}")
        else:
            self.get_logger().warn(f"Received joint angles count does not match. Expected {len(self._joint_names)}.")

    def publish_joint_states(self):
        msg = JointState()
        msg.name = self._joint_names
        msg.position = self._current_positions
        msg.header.stamp = self.get_clock().now().to_msg()
        self._joint_state_pub.publish(msg)

    def execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory
        self.get_logger().info(f"Executing trajectory with {len(traj.points)} points")

        # Kiểm tra số lượng khớp trong trajectory
        if len(traj.joint_names) == len(self._joint_names):
            self._current_positions = traj.points[-1].positions  # Lấy vị trí của điểm cuối cùng
            self.get_logger().info(f"Executing trajectory to: {self._current_positions}")
            goal_handle.succeed()
            return FollowJointTrajectory.Result()
        else:
            self.get_logger().error("Trajectory joint names do not match the controller's joints.")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    node = FakeArmController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
