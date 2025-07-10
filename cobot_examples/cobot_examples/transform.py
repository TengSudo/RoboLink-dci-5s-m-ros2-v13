import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped

class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',     # target_frame
                'link_tool',     # source_frame
                now
            )

            # Convert position (m → mm)
            pos = trans.transform.translation
            x_mm = pos.x * 1000
            y_mm = pos.y * 1000
            z_mm = pos.z * 1000

            # Convert orientation (quaternion → Euler angles in degrees)
            rot = trans.transform.rotation
            q = [rot.x, rot.y, rot.z, rot.w]
            roll, pitch, yaw = euler_from_quaternion(q)

            roll_deg = roll * 180.0 / 3.14159265
            pitch_deg = pitch * 180.0 / 3.14159265
            yaw_deg = yaw * 180.0 / 3.14159265

            self.get_logger().info(f"x={x_mm:.2f}, y={y_mm:.2f}, z={z_mm:.2f}")
            self.get_logger().info(f"roll={roll_deg:.2f}, pitch={pitch_deg:.2f}, yaw={yaw_deg:.2f}")

        except Exception as e:
            self.get_logger().warn(f"⚠️ Transform not available: {e}")


def main():
    rclpy.init()
    node = TfListener()
    rclpy.spin(node)
    rclpy.shutdown()
