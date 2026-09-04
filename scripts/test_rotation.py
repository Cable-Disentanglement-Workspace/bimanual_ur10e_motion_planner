import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math


class URScriptSender(Node):
    def __init__(self):
        super().__init__('urscript_sender')
        self.pub = self.create_publisher(
            String, '/urscript_interface/script_command', 10)

    def rotate_tool_x(self, theta_rad, accel=0.5, vel=0.2):
        script = String()
        script.data = (
            "def rot_x():\n"
            f"  rot = p[0, 0, 0, {theta_rad}, 0, 0]\n"
            "  target = pose_trans(get_actual_tcp_pose(), rot)\n"
            f"  movel(target, a={accel}, v={vel})\n"
            "end\n"
        )
        self.pub.publish(script)
        self.get_logger().info(f"Sent tool-X rotation: {theta_rad} rad")


def main():
    rclpy.init()
    node = URScriptSender()

    # give the publisher a moment to connect before first send
    import time
    time.sleep(1.0)

    node.rotate_tool_x(math.radians(30))    # spin 30° about tool X

    rclpy.spin_once(node, timeout_sec=1.0)  # let the message go out
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()