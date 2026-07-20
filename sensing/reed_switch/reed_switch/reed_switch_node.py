import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class ReedSwitchNode(Node):
    """
    Converts the raw GPIO reading published by jetson_drivers' gpio_node
    (subscribed here on 'gpio/in') into the reed switch state on
    'reed_switch/closed' (True = magnet present / switch closed).
    """

    def __init__(self):
        super().__init__('reed_switch_node')

        self.declare_parameter('active_high', True)
        self._active_high = self.get_parameter('active_high').value

        self._pub = self.create_publisher(Bool, 'reed_switch/closed', 10)
        self._sub = self.create_subscription(Float32, 'gpio/in', self._gpio_callback, 10)

        self.get_logger().info(
            f'Reed switch node started (active_high={self._active_high})'
        )

    def _gpio_callback(self, msg: Float32):
        raw = bool(msg.data)
        closed = raw if self._active_high else not raw

        out = Bool()
        out.data = closed
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ReedSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
