import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class LeakSensorNode(Node):
    """
    Converts the raw GPIO reading published by jetson_drivers' gpio_node
    (subscribed here on 'gpio/in') into leak detection state on
    'leak/detected' (True = leak present).
    """

    def __init__(self):
        super().__init__('leak_sensor_node')

        self.declare_parameter('active_high', True)
        self._active_high = self.get_parameter('active_high').value

        self._pub = self.create_publisher(Bool, 'leak/detected', 10)
        self._sub = self.create_subscription(Float32, 'gpio/in', self._gpio_callback, 10)

        self.get_logger().info(
            f'Leak sensor node started (active_high={self._active_high})'
        )

    def _gpio_callback(self, msg: Float32):
        raw = bool(msg.data)
        detected = raw if self._active_high else not raw

        out = Bool()
        out.data = detected
        self._pub.publish(out)

        if detected:
            self.get_logger().warn('LEAK DETECTED', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = LeakSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
