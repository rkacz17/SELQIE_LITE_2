import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import Jetson.GPIO as GPIO


class LeakSensorNode(Node):
    """
    Polls a digital GPIO pin connected to a leak sensor and publishes
    True on 'leak/detected' whenever the pin reads HIGH (leak present).
    """

    def __init__(self):
        super().__init__('leak_sensor_node')

        self.declare_parameter('gpio_pin', 35)
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('active_high', True)

        self._pin = self.get_parameter('gpio_pin').value
        frequency = self.get_parameter('frequency').value
        self._active_high = self.get_parameter('active_high').value

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pin, GPIO.IN)

        self._pub = self.create_publisher(Bool, 'leak/detected', 10)
        self.create_timer(1.0 / frequency, self._poll)

        self.get_logger().info(
            f'Leak sensor node started on GPIO pin {self._pin} '
            f'(active_high={self._active_high}, {frequency} Hz)'
        )

    def _poll(self):
        raw = GPIO.input(self._pin)
        detected = bool(raw) if self._active_high else not bool(raw)

        msg = Bool()
        msg.data = detected
        self._pub.publish(msg)

        if detected:
            self.get_logger().warn('LEAK DETECTED', throttle_duration_sec=1.0)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeakSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
