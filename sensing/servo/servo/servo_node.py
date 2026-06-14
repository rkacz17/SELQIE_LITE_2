import Jetson.GPIO as GPIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ServoNode(Node):
    """
    Controls a D954SW R/C servo via hardware PWM.
    Publishes nothing; subscribes to 'servo/latch' (Bool):
      True  → latch open
      False → latch closed
    """

    def __init__(self):
        super().__init__('servo_node')

        self.declare_parameter('gpio_pin', 32)        # BOARD pin 32 = PWM0
        self.declare_parameter('frequency', 50.0)     # Standard RC servo: 50 Hz
        self.declare_parameter('open_duty_cycle', 5.0)   # 1.0 ms pulse → open
        self.declare_parameter('close_duty_cycle', 10.0) # 2.0 ms pulse → closed

        self._pin    = self.get_parameter('gpio_pin').value
        freq         = self.get_parameter('frequency').value
        self._open_dc  = self.get_parameter('open_duty_cycle').value
        self._close_dc = self.get_parameter('close_duty_cycle').value

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pin, GPIO.OUT)
        self._pwm = GPIO.PWM(self._pin, freq)
        self._pwm.start(self._close_dc)  # boot in closed position

        self.create_subscription(Bool, 'servo/latch', self._on_latch, 10)

        self.get_logger().info(
            f'Servo node started on GPIO pin {self._pin} '
            f'(open={self._open_dc}%, close={self._close_dc}%, {freq} Hz)'
        )

    def _on_latch(self, msg: Bool):
        if msg.data:
            self._pwm.ChangeDutyCycle(self._open_dc)
            self.get_logger().info('Latch opened')
        else:
            self._pwm.ChangeDutyCycle(self._close_dc)
            self.get_logger().info('Latch closed')

    def destroy_node(self):
        self._pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
