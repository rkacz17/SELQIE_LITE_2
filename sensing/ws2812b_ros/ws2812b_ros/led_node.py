import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32MultiArray
from .ws2812b_spi import WS2812B_SPI


def unpack_rgb(val: int):
    r = (val >> 16) & 0xFF
    g = (val >> 8) & 0xFF
    b = val & 0xFF
    return r, g, b


class LEDNode(Node):
    def __init__(self):
        super().__init__('ws2812b_led_node')

        # Parameters
        self.declare_parameter('num_leds', 2)
        self.declare_parameter('brightness', 1.0)
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_dev', 0)
        self.declare_parameter('spi_hz', 2_400_000)
        self.declare_parameter('pixel_order', 'GRB')
        # Color to show immediately on startup, before any /led_colors message
        # arrives, so the strip gives visible confirmation the node is alive
        # and the SPI wiring is working. (0, 0, 0) disables this behavior.
        self.declare_parameter('startup_color', [0, 255, 0])

        n = int(self.get_parameter('num_leds').value)
        br = float(self.get_parameter('brightness').value)
        bus = int(self.get_parameter('spi_bus').value)
        dev = int(self.get_parameter('spi_dev').value)
        hz = int(self.get_parameter('spi_hz').value)
        po = str(self.get_parameter('pixel_order').value)
        startup_r, startup_g, startup_b = [
            int(c) for c in self.get_parameter('startup_color').value
        ]

        self.dev = WS2812B_SPI(
            n_leds=n,
            spi_bus=bus,
            spi_dev=dev,
            spi_hz=hz,
            brightness=br,
            pixel_order=po,
        )

        self.dev.fill(startup_r, startup_g, startup_b)
        self.dev.show()

        self.sub = self.create_subscription(
            UInt32MultiArray, 'led_colors', self.cb_colors, 10
        )

        self.get_logger().info(
            f'WS2812B SPI node: n={n}, /dev/spidev{bus}.{dev} @ {hz} Hz, '
            f'brightness={br}, pixel_order={po}, '
            f'startup_color=({startup_r},{startup_g},{startup_b})'
        )

    def destroy_node(self):
        try:
            self.dev.fill(0, 0, 0)
            self.dev.show()
        except Exception:
            pass
        self.dev.close()
        super().destroy_node()

    def cb_colors(self, msg: UInt32MultiArray):
        n = min(len(msg.data), self.dev.n)
        for i in range(n):
            r, g, b = unpack_rgb(int(msg.data[i]))
            self.dev.set_rgb(i, r, g, b)
        # Clear any remaining LEDs
        for j in range(n, self.dev.n):
            self.dev.set_rgb(j, 0, 0, 0)
        self.dev.show()


def main():
    rclpy.init()
    node = LEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

