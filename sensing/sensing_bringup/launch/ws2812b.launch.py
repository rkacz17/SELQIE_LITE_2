from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ws2812b_ros',
            executable='led_node',
            name='ws2812b_led_node',
            output='screen',
            parameters=[{
                'num_leds':    1,
                'brightness':  1.0,
                'spi_bus':     0,
                'spi_dev':     0,
                'spi_hz':      2_400_000,
                'pixel_order': 'GRB',
            }],
        ),
    ])
