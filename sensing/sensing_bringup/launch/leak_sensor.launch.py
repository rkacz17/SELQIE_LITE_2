from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leak_sensor',
            executable='leak_sensor_node',
            name='leak_sensor_node',
            output='screen',
            parameters=[{
                'gpio_pin': 15,     # BOARD pin 15 = SOC_GPIO27 on Orin AGX (no conflicts)
                'frequency': 10.0,  # Hz
                'active_high': True,
            }],
        ),
    ])
