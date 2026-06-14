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
                'gpio_pin': 29,     # BOARD pin 29 = SOC_GPIO01 on Orin AGX (no conflicts)
                'frequency': 10.0,  # Hz
                'active_high': True,
            }],
        ),
    ])
