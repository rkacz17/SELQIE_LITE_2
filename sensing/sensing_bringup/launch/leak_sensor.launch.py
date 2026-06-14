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
                'gpio_pin': 35,     # BOARD pin 35
                'frequency': 10.0,  # Hz
                'active_high': True,
            }],
        ),
    ])
