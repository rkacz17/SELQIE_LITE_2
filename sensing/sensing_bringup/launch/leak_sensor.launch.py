from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetson_drivers',
            executable='gpio_node',
            name='leak_sensor_gpio_node',
            output='screen',
            parameters=[{
                'gpio_pin': 35,     # BOARD pin 35
                'is_output': False,
                'frequency': 10.0,  # Hz
            }],
            remappings=[('gpio/in', 'leak/gpio_in')],
        ),
        Node(
            package='leak_sensor',
            executable='leak_sensor_node',
            name='leak_sensor_node',
            output='screen',
            parameters=[{
                'active_high': True,
            }],
            remappings=[('gpio/in', 'leak/gpio_in')],
        ),
    ])
