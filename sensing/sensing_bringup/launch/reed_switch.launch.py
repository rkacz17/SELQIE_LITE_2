from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetson_drivers',
            executable='gpio_node',
            name='reed_switch_gpio_node',
            output='screen',
            parameters=[{
                'gpio_pin': 38,      # BOARD pin 38
                'is_output': False,
                'frequency': 50.0,   # Hz
            }],
            remappings=[('gpio/in', 'reed_switch/gpio_in')],
        ),
        Node(
            package='reed_switch',
            executable='reed_switch_node',
            name='reed_switch_node',
            output='screen',
            parameters=[{
                'active_high': True,
            }],
            remappings=[('gpio/in', 'reed_switch/gpio_in')],
        ),
    ])
