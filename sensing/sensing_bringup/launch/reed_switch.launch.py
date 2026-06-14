from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reed_switch',
            executable='reed_switch_node',
            name='reed_switch_node',
            output='screen',
            parameters=[{
                'gpio_pin': 38,      # BOARD pin 38
                'frequency': 50.0,   # Hz
                'active_high': True,
            }],
        ),
    ])
