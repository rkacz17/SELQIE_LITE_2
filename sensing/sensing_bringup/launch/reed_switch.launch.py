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
                'gpio_pin': 16,      # BOARD pin 16 = SOC_GPIO08 on Orin AGX (no conflicts)
                'frequency': 50.0,   # Hz
                'active_high': True,
            }],
        ),
    ])
