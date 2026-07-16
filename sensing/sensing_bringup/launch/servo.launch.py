from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[{
                'gpio_pin': 13,           # BOARD pin 32 = PWM0 on Orin AGX
                'frequency': 50.0,        # Hz — standard RC servo
                'open_duty_cycle': 5.0,   # 1.0 ms pulse → latch open
                'close_duty_cycle': 10.0, # 2.0 ms pulse → latch closed
            }],
        ),
    ])
