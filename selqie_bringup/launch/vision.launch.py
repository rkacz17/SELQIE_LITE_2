import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ZED_MINI_LAUNCH_FILE = os.path.join(
        get_package_share_directory('vision_bringup'), 'launch', 'zed_mini.launch.py')

LIGHT_PWM_PIN = 18

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ZED_MINI_LAUNCH_FILE)
        ),
        Node(
            package='jetson_drivers',
            executable='gpio_node',
            name='camera_light_node',
            output='screen',
            parameters=[{
                'gpio_pin': LIGHT_PWM_PIN,
                'is_pwm': True,
            }],
            remappings=[
                ('gpio/out', 'lights/pwm'),
            ]
        )
    ])
