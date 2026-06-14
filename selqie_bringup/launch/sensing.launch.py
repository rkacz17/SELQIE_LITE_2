import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

SENSING_BRINGUP = get_package_share_directory('sensing_bringup')

def SensingLaunch(name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SENSING_BRINGUP, 'launch', name)
        )
    )

BATTERY_LAUNCH_FILE = os.path.join(
        get_package_share_directory('battery'), 'launch', 'tinybms_voltage_uart.launch.py')

def generate_launch_description():
    return LaunchDescription([
        SensingLaunch('bar100.launch.py'),
        SensingLaunch('leak_sensor.launch.py'),
        SensingLaunch('reed_switch.launch.py'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(BATTERY_LAUNCH_FILE)
        ),
        # SensingLaunch('imu.launch.py'),
    ])
