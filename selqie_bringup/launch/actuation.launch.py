import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

CAN_LAUNCH_FILE = os.path.join(
        get_package_share_directory('actuation_bringup'), 'launch', 'can.launch.py')

CUBEMARS_LAUNCH_FILE = os.path.join(
        get_package_share_directory('actuation_bringup'), 'launch', 'cubemars.launch.py')

def CanLaunch(interface : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(CAN_LAUNCH_FILE),
        launch_arguments={
            'interface': interface
        }.items()
    )

def CubemarsLaunch(motor_id : str, interface : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(CUBEMARS_LAUNCH_FILE),
        launch_arguments={
            'motor_id': motor_id,
            'interface': interface
        }.items()
    )

def generate_launch_description():
    return LaunchDescription([
        CanLaunch('can0'),
        #CanLaunch('can1'),
        CubemarsLaunch('0', 'can0'),
        CubemarsLaunch('1', 'can0'),
        #CubemarsLaunch('2', 'can1'),
        #CubemarsLaunch('3', 'can1'),
        #CubemarsLaunch('4', 'can1'),
        #CubemarsLaunch('5', 'can1'),
        #CubemarsLaunch('6', 'can0'),
        #CubemarsLaunch('7', 'can0'),
    ])
