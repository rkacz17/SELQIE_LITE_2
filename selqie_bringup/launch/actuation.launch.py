import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

CAN_LAUNCH_FILE = os.path.join(
        get_package_share_directory('actuation_bringup'), 'launch', 'can.launch.py')

CUBEMARS_LAUNCH_FILE = os.path.join(
        get_package_share_directory('actuation_bringup'), 'launch', 'cubemars.launch.py')

# ── Per-group gain tuning ──────────────────────────────────────────────────────
# Inner shafts (motors 0, 2, 4, 6) — reversed polarity
INNER_KP         = '6.0'
INNER_KD         = '0.35'
INNER_VEL_KD     = '0.5'

# Outer shafts (motors 1, 3, 5, 7)
OUTER_KP         = '6.0'
OUTER_KD         = '0.35'
OUTER_VEL_KD     = '0.5'
# ──────────────────────────────────────────────────────────────────────────────

def CanLaunch(interface: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(CAN_LAUNCH_FILE),
        launch_arguments={'interface': interface}.items()
    )

def CubemarsLaunch(motor_id: str, interface: str,
                   position_kp: str, position_kd: str, velocity_kd: str,
                   reverse_polarity: str = 'false'):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(CUBEMARS_LAUNCH_FILE),
        launch_arguments={
            'motor_id':         motor_id,
            'interface':        interface,
            'position_kp':      position_kp,
            'position_kd':      position_kd,
            'velocity_kd':      velocity_kd,
            'reverse_polarity': reverse_polarity,
        }.items()
    )

def InnerShaft(motor_id: str, interface: str):
    return CubemarsLaunch(motor_id, interface,
                          INNER_KP, INNER_KD, INNER_VEL_KD)

def OuterShaft(motor_id: str, interface: str):
    return CubemarsLaunch(motor_id, interface,
                          OUTER_KP, OUTER_KD, OUTER_VEL_KD,
                          reverse_polarity='false')

def generate_launch_description():
    return LaunchDescription([
        CanLaunch('can0'),
        CanLaunch('can1'),
        InnerShaft('0', 'can0'),   # FL inner
        OuterShaft('1', 'can0'),   # FL outer
        InnerShaft('2', 'can1'),   # RL inner
        OuterShaft('3', 'can1'),   # RL outer
        InnerShaft('4', 'can1'),   # RR inner
        OuterShaft('5', 'can1'),   # RR outer
        InnerShaft('6', 'can0'),   # FR inner
        OuterShaft('7', 'can0'),   # FR outer
    ])
