"""
Calibration launch file.

Starts all eight CubeMars motor_node instances (with auto_start=true so each
motor enters MIT control mode automatically), waits briefly for them to come
up, then launches the calibration node which sequences through every joint.

Usage
-----
Full 8-motor calibration (default):
  ros2 launch cubemars_calibration calibration.launch.py

Single motor only:
  ros2 launch cubemars_calibration calibration.launch.py joint_names:='[motor2]'

Dry run (no commands sent; tests the node wiring):
  ros2 launch cubemars_calibration calibration.launch.py dry_run:=true

Custom output path:
  ros2 launch cubemars_calibration calibration.launch.py \
      output_file:=/home/user/my_gains.yaml

Notes
-----
• The robot must be in a safe position with all legs free to move ±0.25 rad
  (~14°) at each joint before launching.
• Motors are calibrated one at a time in order motor0 … motor7.
• The calibration node writes the optimal gains to `output_file` and prints a
  ready-to-paste Python snippet for actuation.launch.py to the console.
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_ACTUATION_DIR = get_package_share_directory('actuation_bringup')
CAN_LAUNCH    = os.path.join(_ACTUATION_DIR, 'launch', 'can.launch.py')
CUBEMARS_LAUNCH = os.path.join(_ACTUATION_DIR, 'launch', 'cubemars.launch.py')
CAL_CONFIG    = os.path.join(
    get_package_share_directory('cubemars_calibration'), 'config', 'calibration.yaml')

# Motor-node startup delay before the calibration node wakes up
MOTOR_STARTUP_DELAY_S = 3.0


def CubemarsNode(motor_id: str, interface: str, reverse: str = 'false'):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(CUBEMARS_LAUNCH),
        launch_arguments={
            'motor_id':         motor_id,
            'interface':        interface,
            'motor_type':       'AK40-10',
            'control_hz':       '100.0',
            'auto_start':       'true',   # Motors enter MIT mode automatically
            'position_kp':      '3.0',    # Conservative baseline while calibrating
            'position_kd':      '0.3',
            'velocity_kd':      '0.5',
            'cmd_timeout':      '1.0',    # Longer timeout so tests don't race
            'reverse_polarity': reverse,
        }.items()
    )


def generate_launch_description():
    joint_names_arg = DeclareLaunchArgument(
        'joint_names',
        default_value="['motor0','motor1','motor2','motor3',"
                      "'motor4','motor5','motor6','motor7']",
        description='List of joint names to calibrate (Python list syntax).'
    )
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='/tmp/cubemars_calibrated_gains.yaml',
        description='Path to write calibrated gains YAML.'
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='If true, no commands are sent to motors (wiring test).'
    )

    calibration_node = Node(
        package='cubemars_calibration',
        executable='calibration_node',
        name='cubemars_calibration_node',
        output='screen',
        parameters=[CAL_CONFIG, {
            'joint_names':  LaunchConfiguration('joint_names'),
            'output_file':  LaunchConfiguration('output_file'),
            'dry_run':      LaunchConfiguration('dry_run'),
        }]
    )

    return LaunchDescription([
        joint_names_arg,
        output_file_arg,
        dry_run_arg,

        # CAN buses
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(CAN_LAUNCH),
            launch_arguments={'interface': 'can0'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(CAN_LAUNCH),
            launch_arguments={'interface': 'can1'}.items()),

        # Motor nodes  (inner = reversed polarity, outer = normal)
        CubemarsNode('0', 'can0', reverse='true'),   # FL inner
        CubemarsNode('1', 'can0', reverse='false'),  # FL outer
        CubemarsNode('2', 'can1', reverse='true'),   # RL inner
        CubemarsNode('3', 'can1', reverse='false'),  # RL outer
        CubemarsNode('4', 'can1', reverse='true'),   # RR inner
        CubemarsNode('5', 'can1', reverse='false'),  # RR outer
        CubemarsNode('6', 'can0', reverse='true'),   # FR inner
        CubemarsNode('7', 'can0', reverse='false'),  # FR outer

        # Calibration node starts after motors have initialised
        TimerAction(
            period=MOTOR_STARTUP_DELAY_S,
            actions=[calibration_node]
        ),
    ])
