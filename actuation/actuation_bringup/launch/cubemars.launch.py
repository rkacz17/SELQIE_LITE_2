from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    motor_id = LaunchConfiguration('motor_id').perform(context)
    interface = LaunchConfiguration('interface').perform(context)
    motor_type = LaunchConfiguration('motor_type').perform(context)
    control_hz = LaunchConfiguration('control_hz').perform(context)
    auto_start = LaunchConfiguration('auto_start').perform(context)
    position_kp = LaunchConfiguration('position_kp').perform(context)
    position_kd = LaunchConfiguration('position_kd').perform(context)
    velocity_kd = LaunchConfiguration('velocity_kd').perform(context)
    cmd_timeout = LaunchConfiguration('cmd_timeout').perform(context)

    joint_name = f'motor{motor_id}'

    return [
        Node(
            package='cubemars_v2_ros',
            executable='motor_node',
            name=f'{joint_name}_node',
            output='screen',
            parameters=[{
                'can_interface': interface,
                'can_id': int(motor_id),
                'motor_type': motor_type,
                'control_hz': float(control_hz),
                'joint_name': joint_name,
                'auto_start': auto_start.lower() in ('true', '1', 'yes'),
                'position_kp': float(position_kp),
                'position_kd': float(position_kd),
                'velocity_kd': float(velocity_kd),
                'cmd_timeout': float(cmd_timeout),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_id', default_value='0', description='Motor index (0-7).'
        ),
        DeclareLaunchArgument(
            'interface', default_value='can0', description='CAN interface name.'
        ),
        DeclareLaunchArgument(
            'motor_type', default_value='AK40-10', description='Cubemars motor type.'
        ),
        DeclareLaunchArgument(
            'control_hz', default_value='100.0', description='Control loop rate in Hz.'
        ),
        DeclareLaunchArgument(
            'auto_start', default_value='false', description='Start motor automatically.'
        ),
        DeclareLaunchArgument(
            'position_kp',
            default_value='1.1',
            description='MIT Kp used for /motorN/command position control (AK40-10: T_MAX=5Nm, keep low).',
        ),
        DeclareLaunchArgument(
            'position_kd',
            default_value='0.12',
            description='MIT Kd used for /motorN/command position control.',
        ),
        DeclareLaunchArgument(
            'velocity_kd',
            default_value='0.5',
            description='MIT Kd used for /motorN/command velocity control.',
        ),
        DeclareLaunchArgument(
            'cmd_timeout',
            default_value='0.5',
            description='Seconds without a command before motor is zeroed (0 = disabled).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
