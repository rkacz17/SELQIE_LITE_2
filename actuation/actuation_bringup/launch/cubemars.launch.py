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

    joint_name = f'motor{motor_id}'

    return [
        Node(
            package='cubemars_v2_ros',
            executable='motor_node',
            name=f'{joint_name}_node',
            output='screen',
            parameters=[{
                'can_interface': interface,
                'can_id': int(motor_id) + 1,
                'motor_type': motor_type,
                'control_hz': float(control_hz),
                'joint_name': joint_name,
                'auto_start': auto_start.lower() in ('true', '1', 'yes'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('motor_id', default_value='0', description='Motor index (0-7).'),
        DeclareLaunchArgument('interface', default_value='can0', description='CAN interface name.'),
        DeclareLaunchArgument('motor_type', default_value='AK70-10', description='Cubemars motor type.'),
        DeclareLaunchArgument('control_hz', default_value='20.0', description='Control loop rate in Hz.'),
        DeclareLaunchArgument('auto_start', default_value='false', description='Start motor automatically.'),
        OpaqueFunction(function=launch_setup),
    ])
