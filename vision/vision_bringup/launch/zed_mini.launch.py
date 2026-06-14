import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

CONFIG_FOLDER = os.path.join(get_package_share_directory('vision_bringup'), 'config')
ZED_MINI_CONFIG = os.path.join(CONFIG_FOLDER, 'zed_mini.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed_wrapper',
            executable='zed_wrapper',
            name='zed_node',
            output='screen',
            parameters=[ZED_MINI_CONFIG],
            remappings=[
                # Keep stereo/points2 notation for downstream pointcloud_filters and terrain_mapping
                ('stereo/point_cloud/cloud_registered', 'stereo/points2'),
                # Expose rectified ZED images under the same raw-image topics the rest of the stack expects
                ('stereo/left/image_rect_color',  'stereo/left/image_raw'),
                ('stereo/right/image_rect_color', 'stereo/right/image_raw'),
            ]
        )
    ])
