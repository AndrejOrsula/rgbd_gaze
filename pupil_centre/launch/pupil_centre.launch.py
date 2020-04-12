"""Launch pupil centre localisation"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    config_openface_separate = LaunchConfiguration('config_openface_separate', default=os.path.join(get_package_share_directory(
        'pupil_centre'), 'config', 'openface_separate.yaml'))
    config_pupil_centre = LaunchConfiguration('config_pupil_centre', default=os.path.join(get_package_share_directory(
        'pupil_centre'), 'config', 'pupil_centre.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_openface_separate',
            default_value=config_openface_separate,
            description='Path to config for openface'),
        DeclareLaunchArgument(
            'config_pupil_centre',
            default_value=config_pupil_centre,
            description='Path to config for pupil centre localisation'),

        Node(
            package='openface',
            node_executable='openface_separate',
            node_name='openface_separate',
            node_namespace='',
            output='screen',
            parameters=[config_openface_separate],
            remappings=[('camera/image_raw', 'camera/color/image_raw'),
                        ('camera/camera_info', 'camera/color/camera_info'),
                        ('openface/landmarks_visible', 'pupil_centre/landmarks_visible')],
        ),

        Node(
            package='openface_eye_region',
            node_executable='openface_eye_region',
            node_name='openface_eye_region',
            node_namespace='',
            output='screen',
            remappings=[('openface/landmarks_visible', 'pupil_centre/landmarks_visible'),
                        ('openface/eye_regions', 'pupil_centre/eye_regions')],
        ),

        Node(
            package='pupil_centre',
            node_executable='pupil_centre',
            node_name='pupil_centre',
            node_namespace='',
            output='screen',
            parameters=[config_pupil_centre],
            remappings=[('camera/aligned_depth_to_color/image_raw',
                         'camera/aligned_depth_to_color/image_raw'),
                        ('camera/aligned_depth_to_color/camera_info',
                         'camera/aligned_depth_to_color/camera_info'),
                        ('eye_regions', 'pupil_centre/eye_regions'),
                        ('pupil_centres', 'pupil_centre/pupil_centres')],
        ),
    ])
