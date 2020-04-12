"""Launch RGB-D gaze"""

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
        'rgbd_gaze'), 'config', 'openface', 'openface_separate.yaml'))
    config_openface_rgbd_head_pose = LaunchConfiguration('config_openface_rgbd_head_pose', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'openface', 'openface_rgbd_head_pose.yaml'))
    config_pupil_centre = LaunchConfiguration('config_pupil_centre', default=os.path.join(get_package_share_directory(
        'pupil_centre'), 'config', 'pupil_centre.yaml'))
    config_rgbd_gaze = LaunchConfiguration('config_rgbd_gaze', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'rgbd_gaze.yaml'))
    user = LaunchConfiguration('user', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'users', 'default.yaml'))
    config_rviz2 = LaunchConfiguration('config_rviz2', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'rviz2.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_openface_separate',
            default_value=config_openface_separate,
            description='Path to config for openface'),
        DeclareLaunchArgument(
            'config_openface_rgbd_head_pose',
            default_value=config_openface_rgbd_head_pose,
            description='Path to config for RGB-D head pose'),
        DeclareLaunchArgument(
            'config_pupil_centre',
            default_value=config_pupil_centre,
            description='Path to config for pupil centre localisation'),
        DeclareLaunchArgument(
            'config_rgbd_gaze',
            default_value=config_rgbd_gaze,
            description='Path to config for RGB-D Gaze'),
        DeclareLaunchArgument(
            'user',
            default_value=user,
            description='Path to config for RGB-D Gaze user'),
        DeclareLaunchArgument(
            'config_rviz2',
            default_value=config_rviz2,
            description='Path to config for RViz2'),

        Node(
            package='openface',
            node_executable='openface_separate',
            node_name='openface_separate',
            node_namespace='',
            output='screen',
            parameters=[config_openface_separate],
            remappings=[('camera/image_raw', 'camera/color/image_raw'),
                        ('camera/camera_info', 'camera/color/camera_info'),
                        ('openface/landmarks_visible',
                         'rgbd_gaze/landmarks_visible'),
                        ('openface/head_pose', 'rgbd_gaze/head_pose_raw')],
        ),

        Node(
            package='openface_eye_region',
            node_executable='openface_eye_region',
            node_name='openface_eye_region',
            node_namespace='',
            output='screen',
            remappings=[('openface/landmarks_visible', 'rgbd_gaze/landmarks_visible'),
                        ('openface/eye_regions', 'rgbd_gaze/eye_regions')],
        ),

        Node(
            package='openface_rgbd_head_pose',
            node_executable='openface_rgbd_head_pose',
            node_name='openface_rgbd_head_pose',
            node_namespace='',
            output='screen',
            parameters=[config_openface_rgbd_head_pose],
            remappings=[('camera/aligned_depth_to_color/image_raw',
                         'camera/aligned_depth_to_color/image_raw'),
                        ('camera/aligned_depth_to_color/camera_info',
                         'camera/aligned_depth_to_color/camera_info'),
                        ('openface/landmarks_visible',
                         'rgbd_gaze/landmarks_visible'),
                        ('openface/head_pose', 'rgbd_gaze/head_pose_raw'),
                        ('openface/rgbd_head_pose', 'rgbd_gaze/head_pose')],
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
                        ('eye_regions', 'rgbd_gaze/eye_regions'),
                        ('pupil_centres', 'rgbd_gaze/pupil_centres')],
        ),

        Node(
            package='rgbd_gaze',
            node_executable='rgbd_gaze',
            node_name='rgbd_gaze',
            node_namespace='',
            output='screen',
            parameters=[config_rgbd_gaze, user],
            remappings=[('head_pose', 'rgbd_gaze/head_pose'),
                        ('pupil_centres', 'rgbd_gaze/pupil_centres'),
                        ('visualisation_markers', 'rgbd_gaze/visualisation_markers')],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('rgbd_gaze'), 'launch',
                              'rviz2.launch.py')]),
            launch_arguments=[('config_rviz2', config_rviz2)]
        ),
    ])
