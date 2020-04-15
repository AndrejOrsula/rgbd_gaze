"""Launch calibration for RGB-D gaze"""

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
        'rgbd_gaze'), 'config', 'openface', 'openface_separate_calibration.yaml'))
    config_openface_rgbd_head_pose = LaunchConfiguration('config_openface_rgbd_head_pose', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'openface', 'openface_rgbd_head_pose.yaml'))
    config_rgbd_gaze_calibration = LaunchConfiguration('config_rgbd_gaze_calibration', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'calibration', 'rgbd_gaze_calibration.yaml'))
    config_rviz2 = LaunchConfiguration('config_rviz2', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'calibration', 'rviz2.yaml'))

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
            'config_rgbd_gaze_calibration',
            default_value=config_rgbd_gaze_calibration,
            description='Path to config for RGB-D Gaze calibration'),
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
                        ('openface/head_pose', 'rgbd_gaze/head_pose_raw'),
                        ('openface/eye_landmarks_visible', 'rgbd_gaze/eye_landmarks_visible')],
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
            package='openface_rgbd_eyelid_contour',
            node_executable='openface_rgbd_eyelid_contour',
            node_name='openface_rgbd_eyelid_contour',
            node_namespace='',
            output='screen',
            parameters=[],
            remappings=[('camera/aligned_depth_to_color/image_raw',
                         'camera/aligned_depth_to_color/image_raw'),
                        ('camera/aligned_depth_to_color/camera_info',
                         'camera/aligned_depth_to_color/camera_info'),
                        ('openface/eye_landmarks_visible',
                         'rgbd_gaze/eye_landmarks_visible'),
                        ('openface/eyelid_contours', 'rgbd_gaze/eyelid_contours')],
        ),

        Node(
            package='rgbd_gaze',
            node_executable='calibration',
            node_name='rgbd_gaze_calibration',
            node_namespace='',
            output='screen',
            parameters=[config_rgbd_gaze_calibration],
            remappings=[('head_pose', 'rgbd_gaze/head_pose'),
                        ('eyelid_contours', 'rgbd_gaze/eyelid_contours'),
                        ('visualisation_markers', 'rgbd_gaze/visualisation_markers'),
                        ('eyelid_contours_cumulative', 'rgbd_gaze/eyelid_contours_cumulative')],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('rgbd_gaze'), 'launch',
                              'rviz2.launch.py')]),
            launch_arguments=[('config_rviz2', config_rviz2)]
        ),
    ])
