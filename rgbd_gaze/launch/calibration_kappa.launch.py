"""Launch calibration of kappa for RGB-D gaze"""

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
    user = LaunchConfiguration('user', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'users', 'default.yaml'))
    config_rgbd_gaze_calibration_kappa = LaunchConfiguration('config_rgbd_gaze_calibration_kappa', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'calibration', 'rgbd_gaze_calibration_kappa.yaml'))
    config_rviz2 = LaunchConfiguration('config_rviz2', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'calibration', 'rviz2.rviz'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'user',
            default_value=user,
            description='Path to config for user of RGB-D gaze'),
        DeclareLaunchArgument(
            'config_rgbd_gaze_calibration_kappa',
            default_value=config_rgbd_gaze_calibration_kappa,
            description='Path to config for RGB-D Gaze calibration for eyeball'),
        DeclareLaunchArgument(
            'config_rviz2',
            default_value=config_rviz2,
            description='Path to config for RViz2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('rgbd_gaze'), 'launch',
                              'rgbd_gaze.launch.py')]),
            launch_arguments=[('user', user),
                              ('config_rviz2', config_rviz2)]
        ),

        Node(
            package='rgbd_gaze',
            node_executable='calibration_kappa',
            node_name='rgbd_gaze_calibration_kappa',
            node_namespace='',
            output='screen',
            parameters=[config_rgbd_gaze_calibration_kappa],
            remappings=[('head_pose', 'rgbd_gaze/head_pose'),
                        ('optical_axes', 'rgbd_gaze/optical_axes'),
                        ('scene_point', 'rgbd_gaze/scene_point'),
                        ('visualisation_markers', 'rgbd_gaze/visualisation_markers')],
        ),
    ])
