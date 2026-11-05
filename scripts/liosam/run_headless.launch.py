"""LIO-SAM 无界面启动（不启 RViz，适合跑 bag / 服务器）。

与上游 `run.launch.py` 的区别：
    - 不启动 rviz2（回放评测时不需要，且会抢 GPU）
    - params_file 可覆盖，便于用项目内的 config/liosam_mid360.yaml

用法：
    ros2 launch scripts/liosam/run_headless.launch.py \
        params_file:=$HOME/slam_benchmark/config/liosam_mid360.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')

    params_file = LaunchConfiguration('params_file')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'params.yaml'),
            description='LIO-SAM 参数文件路径'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'rviz_cfg',
            default_value=os.path.join(share_dir, 'config', 'rviz2.rviz'),
            description='RViz 配置文件路径'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom',
             arguments='0 0 0 0 0 0 map odom'.split(' '),
             output='log'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='log',
             parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}]),

        Node(package='lio_sam', executable='lio_sam_imuPreintegration',
             name='lio_sam_imuPreintegration', parameters=[params_file],
             output='screen'),

        Node(package='lio_sam', executable='lio_sam_imageProjection',
             name='lio_sam_imageProjection', parameters=[params_file],
             output='screen'),

        Node(package='lio_sam', executable='lio_sam_featureExtraction',
             name='lio_sam_featureExtraction', parameters=[params_file],
             output='screen'),

        Node(package='lio_sam', executable='lio_sam_mapOptimization',
             name='lio_sam_mapOptimization', parameters=[params_file],
             output='screen'),

        # RViz 单独一个进程：它崩了不影响算法继续跑
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz')),
             output='log'),
    ])
