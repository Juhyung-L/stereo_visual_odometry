import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('visual_odometry')

    rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    delcare_rviz_config_file = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_config_file_path,
        description='Path to RViz config file'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    vo_node = Node(
        package='visual_odometry',
        executable='main',
        name='visual_odometry',
        output='screen'
    )

    return LaunchDescription([
        delcare_rviz_config_file,
        rviz_node,
        vo_node
    ])