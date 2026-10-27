from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('ustrov_status_display'),
                                  'config', 'status_display.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(package='ustrov_status_display', executable='status_display_node',
             name='ustrov_status_display', output='screen',
             parameters=[LaunchConfiguration('config_file')]),
    ])
