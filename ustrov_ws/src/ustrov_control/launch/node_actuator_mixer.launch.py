from ament_index_python.packages import get_package_share_path
from ustrov_common.launch_helper import declare_use_sim_time
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def declare_launch_args(launch_description: LaunchDescription):
    # 实机默认使用系统时间；仿真时可显式传入 use_sim_time:=true。
    declare_use_sim_time(launch_description=launch_description, default='false')

    default_path = (
        get_package_share_path('ustrov_control')
        / 'config/actuator_mixer/ustrov_default.yaml'
    )
    action = DeclareLaunchArgument(
        name='mixer_path',
        default_value=str(default_path),
        description='Path to mixer configuration .yaml file.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='vehicle_name',
        default_value='',
        description='Optional ROS namespace for this vehicle.',
    )
    launch_description.add_action(action)


def add_node(launch_description: LaunchDescription):
    action = Node(
        package='ustrov_control',
        executable='actuator_mixer_node',
        namespace=LaunchConfiguration('vehicle_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('mixer_path'),
        ],
        output='screen',
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_node(launch_description=launch_description)
    return launch_description
