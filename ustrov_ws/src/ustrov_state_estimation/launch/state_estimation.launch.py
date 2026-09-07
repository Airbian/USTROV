from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """单独启动 EKF，便于在仿真或实机话题上逐项检查。"""
    vehicle_name = LaunchConfiguration('vehicle_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument('vehicle_name', default_value='uuv00'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            Node(
                package='ustrov_state_estimation',
                executable='estimator',
                namespace=vehicle_name,
                name='state_estimator',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            ),
        ]
    )
