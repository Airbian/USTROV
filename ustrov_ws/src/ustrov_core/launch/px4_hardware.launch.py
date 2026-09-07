from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 从安装后的 share 目录读取默认配置，源码和安装空间都可正常使用。
    package_path = get_package_share_path('ustrov_core')
    default_actuator_config = str(
        package_path / 'config' / 'px4_actuator_bridge.yaml'
    )
    default_sensor_config = str(
        package_path / 'config' / 'px4_sensor_bridge.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_name',
            default_value='',
            description='ROS namespace for the USTROV controller and bridge.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_actuator_config,
            description='PX4 actuator bridge parameter file.',
        ),
        DeclareLaunchArgument(
            'sensor_config_file',
            default_value=default_sensor_config,
            description='PX4 sensor bridge parameter file.',
        ),
        DeclareLaunchArgument(
            'px4_topic_prefix',
            default_value='/fmu',
            description='PX4 DDS prefix, for example /fmu or /uuv00/fmu.',
        ),
        Node(
            # 执行器桥接负责 Offboard、Arm/Disarm 和电机命令安全看门狗。
            package='ustrov_core',
            executable='px4_actuator_bridge',
            name='px4_actuator_bridge',
            namespace=LaunchConfiguration('vehicle_name'),
            parameters=[
                LaunchConfiguration('config_file'),
                {'px4_topic_prefix': LaunchConfiguration('px4_topic_prefix')},
            ],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            # 传感器桥接只读取 PX4 数据，不会向飞控发送控制命令。
            package='ustrov_core',
            executable='px4_sensor_bridge',
            name='px4_sensor_bridge',
            namespace=LaunchConfiguration('vehicle_name'),
            parameters=[
                LaunchConfiguration('sensor_config_file'),
                {'px4_topic_prefix': LaunchConfiguration('px4_topic_prefix')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
