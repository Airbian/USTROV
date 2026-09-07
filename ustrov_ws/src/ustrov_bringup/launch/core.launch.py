from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    core_share = get_package_share_path('ustrov_core')
    control_share = get_package_share_path('ustrov_control')

    core_launch_path = core_share / 'launch' / 'px4_hardware.launch.py'
    mixer_launch_path = (
        control_share / 'launch' / 'node_actuator_mixer.launch.py'
    )
    start_agent = LaunchConfiguration('start_agent')
    agent_device = LaunchConfiguration('agent_device')
    agent_baudrate = LaunchConfiguration('agent_baudrate')
    agent_verbose = LaunchConfiguration('agent_verbose')
    vehicle_name = LaunchConfiguration('vehicle_name')
    px4_topic_prefix = LaunchConfiguration('px4_topic_prefix')
    actuator_config_file = LaunchConfiguration('config_file')
    sensor_config_file = LaunchConfiguration('sensor_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    launch_arguments = [
        DeclareLaunchArgument(
            'start_agent',
            default_value='true',
            description=(
                'Start MicroXRCEAgent. Set false if another Agent already '
                'owns the serial port.'
            ),
        ),
        DeclareLaunchArgument(
            'agent_device',
            default_value='/dev/ttyPX4',
            description='Serial device already mapped into the container.',
        ),
        DeclareLaunchArgument(
            'agent_baudrate',
            default_value='115200',
            description='PX4 Micro XRCE-DDS serial baud rate.',
        ),
        DeclareLaunchArgument(
            'agent_verbose',
            default_value='4',
            description='MicroXRCEAgent verbosity level.',
        ),
        DeclareLaunchArgument(
            'vehicle_name',
            default_value='',
            description='Optional common ROS namespace for bridge and mixer.',
        ),
        DeclareLaunchArgument(
            'px4_topic_prefix',
            default_value='/fmu',
            description='PX4 DDS topic prefix, normally /fmu.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=str(
                core_share / 'config' / 'px4_actuator_bridge.yaml'
            ),
            description='PX4 actuator bridge parameter file.',
        ),
        DeclareLaunchArgument(
            'sensor_config_file',
            default_value=str(
                core_share / 'config' / 'px4_sensor_bridge.yaml'
            ),
            description='PX4 sensor bridge parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Hardware bringup normally uses system time.',
        ),
    ]

    # Agent 只负责 PX4 与 ROS 2 DDS 网络之间的串口传输。
    micro_xrce_agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent',
            'serial',
            '--dev',
            agent_device,
            '-b',
            agent_baudrate,
            '-v',
            agent_verbose,
        ],
        name='micro_xrce_agent',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_agent),
    )

    # ustrov_core 同时启动执行器桥和只读传感器桥。
    # 前者负责安全控制链，后者把 PX4 IMU/气压计转换为标准 ROS 消息。
    px4_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(core_launch_path)),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'px4_topic_prefix': px4_topic_prefix,
            'config_file': actuator_config_file,
            'sensor_config_file': sensor_config_file,
        }.items(),
    )

    actuator_mixer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(mixer_launch_path)),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        launch_arguments
        + [
            micro_xrce_agent,
            px4_bridge,
            actuator_mixer,
        ]
    )
