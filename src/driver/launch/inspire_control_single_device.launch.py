"""
ROS2单设备启动文件

用于启动单个设备的控制节点。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    生成启动描述

    启动单个设备的控制节点，设备名称通过launch参数指定。
    """

    pkg_share = FindPackageShare('inspire_control_ros2')

    device_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'device_protocol_config.yaml'
    ])
    controller_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'ros2_controller_config.yaml'
    ])

    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='hand_left',
        description='设备名称（如hand_left、hand_right）'
    )

    device_config_arg = DeclareLaunchArgument(
        'device_config',
        default_value=device_config_path,
        description='设备协议配置文件路径'
    )

    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=controller_config_path,
        description='ROS2控制器配置文件路径'
    )

    node = Node(
        package='inspire_control_ros2',
        executable='inspire_control_node',
        name=[LaunchConfiguration('device_name'), '_node'],
        namespace=LaunchConfiguration('device_name'),
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=[
            '--device-config', LaunchConfiguration('device_config'),
            '--controller-config', LaunchConfiguration('controller_config'),
            '--device', LaunchConfiguration('device_name'),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        device_name_arg,
        device_config_arg,
        controller_config_arg,
        LogInfo(msg=['启动 Inspire 设备节点: ', LaunchConfiguration('device_name')]),
        node,
    ])
