"""
ROS2多设备启动文件

使用ROS2的launch机制，为每个设备启动一个独立的节点。
每个节点可以同时包含多个话题和服务。

从配置文件中动态读取设备列表
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def load_device_names_from_config(config_path):
    """
    从配置文件中读取所有设备名称

    Args:
        config_path: 配置文件路径（字符串）

    Returns:
        设备名称列表
    """
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        device_names = []
        if 'device_nodes' in config:
            for node_config in config['device_nodes']:
                if 'device' in node_config:
                    device_names.append(node_config['device'])

        return device_names
    except Exception as e:
        print(f"警告：无法读取配置文件 {config_path}: {e}")
        print("将使用空设备列表")
        return []


def generate_device_nodes(context, *args, **kwargs):
    """
    动态生成设备节点列表

    Args:
        context: Launch上下文

    Returns:
        节点列表
    """
    controller_config_path = context.launch_configurations.get('controller_config', None)

    if controller_config_path is None:
        pkg_share = FindPackageShare('inspire_control_ros2')
        default_path = PathJoinSubstitution([
            pkg_share, 'config', 'ros2_controller_config.yaml'
        ])
        controller_config_path = default_path.perform(context)
    elif hasattr(controller_config_path, 'perform'):
        controller_config_path = controller_config_path.perform(context)

    devices = load_device_names_from_config(controller_config_path)

    if not devices:
        print("警告：未找到任何设备配置，将不启动任何节点")
        return []

    print(f"从配置文件读取到 {len(devices)} 个设备: {', '.join(devices)}")

    device_config_path = context.launch_configurations.get('device_config', None)
    if device_config_path is None:
        pkg_share = FindPackageShare('inspire_control_ros2')
        default_path = PathJoinSubstitution([
            pkg_share, 'config', 'device_protocol_config.yaml'
        ])
        device_config_path = default_path.perform(context)
    elif hasattr(device_config_path, 'perform'):
        device_config_path = device_config_path.perform(context)

    nodes = []

    for device_name in devices:
        node = Node(
            package='inspire_control_ros2',
            executable='inspire_control_node',
            name=f'{device_name}_node',
            namespace=device_name,
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=[
                '--device-config', device_config_path,
                '--controller-config', controller_config_path,
                '--device', device_name,
            ],
            output='screen',
            emulate_tty=True,
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    """
    生成启动描述

    为配置文件中定义的每个设备创建一个独立的ROS2节点。
    每个节点通过--device参数指定对应的设备名称。
    设备列表从配置文件中动态读取，无需硬编码。
    """

    pkg_share = FindPackageShare('inspire_control_ros2')

    device_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'device_protocol_config.yaml'
    ])
    controller_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'ros2_controller_config.yaml'
    ])

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

    device_nodes_action = OpaqueFunction(function=generate_device_nodes)

    return LaunchDescription([
        device_config_arg,
        controller_config_arg,
        LogInfo(msg='启动多设备 Inspire 控制节点...'),
        device_nodes_action,
        LogInfo(msg='所有设备节点已启动'),
    ])
