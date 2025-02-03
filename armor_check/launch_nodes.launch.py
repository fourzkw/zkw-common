import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    package_dir = get_package_share_directory('armor_check')

    # 定义节点
    ori_node = Node(
        package='armor_check',
        executable='ori_node',
        name='ori_node',
        output='screen'
    )

    yolo_node = Node(
        package='armor_check',
        executable='yolo_node',
        name='yolo_node',
        output='screen'
    )

    tra_node = Node(
        package='armor_check',
        executable='tra_node',
        name='tra_node',
        output='screen'
    )

    # 返回LaunchDescription对象
    return LaunchDescription([
        ori_node,
        yolo_node,
        tra_node
    ])