"""Inicia el nodo C++ Lifecycle por separado, con tiempo de simulación."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('eltronico')
    return LaunchDescription([
        Node(
            package='eltronico', executable='main', name='eltronico',
            parameters=[os.path.join(pkg_dir, 'config', 'params.yaml'), {'use_sim_time': True}],
            output='screen',
        ),
    ])
