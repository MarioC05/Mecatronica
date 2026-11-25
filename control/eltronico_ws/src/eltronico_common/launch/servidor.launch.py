"""Lanza el servidor de dispositivos. La TUI necesita terminal: `ros2 run eltronico_common tui`."""
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodo(context):
    argumentos = []
    if LaunchConfiguration('simular').perform(context).lower() in ('true', '1', 'si', 'sí'):
        argumentos.append('--simulate')
    argumentos += shlex.split(LaunchConfiguration('opciones').perform(context))
    return [Node(package='eltronico_common', executable='servidor',
                 arguments=argumentos, output='screen', emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simular', default_value='false',
                              description='true: sin hardware (--simulate)'),
        DeclareLaunchArgument('opciones', default_value='',
                              description="Opciones de servos_tui, p. ej. \"--channels 0 1 --limit 0.5\""),
        OpaqueFunction(function=_nodo),
    ])
