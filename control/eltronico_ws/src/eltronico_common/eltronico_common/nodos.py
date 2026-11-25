"""Puntos de entrada ROS 2; separa argumentos de la TUI y argumentos ROS."""
import importlib
import sys

from rclpy.utilities import remove_ros_args


def _ejecutar(modo):
    servos_ros2 = importlib.import_module('eltronico_common.servos_ros2')
    originales = list(sys.argv)
    sys.argv = [sys.argv[0], modo] + remove_ros_args(sys.argv)[1:]
    try:
        servos_ros2.main(ros_args=originales)
    finally:
        sys.argv = originales


def servidor():
    """Nodo eltronico_devices: hardware (o --simulate) y topics de estado/órdenes."""
    _ejecutar('server')


def tui():
    """Nodo eltronico_tui: interfaz curses que manda órdenes por topics al servidor."""
    _ejecutar('tui')
