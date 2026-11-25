"""Logger ROS 2: reutiliza el nodo principal y su mapeo, sin cargar acciones."""

import rclpy
from rclpy.executors import ExternalShutdownException

from .nodo_mando_ir import NodoMandoIR


def main(args=None):
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = NodoMandoIR(solo_logger=True)
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
