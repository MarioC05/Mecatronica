"""Integración ROS 2 del servidor y la TUI, sin abrir hardware."""
import sys
import time

import rclpy
from std_msgs.msg import Empty, Float64

from eltronico_common import servos_ros2 as ros


def test_microservo_y_watchdog_por_topics(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, 'argv', ['servidor', '--simulate', '--config', str(tmp_path/'servos.ini')])
    args = ros.app.parse_args()
    rclpy.init()
    servidor = ros.DeviceNode(args)
    cliente = rclpy.create_node('prueba_tui')
    remoto = ros.RemoteControl(cliente, args)
    micro = ros.RemoteMicroServo(cliente, remoto)
    ejecutor = rclpy.executors.SingleThreadedExecutor()
    ejecutor.add_node(servidor)
    ejecutor.add_node(cliente)

    def esperar(condicion, timeout=3):
        fin = time.monotonic() + timeout
        while time.monotonic() < fin:
            remoto.last_ui = time.monotonic()
            ejecutor.spin_once(timeout_sec=.02)
            if condicion():
                return
        assert condicion()

    try:
        esperar(lambda: remoto.received > 0 and micro.publisher.get_subscription_count() > 0)
        assert not servidor.microservo.active
        esperar(lambda: servidor.ctrl.last_ui > servidor.ctrl.last_command)
        micro.command(135)
        esperar(lambda: micro.active and micro.angle == 135)
        micro.publisher.publish(Float64(data=float('nan')))
        micro.publisher.publish(Float64(data=181.0))
        for _ in range(10):
            ejecutor.spin_once(timeout_sec=.02)
        assert servidor.microservo.angle == 135
        micro.stop()
        esperar(lambda: not servidor.microservo.active)
        micro.command(90)
        esperar(lambda: servidor.microservo.active)
        # Sin heartbeat, el watchdog desactiva PCA4.
        remoto.last_ui = time.monotonic() - 10
        fin = time.monotonic() + 1
        while time.monotonic() < fin:
            ejecutor.spin_once(timeout_sec=.02)
        assert not servidor.microservo.active
        assert not servidor.ctrl.armed
        assert not args.config.exists()  # No se guardan neutros automáticamente.
    finally:
        servidor.close()
        ejecutor.shutdown()
        servidor.destroy_node()
        cliente.destroy_node()
        rclpy.shutdown()
