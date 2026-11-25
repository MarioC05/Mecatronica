"""Hilo de lectura evdev con un módulo evdev falso (sin receptor real)."""
import json
import os
import sys
import time
import types

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

from eltronico_ir.nodo_mando_ir import NodoMandoIR

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')


class Evento:
    def __init__(self, tipo, codigo, valor, t):
        self.type, self.code, self.value, self._t = tipo, codigo, valor, t

    def timestamp(self):
        return self._t


def test_lee_scancodes_del_kernel(monkeypatch):
    lectura, escritura = os.pipe()
    pendientes = [
        Evento(0x04, 0x04, 0x0018, 100.00),   # MSC_SCAN botón 2
        Evento(0x00, 0x00, 0, 100.00),        # EV_SYN: se ignora
        Evento(0x04, 0x04, 0x0018, 100.11),   # repetición del kernel
        Evento(0x04, 0x04, 0x0043, 101.00),   # PLAY
    ]

    class Dispositivo:
        def __init__(self, ruta):
            self.path, self.name, self.fd = ruta, 'gpio_ir_recv', lectura

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def close(self):
            pass

        def read(self):
            os.read(lectura, 64)
            eventos, pendientes[:] = list(pendientes), []
            return eventos

    monkeypatch.setitem(sys.modules, 'evdev', types.SimpleNamespace(InputDevice=Dispositivo))
    rclpy.init()
    try:
        nodo = NodoMandoIR(parameter_overrides=[
            Parameter('archivo_mapeo', value=os.path.join(CONFIG, 'mapeo_mando.txt')),
            Parameter('archivo_acciones', value=os.path.join(CONFIG, 'acciones.yaml')),
            Parameter('dispositivo', value='/dev/input/event99'),
        ])
        aux = rclpy.create_node('aux_evdev')
        eventos = []
        aux.create_subscription(String, 'ir/evento',
                                lambda m: eventos.append(json.loads(m.data)), 10)
        ex = SingleThreadedExecutor()
        ex.add_node(nodo)
        ex.add_node(aux)
        fin = time.monotonic() + 5
        while time.monotonic() < fin and not nodo.pub_evento.get_subscription_count():
            ex.spin_once(timeout_sec=0.05)
        os.write(escritura, b'x')
        fin = time.monotonic() + 3
        while time.monotonic() < fin and len(eventos) < 3:
            ex.spin_once(timeout_sec=0.05)
        assert [(e['boton'], e['repeticion']) for e in eventos] == [
            ('BTN_2', False), ('BTN_2', True), ('BTN_PLAY', False)]
        ex.shutdown()
        aux.destroy_node()
        nodo.destroy_node()
        assert not nodo.hilo.is_alive()
    finally:
        rclpy.shutdown()
        os.close(lectura)
        os.close(escritura)
