"""Comprueba que monitorizar la Pi4 no configura ni escribe sus GPIO."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import mock_open

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


@pytest.fixture
def entorno(monkeypatch):
    class Linea:
        def __init__(self, pin):
            self.pin = pin
            self.requests = []
            self.values = []
            self.busy = False
            self.released = False

        def name(self):
            return f'GPIO{self.pin}'

        def request(self, **kwargs):
            if self.busy:
                raise OSError('ocupado')
            self.requests.append(kwargs)

        def set_value(self, value):
            self.values.append(value)

        def get_value(self):
            return self.values[-1] if self.values else self.requests[-1]['default_val']

        def release(self):
            self.released = True

    lineas = [Linea(i) for i in range(28)]
    chip = SimpleNamespace(label=lambda: 'pinctrl-bcm2711', num_lines=lambda: 28,
                           get_line=lambda i: lineas[i], close=lambda: None)
    monkeypatch.setitem(sys.modules, 'gpiod', SimpleNamespace(Chip=lambda p: chip, LINE_REQ_DIR_OUT=3))
    ruta = Path(__file__).parents[1] / 'gpio_topics/node.py'
    spec = importlib.util.spec_from_file_location('gpio_topics_prueba', ruta)
    modulo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modulo)
    monkeypatch.setattr(modulo.glob, 'glob', lambda p: ['/dev/gpiochip0'])
    monkeypatch.setattr(modulo, 'open', mock_open(), raising=False)

    class Registros(bytearray):
        def close(self):
            pass

    monkeypatch.setattr(modulo.mmap, 'mmap', lambda *a, **k: Registros(4096))
    rclpy.init()
    nodos = []

    def crear(**kwargs):
        nodo = modulo.GpioTopicsNode(**kwargs)
        nodos.append(nodo)
        return nodo

    yield crear, lineas
    for nodo in nodos:
        nodo.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('nivel', [False, True])
def test_solo_escribe_tras_set_con_el_valor_solicitado(entorno, nivel):
    crear, lineas = entorno
    nodo = crear()
    nodo._publish_all()
    assert not nodo.output_pins
    assert all(not l.requests and not l.values for l in lineas)
    nodo._set_pin(22, Bool(data=nivel))
    assert lineas[22].requests == [dict(consumer='ros2_gpio_topics', type=3, default_val=int(nivel))]
    assert nodo.output_pins == {22}
    assert all(not l.requests for l in lineas if l.pin != 22)
    nodo._set_pin(22, Bool(data=not nivel))
    assert len(lineas[22].requests) == 1
    assert lineas[22].values == [int(not nivel)]


def test_no_reclama_un_pin_ocupado_ni_un_bus(entorno):
    crear, lineas = entorno
    nodo = crear()
    lineas[18].busy = True
    nodo._set_pin(18, Bool(data=True))
    nodo._set_pin(2, Bool(data=True))
    assert not nodo.output_pins
    assert 18 in nodo.monitor_pins
    assert not lineas[18].released
    assert not lineas[2].requests


def test_lista_vacia_no_permite_escribir(entorno):
    crear, lineas = entorno
    nodo = crear(parameter_overrides=[Parameter('output_pins', Parameter.Type.INTEGER_ARRAY, [])])
    nodo._set_pin(22, Bool(data=True))
    assert not lineas[22].requests
