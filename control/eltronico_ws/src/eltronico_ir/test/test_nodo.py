"""Prueba de integración con rclpy: servidores falsos, scancodes y ir/simular."""
import json
import os
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from eltronico_ir.nodo_mando_ir import NodoMandoIR

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')


@pytest.fixture
def entorno():
    rclpy.init()
    nodo = NodoMandoIR(parameter_overrides=[
        Parameter('archivo_mapeo', value=os.path.join(CONFIG, 'mapeo_mando.txt')),
        Parameter('archivo_acciones', value=os.path.join(CONFIG, 'acciones.yaml')),
        Parameter('dispositivo', value='ninguno'),
    ])
    aux = rclpy.create_node('aux_prueba')
    llamadas = []

    def parar(req, res):
        llamadas.append('parar')
        res.success, res.message = True, 'neutro'
        return res

    def obstaculo(req, res):
        llamadas.append(('obstaculo', req.data))
        res.success = True
        return res

    aux.create_service(Trigger, '/eltronico/parar', parar)
    aux.create_service(SetBool, '/eltronico/proteccion_obstaculo', obstaculo)
    rec = {'boton': [], 'evento': [], 'resultado': [], 'orden': []}
    aux.create_subscription(String, 'ir/boton', lambda m: rec['boton'].append(m.data), 10)
    aux.create_subscription(String, 'ir/evento',
                            lambda m: rec['evento'].append(json.loads(m.data)), 10)
    aux.create_subscription(String, 'ir/resultado',
                            lambda m: rec['resultado'].append(json.loads(m.data)), 10)
    aux.create_subscription(String, '/eltronico/orden_mando',
                            lambda m: rec['orden'].append(m.data), 10)
    pub = aux.create_publisher(String, 'ir/simular', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(nodo)
    ex.add_node(aux)

    def girar(segundos=0.5):
        fin = time.monotonic() + segundos
        while time.monotonic() < fin:
            ex.spin_once(timeout_sec=0.02)

    # Esperar al descubrimiento
    fin = time.monotonic() + 5
    while time.monotonic() < fin and not (
            nodo.clientes['/eltronico/parar'].service_is_ready() and pub.get_subscription_count()
            and nodo.publicadores['/eltronico/orden_mando'].get_subscription_count()):
        ex.spin_once(timeout_sec=0.05)

    yield nodo, pub, girar, llamadas, rec
    ex.shutdown()
    aux.destroy_node()
    nodo.destroy_node()
    rclpy.shutdown()


def test_servicio_trigger_por_simular(entorno):
    nodo, pub, girar, llamadas, rec = entorno
    pub.publish(String(data='0x0043'))     # PLAY/PAUSE
    girar()
    assert llamadas == ['parar']
    assert rec['boton'] == ['BTN_PLAY']
    assert rec['resultado'][-1] == {'boton': 'BTN_PLAY', 'tipo': 'servicio',
                                    'destino': '/eltronico/parar', 'ok': True,
                                    'mensaje': 'neutro'}


def test_setbool_y_repeticion_ignorada(entorno):
    nodo, pub, girar, llamadas, rec = entorno
    nodo.procesar_scancode(0x0015, 10.00)    # +
    nodo.procesar_scancode(0x0015, 10.11)    # mantenido: no repite el servicio
    nodo.procesar_scancode(0x0007, 10.50)    # -
    girar()
    assert llamadas == [('obstaculo', True), ('obstaculo', False)]
    assert rec['boton'] == ['BTN_MAS', 'BTN_MENOS']
    assert [e['repeticion'] for e in rec['evento']] == [False, True, False]


def test_topic_con_repeticion(entorno):
    nodo, pub, girar, llamadas, rec = entorno
    for t in (20.00, 20.11, 20.22):          # 2 -> adelante, mantenido
        nodo.procesar_scancode(0x0018, t)
    nodo.procesar_scancode(0x0018, 21.00)    # pulsación nueva tras soltar
    girar()
    assert rec['orden'] == ['adelante'] * 4
    assert rec['boton'] == ['BTN_2', 'BTN_2']


def test_direccion_ajena_y_desconocido(entorno):
    nodo, pub, girar, llamadas, rec = entorno
    nodo.procesar_scancode(0x1243, 30.0)     # PLAY de otro mando: no para
    nodo.procesar_scancode(0x0099, 31.0)     # código desconocido
    girar()
    assert llamadas == []
    assert [e['reconocido'] for e in rec['evento']] == [False, False]


def test_servicio_no_disponible(entorno):
    nodo, pub, girar, llamadas, rec = entorno
    nodo.procesar_scancode(0x0046, 40.0)     # CH -> /eltronico/armar (no existe)
    girar(0.2)
    assert rec['resultado'][-1]['ok'] is False
    assert rec['resultado'][-1]['mensaje'] == 'servicio no disponible'
