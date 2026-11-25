"""Nodo ROS 2 (Jazzy) del mando IR de Eltronico en la Raspberry Pi 4B.

El receptor IR (VS1838B/KY-022) va al GPIO de la Pi; el overlay ``gpio-ir`` del
kernel decodifica NEC y este nodo lee los scancodes por evdev. Con cada trama:

* publica ``ir/boton`` (std_msgs/String, p. ej. ``BTN_PLAY``) en cada pulsación nueva;
* publica ``ir/evento`` (std_msgs/String, JSON) con todas las tramas, incluidas
  las repeticiones y los códigos desconocidos;
* ejecuta la acción configurada en el YAML: llamar a un servicio o publicar en un
  topic, con cualquier tipo de interfaz;
* publica ``ir/resultado`` (std_msgs/String, JSON) con el resultado de cada acción.

Las repeticiones (botón mantenido) sólo disparan acciones marcadas con
``repetir: true``. ``ir/simular`` admite scancodes para probar sin hardware.
"""

from __future__ import annotations

import json
import os
import queue
import select
import threading
from contextlib import closing
from typing import Dict, Optional

from ament_index_python.packages import get_package_share_directory
from eltronico_ir import entrada_evdev
from eltronico_ir.configuracion import Accion, cargar_acciones, cargar_mapeo
from eltronico_ir.nec import TramaIR, analizar_simulacion, trama_desde_scancode
import rclpy
import rclpy.executors
from rclpy.node import Node
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service
from std_msgs.msg import String


class NodoMandoIR(Node):
    """Traduce pulsaciones del mando IR a topics y servicios."""

    def __init__(self, solo_logger=False, **kwargs):
        super().__init__('logger_ir' if solo_logger else 'mando_ir', **kwargs)
        self.solo_logger = solo_logger
        compartido = get_package_share_directory('eltronico_ir')
        # 'auto': busca gpio_ir_recv; '/dev/input/eventN'; 'ninguno': sólo ir/simular
        self.declare_parameter('dispositivo', 'auto')
        # Vacío: archivos de config/ instalados con el paquete
        self.declare_parameter('archivo_mapeo', '')
        self.declare_parameter('archivo_acciones', '')
        # -1: tomar la del archivo de mapeo; -2: aceptar cualquier dirección
        self.declare_parameter('direccion', -1)
        # El mismo scancode antes de este tiempo cuenta como botón mantenido
        # (NEC repite cada ~108 ms).
        self.declare_parameter('tiempo_mantenido', 0.2)

        p = self.get_parameter
        ruta_mapeo = (p('archivo_mapeo').value
                      or os.path.join(compartido, 'config', 'mapeo_mando.txt'))
        ruta_acciones = (p('archivo_acciones').value
                         or os.path.join(compartido, 'config', 'acciones.yaml'))
        direccion_archivo, self.botones = cargar_mapeo(ruta_mapeo)
        ids = {b.id for b in self.botones.values()}
        self.acciones = {} if solo_logger else cargar_acciones(ruta_acciones, ids)
        direccion = p('direccion').value
        self.direccion: Optional[int] = (direccion_archivo if direccion == -1
                                         else None if direccion == -2 else direccion)
        self.tiempo_mantenido = float(p('tiempo_mantenido').value)

        self.pub_boton = self.create_publisher(String, 'ir/boton', 10)
        self.pub_evento = self.create_publisher(String, 'ir/evento', 10)
        self.pub_resultado = self.create_publisher(String, 'ir/resultado', 10)

        self.clientes: Dict[str, object] = {}
        self.publicadores: Dict[str, object] = {}
        self.pendientes: Dict[str, int] = {}
        for accion in self.acciones.values():
            self._preparar(accion)

        self.ultimo_scancode: Optional[int] = None
        self.ultimo_instante = -1e9

        self.create_subscription(String, 'ir/simular', self._simular, 10)
        self.scancodes: queue.Queue = queue.Queue(maxsize=200)
        self.parar_hilo = threading.Event()
        self.hilo = None
        self.dispositivo = p('dispositivo').value.strip()
        if self.dispositivo.lower() in ('', 'ninguno', 'none'):
            self.dispositivo = ''
        if self.dispositivo:
            self.hilo = threading.Thread(target=self._leer_evdev, daemon=True)
            self.hilo.start()
            self.create_timer(0.01, self._vaciar_cola)

        self.get_logger().info(
            f'{len(self.botones)} botones, {len(self.acciones)} acciones, dirección '
            f'{"cualquiera" if self.direccion is None else f"0x{self.direccion:02X}"}, '
            f'entrada {self.dispositivo or "sólo topic ir/simular"}')

    # ------------------------------------------------------------ preparación

    def _preparar(self, accion: Accion):
        """Comprueba la interfaz y los campos al arrancar, no al pulsar."""
        if accion.tipo == 'servicio':
            tipo = get_service(accion.interfaz)
            set_message_fields(tipo.Request(), accion.campos)
            if accion.nombre not in self.clientes:
                self.clientes[accion.nombre] = self.create_client(tipo, accion.nombre)
        else:
            tipo = get_message(accion.interfaz)
            set_message_fields(tipo(), accion.campos)
            if accion.nombre not in self.publicadores:
                self.publicadores[accion.nombre] = self.create_publisher(tipo, accion.nombre, 10)

    # ------------------------------------------------------------ entrada

    def _abrir(self):
        try:
            import evdev  # sólo hace falta con el receptor real
        except ImportError:
            raise OSError('falta python3-evdev: sudo apt install python3-evdev') from None

        ruta = self.dispositivo
        if ruta == 'auto':
            ruta = entrada_evdev.buscar_dispositivo()
            if ruta is None:
                raise OSError('no hay dispositivo gpio_ir_recv; '
                              '¿falta dtoverlay=gpio-ir en config.txt?')
        protocolos = entrada_evdev.protocolos_activos(ruta)
        if entrada_evdev.nec_activo(protocolos) is False:
            self.get_logger().warn(
                f'NEC no está activo en el receptor (protocols: {protocolos}). '
                'Ejecutar: sudo ir-keytable -s rc0 -p nec')
        return evdev.InputDevice(ruta)

    def _leer_evdev(self):
        while not self.parar_hilo.is_set():
            try:
                disp = self._abrir()
                self.get_logger().info(f'Leyendo {disp.path} ({disp.name})')
                with closing(disp):
                    while not self.parar_hilo.is_set():
                        if not select.select([disp.fd], [], [], 0.1)[0]:
                            continue
                        for ev in disp.read():
                            if (ev.type == entrada_evdev.EV_MSC
                                    and ev.code == entrada_evdev.MSC_SCAN):
                                try:
                                    self.scancodes.put_nowait((ev.value & 0xFFFFFFFF,
                                                               ev.timestamp()))
                                except queue.Full:
                                    pass
            except OSError as exc:
                self.get_logger().warn(f'Receptor IR: {exc}; reintento en 2 s',
                                       throttle_duration_sec=10.0)
                self.parar_hilo.wait(2.0)

    def _vaciar_cola(self):
        while True:
            try:
                scancode, instante = self.scancodes.get_nowait()
            except queue.Empty:
                return
            self.procesar_scancode(scancode, instante)

    def _simular(self, msg: String):
        trama = analizar_simulacion(msg.data)
        if trama is None:
            self.get_logger().warn(f'ir/simular: «{msg.data}» no es un scancode (p. ej. 0x0043)')
            return
        self.procesar_trama(trama)

    # ------------------------------------------------------------ lógica

    def procesar_scancode(self, scancode: int, instante: float):
        """Marca como repetición el mismo scancode recibido poco después."""
        repeticion = (scancode == self.ultimo_scancode
                      and 0 <= instante - self.ultimo_instante <= self.tiempo_mantenido)
        self.ultimo_scancode, self.ultimo_instante = scancode, instante
        self.procesar_trama(trama_desde_scancode(scancode, repeticion))

    def procesar_trama(self, trama: TramaIR):
        boton = self.botones.get(trama.comando)
        direccion_ok = self.direccion is None or trama.direccion == self.direccion
        reconocido = boton is not None and direccion_ok
        evento = {
            'boton': boton.id if reconocido else None,
            'etiqueta': boton.etiqueta if boton else None,
            'comando': trama.comando,
            'direccion': trama.direccion,
            'scancode': trama.scancode,
            'repeticion': trama.repeticion,
            'reconocido': reconocido,
        }
        self.pub_evento.publish(String(data=json.dumps(evento, ensure_ascii=False)))
        if self.solo_logger:
            self.get_logger().info(
                f'{boton.etiqueta if reconocido else "DESCONOCIDO"} '
                f'({boton.id if reconocido else "sin mapeo"}) '
                f'comando=0x{trama.comando:02X}, dirección={trama.direccion}, '
                f'scancode={trama.scancode}'
                + (' [mantenido]' if trama.repeticion else ''))
        if not reconocido:
            if not trama.repeticion:
                self.get_logger().info(
                    f'Código no reconocido: comando 0x{trama.comando:02X}, '
                    f'dirección 0x{trama.direccion:02X}')
            return
        if not trama.repeticion:
            self.pub_boton.publish(String(data=boton.id))
        accion = self.acciones.get(boton.id)
        if accion and (not trama.repeticion or accion.repetir):
            self.ejecutar(accion)

    def ejecutar(self, accion: Accion):
        if accion.tipo == 'topic':
            msg = get_message(accion.interfaz)()
            set_message_fields(msg, accion.campos)
            self.publicadores[accion.nombre].publish(msg)
            self._resultado(accion, True, 'publicado')
            return

        cliente = self.clientes[accion.nombre]
        if not cliente.service_is_ready():
            self._resultado(accion, False, 'servicio no disponible')
            self.get_logger().warn(f'{accion.boton}: servicio {accion.nombre} no disponible',
                                   throttle_duration_sec=2.0)
            return
        # Al mantener pulsado no se acumulan llamadas mientras siga una pendiente.
        if accion.repetir and self.pendientes.get(accion.boton, 0) > 0:
            return
        peticion = get_service(accion.interfaz).Request()
        set_message_fields(peticion, accion.campos)
        self.pendientes[accion.boton] = self.pendientes.get(accion.boton, 0) + 1
        futuro = cliente.call_async(peticion)
        futuro.add_done_callback(lambda f, a=accion: self._respuesta(a, f))

    def _respuesta(self, accion: Accion, futuro):
        self.pendientes[accion.boton] -= 1
        try:
            r = futuro.result()
        except Exception as exc:  # noqa: BLE001 - se informa por topic
            self._resultado(accion, False, f'error: {exc}')
            return
        ok = bool(getattr(r, 'success', True))
        mensaje = getattr(r, 'message', '') or 'respuesta recibida'
        self._resultado(accion, ok, mensaje)

    def _resultado(self, accion: Accion, ok: bool, mensaje: str):
        datos = {'boton': accion.boton, 'tipo': accion.tipo, 'destino': accion.nombre,
                 'ok': ok, 'mensaje': mensaje}
        self.pub_resultado.publish(String(data=json.dumps(datos, ensure_ascii=False)))

    def destroy_node(self):
        self.parar_hilo.set()
        if self.hilo:
            self.hilo.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoMandoIR()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
