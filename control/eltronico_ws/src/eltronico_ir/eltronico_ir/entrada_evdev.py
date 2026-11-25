"""Lectura del receptor IR conectado al GPIO de la Raspberry Pi.

El overlay ``gpio-ir`` crea un dispositivo de entrada (``gpio_ir_recv``). El
kernel decodifica NEC y emite eventos EV_MSC/MSC_SCAN con el scancode, también
en cada trama de repetición mientras se mantiene el botón.
"""

from __future__ import annotations

import glob
import os
from typing import Optional

NOMBRE_GPIO_IR = 'gpio_ir_recv'
EV_MSC = 0x04
MSC_SCAN = 0x04


def buscar_dispositivo(nombre: str = NOMBRE_GPIO_IR) -> Optional[str]:
    """Ruta /dev/input/eventN cuyo nombre contiene ``nombre``, o None."""
    for ruta in sorted(glob.glob('/sys/class/input/event*/device/name')):
        try:
            with open(ruta, encoding='utf-8') as f:
                if nombre in f.read():
                    return '/dev/input/' + ruta.split('/')[4]
        except OSError:
            continue
    return None


def protocolos_activos(dispositivo: str) -> Optional[str]:
    """Contenido de /sys/class/rc/rcN/protocols del receptor; el activo va entre []."""
    evento = os.path.basename(os.path.realpath(dispositivo))
    for rc in glob.glob('/sys/class/rc/rc*'):
        if glob.glob(f'{rc}/input*/{evento}'):
            try:
                with open(f'{rc}/protocols', encoding='utf-8') as f:
                    return f.read().strip()
            except OSError:
                return None
    return None


def nec_activo(protocolos: Optional[str]) -> Optional[bool]:
    if protocolos is None:
        return None
    return any(p.startswith('[nec') for p in protocolos.split())
