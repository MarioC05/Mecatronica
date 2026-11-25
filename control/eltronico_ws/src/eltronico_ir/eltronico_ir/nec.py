"""Conversión de scancodes NEC del kernel en tramas. Sin dependencias de ROS."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class TramaIR:
    """Una pulsación NEC decodificada."""

    comando: int
    direccion: Optional[int] = None
    repeticion: bool = False
    scancode: Optional[int] = None


def trama_desde_scancode(scancode: int, repeticion: bool = False) -> TramaIR:
    """Separa dirección y comando del scancode que entrega rc-core (MSC_SCAN).

    El decodificador NEC del kernel usa tres formatos:
    NEC normal 16 bits ``dir<<8 | cmd``; NEC extendido 24 bits ``dir16<<8 | cmd``;
    NEC32 32 bits ``dir<<24 | ~dir<<16 | cmd<<8 | ~cmd``. El kernel sólo usa NEC32
    con mandos no estándar; uno con dirección 0x00 cabría en 24 bits y se leería
    mal. El mando de Eltronico es NEC normal (dirección 0x00, comando invertido).
    """
    if scancode < 0 or scancode > 0xFFFFFFFF:
        raise ValueError(f'scancode fuera de rango: {scancode}')
    if scancode > 0xFFFFFF:
        return TramaIR((scancode >> 8) & 0xFF, scancode >> 16, repeticion, scancode)
    return TramaIR(scancode & 0xFF, scancode >> 8, repeticion, scancode)


_RE_SIM = re.compile(r'^(?P<sc>0x[0-9a-fA-F]+|\d+)(?:\s*,\s*(?P<rep>[01]))?$')


def analizar_simulacion(texto: str) -> Optional[TramaIR]:
    """Admite ``0x0043`` (scancode) o ``0x0043,1`` (repetición) para el topic ir/simular."""
    m = _RE_SIM.match(texto.strip())
    if not m:
        return None
    try:
        return trama_desde_scancode(int(m.group('sc'), 0), m.group('rep') == '1')
    except ValueError:
        return None
