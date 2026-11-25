"""Carga del mapeo de botones (mapeo_mando.txt) y de las acciones (YAML)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import yaml


@dataclass
class Boton:
    id: str          # p. ej. BTN_PLAY
    comando: int     # código NEC
    etiqueta: str    # lo que pone en el mando


@dataclass
class Accion:
    boton: str
    tipo: str                         # 'servicio' o 'topic'
    nombre: str                       # nombre del servicio o del topic
    interfaz: str                     # p. ej. std_srvs/srv/Trigger
    campos: dict = field(default_factory=dict)
    repetir: bool = False             # actuar también al mantener pulsado


def cargar_mapeo(ruta: str) -> Tuple[Optional[int], Dict[int, Boton]]:
    """Lee el archivo generado con mapear_mando.ino.

    Líneas ``ID,0xCODIGO,etiqueta``; ``DIRECCION,0x..`` fija la dirección del
    mando. Devuelve (dirección, {comando: Boton}).
    """
    direccion = None
    botones: Dict[int, Boton] = {}
    with open(ruta, encoding='utf-8') as f:
        for n, linea in enumerate(f, 1):
            linea = linea.strip()
            if not linea or linea.startswith('#'):
                continue
            partes = [p.strip() for p in linea.split(',', 2)]
            if len(partes) < 2:
                raise ValueError(f'{ruta}:{n}: se esperaba ID,0xCODIGO,etiqueta')
            ident, codigo = partes[0], int(partes[1], 0)
            if ident == 'DIRECCION':
                direccion = codigo
                continue
            if codigo in botones:
                raise ValueError(f'{ruta}:{n}: código 0x{codigo:02X} repetido '
                                 f'({botones[codigo].id} y {ident})')
            etiqueta = partes[2] if len(partes) > 2 else ident
            botones[codigo] = Boton(ident, codigo, etiqueta)
    return direccion, botones


def cargar_acciones(ruta: str, botones_validos) -> Dict[str, Accion]:
    """Lee el YAML de acciones. Falla con un mensaje claro si algo no encaja."""
    with open(ruta, encoding='utf-8') as f:
        datos = yaml.safe_load(f) or {}
    acciones: Dict[str, Accion] = {}
    for boton, d in (datos.get('acciones') or {}).items():
        if boton not in botones_validos:
            raise ValueError(f'{ruta}: el botón {boton} no está en el mapeo del mando')
        if not isinstance(d, dict):
            raise ValueError(f'{ruta}: {boton} debe ser un diccionario')
        if ('servicio' in d) == ('topic' in d):
            raise ValueError(f'{ruta}: {boton} necesita «servicio» o «topic» (uno solo)')
        tipo = 'servicio' if 'servicio' in d else 'topic'
        interfaz = d.get('tipo')
        if not isinstance(interfaz, str) or interfaz.count('/') != 2:
            raise ValueError(f'{ruta}: {boton} necesita «tipo», p. ej. std_srvs/srv/Trigger')
        campos = d.get('peticion' if tipo == 'servicio' else 'mensaje') or {}
        if not isinstance(campos, dict):
            raise ValueError(f'{ruta}: los campos de {boton} deben ser un diccionario')
        acciones[boton] = Accion(boton, tipo, d[tipo], interfaz, campos,
                                 bool(d.get('repetir', False)))
    return acciones
