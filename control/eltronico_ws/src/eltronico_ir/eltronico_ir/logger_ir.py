"""Logger del mando sin ROS ni acciones: python3 -m eltronico_ir.logger_ir."""

import csv
from contextlib import closing
from datetime import datetime
from pathlib import Path
import sys
import time

from .entrada_evdev import buscar_dispositivo, EV_MSC, MSC_SCAN
from .nec import trama_desde_scancode


def main():
    try:
        import evdev
    except ImportError:
        print('Instala la dependencia: sudo apt install python3-evdev', file=sys.stderr)
        return 1
    ruta = buscar_dispositivo()
    if not ruta:
        print('No aparece gpio_ir_recv. Activa dtoverlay=gpio-ir,gpio_pin=18 '
              'y reinicia la Pi.', file=sys.stderr)
        return 1
    try:
        # Localizar el rc correspondiente al receptor, sin asumir rc0 (puede ser CEC).
        for rc in Path('/sys/class/rc').glob('rc*'):
            if list(rc.glob(f'input*/{Path(ruta).name}')):
                (rc / 'protocols').write_text('nec\n')
                break
        botones = {}
        direccion = None
        mapeo = Path(__file__).resolve().parent.parent / 'config/mapeo_mando.txt'
        with mapeo.open() as archivo:
            for fila in csv.reader(archivo):
                if not fila or fila[0].startswith('#'):
                    continue
                ident, codigo = fila[0], int(fila[1], 0)
                if ident == 'DIRECCION':
                    direccion = codigo
                else:
                    botones[codigo] = (ident, fila[2] if len(fila) > 2 else ident)
        with closing(evdev.InputDevice(ruta)) as dispositivo:
            print(f'Leyendo {dispositivo.name} ({ruta}), protocolo NEC. '
                  'Pulsa botones; Ctrl+C para salir. No ejecuta acciones.', flush=True)
            anterior, instante = None, 0.0
            for evento in dispositivo.read_loop():
                if evento.type != EV_MSC or evento.code != MSC_SCAN:
                    continue
                ahora = time.monotonic()
                repeticion = evento.value == anterior and ahora - instante < 0.2
                anterior, instante = evento.value, ahora
                trama = trama_desde_scancode(evento.value)
                ident, etiqueta = botones.get(trama.comando, ('DESCONOCIDO', '?'))
                if trama.direccion != direccion:
                    ident, etiqueta = 'OTRA_DIRECCION', '?'
                print(f'{datetime.now():%H:%M:%S.%f}  {etiqueta} ({ident})  '
                      f'scancode=0x{evento.value:04X}  '
                      f'direccion=0x{trama.direccion:X}'
                      + ('  [mantenido]' if repeticion else ''), flush=True)
    except PermissionError:
        print('Ejecuta con sudo para activar NEC y leer el receptor.', file=sys.stderr)
        return 1
    except OSError as error:
        print(f'Error del receptor: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\nLogger terminado.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
