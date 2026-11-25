#!/usr/bin/env python3
"""Control prudente e independiente de un microservo en PCA9685 canal 4."""
import argparse
import curses

CHANNEL = 4
FREQUENCY_HZ = 50
MIN_ANGLE = 0.0
MAX_ANGLE = 180.0
DEGREES_PER_STEP = 1.0
LARGE_STEP = 10.0
BIG_GLYPHS = {
    '0': (' ### ', '#   #', '#  ##', '# # #', '##  #', '#   #', ' ### '),
    '1': ('  #  ', ' ##  ', '# #  ', '  #  ', '  #  ', '  #  ', '#####'),
    '2': (' ### ', '#   #', '    #', '   # ', '  #  ', ' #   ', '#####'),
    '3': ('#### ', '    #', '    #', ' ### ', '    #', '    #', '#### '),
    '4': ('   # ', '  ## ', ' # # ', '#  # ', '#####', '   # ', '   # '),
    '5': ('#####', '#    ', '#    ', '#### ', '    #', '#   #', ' ### '),
    '6': (' ### ', '#   #', '#    ', '#### ', '#   #', '#   #', ' ### '),
    '7': ('#####', '    #', '   # ', '  #  ', ' #   ', ' #   ', ' #   '),
    '8': (' ### ', '#   #', '#   #', ' ### ', '#   #', '#   #', ' ### '),
    '9': (' ### ', '#   #', '#   #', ' ####', '    #', '#   #', ' ### '),
}


class ServoOutput:
    def __init__(self, simulate=False, address=0x40, clock_hz=25_000_000):
        self.angle = 90.0
        self.output_active = False
        self.i2c = self.kit = self.servo = None
        if simulate:
            return
        import board
        import busio
        from adafruit_servokit import ServoKit

        self.i2c = busio.I2C(board.SCL, board.SDA)
        try:
            self.kit = ServoKit(channels=16, i2c=self.i2c, address=address,
                                reference_clock_speed=clock_hz,
                                frequency=FREQUENCY_HZ)
            self.servo = self.kit.servo[CHANNEL]
            # La API de grados admite None para mantener el canal desactivado.
            self.servo.angle = None
        except BaseException:
            self.close()
            raise

    def command(self, angle):
        angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
        if self.servo is not None:
            self.servo.angle = angle
        self.angle = angle
        self.output_active = True

    def stop_signal(self):
        if self.servo is not None:
            self.servo.angle = None
        self.output_active = False

    def close(self):
        try:
            self.stop_signal()
        finally:
            if self.i2c is not None:
                self.i2c.deinit()
                self.i2c = None
            self.servo = self.kit = None


def draw_big_angle(screen, angle):
    """Dibuja el objetivo absoluto; no representa feedback real."""
    text = f'{angle:03.0f}'
    height, width = screen.getmaxyx()
    glyph_width, gap, top = 5, 1, 4
    total_width = len(text) * glyph_width + (len(text) - 1) * gap
    left = max(0, (width - total_width) // 2)
    for index, character in enumerate(text):
        glyph = BIG_GLYPHS[character]
        for row, pattern in enumerate(glyph):
            if top + row >= height:
                continue
            try:
                screen.addstr(top + row, left + index * (glyph_width + gap),
                              pattern, curses.A_BOLD)
            except curses.error:
                pass


def run_ui(screen, servo):
    curses.curs_set(0)
    screen.nodelay(False)
    screen.keypad(True)
    last_action = 'Arranque: salida inactiva; objetivo inicial 90 grados.'
    while True:
        screen.erase()
        height, width = screen.getmaxyx()
        lines = [
            'ELTRONICO | MICROSERVO PCA9685 | canal 4',
            '',
            'OBJETIVO ABSOLUTO (no es una medida de posicion)',
            '', '', '', '', '', '', '', '',
            f'Objetivo: {servo.angle:.0f} grados absolutos',
            f'Salida: {"ACTIVA" if servo.output_active else "INACTIVA"}',
            f'Recorrido nominal: 0-{MAX_ANGLE:.0f} grados',
            'Al arrancar no se envia orden al servo.',
            'El primer movimiento puede saltar desde la posicion desconocida.',
            'Flechas izq./der. o J/L: objetivo -/+ 1 grado',
            f'RePág/AvPág: -/+ 10 grados | Inicio: 0 | Fin: {MAX_ANGLE:.0f} | C: 90',
            'G: escribir un angulo absoluto y confirmar con Enter',
            'ESPACIO: desactivar salida | Q/Esc: desactivar y salir',
            last_action,
            'Prueba inicial con el horn desacoplado y sin carga mecanica.',
        ]
        for row, line in enumerate(lines[:height]):
            if 4 <= row <= 10:
                continue
            try:
                screen.addnstr(row, 0, line, max(0, width - 1))
            except curses.error:
                pass
        draw_big_angle(screen, servo.angle)
        screen.refresh()
        key = screen.getch()
        if key in (ord('q'), ord('Q'), 27):
            return
        if key in (curses.KEY_RIGHT, ord('l'), ord('L'), ord('+'), ord('=')):
            servo.command(servo.angle + DEGREES_PER_STEP)
            last_action = 'Objetivo absoluto enviado en grados.'
        elif key in (curses.KEY_LEFT, ord('j'), ord('J'), ord('-'), ord('_')):
            servo.command(servo.angle - DEGREES_PER_STEP)
            last_action = 'Objetivo absoluto enviado en grados.'
        elif key in (curses.KEY_NPAGE, ord('<')):
            servo.command(servo.angle - LARGE_STEP)
            last_action = 'Objetivo absoluto -10 grados ordenado.'
        elif key in (curses.KEY_PPAGE, ord('>')):
            servo.command(servo.angle + LARGE_STEP)
            last_action = 'Objetivo absoluto +10 grados ordenado.'
        elif key in (curses.KEY_HOME, ord('0')):
            servo.command(MIN_ANGLE)
            last_action = 'Objetivo absoluto 0 grados ordenado.'
        elif key == curses.KEY_END:
            servo.command(MAX_ANGLE)
            last_action = f'Objetivo absoluto {MAX_ANGLE:.0f} grados ordenado.'
        elif key in (ord('c'), ord('C')):
            servo.command(90.0)
            last_action = 'Objetivo absoluto 90 grados ordenado.'
        elif key in (ord('g'), ord('G')):
            prompt = f'Angulo absoluto [0-{MAX_ANGLE:.0f}] grados: '
            prompt_row = max(0, screen.getmaxyx()[0] - 2)
            screen.move(prompt_row, 0)
            screen.clrtoeol()
            screen.addnstr(prompt_row, 0, prompt, max(0, width - 1))
            screen.refresh()
            curses.echo()
            curses.curs_set(1)
            try:
                raw = screen.getstr(prompt_row, len(prompt), 3)
            finally:
                curses.noecho()
                curses.curs_set(0)
            try:
                angle = int(raw.decode('ascii').strip())
                if not MIN_ANGLE <= angle <= MAX_ANGLE:
                    raise ValueError
            except (UnicodeDecodeError, ValueError):
                last_action = f'Entrada no valida: escribe un entero entre 0 y {MAX_ANGLE:.0f}.'
            else:
                servo.command(angle)
                last_action = f'Objetivo absoluto {angle} grados ordenado.'
        elif key == ord(' '):
            servo.stop_signal()
            last_action = 'Salida desactivada.'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', type=lambda s: int(s, 0), default=0x40,
                        help='direccion I2C de PCA9685 (por defecto 0x40)')
    parser.add_argument('--clock-hz', type=int, default=25_000_000,
                        help='reloj PCA calibrado (por defecto 25000000)')
    parser.add_argument('--simulate', action='store_true',
                        help='mostrar la interfaz sin abrir I2C ni mover hardware')
    args = parser.parse_args()
    servo = ServoOutput(args.simulate, args.address, args.clock_hz)
    try:
        curses.wrapper(run_ui, servo)
    except KeyboardInterrupt:
        pass
    finally:
        servo.close()


if __name__ == '__main__':
    main()
