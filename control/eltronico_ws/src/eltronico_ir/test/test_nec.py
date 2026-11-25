import pytest

from eltronico_ir.entrada_evdev import nec_activo
from eltronico_ir.nec import TramaIR, analizar_simulacion, trama_desde_scancode


@pytest.mark.parametrize('scancode, comando, direccion', [
    (0x0043, 0x43, 0x00),          # NEC normal: dir<<8 | cmd
    (0x0745, 0x45, 0x07),
    (0x12345A, 0x5A, 0x1234),      # NEC extendido: dir16<<8 | cmd
    (0x12ED43BC, 0x43, 0x12ED),    # NEC32: dir, ~dir, cmd, ~cmd
])
def test_scancodes(scancode, comando, direccion):
    t = trama_desde_scancode(scancode)
    assert (t.comando, t.direccion, t.scancode) == (comando, direccion, scancode)


def test_scancode_fuera_de_rango():
    with pytest.raises(ValueError):
        trama_desde_scancode(-1)


@pytest.mark.parametrize('texto, esperado', [
    ('0x0043', TramaIR(0x43, 0x00, False, 0x43)),
    (' 0x0018,1 ', TramaIR(0x18, 0x00, True, 0x18)),
    ('67', TramaIR(0x43, 0x00, False, 67)),
])
def test_simulacion(texto, esperado):
    assert analizar_simulacion(texto) == esperado


@pytest.mark.parametrize('texto', ['', 'BTN_PLAY', '0x43,2', '0x1FFFFFFFF', 'hola'])
def test_simulacion_invalida(texto):
    assert analizar_simulacion(texto) is None


@pytest.mark.parametrize('protocolos, esperado', [
    ('[nec] rc-5 rc-6 sony', True),
    ('[nec] [rc-6]', True),
    ('nec rc-5 [rc-6] sony', False),
    ('lirc nec necx', False),
    (None, None),
])
def test_nec_activo(protocolos, esperado):
    assert nec_activo(protocolos) is esperado
