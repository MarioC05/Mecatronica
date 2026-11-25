import os

import pytest

from eltronico_ir.configuracion import cargar_acciones, cargar_mapeo

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')


def test_mapeo_real():
    direccion, botones = cargar_mapeo(os.path.join(CONFIG, 'mapeo_mando.txt'))
    assert direccion == 0x00
    assert len(botones) == 21
    assert botones[0x43].id == 'BTN_PLAY'
    assert botones[0x43].etiqueta == 'PLAY/PAUSE'


def test_acciones_reales():
    _, botones = cargar_mapeo(os.path.join(CONFIG, 'mapeo_mando.txt'))
    acciones = cargar_acciones(os.path.join(CONFIG, 'acciones.yaml'),
                               {b.id for b in botones.values()})
    assert acciones['BTN_PLAY'].tipo == 'servicio'
    assert acciones['BTN_MAS'].campos == {'data': True}
    assert acciones['BTN_2'].repetir and acciones['BTN_2'].tipo == 'topic'
    assert not acciones['BTN_PLAY'].repetir


def test_codigo_repetido(tmp_path):
    f = tmp_path / 'm.txt'
    f.write_text('BTN_A,0x10,a\nBTN_B,0x10,b\n')
    with pytest.raises(ValueError, match='repetido'):
        cargar_mapeo(str(f))


@pytest.mark.parametrize('yaml_txt, error', [
    ('acciones: {BTN_X: {servicio: /a, tipo: std_srvs/srv/Trigger}}', 'no está en el mapeo'),
    ('acciones: {BTN_A: {tipo: std_srvs/srv/Trigger}}', 'servicio'),
    ('acciones: {BTN_A: {servicio: /a, topic: /b, tipo: std_srvs/srv/Trigger}}', 'uno solo'),
    ('acciones: {BTN_A: {servicio: /a}}', 'tipo'),
])
def test_acciones_invalidas(tmp_path, yaml_txt, error):
    f = tmp_path / 'a.yaml'
    f.write_text(yaml_txt)
    with pytest.raises(ValueError, match=error):
        cargar_acciones(str(f), {'BTN_A'})
