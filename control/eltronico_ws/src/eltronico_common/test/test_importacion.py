import importlib
import sys


def test_importa_scripts_enlazados():
    sys.modules.setdefault('servos_tui', importlib.import_module('eltronico_common.servos_tui'))
    modulo = importlib.import_module('eltronico_common.servos_ros2')
    assert hasattr(modulo, 'DeviceNode') and hasattr(modulo, 'main')
