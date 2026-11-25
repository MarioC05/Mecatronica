from glob import glob

from setuptools import find_packages, setup

paquete = 'eltronico_common'

setup(
    name=paquete,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paquete]),
        ('share/' + paquete, ['package.xml', 'LEEME.md']),
        ('share/' + paquete + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    description='Servidor de dispositivos y TUI remota de Eltronico para ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'servidor = eltronico_common.nodos:servidor',
            'tui = eltronico_common.nodos:tui',
        ],
    },
)
