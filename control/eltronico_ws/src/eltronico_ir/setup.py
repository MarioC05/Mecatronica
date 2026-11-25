from glob import glob

from setuptools import find_packages, setup

paquete = 'eltronico_ir'

setup(
    name=paquete,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paquete]),
        ('share/' + paquete, ['package.xml']),
        ('share/' + paquete + '/config', glob('config/*')),
        ('share/' + paquete + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    description='Mando IR de Eltronico para ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mando_ir = eltronico_ir.nodo_mando_ir:main',
            'logger_ir = eltronico_ir.nodo_logger_ir:main',
        ],
    },
)
