"""Inicia Gazebo Harmonic con Eltronico y la configuracion de ros2_control.

El robot se inserta desde un archivo URDF generado temporalmente para conservar
la carga fiable que ya funciona, sin depender de la llegada de un topic.
"""

import atexit
import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('eltronico'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    world = pkg_share / 'worlds' / 'eltronico.world.sdf'
    urdf = pkg_share / 'urdf' / 'eltronico.urdf'
    controllers = pkg_share / 'config' / 'controllers.yaml'

    # Gazebo necesita una ruta absoluta al YAML. No se modifica el URDF original.
    marcador = '$(find eltronico)/config/controllers.yaml'
    contenido = urdf.read_text(encoding='utf-8')
    if contenido.count(marcador) != 1:
        raise RuntimeError('No se ha encontrado una unica referencia al YAML en el URDF')
    if not controllers.is_file():
        raise FileNotFoundError(f'No existe la configuracion de control: {controllers}')
    descripcion_xml = contenido.replace(marcador, str(controllers))

    # Se mantiene la insercion con -file, que ya funciona en este proyecto.
    with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', suffix='.urdf',
        prefix='eltronico_gz_', delete=False
    ) as temporal:
        temporal.write(descripcion_xml)
        urdf_gazebo = Path(temporal.name)
    atexit.register(lambda: urdf_gazebo.unlink(missing_ok=True))

    resources = [str(pkg_share.parent)]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resources.append(os.environ['GZ_SIM_RESOURCE_PATH'])

    # Permite aislar la simulacion fisica de la interfaz grafica de Gazebo.
    # Uso: ELTRONICO_HEADLESS=1 ros2 launch eltronico eltronico_launch.py
    solo_servidor = os.environ.get('ELTRONICO_HEADLESS', '0').lower() in ('1', 'true', 'yes')
    gz_args = f"{'-s ' if solo_servidor else ''}-r -v 3 {world}"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            # Se inicia en Play; ros2_control necesita que avance la física.
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    description = ParameterValue(descripcion_xml, value_type=str)
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_eltronico',
        output='screen',
        arguments=[
            '-world', 'eltronico_world',
            '-file', str(urdf_gazebo),
            '-name', 'eltronico',
            '-x', '0', '-y', '0', '-z', '0',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='eltronico_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/eltronico/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/eltronico/caster_left/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/eltronico/caster_right/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
    )

    joint_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60', '--switch-timeout', '20'],
        output='screen',
    )
    position_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60', '--switch-timeout', '20'],
        output='screen',
    )
    controller_node = Node(
        package='eltronico', executable='main', name='eltronico', output='screen',
        parameters=[str(pkg_share / 'config' / 'params.yaml'), {'use_sim_time': True}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resources)),
        gazebo,
        state_publisher,
        spawn,
        bridge,
        controller_node,
        # Diferir los spawners hasta que Gazebo haya insertado el robot.
        TimerAction(period=6.0, actions=[joint_spawner]),
        TimerAction(period=8.0, actions=[position_spawner]),
    ])
