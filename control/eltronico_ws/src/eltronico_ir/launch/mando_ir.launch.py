from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('dispositivo', default_value='auto',
                              description='auto (busca gpio_ir_recv), /dev/input/eventN '
                                          'o «ninguno» para usar sólo ir/simular'),
        DeclareLaunchArgument('archivo_acciones', default_value='',
                              description='YAML de acciones; vacío usa el del paquete'),
    ]
    parametros = {
        'dispositivo': LaunchConfiguration('dispositivo'),
        'archivo_acciones': LaunchConfiguration('archivo_acciones'),
    }
    nodo = Node(package='eltronico_ir', executable='mando_ir', name='mando_ir',
                output='screen', parameters=[parametros])
    return LaunchDescription(args + [nodo])
