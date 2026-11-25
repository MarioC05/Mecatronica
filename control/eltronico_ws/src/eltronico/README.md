# Eltronico — ROS 2 Jazzy + Gazebo Harmonic

Paquete de simulación y control de Eltronico. Incluye el URDF y sus mallas, el mundo con escaleras, `gz_ros2_control` y un nodo C++ con modos `IDLE`, `WALK`, `AVOID_OBSTACLE` (reservado) y `CLIMB`.

## Compilar

Desde `control_work/controller-ws/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select eltronico --symlink-install
source install/setup.bash
```

## Ejecutar

```bash
ros2 launch eltronico eltronico_launch.py
```

El lanzamiento inicia Gazebo, el nodo `/eltronico`, los controladores y los puentes de `/clock`, odometría y contactos. **No es necesario iniciarlos por separado.**

En otra terminal, con el entorno cargado:

```bash
ros2 control list_controllers
ros2 topic echo /eltronico/odom --once
ros2 lifecycle set /eltronico configure
ros2 lifecycle set /eltronico activate
ros2 param set /eltronico behavior walk
```

Para detener la marcha:

```bash
ros2 param set /eltronico behavior idle
```

## Estado y limitaciones

- `WALK`: genera consignas para las cuatro articulaciones de las patas; pendiente validar el desplazamiento en Gazebo.
- `CLIMB`: contiene la secuencia de aproximación, repliegue de la palanca, avance y comprobación de apoyos; **no está validado subiendo escalones**.
- El motor físico puede mostrar un aviso `mimic` por los engranajes 3:1: la transmisión física no está reproducida fielmente.
- La lógica C++ se probó con datos simulados; hay que verificar sensores, contactos y actuadores en Gazebo antes de dar la subida por funcional.

ERROR DE MOVIMIENTO EN LA PATA DERECHA