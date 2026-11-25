# Workspace Eltronico — ROS 2 Jazzy

Fuentes de cuatro paquetes ROS 2:

| Paquete | Función |
|---|---|
| eltronico | Nodo C++ lifecycle, modelo CAD y simulación Gazebo Harmonic. |
| eltronico_common | Servidor de hardware y TUI remota ROS 2 (incluye PCA4). |
| eltronico_ir | Mando IR y logger ROS 2 con la misma tabla de botones. |
| gpio_topics | Lectura/escritura de GPIO digitales por topics; recuperado de la Pi. |

La copia del 24/11/2026 en la Pi contiene fuentes, modelos y documentación,
sin `build`, `install`, `log`, cachés ni binarios del PC. No se compila allí.
En la Pi: `~/eltronico_ws`; dentro del Docker existente:
`/home/ros/ros2_ws/eltronico_ws`. La primera ruta es un enlace a la carpeta
compartida con Docker. El workspace anterior se conserva por separado.

Cuando se decida compilar, dentro de Docker:

```bash
cd /home/ros/ros2_ws/eltronico_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Se necesitan las dependencias de los package.xml (incluido Gazebo para
`eltronico`) y las bibliotecas Raspberry para el hardware. La instalación de
Python del host no es la del contenedor. No se afirma que esas dependencias
estén todas instaladas en la imagen actual.

Después de compilar, en dos terminales ROS:

```bash
ros2 run eltronico_common servidor --simulate
ros2 run eltronico_common tui
```

Para hardware real, omitir `--simulate` únicamente en el servidor; los neutros
se leen del INI de la Pi, montado en `/home/ros/.config/eltronico/servos.ini`.
No ejecutar otro controlador de PCA9685 simultáneamente. Las órdenes de PCA0-3
son velocidades de servos continuos; las de PCA4 son objetivos de 0..180 grados
sin feedback. Se conservan las protecciones, límites y pines originales.

Logger del mando:

```bash
ros2 run eltronico_ir logger_ir
```

Las entradas operativas son ROS 2 (rclpy/rclcpp y ament), sin ROS 1 ni puente
MQTT necesario. Las clases Python de hardware/curses y el logger IR independiente
son utilidades internas/diagnóstico, no nodos ROS 1. El IR aún publica órdenes
provisionales sin conexión completa al controlador de hardware; la simulación
no constituye una marcha física validada. No se integra automáticamente el
control articular de Gazebo con los servos continuos.
