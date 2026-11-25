# gpio_topics — GPIO por topics ROS 2

Copia del paquete del workspace antiguo de la Pi, recuperada el 24/11/2026.
Revisado para diferir toda escritura hasta recibir un mensaje `/set`.
No se ha arrancado el nodo sobre hardware ni abierto GPIO reales.
Usa libgpiod 1.x (`python3-libgpiod` dentro del contenedor), rclpy y std_msgs.
El paquete se construye con ament_cmake/ament_cmake_python.

Por defecto intenta gestionar BCM0..27, no los40pines físicos del conector.
En Raspberry Pi4 todos se monitorizan mediante /dev/gpiomem sin reclamarlos
ni cambiar su función, dirección o nivel. En otros chips usa solicitudes
libgpiod AS_IS para mantener la dirección existente; omite pines no disponibles.

| Topic | Tipo | Función |
|---|---|---|
| /gpio/bcmN/state | std_msgs/msg/Bool | Nivel digital a20Hz por defecto. |
| /gpio/bcmN/get | std_msgs/msg/Empty | Solicitar publicación inmediata. |
| /gpio/bcmN/set | std_msgs/msg/Bool | Primera orden activa la salida con ese nivel; siguientes la actualizan. |

`output_pins: [-1]` permite solicitar escritura en los pines seleccionados;
**no los configura como salidas al arrancar**. `output_pins: []` impide toda
escritura. BCM2/3/7/8/9/10/11 siguen protegidos como buses, incluso con `/set`.
Si otro driver ocupa un GPIO (por ejemplo el receptor IR), la solicitud de
salida falla y se conserva el monitor. La lectura y `/get` nunca activan salidas.

Ejemplo de escritura explícita en BCM22:

```bash
ros2 topic pub --once /gpio/bcm22/set std_msgs/msg/Bool '{data: true}'
```

El pin mantiene el último nivel ordenado mientras lo gestiona el nodo. Al salir
se libera sin enviar un cero adicional; el estado posterior depende del kernel
y del hardware. No se cambian pull-ups/pull-downs expresamente.

Ejemplo de lectura solo de un GPIO libre (BCM22), después de compilar:

```bash
ros2 run gpio_topics gpio_topics_node --ros-args -p pins:="[22]" -p output_pins:="[]"
ros2 topic echo /gpio/bcm22/state
```

`pins` y `output_pins` son listas de números BCM. Solo seleccionar pines cuyo
uso real se haya comprobado; no iniciar simultáneamente dos nodos que reclamen
la misma línea. El muestreo a20Hz no decodifica IR, PWM, I2C ni SPI: para esos
dispositivos usar sus nodos específicos.
