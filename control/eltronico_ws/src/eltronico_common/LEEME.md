# eltronico_common — servidor de dispositivos y TUI de Eltronico (ROS 2 Jazzy)

Adaptación ROS 2 de la TUI 2.8.1 (24/11/2026). `servos_tui.py` conserva las
clases de hardware y la interfaz curses; `servos_ros2.py` conecta servidor y
TUI por topics. Esta adaptación incorpora el microservo PCA4 y ya no es una
copia idéntica del script histórico: no sobrescribirla al actualizar aquel.

| Ejecutable | Nodo | Qué hace |
|---|---|---|
| `servidor` | `/eltronico_devices` | Hardware real (o `--simulate`): servos PCA0-3, microservo PCA4, LEDs, IMU, ultrasonidos. Publica estados y escucha órdenes. |
| `tui` | `/eltronico_tui` | Interfaz curses remota; manda órdenes al servidor por topics y heartbeat a 10 Hz. |

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run eltronico_common servidor --simulate      # o: ros2 launch eltronico_common servidor.launch.py simular:=true
ros2 run eltronico_common tui                      # en otra terminal, mismos --channels/--pins
```

Topics (std_msgs): estados `String` JSON a 20 Hz en `/robot/state`, `/servos/servoN/state`, `/leds/state`,
`/imu/state`, `/distance/state`; órdenes en `/robot/set` (String JSON), `/servos/servoN/set` (Float64 velocidad
normalizada -1..1), `/leds/set`, `/robot/armed/set` (Bool), `/robot/stop/set` y `/robot/heartbeat/set` (Empty).
Detalle en `actualizado_reunion/control_hardware/ROS2.md`.

Notas:
- Los servos son continuos: las órdenes son VELOCIDAD, no posición articular.
- Los neutros se leen del INI del usuario que ejecuta el servidor (`~/.config/eltronico/servos.ini`).
- Las bibliotecas de la Raspberry (Adafruit, lgpio, NeoPixel SPI, MPU6050) se instalan aparte; no son dependencias rosdep.
- Admite `--ros-args` para remapeos y nombres de nodo; las opciones hardware siguen siendo argumentos CLI.
- La pestaña MICROSERVO manda objetivos absolutos en grados por `/microservo/set`
  (Float64); `/microservo/stop/set` (Empty) desactiva la salida. Su estado se
  publica en `/microservo/state` y en `/robot/state`: es objetivo, no feedback.
- PCA4 permanece desactivado hasta recibir orden; pérdida de heartbeat durante
  más de 0,5 s, parada global o cierre desactivan su salida.
- La comunicación de estos ejecutables usa ROS 2; `--mqtt-host` se rechaza.
  El código MQTT del archivo histórico no se ejecuta desde estos puntos de entrada.
- No iniciar el servidor real junto a otra TUI/controlador que abra la misma PCA.
