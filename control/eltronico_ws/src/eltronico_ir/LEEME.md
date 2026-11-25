# eltronico_ir — mando IR en ROS 2 Jazzy (Raspberry Pi 4B)

Nodo Python `mando_ir`. El receptor IR (VS1838B/KY-022) va conectado al GPIO de la Raspberry Pi 4B. El overlay `gpio-ir` del kernel decodifica el protocolo NEC y el nodo lee los scancodes por evdev. Identifica el botón con `config/mapeo_mando.txt` (21 botones, dirección 0x00) y ejecuta la acción de `config/acciones.yaml`: llamar a un servicio o publicar en un topic de cualquier tipo.

## Conexión

| Receptor | Raspberry Pi 4B |
|---|---|
| OUT / S | **BCM18**, pin físico 12 |
| VCC / + | **3V3**, pin físico 1 o 17 (no 5 V: la salida iría a 5 V al GPIO) |
| GND / − | GND, p. ej. pin físico 14 |

BCM18 está libre en el esquema de cuatro servos y en la TUI. Ocupados: BCM2/3 I²C (PCA), 5/6/13/19 y 17/27 feedback, 10 NeoPixel, 24/25 y 13/14 ultrasonidos. Para usar otro pin, cambiar `gpio_pin` en el overlay.

## Preparar la Raspberry Pi (una vez)

### Logger ROS 2

El ejecutable `logger_ir` reutiliza `NodoMandoIR`, su lector evdev y
`config/mapeo_mando.txt`. Muestra etiqueta, ID, código y repeticiones en el log;
publica `ir/boton` e `ir/evento`, sin cargar ni ejecutar acciones.

Con ROS 2 y el paquete compilado y su entorno cargado:

```bash
ros2 run eltronico_ir logger_ir
```

Para probar sin receptor: `ros2 run eltronico_ir logger_ir --ros-args -p
dispositivo:=ninguno` y publicar en `/ir/simular` desde otra terminal.
El protocolo NEC debe estar activo (servicio descrito abajo), y el usuario
debe tener permisos de lectura del receptor. El logger ROS requiere ROS 2;
el logger independiente de la siguiente sección no lo requiere.

### Logger sin ROS

Con el overlay de abajo activado y la Pi reiniciada:

```bash
sudo apt install python3-evdev
cd ~/eltronico_ir
sudo python3 -m eltronico_ir.logger_ir
```

Detecta el receptor, activa NEC automáticamente y muestra etiqueta, ID y código
de cada botón, incluyendo repeticiones. No ejecuta acciones ni necesita
`ir-keytable` ni ROS. Ctrl+C termina la lectura. Esta utilidad se ejecuta desde
la carpeta fuente, donde está `config/mapeo_mando.txt`.

1. Activar el receptor en `/boot/firmware/config.txt` (Ubuntu 24.04 y Raspberry Pi OS Bookworm) y reiniciar:
   ```
   dtoverlay=gpio-ir,gpio_pin=18
   ```
2. Instalar evdev y dar acceso al usuario (cerrar sesión después):
   ```bash
   sudo apt install python3-evdev
   sudo usermod -aG input $USER
   ```
3. Activar NEC. El overlay arranca con el mapa RC6 por defecto, que no decodifica este mando:
   ```bash
   cat /sys/class/rc/rc*/protocols          # NEC activo si aparece [nec]
   sudo cp config/eltronico-ir-nec.service /etc/systemd/system/
   sudo systemctl daemon-reload && sudo systemctl enable --now eltronico-ir-nec
   ```
   El servicio escribe `nec` en el receptor `gpio_ir_recv` en cada arranque. No asigna teclas, de modo que el mando no escribe en consolas ni en otras aplicaciones. Si NEC no está activo, el nodo lo avisa en el log.
4. Comprobación opcional sin ROS: `sudo apt install ir-keytable` y `ir-keytable -t -s rc0`. Al pulsar PLAY/PAUSE debe salir `scancode = 0x43` con protocolo nec.

## Compilar y lanzar

```bash
cd ~/Desktop/robot_mecatronica/Mecatronica_loca/ros2_ws    # ajustar la ruta en la Pi
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select eltronico_ir
source install/setup.bash
ros2 launch eltronico_ir mando_ir.launch.py
```

Argumentos: `dispositivo` (`auto` busca `gpio_ir_recv`; también admite `/dev/input/eventN` o `ninguno`) y `archivo_acciones` (ruta a otro YAML). Con `--ros-args -p` también se pueden pasar `archivo_mapeo`, `direccion` (-1 toma la del mapeo, -2 acepta cualquiera) y `tiempo_mantenido` (0,2 s). Si el receptor no está, el nodo sigue funcionando con `ir/simular` y reintenta cada 2 s.

Prueba sin receptor (en cualquier PC):
```bash
ros2 launch eltronico_ir mando_ir.launch.py dispositivo:=ninguno
ros2 topic pub --once /ir/simular std_msgs/msg/String "{data: '0x0043'}"   # PLAY/PAUSE
ros2 topic echo /ir/evento
```

## Interfaz ROS

| Nombre | Tipo | Contenido |
|---|---|---|
| `ir/simular` (sub) | std_msgs/String | Scancode `0x0043`, o `0x0043,1` como repetición |
| `ir/boton` (pub) | std_msgs/String | ID del botón (`BTN_PLAY`) en cada pulsación nueva; no en repeticiones |
| `ir/evento` (pub) | std_msgs/String (JSON) | Todas las tramas: botón, comando, dirección, scancode, repetición, reconocido |
| `ir/resultado` (pub) | std_msgs/String (JSON) | Resultado de cada acción: ok, destino, mensaje |

Mientras se mantiene el botón, el kernel repite el scancode unas 9 veces por segundo. El mismo scancode antes de `tiempo_mantenido` se considera repetición. Solo disparan acción las repeticiones de entradas con `repetir: true`. Una repetición de servicio no se envía mientras siga pendiente la llamada anterior. Si el servicio no está disponible, el nodo no espera: publica `ok: false` y continúa. Se descartan los códigos con otra dirección del mando y los códigos desconocidos.

## Acciones por defecto (provisionales)

| Botón | Acción |
|---|---|
| PLAY/PAUSE, 5 | `/eltronico/parar` (std_srvs/Trigger) |
| CH | `/eltronico/armar` (std_srvs/Trigger) |
| + / − | `/eltronico/proteccion_obstaculo` (std_srvs/SetBool true/false) |
| 2 / 8 / 4 / 6 | `/eltronico/orden_mando` (std_msgs/String adelante/atras/izquierda/derecha, repetir) |

Todavía **no existen** nodos ROS 2 que atiendan estos servicios y topics. Hay que ajustar los nombres al migrar el control. El mando no manda velocidad a los servos directamente: solo publica intenciones u órdenes que el controlador debe validar con sus límites. La subida de escalón no se asigna porque no hay maniobra validada para las patas r2. El IR necesita línea de vista y puede perder tramas: no usarlo como única parada de emergencia.

## Pruebas

Desde `ros2_ws/`, con el paquete compilado y el overlay cargado (sin `install/setup.bash`, las pruebas de `test_nodo.py` fallan con `PackageNotFoundError`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
cd src/eltronico_ir && python3 -m pytest -q test
```

Cubren la conversión de scancodes NEC, NEC extendido y NEC32, la simulación, la detección de NEC activo, la carga y los errores de configuración, la integración rclpy con servidores falsos de Trigger y SetBool, y el hilo de lectura evdev con un dispositivo falso. También se lanzó a mano el ejecutable instalado con `ros2 launch`. **No se ha probado en la Raspberry Pi con el receptor real**: falta comprobar el overlay, el servicio NEC, los permisos y la repetición real del kernel.
