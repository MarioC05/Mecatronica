# Control manual de Eltronico

La TUI controla cuatro Parallax Feedback 360 mediante la PCA9685, un anillo NeoPixel
de 12 LED por SPI y una IMU GY-521 (MPU6050). El ultrasonidos HC-SR04 esta preparado
pero desactivado por defecto porque todavia no esta conectado. No incluye audio.

La versión 2.4 añade trazas MQTT opcionales (motores, feedback y LEDs).
Consulta [MQTT.md](MQTT.md) para instalar el broker Mosquitto en la Pi, activar
`--mqtt-host localhost` y guardar las trazas desde el PC.

La version 2.5 muestra distancia original y filtrada con Kalman, tambien en MQTT.
La version 2.6 anade la pestana IMU y deja el ultrasonidos apagado por defecto.
La version 2.7 amplia el control a cuatro motores: conserva PCA0/1 y anade las
rodillas en PCA2/3. Se mantiene un modo opcional de dos motores.
La version 2.7.1 aclara la calibracion: `J/L` ajusta el neutro sin pulsar `A`,
incluso si la habilitacion esta bloqueada por RPM o falta de feedback.
La version 2.8 integra el microservo PCA4 en una pestana de la TUI principal;
los motores con feedback siguen en PCA0-3. El microservo instalado tiene un
recorrido de 180 grados.
La version 2.8.1 amplia el ajuste de neutros a 1200-1800 us.

El microservo PCA4 se controla desde la pestana **MICROSERVO**. El script
`microservo_tui.py` queda como alternativa para manejarlo sin la TUI principal.

## Arranque en la Raspberry Pi

Copia el archivo y ejecútalo en la Pi:

```bash
scp /home/manza/Desktop/Mecatronica_loca/actualizado_reunion/control_hardware/servos_tui.py eltronico@10.169.175.134:~/
ssh eltronico@10.169.175.134
~/robot-venv/bin/python ~/servos_tui.py
```

Dependencias de hardware: `adafruit-circuitpython-servokit`, `adafruit-circuitpython-neopixel-spi` y `lgpio`. Habilita SPI en `raspi-config` para usar el anillo. Si ya calibraste los neutros, la TUI los guarda en `~/.config/eltronico/servos.ini` al pulsar `S` mayúscula.

Para la IMU instala tambien `requirements-imu.txt` en el entorno de la TUI.
Desde el PC:

```bash
scp /home/manza/Desktop/Mecatronica_loca/actualizado_reunion/control_hardware/{servos_tui.py,requirements-imu.txt} eltronico@10.169.175.134:~/
ssh eltronico@10.169.175.134
```

En la Raspberry, cierra la TUI anterior e instala/arranca:

```bash
~/robot-venv/bin/python -m pip install -r ~/requirements-imu.txt
unset MQTT_USERNAME MQTT_PASSWORD
~/robot-venv/bin/python ~/servos_tui.py --mqtt-host localhost
```

El ejemplo usa el broker local sin contrasena configurado en esta instalacion.

No ejecutes a la vez otro programa que controle la PCA9685.

## Conexiones configuradas

| Componente | Raspberry / PCA9685 |
|---|---|
| PCA9685 | I2C, dirección `0x40` |
| Feedback servo 1 / 2 | BCM27 / BCM17 (pines físicos 13 / 11) |
| Servo 1 / 2 | Canales PCA9685 0 / 1 |
| Feedback rodilla 1, servo 3 | BCM5, pin fisico 29 |
| Feedback rodilla 2, servo 4 | BCM6, pin fisico 31 |
| Servo 3 / 4 (rodillas) | Canales PCA9685 2 / 3 |
| Microservo posicional sin feedback | Canal PCA9685 4; pestaña MICROSERVO |
| NeoPixel DIN | BCM10, pin físico 19 (SPI0 MOSI) |
| GY-521 SDA / SCL | Bus I2C compartido, conectores SDA/SCL de la PCA |
| GY-521 direccion | 0x68 o 0x69, deteccion automatica |
| Distancia TRIG (desactivado) | BCM13, pin fisico 33 |
| Distancia ECHO (desactivado) | BCM14, pin fisico 8, entrada de 3.3 V |

La TUI aplica el ajuste SPI que resolvió el blanco errático en esta Pi: `frequency=6_400_000` y `bit0=0b10000000`. Deja el orden de color en GRB salvo que el anillo muestre rojo y verde intercambiados. El código no puede compensar problemas de alimentación o conexión.

Los feedback se conectan a la Raspberry y las senales de mando a la PCA.
BCM5/6 no son los pines fisicos 5/6: usa los fisicos 29/31. Comparte GND entre
Pi, PCA y fuente de servos. Este mapa conserva el cableado anterior de la TUI;
no es el orden del esquema Fritzing historico (cadera/rodilla por lado).

Los neutros se cargan por numero de canal de `~/.config/eltronico/servos.ini`.
Los valores guardados de PCA0/1 se conservan; no se sustituyen por 1500 us ni
por valores de capturas anteriores. Si PCA2/3 aun no tienen valor guardado,
arrancan con **1500 us provisional**, se marcan `CALIBRAR` y pueden girar: no
equivale a una parada confirmada. Selecciona `3` o `4`, ajusta con `J/L` hasta
tener RPM cerca de cero y guarda con `S` mayuscula. Guardar conserva otros
canales y secciones del INI. Para habilitar se exige feedback reciente y RPM
cerca de cero en todos los motores configurados.

La interfaz de cuatro motores cabe en una terminal de 110 columnas x 24 filas.
MQTT incluye los cuatro motores en la lista `servos`.

## IMU GY-521 / MPU6050

SDA/SCL de la GY-521 comparten el bus de la PCA9685: la PCA usa 0x40 y la IMU
0x68 (AD0 bajo) o 0x69 (AD0 alto). Comparte tambien GND y alimenta el modulo
segun su placa, manteniendo las senales I2C de la Pi a 3.3 V. No conectes la IMU
a las salidas PWM de servo ni a la alimentacion V+ de los motores.

Pulsa `TAB` hasta llegar a **IMU** (tres veces desde SERVOS en la TUI 2.8).
Muestra:

- Aceleracion X/Y/Z en m/s2, incluida la gravedad.
- Velocidad angular X/Y/Z en grados/s.
- Roll/pitch aproximados a partir de la gravedad, en grados.
- Temperatura interna del chip, edad de la muestra y estado del sensor.

Las lecturas usan los ejes impresos en el modulo. Roll/pitch presuponen que la
aceleracion dominante es la gravedad; al mover el robot pueden variar por
aceleraciones lineales. No hay calibracion automatica de sesgo ni estimacion
de rumbo absoluto. La temperatura es la del chip, no la ambiental.

El sensor se consulta a 20 Hz, con filtro digital interno de 21 Hz, rango de
acelerometro +/-2 g y giroscopio +/-250 grados/s. El driver devuelve rad/s y
la TUI los convierte a grados/s. Se reutiliza el objeto I2C de la PCA para que
los drivers compartan su bloqueo de bus; las lecturas se realizan en otro hilo.
La IMU se detiene antes de liberar ese bus al cerrar.

Si falta el driver, el sensor no responde o hay un error, la TUI muestra
`ERROR IMU` y reintenta cada 2 segundos. Los datos invalidos o de mas de 0.5 s
se publican como `null`. Las lecturas de la IMU no generan ordenes de motores.

```bash
python3 ~/servos_tui.py --imu-address 0x69
python3 ~/servos_tui.py --imu-backend off
python3 ~/servos_tui.py --simulate
```

La simulacion funciona sin la dependencia MPU6050. Referencias:
[API Adafruit MPU6050](https://docs.circuitpython.org/projects/mpu6050/en/latest/api.html),
[registros InvenSense](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).

## Sensor de distancia

Los numeros de TRIG/ECHO se interpretan como **GPIO BCM de la Raspberry**, no
como pines fisicos ni como numeracion Arduino. El pin fisico 13 ya corresponde
al feedback del servo (BCM27). Si el sketch se ejecutaba en otra placa, conecta
el sensor a la Pi con la tabla anterior para usar esta implementacion.

En un HC-SR04 alimentado a 5 V, baja ECHO a 3.3 V con un divisor resistivo o
adaptador de nivel antes de conectarlo a la Pi; comparte GND. Los GPIO de la
Pi no toleran 5 V. Referencias: [GPIO Raspberry Pi](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
y [HC-SR04](https://www.sparkfun.com/ultrasonic-distance-sensor-hc-sr04.html).

Cuando se activa, la distancia aparece debajo de los servos: `Original`, `Kalman` y
estado (`OK`, `OUTLIER`, `SIN ECO` o error GPIO). No hace falta cambiar de pestana.
La lectura usa callbacks de lgpio y un hilo propio; ni la TUI ni el control de
motores esperan al eco. Un fallo del sensor se muestra sin deshabilitar la TUI.

Se mantiene la logica de tu sketch: `P=10`, `Q=0.05`, `R=4`, umbral de salto
de 15 cm y cinco rechazos consecutivos antes de permitir una actualizacion.
La primera lectura valida inicializa el filtro. Una lectura rechazada sigue
mostrando la distancia original mientras Kalman conserva la estimacion anterior.
Solo se aceptan distancias estrictamente entre 2 y 400 cm.

TRIG solicita un pulso de 10 us; ECHO tiene un timeout de 25 ms. Se inicia una
medicion cada 65 ms (el [datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
recomienda un ciclo superior a 60 ms). La formula es `duracion_us * 0.0343 / 2`.
Sin eco, fuera de rango o tras 0.5 s sin datos recientes, se muestra `---` y MQTT
publica `null`; no se presenta la ultima distancia como actual. Tras una perdida
de senal superior a 0.5 s, la siguiente lectura valida reinicializa Kalman.

El sensor esta **desactivado por defecto**: no arranca su hilo, no reserva GPIO
y no genera pulsos TRIG, incluso en simulacion. Para cuando este instalado:

```bash
python3 ~/servos_tui.py --distance-backend gpio --mqtt-host localhost
python3 ~/servos_tui.py --distance-backend gpio --trig-pin 13 --echo-pin 24
python3 ~/servos_tui.py --distance-backend off
python3 ~/servos_tui.py --simulate --distance-backend gpio
```

BCM14 tambien puede usarse como TX de UART. Si esta ocupado, libera ese UART o
cambia el cable ECHO a un GPIO libre (por ejemplo BCM24, pin fisico 18) y pasa
`--echo-pin 24`. Se rechazan conflictos con feedback, I2C y SPI antes de abrir
hardware. La simulacion genera distancias con ruido y picos sin importar lgpio.

Para verificar en el PC:

```bash
cd actualizado_reunion/control_hardware
python3 -m unittest -v test_servos_tui test_mqtt test_distance test_imu test_four_servos
```

## Teclas

### Motores

- `A`: habilitar, si todos los feedback estan recientes y las RPM estan cerca de cero.
- `1` / `2` / `3` / `4`: seleccionar un servo (PCA0/1/2/3 por defecto).
- `B` / `E`: seleccionar los servos 1-2, mismo signo / espejo.
- `R` / `F`: seleccionar las rodillas 3-4, mismo signo / espejo.
- `T`: seleccionar todos, mismo signo.
- `W` / `S` o flechas arriba/abajo: aumentar/reducir velocidad en pasos de 5 %.
- `Espacio` / `Esc`: enviar neutro a todos los motores configurados.
- `J` / `L` o `[` / `]`: ajustar el neutro del seleccionado en pasos de 5 µs **sin pulsar A**.
- `S` mayúscula: guardar neutros.
- `Q`: salir y enviar neutros.

Por defecto el mando llega al 100 % y queda activo hasta parar. Usa `--limit 0.3` para limitarlo durante pruebas. `--deadman 1` añade parada tras un segundo sin órdenes. Un neutro no es un freno mecánico.

Para calibrar desde el arranque, selecciona `1`, `2`, `3` o `4`, pulsa `J`
para bajar el neutro o `L` para subirlo y observa las RPM. Cada ajuste pone
todas las ordenes de velocidad a cero y aplica los neutros en el hilo de
control aunque no se haya habilitado con `A`. Guarda con `S` mayuscula.
`W/S` y las flechas ajustan velocidad, no neutro; esas ordenes si requieren
`A`. Usa `A` solo despues de calibrar, cuando quieras mover los motores.
Sin feedback puedes cambiar el valor, pero no confirmar la parada por RPM;
la habilitacion del movimiento sigue bloqueada hasta recuperar senal valida.
El neutro ajustable admite 1200-1800 us. J/L manda ese valor directamente al
motor aunque `A` no este habilitada; si el valor aun no es su neutro real, puede
moverse mientras calibras. Hazlo con patas/motores despejados y ten a mano
Espacio para desactivar el mando.

## Microservo sin feedback en PCA4 (TUI principal)

La TUI principal incluye la pestaña **MICROSERVO** y usa la interfaz de grados
`Servo.angle` sobre el canal PCA4 compartido con los otros controles. Al
arrancar deja el canal desactivado; no ordena posición hasta pulsar una tecla.
Controla un objetivo absoluto de 0 a 180 grados.
`J`/flecha izquierda ordena -1 grado; `L`/flecha derecha +1 grado; RePág/AvPág
cambian 10 grados; Inicio/Fin ordenan 0/180 grados y `C` ordena 90. Pulsa `G`
para escribir un ángulo entero entre 0 y 180 y confirmarlo con Enter. El objetivo
se muestra grande y centrado. Espacio desactiva la salida y `Q`/Esc sale.

El ángulo mostrado es el objetivo enviado, no una posición medida: el servo no
tiene feedback. El recorrido físico depende del modelo y del montaje; calibra
con el horn desacoplado y sin carga, y detente si zumba o hace fuerza en un
extremo. La librería convierte internamente los grados a la señal que requiere
el servo. No ejecutes otra TUI/controlador PCA a la vez: la frecuencia de 50 Hz
se configura para el chip PCA completo.

```bash
~/robot-venv/bin/python ~/servos_tui.py --mqtt-host localhost
```

La interfaz independiente sigue disponible como `microservo_tui.py` si quieres
usar solo PCA4. Para previsualizar la TUI principal sin hardware:
`python3 servos_tui.py --simulate`.

### NeoPixel

Pulsa `TAB` para cambiar de pestaña. En la pestaña NeoPixel:

- Flechas arriba/abajo: elegir rojo, verde, azul o brillo.
- Flechas izquierda/derecha: ajustar el valor.
- `Home` / `End`: mínimo/máximo.
- `Enter`: encender o apagar; `0`: apagar.

El anillo empieza apagado y se inicializa al encenderlo. `--led-backend off` desactiva el control de LED; `--led-count 12` y `--led-order GRB` son los valores por defecto.

## Modos

```bash
python3 ~/servos_tui.py --simulate
python3 ~/servos_tui.py --limit 0.3
python3 ~/servos_tui.py --channels 0 1
python3 ~/servos_tui.py --channels 0 1 2 3 --pins 27 17 5 6
```

Sin opciones se usan cuatro motores. `--channels 0 1` o `--pins 27 17`
mantiene la prueba anterior de dos motores. `--channels` y `--pins` admiten
dos o cuatro valores, con la misma longitud. `--neutral` requiere un valor
por motor y reemplaza los valores cargados del INI solo durante esa ejecucion;
omitirlo para conservar la calibracion real. Las teclas de pareja indican
posiciones en `--channels`, no asignan automaticamente izquierda/derecha.

En simulación no se accede al hardware. Los valores PWM que muestra la TUI son calculados; no son una medición de la señal en el servo.
