# Trazas MQTT de Eltronico

Primero copia los archivos desde el PC, en la raíz de este repositorio:

```bash
ssh eltronico@raspberrypi 'mkdir -p ~/control_hardware'
scp actualizado_reunion/control_hardware/{servos_tui.py,requirements-mqtt.txt,requirements-imu.txt,mosquitto-eltronico.conf,MQTT.md} eltronico@raspberrypi:~/control_hardware/
ssh eltronico@raspberrypi
cd ~/control_hardware
```

## 1. Levantar el broker en la Raspberry Pi

En Raspberry Pi OS, instala Mosquitto y crea el usuario del robot:

```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients python3-venv
sudo mosquitto_passwd -c /etc/mosquitto/passwd eltronico
sudo chown root:mosquitto /etc/mosquitto/passwd
sudo chmod 640 /etc/mosquitto/passwd
```

La contraseña se introduce de forma interactiva. Usa `-c` solo al crear el archivo;
si ya existe, omite `-c` para conservar los demás usuarios.

Desde este directorio, copia la configuración y arranca el servicio:

```bash
sudo cp mosquitto-eltronico.conf /etc/mosquitto/conf.d/eltronico.conf
sudo systemctl enable mosquitto
sudo systemctl restart mosquitto
systemctl status mosquitto --no-pager
hostname -I
```

El listener acepta conexiones del PC por el puerto TCP 1883 con usuario y contraseña.
Esta configuración es para la red local de confianza: el tráfico no lleva TLS.
No abras ese puerto en el router a Internet. Si tienes firewall, permite TCP 1883
solo desde tu red local. Si ya tienes un listener 1883 configurado, intégralo en
ese archivo en vez de duplicarlo. Para diagnosticar errores de arranque:

```bash
sudo journalctl -u mosquitto -n 50 --no-pager
```

## 2. Instalar el cliente Python

En la Pi, instala el cliente en un entorno que permita usar las
librerías de hardware ya instaladas en el sistema:

```bash
cd ~/control_hardware
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-mqtt.txt -r requirements-imu.txt
```

Si las librerías de hardware están en otro entorno virtual, instala el requisito
MQTT en ese entorno y ejecuta con su Python.

## 3. Arrancar la TUI

En la Pi, introduce la contraseña del usuario MQTT sin escribirla en el historial:

```bash
read -rsp 'Contraseña MQTT: ' MQTT_PASSWORD; echo
export MQTT_PASSWORD
.venv/bin/python servos_tui.py --mqtt-host localhost --mqtt-username eltronico
```

Para comprobarlo sin tocar hardware:

```bash
.venv/bin/python servos_tui.py --simulate --mqtt-host localhost --mqtt-username eltronico
```

Opciones: `--mqtt-port 1883`, `--mqtt-topic eltronico/tui`, `--mqtt-hz 5`
(de 0.2 a 20 Hz). El usuario también puede darse con `MQTT_USERNAME`.
Sin `--mqtt-host` no se importa Paho ni se abre ninguna conexión MQTT.
La última fila de la TUI muestra el estado de conexión.

## 4. Ver y guardar trazas desde el PC

Instala `mosquitto-clients` en el PC y ejecuta, sustituyendo `IP_DE_LA_PI`:

```bash
read -rsp 'Contraseña MQTT: ' MQTT_PASSWORD; echo
export MQTT_PASSWORD
mosquitto_sub -h IP_DE_LA_PI -u eltronico -P "$MQTT_PASSWORD" -t 'eltronico/tui/#' -v
```

Para guardar telemetría en JSONL, un JSON por línea:

```bash
mosquitto_sub -h IP_DE_LA_PI -u eltronico -P "$MQTT_PASSWORD" \
  -t 'eltronico/tui/telemetry' > "trazas-$(date +%Y%m%d-%H%M%S).jsonl"
```

La opción `-P` evita guardar el secreto literal en el historial con este ejemplo,
pero los procesos del mismo usuario pueden ver los argumentos del cliente.
El broker distribuye mensajes; para conservar el histórico deja este suscriptor activo.

## Mensajes

| Topic | Contenido |
|---|---|
| `eltronico/tui/telemetry` | JSON periódico: motores, feedback, LEDs y estado del control |
| `eltronico/tui/events` | Cambios observados de habilitación, órdenes, neutros, mensajes, fallos y LEDs |
| `eltronico/tui/status` | `online` / `offline`, retenido; Last Will en desconexión inesperada |

Cada JSON contiene `schema: 1`, `session` y `ts` en UTC. Telemetría y eventos
comparten `seq`. La telemetría añade `monotonic_s`, `simulate`, `armed`, `message`,
`fault`, `servos` y `led`. Cada servo contiene `command` (-1..1),
`pwm_calculated_us` (calculado, no medido), `neutral_us`, `angle_deg`, `rpm`,
`feedback_age_s`, `feedback_ok`, canal PCA y GPIO BCM.
Los ángulos y RPM de feedback caducado se publican como `null`; la edad también
es `null` cuando no se ha recibido ninguna señal.

Desde 2.7, `servos` contiene cuatro elementos por defecto: canales 0/1/2/3,
GPIO 27/17/5/6. `--channels 0 1` conserva el modo de dos. Cada elemento anade
`neutral_configured`: si el neutro procede del INI o de `--neutral` en vez
del valor provisional. Este indicador no certifica la calibracion fisica;
la habilitacion sigue comprobando feedback y RPM.

Desde la TUI 2.5, `telemetry` incluye `distance`: `enabled`, `valid`, `raw_cm`,
`kalman_cm`, `age_s`, `outlier_rejected`, `rejections`, `status`, `trig_gpio` y
`echo_gpio`. Las dos distancias se publican en centimetros; valen `null` cuando
no hay una lectura valida y reciente. `outlier_rejected` indica que Kalman ha
retenido la estimacion anterior frente a un salto. El sensor se muestrea cada
65 ms y MQTT envia la ultima muestra a la frecuencia de `--mqtt-hz`.
Vease el cableado y el filtro en [LEEME.md](LEEME.md#sensor-de-distancia).

Desde la TUI 2.6 el ultrasonidos queda desactivado por defecto (`enabled: false`)
hasta pasar `--distance-backend gpio`. Se anade `imu` a `telemetry`, con
`enabled`, `valid`, `address`, `accel_m_s2` (X/Y/Z, gravedad incluida),
`gyro_deg_s` (X/Y/Z), `temperature_c`, `roll_deg`, `pitch_deg`, `age_s` y `status`.
Los datos invalidos o caducados se publican como `null`. La IMU se consulta
a 20 Hz y MQTT publica la muestra mas reciente segun `--mqtt-hz`.
Roll/pitch son estimaciones por gravedad, sobre todo validas en reposo.

Telemetría y eventos usan QoS 0 y no se retienen. Los eventos se detectan a la
frecuencia de muestreo: cambios más rápidos pueden agruparse. Durante cortes se
descartan muestras; se reconecta automáticamente sin bloquear motores ni interfaz.
No es un registro exhaustivo de cada pulsación. `status` usa QoS 1 y retain;
el Last Will puede tardar en detectarse (keepalive de 15 segundos). Ese estado
indica conexión de la TUI, no confirma que el robot esté parado. Cada prefijo
debe pertenecer a una sola TUI; usa otro `--mqtt-topic` para una segunda instancia.

La integración publica trazas; no se suscribe a órdenes de movimiento.

Referencias: [Mosquitto](https://mosquitto.org/man/mosquitto-conf-5.html),
[cliente Paho](https://eclipse.dev/paho/files/paho.mqtt.python/html/client.html).
