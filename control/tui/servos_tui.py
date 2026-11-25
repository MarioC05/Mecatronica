#!/usr/bin/env python3
"""TUI para motores, microservo PCA4, NeoPixel y sensores en Raspberry Pi 4."""
import argparse
import configparser
import curses
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
import uuid
from datetime import datetime, timezone
from collections import deque

OUTPUT_MIN_US, OUTPUT_MAX_US = 1000, 2000
MIN_US, MAX_US = 1200, 1800
VERSION = "2.8.1"
DEFAULT_CHANNELS = [0, 1, 2, 3]
DEFAULT_PINS = [27, 17, 5, 6]


class IMUSensor:
    """MPU6050 en el mismo objeto I2C que la PCA; lecturas fuera del control."""
    PERIOD = .05
    STALE = .5
    RETRY = 2.0

    def __init__(self, args, i2c=None):
        self.args, self.i2c = args, i2c
        self.enabled = args.imu_backend != 'off'
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.thread = self.sensor = self.address = None
        self.accel = self.gyro = self.temperature = self.seen = None
        self.roll = self.pitch = None
        self.valid = False
        self.status = 'Buscando GY-521 / MPU6050' if self.enabled else 'Desactivada'

    def open(self):
        try:
            import adafruit_mpu6050
        except ImportError as exc:
            raise RuntimeError('Instala requirements-imu.txt en el entorno de la TUI') from exc
        if self.i2c is None:
            raise RuntimeError('Bus I2C de la PCA no disponible')
        addresses = [self.args.imu_address] if self.args.imu_address is not None else [0x68, 0x69]
        errors = []
        for address in addresses:
            try:
                sensor = adafruit_mpu6050.MPU6050(self.i2c, address=address)
                sensor.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
                sensor.gyro_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS
                sensor.filter_bandwidth = adafruit_mpu6050.Bandwidth.BAND_21_HZ
                sensor.sample_rate_divisor = 19
                self.sensor = sensor
                with self.lock:
                    self.address = address
                return
            except (OSError, ValueError, RuntimeError) as exc:
                errors.append(f'0x{address:02x}: {exc}')
        raise RuntimeError('; '.join(errors))

    def record(self, acceleration, gyro_rad_s, temperature, now):
        if (len(acceleration) != 3 or len(gyro_rad_s) != 3 or
                not all(math.isfinite(v) for v in (*acceleration, *gyro_rad_s, temperature))):
            raise ValueError('Muestra IMU no valida')
        ax, ay, az = acceleration
        roll = pitch = None
        if math.sqrt(ax * ax + ay * ay + az * az) > 1e-6:
            roll = math.degrees(math.atan2(ay, az))
            pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        with self.lock:
            self.accel = tuple(acceleration)
            self.gyro = tuple(math.degrees(v) for v in gyro_rad_s)
            self.temperature = temperature
            self.roll, self.pitch = roll, pitch
            self.seen, self.valid, self.status = now, True, 'OK'

    def snapshot(self):
        with self.lock:
            age = None if self.seen is None else max(0, time.monotonic() - self.seen)
            fresh = self.enabled and self.valid and age is not None and age <= self.STALE
            return {'enabled': self.enabled, 'valid': fresh,
                    'address': None if self.address is None else f'0x{self.address:02x}',
                    'accel_m_s2': dict(zip('xyz', self.accel)) if fresh else None,
                    'gyro_deg_s': dict(zip('xyz', self.gyro)) if fresh else None,
                    'temperature_c': self.temperature if fresh else None,
                    'roll_deg': self.roll if fresh else None,
                    'pitch_deg': self.pitch if fresh else None,
                    'age_s': age,
                    'status': 'LECTURA CADUCADA' if self.valid and not fresh else self.status}

    def run(self):
        while not self.done.is_set():
            started = time.monotonic()
            try:
                if self.args.simulate:
                    angle = .15 * math.sin(started / 3)
                    accel = (0.0, 9.80665 * math.sin(angle), 9.80665 * math.cos(angle))
                    gyro = (.05 * math.cos(started / 3), 0.0, 0.0)
                    temperature = 28.0
                else:
                    if self.sensor is None:
                        self.open()
                    accel = self.sensor.acceleration
                    gyro = self.sensor.gyro
                    temperature = self.sensor.temperature
                self.record(accel, gyro, temperature, time.monotonic())
            except Exception as exc:
                self.sensor = None
                with self.lock:
                    self.valid = False
                    self.status = 'ERROR IMU: ' + str(exc)
                self.done.wait(self.RETRY)
                continue
            self.done.wait(max(0, self.PERIOD - (time.monotonic() - started)))

    def start(self):
        if self.enabled:
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def close(self):
        self.done.set()
        if self.thread:
            self.thread.join(timeout=1)
        # El bus pertenece a Hardware; este lo libera tras parar la IMU.


class DistanceKalman:
    """Kalman escalar y rechazo de saltos, con los parametros del sketch."""
    def __init__(self):
        self.estimate = None
        self.p, self.q, self.r = 10.0, .05, 4.0
        self.rejections = 0

    def update(self, measurement):
        if not math.isfinite(measurement) or not 2 < measurement < 400:
            raise ValueError('Distancia fuera de rango (2..400 cm)')
        if self.estimate is None:
            self.estimate = measurement
            return self.estimate, True
        self.p += self.q
        if abs(measurement - self.estimate) > 15:
            self.rejections += 1
            if self.rejections < 5:
                return self.estimate, False
        else:
            self.rejections = 0
        gain = self.p / (self.p + self.r)
        self.estimate += gain * (measurement - self.estimate)
        self.p *= 1 - gain
        self.rejections = 0
        return self.estimate, True


class DistanceSensor:
    """Captura ECHO con timestamps lgpio; timeout fuera del hilo de motores."""
    TIMEOUT = .025
    PERIOD = .065
    STALE = .5

    def __init__(self, args):
        self.args = args
        self.enabled = args.distance_backend != 'off'
        self.lock = threading.Lock()
        self.echo_lock = threading.Lock()
        self.done, self.echo_done = threading.Event(), threading.Event()
        self.thread = None
        self.filter = DistanceKalman()
        self.raw = self.filtered = self.seen = None
        self.valid, self.rejected, self.rejections = False, False, 0
        self.status = 'Esperando lectura' if self.enabled else 'Desactivado'
        self.lgpio = self.handle = self.callback = None
        self.claimed = []
        self.waiting = False
        self.rise = self.width_us = None
        self.started_ns = 0

    def edge(self, chip, gpio, level, tick):
        with self.echo_lock:
            if (not self.waiting or tick < self.started_ns or
                    tick - self.started_ns > self.TIMEOUT * 1e9):
                return
            if level == 1:
                self.rise = tick
            elif level == 0 and self.rise is not None:
                self.width_us = (tick - self.rise) / 1000
                self.waiting = False
                self.echo_done.set()

    def measure(self):
        if self.lgpio.gpio_read(self.handle, self.args.echo_pin):
            return None  # ECHO atascado alto: no confundirlo con una distancia.
        with self.echo_lock:
            self.echo_done.clear()
            self.rise = self.width_us = None
            self.started_ns = time.monotonic_ns()
            self.waiting = True
        try:
            self.lgpio.tx_pulse(self.handle, self.args.trig_pin, 10, 10, pulse_cycles=1)
            self.echo_done.wait(self.TIMEOUT)
            with self.echo_lock:
                width = self.width_us
            if width is not None and 0 < width <= self.TIMEOUT * 1e6:
                return width * .0343 / 2
            return None
        finally:
            with self.echo_lock:
                self.waiting = False

    def record(self, measurement, now):
        with self.lock:
            if measurement is None or not math.isfinite(measurement) or not 2 < measurement < 400:
                self.raw = None
                self.valid, self.rejected = False, False
                self.status = 'SIN ECO / fuera de rango'
                return
            # Tras un corte largo no arrastrar una estimacion antigua.
            if self.seen is not None and now - self.seen > self.STALE:
                self.filter = DistanceKalman()
            self.filtered, accepted = self.filter.update(measurement)
            self.raw, self.seen, self.valid = measurement, now, True
            self.rejected = not accepted
            self.rejections = self.filter.rejections
            self.status = (f'OUTLIER {self.rejections}/5' if self.rejected else 'OK')

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            age = None if self.seen is None else max(0, now - self.seen)
            fresh = self.enabled and self.valid and age is not None and age <= self.STALE
            return {'enabled': self.enabled, 'valid': fresh,
                    'raw_cm': self.raw if fresh else None,
                    'kalman_cm': self.filtered if fresh else None,
                    'age_s': age, 'outlier_rejected': self.rejected if fresh else False,
                    'rejections': self.rejections,
                    'status': 'LECTURA CADUCADA' if self.valid and not fresh else self.status,
                    'trig_gpio': self.args.trig_pin, 'echo_gpio': self.args.echo_pin}

    def open(self):
        import lgpio
        self.lgpio = lgpio
        self.handle = lgpio.gpiochip_open(self.args.gpiochip)
        lgpio.gpio_claim_output(self.handle, self.args.trig_pin, 0)
        self.claimed.append(self.args.trig_pin)
        lgpio.gpio_claim_alert(self.handle, self.args.echo_pin, lgpio.BOTH_EDGES)
        self.claimed.append(self.args.echo_pin)
        self.callback = lgpio.callback(self.handle, self.args.echo_pin, lgpio.BOTH_EDGES, self.edge)

    def release(self):
        errors = []
        if self.callback:
            try:
                self.callback.cancel()
            except Exception as exc:
                errors.append(str(exc))
        if self.handle is not None:
            for pin in self.claimed:
                if pin == self.args.trig_pin:
                    try:
                        self.lgpio.tx_pulse(self.handle, pin, 0, 0)
                        self.lgpio.gpio_write(self.handle, pin, 0)
                    except Exception as exc:
                        errors.append(str(exc))
                try:
                    self.lgpio.gpio_free(self.handle, pin)
                except Exception as exc:
                    errors.append(str(exc))
            try:
                self.lgpio.gpiochip_close(self.handle)
            except Exception as exc:
                errors.append(str(exc))
            self.handle = None
        if errors:
            with self.lock:
                self.status = 'ERROR liberando sensor: ' + '; '.join(errors)
                self.valid = False

    def run(self):
        try:
            if not self.args.simulate:
                self.open()
            while not self.done.is_set():
                started = time.monotonic()
                if self.args.simulate:
                    # Senal determinista con ruido y picos para comprobar el filtro.
                    sample = round(started / self.PERIOD)
                    raw = 80 + 12 * math.sin(started / 4) + 2 * math.sin(sample * 2.4)
                    if sample % 40 == 0:
                        raw += 45
                else:
                    raw = self.measure()
                self.record(raw, time.monotonic())
                self.done.wait(max(0, self.PERIOD - (time.monotonic() - started)))
        except Exception as exc:
            with self.lock:
                self.valid = False
                self.status = 'ERROR sensor: ' + str(exc)
        finally:
            self.release()

    def start(self):
        if self.enabled:
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def close(self):
        self.done.set()
        self.echo_done.set()
        if self.thread:
            self.thread.join(timeout=1)


class MQTTTraces:
    """Telemetria opcional: ninguna operacion de red usa el hilo de control."""

    def __init__(self, args, client=None, distance=None, imu=None):
        self.args = args
        self.distance = distance
        self.imu = imu
        self.topic = args.mqtt_topic
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.status = 'Conectando'
        self.connected = threading.Event()
        self.done = threading.Event()
        self.thread = None
        self.previous = None
        if client is None:
            try:
                import paho.mqtt.client as mqtt
            except ImportError as exc:
                raise RuntimeError('MQTT requiere paho-mqtt >= 2: instala requirements-mqtt.txt') from exc
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                 client_id='eltronico-tui-' + self.session[:12])
        self.client = client
        client.on_connect = self.on_connect
        client.on_disconnect = self.on_disconnect
        client.on_connect_fail = self.on_connect_fail
        client.max_queued_messages_set(10)
        client.reconnect_delay_set(min_delay=1, max_delay=30)
        if args.mqtt_username:
            client.username_pw_set(args.mqtt_username, os.environ.get('MQTT_PASSWORD'))
        client.will_set(self.topic + '/status', self.encode({'state': 'offline'}),
                        qos=1, retain=True)

    def encode(self, payload):
        return json.dumps({'schema': 1, 'session': self.session,
                           'ts': datetime.now(timezone.utc).isoformat(),
                           **payload}, allow_nan=False, ensure_ascii=False)

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            self.status = 'Conexion rechazada: ' + str(reason_code)
            self.connected.clear()
            return
        self.status = 'Conectado'
        client.publish(self.topic + '/status', self.encode({'state': 'online'}),
                       qos=1, retain=True)
        self.connected.set()

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.connected.clear()
        self.status = 'Desconectado; reintentando'

    def on_connect_fail(self, client, userdata):
        self.connected.clear()
        self.status = 'Broker no disponible; reintentando'

    def snapshot(self, ctrl, extras):
        now = time.monotonic()
        with ctrl.lock:
            servos = []
            for i, fb in enumerate(ctrl.hw.feedback):
                angle, rpm, age = fb.read(now)
                fresh = age <= .25
                servos.append({'servo': i + 1, 'channel': self.args.channels[i],
                               'gpio': self.args.pins[i], 'command': ctrl.speeds[i],
                               'pwm_calculated_us': ctrl.hw.pulses[i],
                               'neutral_us': ctrl.neutrals[i],
                               'neutral_configured': self.args.channels[i] not in getattr(self.args, 'uncalibrated_channels', []),
                               'angle_deg': angle if fresh else None,
                               'rpm': rpm if fresh else None,
                               'feedback_age_s': age if math.isfinite(age) else None,
                               'feedback_ok': fresh})
            state = {'simulate': self.args.simulate, 'armed': ctrl.armed,
                     'message': ctrl.message, 'fault': ctrl.fault, 'servos': servos}
        state['led'] = extras.snapshot()
        if self.distance is not None:
            state['distance'] = self.distance.snapshot()
        if self.imu is not None:
            state['imu'] = self.imu.snapshot()
        state['monotonic_s'] = now
        return state

    def sample(self, ctrl, extras):
        # Descartar durante cortes: no reproducir telemetria antigua al reconectar.
        if not self.connected.is_set():
            return
        state = self.snapshot(ctrl, extras)
        self.sequence += 1
        self.client.publish(self.topic + '/telemetry',
                            self.encode({'seq': self.sequence, **state}), qos=0)
        event = {key: state[key] for key in ('armed', 'message', 'fault', 'led')}
        event['commands'] = [s['command'] for s in state['servos']]
        event['neutrals_us'] = [s['neutral_us'] for s in state['servos']]
        if event != self.previous:
            self.client.publish(self.topic + '/events',
                                self.encode({'seq': self.sequence, **event}), qos=0)
            self.previous = event

    def run(self, ctrl, extras):
        while not self.done.is_set():
            try:
                self.sample(ctrl, extras)
            except Exception as exc:
                self.status = 'Error de telemetria: ' + str(exc)
            self.done.wait(1 / self.args.mqtt_hz)

    def start(self, ctrl, extras):
        self.client.connect_async(self.args.mqtt_host, self.args.mqtt_port, keepalive=15)
        self.client.loop_start()
        self.thread = threading.Thread(target=self.run, args=(ctrl, extras), daemon=True)
        self.thread.start()

    def close(self):
        self.done.set()
        if self.thread:
            self.thread.join(timeout=1)
        try:
            if self.connected.is_set():
                info = self.client.publish(self.topic + '/status',
                                           self.encode({'state': 'offline'}), qos=1, retain=True)
                info.wait_for_publish(timeout=.5)
        finally:
            self.client.disconnect()
            self.client.loop_stop()


def speed_pulse(speed, neutral, deadband=25):
    if abs(speed) < 1e-9:
        return neutral
    speed = max(-1.0, min(1.0, speed))
    offset = (deadband + (220 - deadband) * abs(speed)) * neutral / 1500
    pulse = neutral + math.copysign(offset, speed)
    return max(OUTPUT_MIN_US, min(OUTPUT_MAX_US, pulse))


class Feedback:
    def __init__(self):
        self.lock = threading.Lock()
        self.rise = self.high = None
        self.samples = deque(maxlen=240)
        self.angle = self.previous = None
        self.unwrapped = 0.0
        self.seen = None

    def edge(self, chip, gpio, level, tick):
        with self.lock:
            if level == 0 and self.rise is not None:
                high = tick - self.rise
                if high > 0:
                    self.high = high
            elif level == 1:
                if self.rise is not None and self.high is not None:
                    period = tick - self.rise
                    if 900_000 <= period <= 1_300_000 and 0 < self.high < period:
                        duty = self.high / period
                        if 0.015 <= duty <= 0.985:
                            angle = (max(.027, min(.971, duty)) - .027) / .944 * 360
                            if self.previous is not None:
                                self.unwrapped += (angle - self.previous + 180) % 360 - 180
                            self.previous = self.angle = angle % 360
                            self.seen = time.monotonic()
                            self.samples.append((tick / 1e9, self.unwrapped))
                self.rise, self.high = tick, None

    def read(self, now):
        with self.lock:
            age = math.inf if self.seen is None else max(0, now - self.seen)
            rpm = None
            if len(self.samples) > 2:
                last = self.samples[-1]
                first = next((p for p in self.samples if p[0] >= last[0] - .15), self.samples[0])
                dt = last[0] - first[0]
                if dt >= .05:
                    rpm = (last[1] - first[1]) / dt / 6
            return self.angle, rpm, age


class SimFeedback:
    def __init__(self, index, hardware):
        self.index, self.hardware = index, hardware
        self.angle, self.last = 0.0, time.monotonic()

    def read(self, now):
        pulse = self.hardware.pulses[self.index]
        neutral = self.hardware.sim_neutrals[self.index]
        scale = neutral / 1500
        rpm = 0 if abs(pulse - neutral) <= 20 * scale else (pulse - neutral) / (220 * scale) * 120
        self.angle = (self.angle + max(0, now - self.last) * rpm * 6) % 360
        self.last = now
        return self.angle, rpm, 0.0


class Hardware:
    def __init__(self, args, neutrals):
        if not len(neutrals) == len(args.channels) == len(args.pins):
            raise ValueError('Debe haber un neutro y un feedback por canal PCA')
        self.args = args
        self.channels = args.channels
        self.pulses = list(neutrals)
        self.sim_neutrals = list(neutrals)
        self.callbacks = []
        self.claimed = []
        self.handle = self.i2c = self.pca = None
        self.lgpio = None
        self.feedback = []
        self.simulate = args.simulate
        if self.simulate:
            self.feedback = [SimFeedback(i, self) for i in range(len(self.channels))]
            return
        try:
            import board
            import busio
            import lgpio
            from adafruit_pca9685 import PCA9685
            self.lgpio = lgpio
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(self.i2c, address=args.address,
                               reference_clock_speed=args.clock_hz)
            self.pca.frequency = 50
            self.frequency = self.pca.frequency
            self.write(neutrals)
            self.handle = lgpio.gpiochip_open(args.gpiochip)
            for pin in args.pins:
                fb = Feedback()
                lgpio.gpio_claim_alert(self.handle, pin, lgpio.BOTH_EDGES)
                self.claimed.append(pin)
                self.callbacks.append(lgpio.callback(self.handle, pin, lgpio.BOTH_EDGES, fb.edge))
                self.feedback.append(fb)
        except BaseException:
            self.close(neutrals)
            raise

    def write(self, pulses):
        if len(pulses) != len(self.channels):
            raise ValueError('Debe haber un pulso por canal PCA')
        errors = []
        for i, pulse in enumerate(pulses):
            try:
                if not self.simulate:
                    counts = round(pulse * self.frequency * 4096 / 1e6)
                    self.pca.channels[self.channels[i]].duty_cycle = counts << 4
                    self.pulses[i] = counts / (self.frequency * 4096) * 1e6
                else:
                    self.pulses[i] = pulse
            except Exception as exc:
                errors.append(f'CH{self.channels[i]}: {exc}')
        if errors:
            raise RuntimeError('; '.join(errors))

    def close(self, neutrals):
        errors = []
        if self.simulate or self.pca is not None:
            for _ in range(3):
                try:
                    self.write(neutrals)
                    break
                except Exception as exc:
                    errors.append(str(exc))
                    time.sleep(.03)
            time.sleep(.1)
        for cb in self.callbacks:
            try:
                cb.cancel()
            except Exception as exc:
                errors.append(str(exc))
        if self.handle is not None:
            for pin in self.claimed:
                try:
                    self.lgpio.gpio_free(self.handle, pin)
                except Exception as exc:
                    errors.append(str(exc))
            try:
                self.lgpio.gpiochip_close(self.handle)
            except Exception as exc:
                errors.append(str(exc))
        if self.i2c is not None:
            try:
                self.i2c.deinit()
            except Exception as exc:
                errors.append(str(exc))
        return errors


class MicroServoControl:
    """Microservo posicional PCA4, ordenado en grados y sin feedback."""
    MIN_ANGLE = 0.0
    MAX_ANGLE = 180.0

    def __init__(self, hardware):
        self.hardware = hardware
        self.angle = 90.0  # Objetivo inicial mostrado; no se manda al arrancar.
        self.active = False
        self.servo = None
        if not hardware.simulate:
            from adafruit_motor import servo
            self.servo = servo.Servo(hardware.pca.channels[4])
            self.servo.angle = None

    def command(self, angle):
        angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, float(angle)))
        if self.servo is not None:
            self.servo.angle = angle
        self.angle = angle
        self.active = True

    def stop(self):
        if self.servo is not None:
            self.servo.angle = None
        self.active = False


class Control:
    def __init__(self, hardware, neutrals, args):
        if len(neutrals) != len(hardware.feedback):
            raise ValueError('Debe haber un feedback por motor')
        self.hw, self.args = hardware, args
        self.lock = threading.RLock()
        self.neutrals = list(neutrals)
        self.speeds = [0.0] * len(neutrals)
        self.armed = False
        self.message = 'Calibra con J/L sin pulsar A; S guarda. A solo habilita el movimiento.'
        self.fault = None
        self.last_ui = self.last_command = time.monotonic()
        self.last_sent = None
        self.last_sent_at = 0.0
        self.done = threading.Event()
        self.thread = None

    def stop(self, message='Parada solicitada'):
        with self.lock:
            self.speeds[:] = [0.0] * len(self.neutrals)
            self.armed = False
            self.message = message

    def arm(self, now):
        with self.lock:
            if self.fault:
                self.message = 'Fallo de salida: reinicia tras revisar I2C.'
            elif any(fb.read(now)[2] > .25 for fb in self.hw.feedback):
                self.message = 'Falta feedback: movimiento bloqueado. J/L permite ajustar neutros sin A.'
            elif any(fb.read(now)[1] is None or abs(fb.read(now)[1]) > 4
                     for fb in self.hw.feedback):
                self.message = 'RPM altas: ajusta neutros con J/L sin A hasta parar; S guarda.'
            else:
                self.speeds[:] = [0.0] * len(self.neutrals)
                self.armed = True
                self.last_command = now
                self.message = 'Habilitado. W/S cambia velocidad; espacio para parar.'

    def jog(self, selected, delta, mirror, now):
        with self.lock:
            if not self.armed:
                self.message = 'W/S mueve y requiere A. Para ajustar el neutro usa J/L sin A.'
                return
            for position, i in enumerate(selected):
                sign = -1 if mirror and position % 2 else 1
                self.speeds[i] = max(-self.args.limit, min(self.args.limit,
                                                        round(self.speeds[i] + delta * sign, 3)))
            self.last_command = now

    def trim(self, selected, delta):
        with self.lock:
            self.stop('Neutro ajustado sin A. J/L: seguir ajustando | S: guardar | A: mover.')
            for i in selected:
                self.neutrals[i] = max(MIN_US, min(MAX_US, self.neutrals[i] + delta))

    def tick(self, now):
        with self.lock:
            moving = any(abs(s) > 1e-9 for s in self.speeds)
            if now - self.last_ui > .5:
                self.stop('Parada: interfaz sin respuesta.')
            elif moving and self.args.deadman > 0 and now - self.last_command > self.args.deadman:
                self.stop('Parada: tiempo sin orden. Pulsa A para habilitar.')
            elif moving and any(self.hw.feedback[i].read(now)[2] > .25
                                for i, s in enumerate(self.speeds) if abs(s) > 1e-9):
                self.stop('Parada: feedback perdido.')
            pulses = [speed_pulse(s if self.armed else 0, n, getattr(self.args, 'deadband_us', 25))
                      for s, n in zip(self.speeds, self.neutrals)]
            if pulses != self.last_sent or now - self.last_sent_at >= .1:
                self.hw.write(pulses)
                self.last_sent = pulses
                self.last_sent_at = now

    def run(self):
        while not self.done.is_set():
            try:
                self.tick(time.monotonic())
            except Exception as exc:
                with self.lock:
                    self.fault = str(exc)
                    self.stop('FALLO I2C: no se garantiza la parada; corta alimentacion.')
                    try:
                        self.hw.write(self.neutrals)
                    except Exception:
                        pass
                return
            self.done.wait(.02)

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def finish(self):
        self.done.set()
        if self.thread:
            self.thread.join(timeout=1)
        return self.hw.close(self.neutrals)


def save_config(path, values, channels):
    cfg = configparser.ConfigParser()
    if path.exists():
        cfg.read(path)
    if not cfg.has_section('neutral'):
        cfg.add_section('neutral')
    for channel, value in zip(channels, values):
        cfg.set('neutral', f'channel_{channel}', str(value))
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    with tmp.open('w') as f:
        cfg.write(f)
    os.replace(tmp, path)


def load_neutrals(args):
    """Cargar por canal PCA; conservar los neutros antiguos al anadir motores."""
    args.uncalibrated_channels = []
    if args.neutral is not None:
        return list(args.neutral)
    cfg = configparser.ConfigParser()
    if args.config.exists():
        with args.config.open() as f:
            cfg.read_file(f)
    values = []
    for channel in args.channels:
        key = f'channel_{channel}'
        if not cfg.has_option('neutral', key):
            args.uncalibrated_channels.append(channel)
        values.append(cfg.getfloat('neutral', key, fallback=1500))
    if any(not math.isfinite(n) or not MIN_US <= n <= MAX_US for n in values):
        raise ValueError('Neutros del INI fuera de rango')
    return values


def servo_selection(key, count):
    """Seleccion individual o por parejas en el orden de --channels."""
    if ord('1') <= key < ord('1') + count:
        return [key - ord('1')], False
    if key in (ord('b'), ord('B'), ord('e'), ord('E')):
        return [0, 1], key in (ord('e'), ord('E'))
    if count >= 4 and key in (ord('r'), ord('R'), ord('f'), ord('F')):
        return [2, 3], key in (ord('f'), ord('F'))
    if key in (ord('t'), ord('T')):
        return list(range(count)), False
    return None


class Extras:
    """Manejo de tiras LED SPI (NeoPixel) fuera del hilo de la TUI."""
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.thread = None
        self.rgb = [255, 80, 0]
        self.brightness = 10
        self.led_on = False
        self.generation = 0
        self.applied = 0
        self.led_status = 'Sin encender | DATA BCM10, pin 19 | SPI0 MOSI'
        self.pixels = self.spi = None

    def snapshot(self):
        with self.lock:
            return {'rgb': list(self.rgb), 'brightness': self.brightness,
                    'led_on': self.led_on, 'led_status': self.led_status}

    def led_change(self, field=None, delta=0, absolute=None, toggle=False, off=False):
        with self.lock:
            if field is not None:
                value = self.rgb[field] if field < 3 else self.brightness
                value = value + delta if absolute is None else absolute
                value = max(0, min(255 if field < 3 else 100, value))
                if field < 3:
                    self.rgb[field] = value
                else:
                    self.brightness = value
            if toggle:
                self.led_on = not self.led_on
            if off:
                self.led_on = False
            self.generation += 1

    def apply_leds(self, rgb, brightness, on):
        if self.args.simulate:
            return 'SIMULADO | BCM10 / pin 19 | ' + ('ENCENDIDO' if on else 'APAGADO')
        if self.args.led_backend == 'off':
            return 'Desactivado con --led-backend off'

        if self.pixels is None:
            if not on:
                return 'Sin inicializar | BCM10 / pin 19'
            import board
            import neopixel_spi

            self.spi = board.SPI()

            self.pixels = neopixel_spi.NeoPixel_SPI(
                self.spi,
                self.args.led_count,
                auto_write=False,
                brightness=1.0,
                pixel_order=self.args.led_order,
                bpp=4 if 'W' in self.args.led_order else 3,
                frequency=6_400_000,
                bit0=0b10000000,
            )

        # Escalamos el brillo aplicando el factor a los componentes RGB directamente
        factor = (brightness / 100.0) if on else 0.0
        r_val = int(rgb[0] * factor)
        g_val = int(rgb[1] * factor)
        b_val = int(rgb[2] * factor)

        has_w = 'W' in self.args.led_order
        color = (r_val, g_val, b_val, 0) if has_w else (r_val, g_val, b_val)

        # Asignar color y refrescar mediante la librería
        self.pixels.fill(color)
        self.pixels.show()

        return f'{"ENCENDIDO" if on else "APAGADO"} | {self.args.led_count} LED | BCM10 / pin 19'

    def run(self):
        try:
            while not self.done.is_set():
                with self.lock:
                    lg = self.generation
                    rgb, brightness, on = list(self.rgb), self.brightness, self.led_on
                if lg != self.applied:
                    try:
                        status = self.apply_leds(rgb, brightness, on)
                    except Exception as exc:
                        status = 'ERROR LED: ' + str(exc)
                    with self.lock:
                        self.led_status = status
                    self.applied = lg
                    time.sleep(0.02)  # Ventana de espera para garantizar latch en bus SPI
                self.done.wait(.025)
        finally:
            if self.pixels is not None:
                try:
                    self.pixels.deinit()
                except Exception:
                    pass
            if self.spi is not None:
                try:
                    self.spi.deinit()
                except Exception:
                    pass

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def close(self):
        self.done.set()
        if self.thread:
            self.thread.join(timeout=1.5)


def slider(value, maximum, width=16):
    filled = round(width * value / maximum)
    return '[' + '#' * filled + '-' * (width - filled) + ']'


BIG_ANGLE_GLYPHS = {
    '0': (' ### ', '#   #', '#  ##', '# # #', '##  #', '#   #', ' ### '),
    '1': ('  #  ', ' ##  ', '# #  ', '  #  ', '  #  ', '  #  ', '#####'),
    '2': (' ### ', '#   #', '    #', '   # ', '  #  ', ' #   ', '#####'),
    '3': ('#### ', '    #', '    #', ' ### ', '    #', '    #', '#### '),
    '4': ('   # ', '  ## ', ' # # ', '#  # ', '#####', '   # ', '   # '),
    '5': ('#####', '#    ', '#    ', '#### ', '    #', '#   #', ' ### '),
    '6': (' ### ', '#   #', '#    ', '#### ', '#   #', '#   #', ' ### '),
    '7': ('#####', '    #', '   # ', '  #  ', ' #   ', ' #   ', ' #   '),
    '8': (' ### ', '#   #', '#   #', ' ### ', '#   #', '#   #', ' ### '),
    '9': (' ### ', '#   #', '#   #', ' ####', '    #', '#   #', ' ### '),
}


def draw_big_angle(screen, angle, top=4):
    text = f'{angle:03.0f}'
    height, width = screen.getmaxyx()
    glyph_width, gap = 5, 1
    left = max(0, (width - (len(text) * glyph_width + (len(text) - 1) * gap)) // 2)
    for index, character in enumerate(text):
        for row, pattern in enumerate(BIG_ANGLE_GLYPHS[character]):
            if top + row < height:
                try:
                    screen.addstr(top + row, left + index * (glyph_width + gap),
                                  pattern, curses.A_BOLD)
                except curses.error:
                    pass


def prompt_angle(screen):
    height, width = screen.getmaxyx()
    prompt = f'Angulo absoluto [0-{MicroServoControl.MAX_ANGLE:.0f}] grados: '
    row = max(0, height - 2)
    screen.move(row, 0)
    screen.clrtoeol()
    screen.addnstr(row, 0, prompt, max(0, width - 1))
    screen.refresh()
    screen.timeout(-1)
    curses.echo()
    curses.curs_set(1)
    try:
        raw = screen.getstr(row, min(len(prompt), max(0, width - 1)), 3)
    finally:
        curses.noecho()
        curses.curs_set(0)
        screen.timeout(40)
    try:
        angle = int(raw.decode('ascii').strip())
        if not MicroServoControl.MIN_ANGLE <= angle <= MicroServoControl.MAX_ANGLE:
            raise ValueError
    except (AttributeError, UnicodeDecodeError, ValueError):
        return None
    return angle


def tui(screen, ctrl, args, quit_event, extras, traces=None, distance=None, imu=None,
        microservo=None):
    curses.curs_set(0)
    curses.set_escdelay(25)
    screen.keypad(True)
    screen.timeout(40)
    selected, mirror, page, led_field = [0], False, 0, 0
    count = len(ctrl.neutrals)
    sensor_row = 5 + count
    body = sensor_row + 1
    footer = max(body + 8, 18 if microservo else 0)
    while not quit_event.is_set():
        now = time.monotonic()
        ex = extras.snapshot()
        ds = distance.snapshot() if distance else None
        im = imu.snapshot() if imu else None
        with ctrl.lock:
            ctrl.last_ui = now
            screen.erase()
            height, width = screen.getmaxyx()
            def put(row, text, attr=0):
                if row < height - 1 and width > 2:
                    try:
                        screen.addnstr(row, 1, text, width - 2, attr)
                    except curses.error:
                        pass
            put(0, 'ELTRONICO | TUI ' + VERSION + ' | PI4B | ' + ('SIMULACION' if args.simulate else 'HARDWARE'), curses.A_BOLD)
            tabs = ['SERVOS'] + (['MICROSERVO'] if microservo else []) + ['NEOPIXEL', 'IMU']
            put(1, 'TAB: ' + '  '.join('['+t+']' if i == page else t for i, t in enumerate(tabs)))
            if page == 0:
                put(2, f"{'HABILITADO' if ctrl.armed else 'NEUTRO SOLICITADO'} | Servo(s): {','.join(str(i+1) for i in selected)} | {'espejo' if mirror else 'mismo signo'}")
                mode = f'timeout {args.deadman:.1f}s' if args.deadman else 'mando fijo hasta ESPACIO'
                put(3, f"Limite {args.limit:.0%} | {mode} | GPIO {'/'.join(map(str, args.pins))}")
                put(4, 'Servo  Canal GPIO  Orden    PWM calc. Neutro(us) Angulo    RPM     Feedback', curses.A_BOLD)
                for i, fb in enumerate(ctrl.hw.feedback):
                    angle, rpm, age = fb.read(now)
                    angle_text = '   ---' if angle is None or age > .25 else f'{angle:6.1f}'
                    rpm_text = '   ---' if rpm is None or age > .25 else f'{rpm:7.1f}'
                    status = 'SIN SENAL' if age > .25 else 'OK'
                    if not ctrl.armed and rpm is not None and age <= .25 and abs(rpm) > 4:
                        status = 'NO PARADO: ajustar neutro'
                    if args.channels[i] in getattr(args, 'uncalibrated_channels', []):
                        status += ' | CALIBRAR'
                    put(5+i, f'{i+1:>3} {args.channels[i]:>6} {args.pins[i]:>4} {ctrl.speeds[i]:>7.0%} {ctrl.hw.pulses[i]:>10.1f} {ctrl.neutrals[i]:>10.0f} {angle_text} {rpm_text}  {status}')
            if ds and page == 0:
                raw = '---' if ds['raw_cm'] is None else f"{ds['raw_cm']:.1f}"
                kalman = '---' if ds['kalman_cm'] is None else f"{ds['kalman_cm']:.1f}"
                if ds['enabled']:
                    put(sensor_row, f"DISTANCIA | Original {raw} cm | Kalman {kalman} cm | {ds['status']}")
                elif im:
                    put(sensor_row, 'IMU: ' + im['status'] + ' | TAB para ver GY-521 | Ultrasonidos desactivado')
            if page == 0:
                put(body, f'1..{count}: servo | B/E: pareja 1-2 normal/espejo | R/F: rodillas 3-4' if count == 4 else '1/2: servo | B: ambos | E: espejo')
                put(body+1, 'T: todos | A: habilitar | W/S o arriba/abajo: velocidad +/-5%')
                put(body+2, f'J/L o [/]: neutro -/+5 us SIN A | rango {MIN_US}-{MAX_US} us')
                put(body+3, 'S MAYUSCULA: guardar neutros | Limite de mando por defecto: 100%')
                put(body+5, 'TAB: microservo, luces e IMU.')
            elif microservo and page == 1:
                put(2, 'MICROSERVO PCA4 | objetivo absoluto; no mide el angulo fisico', curses.A_BOLD)
                put(3, 'Canal independiente, sin feedback | el arranque no ordena movimiento')
                draw_big_angle(screen, microservo.angle)
                put(11, f'Objetivo: {microservo.angle:.0f} grados | Salida: {"ACTIVA" if microservo.active else "INACTIVA"}')
                put(12, f'Rango nominal SG90: 0-{MicroServoControl.MAX_ANGLE:.0f} grados')
                put(13, 'G: escribir angulo y confirmar con Enter')
                put(14, 'Flechas izq./der. o J/L: -/+ 1 grado')
                put(15, f'RePág/AvPág: -/+ 10 | Inicio: 0 | Fin: {MicroServoControl.MAX_ANGLE:.0f} | C: 90')
                put(16, 'ESPACIO: desactivar salida | Q: salir')
                put(17, ctrl.message if ctrl.fault else 'Prueba sin carga; detente si fuerza el tope.')
            elif page == (2 if microservo else 1):
                put(body, f'NEOPIXEL: {args.led_count} LED | DATA BCM10, pin fisico 19 | SPI0 MOSI', curses.A_BOLD)
                for i, label in enumerate(['Rojo', 'Verde', 'Azul', 'Brillo']):
                    value, maximum = (ex['rgb'][i], 255) if i < 3 else (ex['brightness'], 100)
                    suffix = '%' if i == 3 else '/255'
                    put(body+1+i, f'{">" if i == led_field else " "} {label:<7} {slider(value,maximum)} {value:3}{suffix}', curses.A_BOLD if i == led_field else 0)
                put(body+5, 'Arriba/abajo: elegir slider | izda/dcha: ajustar | ENTER: encender/apagar')
                put(body+6, '0: apagar | Home/End: minimo/maximo | ' + ('ENCENDIDO' if ex['led_on'] else 'APAGADO'))
            elif page == (3 if microservo else 2) and im:
                address = im['address'] or '0x68/0x69 auto'
                put(body, f'GY-521 / MPU6050 | I2C {address} | {im["status"]}', curses.A_BOLD)
                if im['valid']:
                    ac, gy = im['accel_m_s2'], im['gyro_deg_s']
                    put(body+1, f'Accel (m/s2): X {ac["x"]:+8.3f}  Y {ac["y"]:+8.3f}  Z {ac["z"]:+8.3f}')
                    put(body+2, f'Giro  (deg/s): X {gy["x"]:+8.3f}  Y {gy["y"]:+8.3f}  Z {gy["z"]:+8.3f}')
                    roll = '---' if im['roll_deg'] is None else f'{im["roll_deg"]:+.1f}'
                    pitch = '---' if im['pitch_deg'] is None else f'{im["pitch_deg"]:+.1f}'
                    put(body+3, f'Inclinacion por gravedad: Roll {roll} deg | Pitch {pitch} deg')
                    put(body+4, f'Temperatura chip: {im["temperature_c"]:.1f} C | Edad: {im["age_s"]:.2f} s')
                else:
                    put(body+1, 'Sin datos recientes. Revisa alimentacion, SDA/SCL y dependencia MPU6050.')
                put(body+5, 'Roll/pitch aproximados en reposo; aceleracion incluye gravedad.')
                put(body+6, 'Ejes del modulo | Filtro interno 21 Hz | Lectura 20 Hz | TAB para cambiar')
            global_keys = 'ESPACIO/ESC = motores neutros'
            if microservo:
                global_keys += ' + microservo desactivado'
            put(footer, 'GLOBAL: ' + global_keys + ' | Q = salir', curses.A_BOLD)
            put(footer+1, ctrl.message)
            put(footer+2, 'LED: ' + ex['led_status'])
            if ctrl.fault:
                put(footer+3, ctrl.fault, curses.A_BOLD)
            else:
                put(footer+3, 'PWM calculado, no medido. Un neutro no es un freno mecanico.')
            put(footer+4, 'MQTT: ' + (traces.status if traces else 'desactivado (--mqtt-host HOST)'))
            if height < footer + 6 or width < 110:
                put(height-2, f'Amplia la terminal: recomendado 110 columnas x {footer + 6} filas.')
            screen.refresh()
        key = screen.getch()
        now = time.monotonic()
        if key in (ord('q'), ord('Q'), 3):
            break
        if key in (32, 27):
            ctrl.stop()
            if microservo:
                microservo.stop()
        elif key in (9, curses.KEY_BTAB):
            ctrl.stop('Pestana cambiada: motores en neutro.')
            page = (page + (-1 if key == curses.KEY_BTAB else 1)) % len(tabs)
        elif page == 0:
            selection = servo_selection(key, count)
            if key in (ord('a'), ord('A')):
                ctrl.arm(now)
            elif selection is not None:
                ctrl.stop('Seleccion cambiada. J/L: neutro sin A | A: habilitar movimiento.')
                selected, mirror = selection
            elif key in (curses.KEY_UP, ord('w'), ord('W')):
                ctrl.jog(selected, .05, mirror, now)
            elif key in (curses.KEY_DOWN, ord('s'), ord('S')):
                if key == ord('S'):
                    ctrl.stop('Guardando neutros.')
                    try:
                        save_config(args.config, ctrl.neutrals, args.channels)
                        args.uncalibrated_channels = []
                        ctrl.message = 'Neutros guardados en ' + str(args.config)
                    except Exception as exc:
                        ctrl.message = 'No se pudo guardar: ' + str(exc)
                else:
                    ctrl.jog(selected, -.05, mirror, now)
            elif key in (ord('['), ord(']'), ord('j'), ord('J'), ord('l'), ord('L')):
                ctrl.trim(selected, -5 if key in (ord('['), ord('j'), ord('J')) else 5)
        elif microservo and page == 1:
            if key in (curses.KEY_RIGHT, ord('l'), ord('L'), ord('+'), ord('=')):
                microservo.command(microservo.angle + 1)
            elif key in (curses.KEY_LEFT, ord('j'), ord('J'), ord('-'), ord('_')):
                microservo.command(microservo.angle - 1)
            elif key in (curses.KEY_NPAGE, ord('<')):
                microservo.command(microservo.angle - 10)
            elif key in (curses.KEY_PPAGE, ord('>')):
                microservo.command(microservo.angle + 10)
            elif key in (curses.KEY_HOME, ord('0')):
                microservo.command(0)
            elif key == curses.KEY_END:
                microservo.command(MicroServoControl.MAX_ANGLE)
            elif key in (ord('c'), ord('C')):
                microservo.command(90)
            elif key in (ord('g'), ord('G')):
                angle = prompt_angle(screen)
                if angle is None:
                    ctrl.message = f'Entrada invalida: usa un entero entre 0 y {MicroServoControl.MAX_ANGLE:.0f} grados.'
                else:
                    microservo.command(angle)
                    ctrl.message = f'Microservo: objetivo de {angle} grados ordenado.'
        elif page == (2 if microservo else 1):
            if key == curses.KEY_UP:
                led_field = (led_field - 1) % 4
            elif key == curses.KEY_DOWN:
                led_field = (led_field + 1) % 4
            elif key in (curses.KEY_LEFT, ord('-')):
                extras.led_change(led_field, -5)
            elif key in (curses.KEY_RIGHT, ord('+'), ord('=')):
                extras.led_change(led_field, 5)
            elif key == curses.KEY_HOME:
                extras.led_change(led_field, absolute=0)
            elif key == curses.KEY_END:
                extras.led_change(led_field, absolute=255 if led_field < 3 else 100)
            elif key in (10, 13, curses.KEY_ENTER):
                extras.led_change(toggle=True)
            elif key == ord('0'):
                extras.led_change(off=True)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulate', action='store_true', help='TUI sin hardware ni librerias de Raspberry')
    p.add_argument('--channels', type=int, nargs='+', metavar='CH',
                   help='2 o 4 canales PCA; por defecto 0 1 2 3')
    p.add_argument('--pins', type=int, nargs='+', metavar='BCM',
                   help='Feedback por canal; por defecto 27 17 5 6')
    p.add_argument('--gpiochip', type=int, default=0)
    p.add_argument('--address', type=lambda x: int(x, 0), default=0x40)
    p.add_argument('--clock-hz', type=int, default=25_000_000, help='Reloj PCA calibrado; por defecto 25 MHz')
    p.add_argument('--neutral', type=float, nargs='+', metavar='US',
                   help='Un neutro por canal; sin esta opcion se conserva el INI')
    p.add_argument('--limit', type=float, default=1.0, help='Limite de mando, 0.05..1; por defecto 1.0 (100%%)')
    p.add_argument('--deadman', type=float, default=0, help='0: mando fijo; o 0.2..3 segundos sin orden para parar')
    p.add_argument('--deadband-us', type=float, default=25, help='Margen de arranque, 0..60 us equivalentes; por defecto 25')
    p.add_argument('--led-count', type=int, default=12)
    p.add_argument('--led-backend', choices=['spi', 'off'], default='spi')
    p.add_argument('--led-order', choices=['GRB', 'RGB', 'GRBW', 'RGBW'], default='GRB')
    p.add_argument('--distance-backend', choices=['gpio', 'off'], default='off',
                   help='Ultrasonidos preparado pero desactivado por defecto')
    p.add_argument('--trig-pin', type=int, default=13, help='TRIG ultrasonico, GPIO BCM (fisico 33)')
    p.add_argument('--echo-pin', type=int, default=14, help='ECHO ultrasonico, GPIO BCM (fisico 8); nivel 3.3 V')
    p.add_argument('--imu-backend', choices=['i2c', 'off'], default='i2c')
    p.add_argument('--imu-address', type=lambda x: int(x, 0),
                   help='Direccion MPU6050: 0x68 o 0x69; por defecto prueba ambas')
    p.add_argument('--config', type=Path, default=Path.home()/'.config/eltronico/servos.ini')
    p.add_argument('--mqtt-host', help='Activar trazas MQTT en este broker (p. ej. localhost)')
    p.add_argument('--mqtt-port', type=int, default=1883)
    p.add_argument('--mqtt-topic', default='eltronico/tui', help='Prefijo de topics')
    p.add_argument('--mqtt-hz', type=float, default=5, help='Frecuencia de telemetria, 0.2..20 Hz')
    p.add_argument('--mqtt-username', default=os.environ.get('MQTT_USERNAME'),
                   help='Usuario MQTT; contrasena en variable MQTT_PASSWORD')
    a = p.parse_args()
    if a.channels is None:
        a.channels = DEFAULT_CHANNELS[:len(a.pins)] if a.pins is not None else list(DEFAULT_CHANNELS)
    if a.pins is None:
        a.pins = DEFAULT_PINS[:len(a.channels)]
    if len(a.channels) not in (2, 4) or len(a.pins) != len(a.channels):
        p.error('Selecciona 2 o 4 canales y el mismo numero de GPIO de feedback')
    if a.neutral is not None and len(a.neutral) != len(a.channels):
        p.error('--neutral requiere un valor por canal PCA')
    if a.imu_address is not None and a.imu_address not in (0x68, 0x69):
        p.error('--imu-address debe ser 0x68 o 0x69')
    if a.imu_backend != 'off' and a.address in (0x68, 0x69):
        p.error('La direccion PCA no debe coincidir con las direcciones de la IMU')
    a.mqtt_topic = a.mqtt_topic.strip('/')
    if not a.mqtt_topic or any(c in a.mqtt_topic for c in '+#\x00'):
        p.error('--mqtt-topic debe ser un prefijo sin comodines ni NUL')
    if not 1 <= a.mqtt_port <= 65535 or not .2 <= a.mqtt_hz <= 20:
        p.error('--mqtt-port entre 1 y 65535; --mqtt-hz entre 0.2 y 20')
    if len(set(a.channels)) != len(a.channels) or any(not 0 <= c < 16 for c in a.channels):
        p.error('Canales distintos de 0 a 15')
    if len(set(a.pins)) != len(a.pins) or any(not 0 <= pin <= 27 for pin in a.pins):
        p.error('GPIO BCM distintos de 0 a 27')
    if set(a.pins) & {0, 1, 2, 3}:
        p.error('Feedback no puede ocupar los GPIO de HAT/I2C')
    if a.distance_backend != 'off':
        distance_pins = {a.trig_pin, a.echo_pin}
        if len(distance_pins) != 2 or any(not 0 <= pin <= 27 for pin in distance_pins):
            p.error('TRIG/ECHO deben ser GPIO BCM distintos entre 0 y 27')
        reserved = {0, 1, 2, 3} | set(a.pins)  # HAT EEPROM, I2C PCA y feedback.
        if a.led_backend == 'spi':
            reserved |= {7, 8, 9, 10, 11}
        if distance_pins & reserved:
            p.error('TRIG/ECHO en conflicto con HAT/I2C, feedback de servos o SPI')
    if not .05 <= a.limit <= 1 or not (a.deadman == 0 or .2 <= a.deadman <= 3):
        p.error('--limit entre 0.05 y 1; --deadman 0 o entre 0.2 y 3')
    if not 1 <= a.led_count <= 256:
        p.error('--led-count entre 1 y 256')
    if a.led_backend == 'spi' and set(a.pins) & {7, 8, 9, 10, 11}:
        p.error('Feedback no puede ocupar pines SPI cuando el anillo usa SPI')
    if not 0 <= a.deadband_us <= 60:
        p.error('--deadband-us entre 0 y 60')
    if not 0x03 <= a.address <= 0x77 or a.clock_hz <= 0:
        p.error('Direccion I2C o reloj incorrectos')
    if a.neutral and any(not math.isfinite(v) or not MIN_US <= v <= MAX_US for v in a.neutral):
        p.error(f'Neutros entre {MIN_US} y {MAX_US} us')
    return a


def main():
    args = parse_args()
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise SystemExit('Ejecuta en una terminal interactiva, sin redirigir stdin/stdout.')
    try:
        neutrals = load_neutrals(args)
    except (OSError, ValueError, configparser.Error) as exc:
        raise SystemExit(str(exc)) from exc
    quit_event = threading.Event()

    def request_quit(signum, frame):
        quit_event.set()

    old_handlers = {s: signal.signal(s, request_quit) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    hardware = ctrl = extras = traces = distance = imu = microservo = None
    errors = []
    try:
        distance = DistanceSensor(args)
        imu = IMUSensor(args)
        if args.mqtt_host:
            traces = MQTTTraces(args, distance=distance, imu=imu)
        hardware = Hardware(args, neutrals)
        microservo = MicroServoControl(hardware)
        imu.i2c = hardware.i2c
        ctrl = Control(hardware, neutrals, args)
        ctrl.start()
        extras = Extras(args)
        extras.start()
        distance.start()
        imu.start()
        if traces:
            traces.start(ctrl, extras)
        curses.wrapper(tui, ctrl, args, quit_event, extras, traces, distance, imu, microservo)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    finally:
        if ctrl:
            ctrl.stop('Cierre de la TUI: neutros solicitados.')
        if microservo:
            microservo.stop()
        if imu:
            imu.close()
        if ctrl:
            errors = ctrl.finish()
        elif hardware:
            errors = hardware.close(neutrals)
        if extras:
            extras.close()
        if distance:
            distance.close()
        if traces:
            try:
                traces.close()
            except Exception as exc:
                print('Error cerrando MQTT: ' + str(exc), file=sys.stderr)
        for s, handler in old_handlers.items():
            signal.signal(s, handler)
        if errors:
            print('Fallo al parar/liberar: ' + '; '.join(errors), file=sys.stderr)
        elif hardware:
            print('Enviados los neutros. La PCA mantiene esa senal; no es un freno mecanico.')
    return 1 if errors or (ctrl and ctrl.fault) else 0


if __name__ == '__main__':
    sys.exit(main())
