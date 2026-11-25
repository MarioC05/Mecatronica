#!/usr/bin/env python3
"""ROS 2 device interface and remote client for the Eltronico curses TUI."""
import curses
import json
import math
import sys
import threading
import time
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Bool, Float64
from . import servos_tui as app


class DeviceNode(Node):
    def __init__(self, args):
        super().__init__('eltronico_devices')
        self.args = args
        if 4 in args.channels:
            raise ValueError('PCA4 está reservado al microservo; no usarlo como motor continuo')
        neutrals = app.load_neutrals(args)
        self.hw = app.Hardware(args, neutrals)
        self.ctrl = app.Control(self.hw, neutrals, args)
        self.microservo = app.MicroServoControl(self.hw)
        self.extras = app.Extras(args)
        self.distance = app.DistanceSensor(args)
        self.imu = app.IMUSensor(args, self.hw.i2c)
        self.sources = {}
        self.bind('robot', self.robot_state)
        self.bind('leds', self.extras.snapshot)
        self.bind('imu', self.imu.snapshot)
        self.bind('distance', self.distance.snapshot)
        self.bind('microservo', self.microservo_state)
        self.create_subscription(Float64, '/microservo/set', self.microservo_command, 10)
        self.create_subscription(Empty, '/microservo/stop/set', lambda m: self.microservo.stop(), 10)
        for i in range(len(args.channels)):
            self.bind(f'servos/servo{i+1}', lambda i=i: self.servo_state(i))
            self.create_subscription(Float64, f'/servos/servo{i+1}/set', lambda m, i=i: self.set_speed(i, m.data), 10)
        self.create_subscription(String, '/robot/set', self.command, 10)
        self.create_subscription(String, '/leds/set', self.led_command, 10)
        self.create_subscription(Empty, '/robot/heartbeat/set', self.heartbeat, 10)
        self.create_subscription(Bool, '/robot/armed/set', lambda m: self.ctrl.arm(time.monotonic()) if m.data else self.ctrl.stop(), 10)
        self.create_subscription(Empty, '/robot/stop/set', lambda m: self.stop(), 10)
        self.ctrl.start()
        self.extras.start()
        self.distance.start()
        self.imu.start()
        self.create_timer(.05, self.publish_all)

    def bind(self, name, source):
        publisher = self.create_publisher(String, '/' + name + '/state', 10)
        self.sources[name] = (publisher, source)
        self.create_subscription(Empty, '/' + name + '/get', lambda m: self.publish(name), 10)

    def servo_state(self, i):
        angle, rpm, age = self.hw.feedback[i].read(time.monotonic())
        return dict(servo=i+1, channel=self.args.channels[i], gpio=self.args.pins[i],
                    command=self.ctrl.speeds[i], pwm_calculated_us=self.hw.pulses[i],
                    neutral_us=self.ctrl.neutrals[i], angle_deg=angle, rpm=rpm,
                    feedback_age_s=age if math.isfinite(age) else None)

    def robot_state(self):
        with self.ctrl.lock:
            return dict(armed=self.ctrl.armed, message=self.ctrl.message, fault=self.ctrl.fault,
                        uncalibrated_channels=self.args.uncalibrated_channels,
                        servos=[self.servo_state(i) for i in range(len(self.args.channels))],
                        led=self.extras.snapshot(), imu=self.imu.snapshot(), distance=self.distance.snapshot(),
                        microservo=self.microservo_state())

    def microservo_state(self):
        return dict(angle=self.microservo.angle, active=self.microservo.active)

    def microservo_command(self, msg):
        if not math.isfinite(msg.data) or not 0 <= msg.data <= 180:
            self.get_logger().error('Microservo: objetivo fuera de 0..180 grados')
            return
        if time.monotonic() - self.ctrl.last_ui > .5:
            self.get_logger().warn('Microservo: falta heartbeat de la TUI')
            return
        self.microservo.command(msg.data)

    def stop(self, message='Parada ROS'):
        self.ctrl.stop(message)
        self.microservo.stop()

    def publish(self, name):
        pub, source = self.sources[name]
        pub.publish(String(data=json.dumps(source(), allow_nan=False)))

    def publish_all(self):
        if time.monotonic() - self.ctrl.last_ui > .5:
            self.microservo.stop()
        for name in self.sources:
            self.publish(name)

    def heartbeat(self, msg):
        self.ctrl.last_ui = time.monotonic()

    def set_speed(self, i, value):
        if not math.isfinite(value) or abs(value) > self.args.limit:
            self.get_logger().error('Velocidad fuera de rango')
            return
        with self.ctrl.lock:
            if self.ctrl.armed:
                self.ctrl.speeds[i] = value
                self.ctrl.last_command = time.monotonic()

    def led_command(self, msg):
        try:
            data = json.loads(msg.data)
            self.extras.led_change(**data)
        except (ValueError, TypeError, IndexError) as exc:
            self.get_logger().error(str(exc))

    def command(self, msg):
        try:
            data = json.loads(msg.data)
            op = data['op']
            now = time.monotonic()
            if op == 'stop':
                self.stop(data.get('message', 'Parada ROS'))
            elif op == 'arm':
                self.ctrl.arm(now)
            elif op in ('jog', 'trim'):
                selected = data['selected']
                if not selected or any(type(i) is not int or i < 0 or i >= len(self.args.channels) for i in selected):
                    raise ValueError('Selección inválida')
                delta = float(data['delta'])
                if not math.isfinite(delta) or abs(delta) > (10 if op == 'trim' else .1):
                    raise ValueError('Incremento inválido')
                if op == 'trim':
                    self.ctrl.trim(selected, delta)
                else:
                    self.ctrl.jog(selected, delta, bool(data.get('mirror', False)), now)
            elif op == 'save':
                self.ctrl.stop('Guardando neutros ROS')
                app.save_config(self.args.config, self.ctrl.neutrals, self.args.channels)
                self.args.uncalibrated_channels = []
            else:
                raise ValueError('Operación desconocida')
        except (ValueError, TypeError, KeyError, OSError) as exc:
            self.get_logger().error(str(exc))

    def close(self):
        self.stop()
        self.imu.close()
        self.distance.close()
        self.extras.close()
        self.ctrl.finish()


class RemoteControl:
    def __init__(self, node, args):
        self.node, self.args = node, args
        self.lock = threading.RLock()
        self.neutrals = [1500] * len(args.channels)
        self.speeds = [0.] * len(args.channels)
        self.armed, self.fault = False, None
        self.message = 'Esperando estado ROS 2'
        self.last_ui = time.monotonic()
        self.received = 0.
        self.state = {}
        self.hw = SimpleNamespace(pulses=list(self.neutrals), feedback=[])
        for i in range(len(args.channels)):
            self.hw.feedback.append(SimpleNamespace(read=lambda now, i=i: self.feedback(i, now)))
        self.publisher = node.create_publisher(String, '/robot/set', 10)
        self.heartbeat_pub = node.create_publisher(Empty, '/robot/heartbeat/set', 10)
        node.create_subscription(String, '/robot/state', self.update, 10)
        node.create_timer(.1, self.heartbeat)

    def update(self, msg):
        state = json.loads(msg.data)
        with self.lock:
            self.state, self.received = state, time.monotonic()
            self.armed, self.fault, self.message = state['armed'], state['fault'], state['message']
            self.neutrals = [s['neutral_us'] for s in state['servos']]
            self.speeds = [s['command'] for s in state['servos']]
            self.hw.pulses = [s['pwm_calculated_us'] for s in state['servos']]
            self.args.uncalibrated_channels = state.get('uncalibrated_channels', [])

    def heartbeat(self):
        if time.monotonic() - self.last_ui < .3:
            self.heartbeat_pub.publish(Empty())
        if time.monotonic() - self.received > .5:
            self.armed = False
            self.message = 'Estado ROS caducado'

    def feedback(self, i, now):
        servos = self.state.get('servos', [])
        if i >= len(servos):
            return None, None, float('inf')
        s = servos[i]
        age = s['feedback_age_s']
        return s['angle_deg'], s['rpm'], float('inf') if age is None else age + now - self.received

    def send(self, op, **values):
        self.publisher.publish(String(data=json.dumps(dict(op=op, **values))))

    def stop(self, message='Parada solicitada'):
        self.send('stop', message=message)

    def arm(self, now):
        self.send('arm')

    def jog(self, selected, delta, mirror, now):
        self.send('jog', selected=selected, delta=delta, mirror=mirror)

    def trim(self, selected, delta):
        self.send('trim', selected=selected, delta=delta)


class RemoteSensor:
    def __init__(self, ctrl, name):
        self.ctrl, self.name = ctrl, name

    def snapshot(self):
        state = self.ctrl.state.get(self.name)
        if state is None or time.monotonic() - self.ctrl.received > .5:
            return None
        return state


class RemoteExtras(RemoteSensor):
    def snapshot(self):
        return super().snapshot() or dict(rgb=[0, 0, 0], brightness=0, led_on=False, led_status='Sin conexión ROS')

    def led_change(self, field=None, delta=0, absolute=None, toggle=False, off=False):
        self.ctrl.led_pub.publish(String(data=json.dumps(dict(field=field, delta=delta, absolute=absolute, toggle=toggle, off=off))))


class RemoteMicroServo:
    def __init__(self, node, ctrl):
        self.ctrl = ctrl
        self.publisher = node.create_publisher(Float64, '/microservo/set', 10)
        self.stop_pub = node.create_publisher(Empty, '/microservo/stop/set', 10)

    @property
    def angle(self):
        return self.ctrl.state.get('microservo', {}).get('angle', 90.0)

    @property
    def active(self):
        return (time.monotonic() - self.ctrl.received <= .5
                and self.ctrl.state.get('microservo', {}).get('active', False))

    def command(self, angle):
        if math.isfinite(angle) and 0 <= angle <= 180:
            self.publisher.publish(Float64(data=float(angle)))

    def stop(self):
        self.stop_pub.publish(Empty())


def main(ros_args=None):
    mode = sys.argv.pop(1) if len(sys.argv) > 1 else 'tui'
    if mode not in ('server', 'tui'):
        raise SystemExit('Uso: servos_ros2.py server|tui [opciones de servos_tui]')
    args = app.parse_args()
    if args.mqtt_host:
        raise SystemExit('La TUI ROS 2 publica telemetría por topics; no usa --mqtt-host.')
    rclpy.init(args=ros_args or [])
    if mode == 'server':
        node = DeviceNode(args)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.close()
            node.destroy_node()
            rclpy.try_shutdown()
        return
    node = Node('eltronico_tui')
    ctrl = RemoteControl(node, args)
    ctrl.led_pub = node.create_publisher(String, '/leds/set', 10)
    microservo = RemoteMicroServo(node, ctrl)
    save_config_original = app.save_config
    app.save_config = lambda *a: ctrl.send('save')
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        curses.wrapper(app.tui, ctrl, args, threading.Event(), RemoteExtras(ctrl, 'led'),
                       None, RemoteSensor(ctrl, 'distance'), RemoteSensor(ctrl, 'imu'), microservo)
    finally:
        ctrl.stop('TUI cerrada')
        microservo.stop()
        time.sleep(.1)
        rclpy.try_shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        app.save_config = save_config_original


if __name__ == '__main__':
    main()
