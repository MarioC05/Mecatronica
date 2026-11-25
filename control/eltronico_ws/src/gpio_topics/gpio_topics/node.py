"""Per-pin ROS topics for Raspberry Pi GPIO, using libgpiod 1.x."""
import glob
import mmap
import struct
import gpiod
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, Empty

class GpioTopicsNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gpio_topics', **kwargs)
        self.declare_parameter('pins', list(range(28)))
        # ROS puede representar [] como BYTE_ARRAY; admitirlo como lista vacía.
        self.declare_parameter('output_pins', [-1],
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('chip', '')
        pins = [int(p) for p in self.get_parameter('pins').value]
        outputs = {int(p) for p in self.get_parameter('output_pins').value}
        if outputs == {-1}:
            outputs = set(pins)
        rate = float(self.get_parameter('publish_rate_hz').value)
        chip_override = str(self.get_parameter('chip').value)
        if not pins or len(pins) != len(set(pins)):
            raise ValueError('pins debe ser una lista no vacía sin duplicados')
        if not outputs.issubset(set(pins)):
            raise ValueError('output_pins debe ser subconjunto de pins')
        if rate <= 0:
            raise ValueError('publish_rate_hz debe ser positivo')
        self.chips, self.lines, self.state_publishers = [], {}, {}
        self.set_subscriptions, self.get_subscriptions = [], []
        self.output_pins = set()  # Solo salidas activadas por un mensaje /set.
        self.allowed_outputs = outputs - {2, 3, 7, 8, 9, 10, 11}
        self.pin_lines = {}
        self.level_registers = None
        self.monitor_pins = set()
        try:
            self.pin_lines = self._find_lines(pins, chip_override)
            for pin in pins:
                chip, offset = self.pin_lines[pin]
                if chip.label() == 'pinctrl-bcm2711':
                    if self.level_registers is None:
                        with open('/dev/gpiomem', 'rb') as device:
                            self.level_registers = mmap.mmap(device.fileno(), mmap.PAGESIZE, access=mmap.ACCESS_READ)
                    self.monitor_pins.add(pin)
                    prefix = f'/gpio/bcm{pin}'
                    self.state_publishers[pin] = self.create_publisher(Bool, prefix + '/state', 10)
                    self.get_subscriptions.append(self.create_subscription(
                        Empty, prefix + '/get', lambda _msg, p=pin: self._publish_pin(p), 10))
                    self.set_subscriptions.append(self.create_subscription(
                        Bool, prefix + '/set', lambda msg, p=pin: self._set_pin(p, msg), 10))
                    continue
                line = chip.get_line(offset)
                try:
                    # Otros chips: conservar la dirección existente, sin escribir.
                    line.request(consumer='ros2_gpio_topics', type=gpiod.LINE_REQ_DIR_AS_IS)
                except OSError as exc:
                    self.get_logger().warning(f'BCM {pin} no disponible: {exc}; se omite')
                    continue
                self.lines[pin] = line
                prefix = f'/gpio/bcm{pin}'
                self.state_publishers[pin] = self.create_publisher(Bool, prefix + '/state', 10)
                self.get_subscriptions.append(self.create_subscription(
                    Empty, prefix + '/get', lambda _msg, p=pin: self._publish_pin(p), 10))
                self.set_subscriptions.append(self.create_subscription(
                    Bool, prefix + '/set', lambda msg, p=pin: self._set_pin(p, msg), 10))
                self.get_logger().info(f'BCM {pin}: {chip.name()}, línea {offset}, ' +
                                       'lectura sin cambiar dirección')
        except Exception:
            self._release()
            raise
        if not self.lines and not self.monitor_pins:
            self._release()
            raise RuntimeError("No hay GPIO disponibles. Comprueba si ya hay otra instancia activa.")
        self.timer = self.create_timer(1.0 / rate, self._publish_all)
        self.get_logger().info(f'GPIO listo; pines activos: {sorted(set(self.lines) | self.monitor_pins)}; salidas activas: {sorted(self.output_pins)}')

    def _find_lines(self, pins, override):
        targets = {f'GPIO{p}': p for p in pins}
        # BCM2711 labels the HAT ID EEPROM lines by function name.
        if 0 in pins:
            targets['ID_SDA'] = 0
        if 1 in pins:
            targets['ID_SCL'] = 1
        found = {}
        for path in ([override] if override else sorted(glob.glob('/dev/gpiochip*'))):
            chip = gpiod.Chip(path)
            self.chips.append(chip)
            for offset in range(chip.num_lines()):
                name = chip.get_line(offset).name()
                if name in targets and targets[name] not in found:
                    found[targets[name]] = (chip, offset)
        missing = sorted(set(pins) - set(found))
        if missing:
            raise RuntimeError(f'No se encontraron líneas GPIO BCM {missing}; revisa /dev/gpiochip*')
        return found

    def _publish_pin(self, pin):
        msg = Bool()
        if pin in self.monitor_pins:
            # BCM2711 GPLEV0 samples pad levels without changing pin mux.
            levels = struct.unpack_from('<I', self.level_registers, 0x34)[0]
            msg.data = bool(levels & (1 << pin))
        else:
            msg.data = bool(self.lines[pin].get_value())
        self.state_publishers[pin].publish(msg)

    def _publish_all(self):
        for pin in self.state_publishers:
            try:
                self._publish_pin(pin)
            except OSError as exc:
                self.get_logger().error(f'Error leyendo BCM {pin}: {exc}')

    def _set_pin(self, pin, msg):
        if pin not in self.allowed_outputs:
            self.get_logger().warning(f'BCM {pin}: escritura no permitida (output_pins o bus reservado)')
            return
        try:
            value = 1 if msg.data else 0
            if pin not in self.output_pins:
                if pin in self.monitor_pins:
                    chip, offset = self.pin_lines[pin]
                    line = chip.get_line(offset)
                    # Si lo usa otro driver, request falla: no se fuerza ni se libera.
                    line.request(consumer='ros2_gpio_topics',
                                 type=gpiod.LINE_REQ_DIR_OUT, default_val=value)
                    self.lines[pin] = line
                    self.monitor_pins.remove(pin)
                else:
                    self.lines[pin].set_direction_output(value)
                self.output_pins.add(pin)
            else:
                self.lines[pin].set_value(value)
            self._publish_pin(pin)
        except OSError as exc:
            self.get_logger().error(f'Error escribiendo BCM {pin}: {exc}')

    def _release(self):
        if self.level_registers is not None:
            self.level_registers.close()
            self.level_registers = None
        for line in self.lines.values():
            try: line.release()
            except Exception: pass
        for chip in self.chips:
            try: chip.close()
            except Exception: pass

    def destroy_node(self):
        self._release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpioTopicsNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
