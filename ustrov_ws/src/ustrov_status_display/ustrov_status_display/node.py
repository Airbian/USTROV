"""只订阅数据、刷新 OLED；不发布控制消息，不创建 Arm/Disarm 客户端。"""
import fcntl
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import BatteryStatus, SensorCombined, VehicleStatus
from ustrov_control_msgs.msg import ActuatorSetpoint

from .state import Battery, DisplayState, Vehicle


class StatusDisplay(Node):
    def __init__(self):
        super().__init__('ustrov_status_display')
        defaults = {
            'i2c_bus': 1, 'i2c_address': 0x3c, 'rotation': 0,
            'refresh_hz': 2.0, 'contrast': 100, 'driver_python_path': '',
            'battery_topic': '/fmu/out/battery_status',
            'vehicle_status_topic': '/fmu/out/vehicle_status_v1',
            'heartbeat_topic': '/fmu/out/sensor_combined',
            'thrust_topic': '/thrust_setpoint', 'torque_topic': '/torque_setpoint',
            'battery_id': 1, 'ethernet_interface': 'eth0',
            'px4_timeout_sec': 2.0, 'battery_timeout_sec': 5.0,
            'command_timeout_sec': 0.5,
        }
        self.params = {key: self.declare_parameter(key, value).value
                       for key, value in defaults.items()}
        if not 0.2 <= self.params['refresh_hz'] <= 10:
            raise ValueError('refresh_hz must be in [0.2, 10]')
        if self.params['rotation'] not in (0, 1, 2, 3):
            raise ValueError('rotation must be 0, 1, 2 or 3')
        if not 0x08 <= self.params['i2c_address'] <= 0x77:
            raise ValueError('i2c_address must be a valid 7-bit I2C address')
        if min(self.params['px4_timeout_sec'], self.params['battery_timeout_sec'],
               self.params['command_timeout_sec']) <= 0:
            raise ValueError('message timeouts must be positive')
        # 防止两个显示进程同时操作同一块屏幕。
        lock_dir = Path.home() / '.ros'
        lock_dir.mkdir(exist_ok=True)
        self._lock = open(lock_dir / 'ustrov_status_display.lock', 'a')
        fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        driver_path = self.params['driver_python_path'] or str(
            Path.home() / '.local/share/ustrov_oled/deps/usr/lib/python3/dist-packages')
        if Path(driver_path).is_dir():
            sys.path.insert(0, driver_path)
        from luma.core.interface.serial import i2c
        from luma.oled.device import ssd1306
        from PIL import Image, ImageDraw, ImageFont
        self._i2c, self._ssd1306 = i2c, ssd1306
        self._Image, self._ImageDraw = Image, ImageDraw
        self._font = ImageFont.load_default()
        self._device = None
        self._last_lines = None
        self._next_retry = 0.0
        self._next_network_check = 0.0
        self._ethernet_up, self._ip = False, '--'
        self.state = DisplayState(self.params['px4_timeout_sec'],
                                  self.params['battery_timeout_sec'],
                                  self.params['command_timeout_sec'])
        qos = qos_profile_sensor_data
        self._subscriptions = [
            self.create_subscription(BatteryStatus, self.params['battery_topic'], self.on_battery, qos),
            self.create_subscription(VehicleStatus, self.params['vehicle_status_topic'], self.on_vehicle, qos),
            self.create_subscription(SensorCombined, self.params['heartbeat_topic'], self.on_heartbeat, qos),
            self.create_subscription(ActuatorSetpoint, self.params['thrust_topic'], self.on_thrust, qos),
            self.create_subscription(ActuatorSetpoint, self.params['torque_topic'], self.on_torque, qos),
        ]
        self.create_timer(1.0 / self.params['refresh_hz'], self.refresh)
        self.get_logger().info('Read-only OLED display started; no actuator commands or services are used.')

    def on_battery(self, msg):
        if self.params['battery_id'] >= 0 and msg.id != self.params['battery_id']:
            return
        self.state.battery = Battery(msg.connected, msg.voltage_v, msg.remaining,
                                     msg.warning, msg.faults)
        self.state.battery_time = time.monotonic()

    def on_vehicle(self, msg):
        mode = 'OFFBOARD' if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD else str(msg.nav_state)
        self.state.vehicle = Vehicle(msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                                      mode, msg.failsafe, msg.pre_flight_checks_pass)
        self.state.heartbeat_time = time.monotonic()

    def on_heartbeat(self, _msg):
        # VehicleStatus 可能只在状态变化时发送；用 IMU 数据确认链路仍存活。
        self.state.heartbeat_time = time.monotonic()

    def on_thrust(self, _msg):
        self.state.thrust_time = time.monotonic()

    def on_torque(self, _msg):
        self.state.torque_time = time.monotonic()

    def update_network(self):
        interface = self.params['ethernet_interface']
        try:
            self._ethernet_up = (Path('/sys/class/net') / interface / 'carrier').read_text().strip() == '1'
            result = subprocess.run(['ip', '-4', '-o', 'addr', 'show', 'dev', interface],
                                    capture_output=True, text=True, timeout=0.3, check=False)
            fields = result.stdout.split()
            self._ip = fields[fields.index('inet') + 1].split('/')[0] if 'inet' in fields else '--'
        except (OSError, subprocess.TimeoutExpired, ValueError, IndexError):
            self._ethernet_up, self._ip = False, '--'

    def refresh(self):
        now = time.monotonic()
        if now >= self._next_network_check:
            self.update_network()
            self._next_network_check = now + 3.0
        if self._device is None:
            if now < self._next_retry:
                return
            self._next_retry = now + 5.0
            try:
                serial = self._i2c(port=self.params['i2c_bus'], address=self.params['i2c_address'])
                self._device = self._ssd1306(serial, width=128, height=64, rotate=self.params['rotation'])
                self._device.contrast(max(0, min(255, self.params['contrast'])))
                self._last_lines = None
                self.get_logger().info(f'OLED connected: bus {self.params["i2c_bus"]}, address {self.params["i2c_address"]:#04x}')
            except Exception as exc:
                self._device = None
                self.get_logger().warning(f'OLED unavailable; retry in 5 s: {exc}')
                return
        lines = self.state.lines(now, self._ethernet_up, self._ip)
        # 右上角心跳点每秒翻转，便于区分进程运行与旧画面冻结。
        frame_key = (tuple(lines), int(now) % 2)
        if frame_key == self._last_lines:
            return
        try:
            image = self._Image.new('1', (128, 64))
            draw = self._ImageDraw.Draw(image)
            for row, text in enumerate(lines):
                while text and draw.textbbox((0, 0), text, font=self._font)[2] > 120:
                    text = text[:-1]
                draw.text((0, row * 10), text, font=self._font, fill=255)
            if frame_key[1]:
                draw.rectangle((124, 1, 126, 3), fill=255)
            self._device.display(image)
            self._last_lines = frame_key
            self.get_logger().debug('OLED frame: ' + ' | '.join(lines))
        except Exception as exc:
            self._device = None
            self.get_logger().warning(f'OLED write failed; retry in 5 s: {exc}')

    def destroy_node(self):
        if getattr(self, '_device', None) is not None:
            try:
                self._device.cleanup()
            except Exception:
                pass
        if getattr(self, '_lock', None) is not None:
            self._lock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StatusDisplay()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
