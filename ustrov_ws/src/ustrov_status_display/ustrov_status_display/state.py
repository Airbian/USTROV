"""与 ROS/I2C 解耦的显示逻辑；过期数据不能继续显示为正常。"""
from dataclasses import dataclass
import math
from typing import Optional


@dataclass
class Battery:
    connected: bool
    voltage: float
    remaining: float
    warning: int = 0
    faults: int = 0


@dataclass
class Vehicle:
    armed: bool
    mode: str
    failsafe: bool = False
    preflight_ok: bool = False


class DisplayState:
    def __init__(self, px4_timeout=2.0, battery_timeout=5.0, command_timeout=0.5):
        self.px4_timeout = px4_timeout
        self.battery_timeout = battery_timeout
        self.command_timeout = command_timeout
        self.battery: Optional[Battery] = None
        self.vehicle: Optional[Vehicle] = None
        self.battery_time = None
        self.heartbeat_time = None
        self.thrust_time = None
        self.torque_time = None

    @staticmethod
    def fresh(received, now, timeout):
        return received is not None and 0.0 <= now - received <= timeout

    def lines(self, now, ethernet_up, ip):
        alive = self.fresh(self.heartbeat_time, now, self.px4_timeout)
        status_known = alive and self.vehicle is not None
        armed = status_known and self.vehicle.armed
        arm_text = ('ARMED' if armed else 'DISARMED') if status_known else 'UNKNOWN'
        mode = self.vehicle.mode if status_known else '--'
        battery_fresh = self.fresh(self.battery_time, now, self.battery_timeout)
        battery_valid = battery_fresh and self.battery is not None and self.battery.connected
        volts, percent = '--', '--'
        if battery_valid:
            if math.isfinite(self.battery.voltage) and self.battery.voltage > 0:
                volts = f'{self.battery.voltage:.2f}'
            if math.isfinite(self.battery.remaining) and 0 <= self.battery.remaining <= 1:
                percent = str(round(100 * self.battery.remaining))
        commands_ok = (
            self.fresh(self.thrust_time, now, self.command_timeout)
            and self.fresh(self.torque_time, now, self.command_timeout)
        )
        cmd = 'OK' if commands_ok else ('LOST' if armed else 'WAIT')
        warning = 'NONE'
        if not alive:
            warning = 'PX4 LOST'
        elif not status_known:
            warning = 'STATUS WAIT'
        elif self.vehicle.failsafe:
            warning = 'FAILSAFE'
        elif not battery_fresh:
            warning = 'BAT STALE'
        elif not battery_valid:
            warning = 'NO BAT'
        elif not math.isfinite(self.battery.voltage) or self.battery.voltage <= 0:
            warning = 'BAT INVALID'
        elif self.battery.faults:
            warning = 'BAT FAULT'
        elif self.battery.warning:
            warning = {1: 'BAT LOW', 2: 'BAT CRITICAL', 3: 'BAT EMERGENCY',
                       4: 'BAT FAILED', 6: 'BAT UNHEALTHY', 7: 'CHARGING'}.get(
                           self.battery.warning, 'BAT WARNING')
        elif armed and not commands_ok:
            warning = 'CMD LOST'
        elif not ethernet_up:
            warning = 'ETH DOWN'
        elif not armed and not self.vehicle.preflight_ok:
            warning = 'PRECHECK'
        return [
            f'USTROV {arm_text}',
            f'BAT {volts}V {percent}%',
            f'MODE {mode}',
            f'PX4 {"OK" if alive else "LOST"} CMD {cmd}',
            f'ETH {ip if ethernet_up else "DOWN"}',
            f'WARN {warning}',
        ]
