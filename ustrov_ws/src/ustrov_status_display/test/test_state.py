import unittest
from ustrov_status_display.state import Battery, DisplayState, Vehicle


class TestDisplayState(unittest.TestCase):
    def ready(self, armed=False):
        state = DisplayState()
        state.vehicle = Vehicle(armed, 'OFFBOARD', preflight_ok=True)
        state.battery = Battery(True, 11.23, 0.386)
        state.heartbeat_time = state.battery_time = 10.0
        state.thrust_time = state.torque_time = 10.0
        return state

    def test_live_voltage_and_px4_percentage(self):
        lines = self.ready().lines(10.1, True, '192.168.50.2')
        self.assertEqual(lines[1], 'BAT 11.23V 39%')
        self.assertEqual(lines[-1], 'WARN NONE')

    def test_px4_disconnect_hides_cached_disarmed(self):
        lines = self.ready().lines(13.0, True, '192.168.50.2')
        self.assertEqual(lines[0], 'USTROV UNKNOWN')
        self.assertEqual(lines[-1], 'WARN PX4 LOST')

    def test_battery_expires_while_imu_stays_live(self):
        state = self.ready()
        state.heartbeat_time = 16.0
        lines = state.lines(16.0, True, '192.168.50.2')
        self.assertEqual(lines[1], 'BAT --V --%')
        self.assertEqual(lines[-1], 'WARN BAT STALE')

    def test_unknown_soc_is_not_zero_percent(self):
        state = self.ready()
        state.battery.remaining = -1.0
        self.assertEqual(state.lines(10.1, True, 'ip')[1], 'BAT 11.23V --%')

    def test_command_loss_only_alarms_when_armed(self):
        for armed, expected in ((False, 'WAIT'), (True, 'LOST')):
            state = self.ready(armed)
            lines = state.lines(11.0, True, 'ip')
            self.assertIn(f'CMD {expected}', lines[3])
            self.assertEqual(lines[-1], 'WARN CMD LOST' if armed else 'WARN NONE')

    def test_status_snapshot_remains_valid_with_live_imu(self):
        state = self.ready()
        state.heartbeat_time = state.battery_time = 100.0
        self.assertEqual(state.lines(100.1, True, 'ip')[0], 'USTROV DISARMED')

    def test_invalid_and_nan_battery_values(self):
        state = self.ready()
        state.battery = Battery(True, float('nan'), float('nan'))
        self.assertEqual(state.lines(10.1, True, 'ip')[1], 'BAT --V --%')
        self.assertEqual(state.lines(10.1, True, 'ip')[-1], 'WARN BAT INVALID')

    def test_low_battery_and_missing_ethernet(self):
        state = self.ready()
        state.battery.warning = 2
        self.assertEqual(state.lines(10.1, False, '--')[-1], 'WARN BAT CRITICAL')
        state.battery.warning = 0
        self.assertEqual(state.lines(10.1, False, '--')[-1], 'WARN ETH DOWN')


if __name__ == '__main__':
    unittest.main()
