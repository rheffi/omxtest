"""Offline ID14 transport tests: no SDK, serial port or motor connection."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools/measurement'), str(ROOT / 'tools/picking')]
from omx_motor_frame import FramePacket, PositionFrame, frame_packet, signed32
from pick_robot import DynamixelBus, MotionController, inspect

SERIAL = '72ED7C2850323952582E3120FF07090D'
PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_' + SERIAL + '-if00'


class Packet:
    def __init__(self, present=4080):
        self.positions = dict(zip(range(11, 17), [2142, 1746, 2281, present, 2101, 2256]))
        self.goals = dict(self.positions)
        self.writes = []
        self.error = 0

    def ping(self, port, motor):
        return (1060 if motor < 14 else 1200), 0, 0

    def read1ByteTxRx(self, port, motor, address):
        return {11: 5 if motor == 16 else 4, 64: 1, 70: 0, 146: 25}[address], 0, 0

    def read2ByteTxRx(self, port, motor, address):
        return {144: 61, 38: 1750, 102: 1750}[address], 0, 0

    def read4ByteTxRx(self, port, motor, address):
        value = self.positions[motor] if address == 132 else self.goals[motor] if address == 116 else 0
        return value & 0xFFFFFFFF, 0, self.error

    def write4ByteTxRx(self, port, motor, address, value):
        self.writes.append((motor, address, signed32(value)))
        if address == 116:
            self.goals[motor] = signed32(value)
            self.positions[motor] = signed32(value)
        return 0, 0

    def write1ByteTxRx(self, port, motor, address, value):
        self.writes.append((motor, address, value))
        return 0, 0

    write2ByteTxRx = write1ByteTxRx


def bus_for(raw):
    bus = DynamixelBus(Path(PORT))
    bus.port = object()
    bus.packet = FramePacket(raw, 1024, SERIAL)
    return bus


class MotorFrameTests(unittest.TestCase):
    def test_high_manual_read_and_exact_90_degree_goal(self):
        raw = Packet(4080)
        bus = bus_for(raw)
        self.assertEqual(bus.read(14, 132, 4), 3056)
        bus.write(14, 116, 4, 3040)
        self.assertEqual(raw.writes[-1], (14, 116, 4064))

    def test_cross_4096_in_both_directions_without_wrap_jump(self):
        raw = Packet(4064)
        bus = bus_for(raw)
        actual = []
        for logical in [3040, 3060, 3080, 3100, 3080, 3060, 3040]:
            bus.write(14, 116, 4, logical)
            self.assertEqual(bus.read(14, 132, 4), logical)
            actual.append(raw.positions[14])
        self.assertEqual(actual, [4064, 4084, 4104, 4124, 4104, 4084, 4064])

    def test_reconnect_after_reset_selects_nearby_signed_turn(self):
        raw = Packet(28)
        bus = bus_for(raw)
        self.assertEqual(bus.read(14, 132, 4), 3100)
        bus.write(14, 116, 4, 3040)
        self.assertEqual(raw.writes[-1], (14, 116, -32))
        self.assertEqual(bus.read(14, 132, 4), 3040)
        self.assertEqual(bus.read(14, 116, 4), 3040)

    def test_stale_goal_does_not_determine_current_turn(self):
        raw = Packet(28)
        raw.goals[14] = 4124
        bus = bus_for(raw)
        bus.read(14, 116, 4)  # Recorder reads goal before present.
        self.assertEqual(bus.read(14, 132, 4), 3100)
        bus.write(14, 116, 4, 3040)
        self.assertEqual(raw.positions[14], -32)

    def test_failed_initial_position_read_sends_no_goal(self):
        raw = Packet()
        raw.error = 0x80
        bus = bus_for(raw)
        with self.assertRaisesRegex(RuntimeError, 'READ'):
            bus.write(14, 116, 4, 3040)
        self.assertEqual(raw.writes, [])
        self.assertIsNone(bus.packet.frame.bias)

    def test_read_failure_does_not_anchor(self):
        raw = Packet()
        raw.error = 0x80
        adapter = FramePacket(raw, 1024, SERIAL)
        self.assertEqual(adapter.read4ByteTxRx(None, 14, 132)[2], 0x80)
        self.assertIsNone(adapter.frame.bias)

    def test_other_motors_and_nonposition_registers_unchanged(self):
        raw = Packet()
        bus = bus_for(raw)
        for motor in (11, 12, 13, 15, 16):
            self.assertEqual(bus.read(motor, 132, 4), raw.positions[motor])
            bus.write(motor, 116, 4, 2222)
            self.assertEqual(raw.writes[-1], (motor, 116, 2222))
        self.assertEqual(bus.read(14, 20, 4), 0)
        bus.write(14, 64, 1, 0)
        self.assertEqual(raw.writes[-1], (14, 64, 0))

    def test_enable_and_hold_do_not_move_and_ramp_stays_continuous(self):
        raw = Packet(28)
        bus = bus_for(raw)
        cfg = {'stage_seconds': .1, 'stage_pause_seconds': 0}
        driver = MotionController(bus, cfg)
        driver.prepare(use_gripper=False)
        driver.enable()
        self.assertEqual(raw.positions[14], 28)
        with patch('pick_robot.time.sleep'):
            driver.ramp({14: 3040})
        commands = [value for motor, address, value in raw.writes if (motor, address) == (14, 116)]
        self.assertEqual(commands[-1], -32)
        self.assertTrue(all(-32 <= value <= 28 for value in commands))
        driver.hold()
        self.assertEqual(raw.positions[14], -32)

    def test_inspect_separates_physical_and_model(self):
        raw = Packet(4064)
        row = inspect(bus_for(raw))[3]
        self.assertEqual(row['position'], 3040)
        self.assertEqual(row['physical_position'], 4064)

    def test_saved_pose_round_trip_does_not_accumulate_offset(self):
        path = ROOT / 'tools/measurement/07_omx_pose_recorder.py'
        spec = importlib.util.spec_from_file_location('frame_pose_recorder', path)
        recorder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(recorder)
        saved = json.loads((ROOT / 'config/single/omx_saved_poses.json').read_text())
        for name, pose in saved['poses'].items():
            with self.subTest(name=name):
                logical = pose['raw_pulses']
                raw = Packet()
                raw.positions.update(zip(range(11, 16), logical))
                raw.positions[14] += 1024
                raw.goals = dict(raw.positions)
                adapter = FramePacket(raw, 1024, SERIAL)
                captured = recorder.read_once(adapter, None)
                self.assertEqual(captured['positions'], logical)
                adapter.write4ByteTxRx(None, 14, 116, captured['positions'][3])
                self.assertEqual(raw.positions[14], logical[3] + 1024)
                self.assertEqual(captured['motor_frame']['physical_offset_pulses'], 1024)

    def test_config_identity_and_no_file_default_are_unchanged_for_other_arms(self):
        raw = Packet()
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'frame.json'
            with patch('omx_motor_frame.DEFAULT_CONFIG', path):
                self.assertIs(frame_packet(raw, PORT), raw)
            path.write_text(json.dumps(dict(schema_version=1, usb_serial=SERIAL,
                                            motor_id=14, physical_offset_pulses=1024)))
            self.assertIsInstance(frame_packet(raw, PORT, path), FramePacket)
            self.assertIs(frame_packet(raw, PORT.replace(SERIAL, 'OTHER'), path), raw)
            with patch('omx_motor_frame.usb_serial', return_value=SERIAL):
                self.assertIsInstance(frame_packet(raw, '/dev/omx_follower', path), FramePacket)
            with self.assertRaises(FileNotFoundError):
                frame_packet(raw, PORT, path.with_name('missing.json'))

    def test_all_turn_branches_roundtrip(self):
        for turn in (-2, -1, 0, 1, 2):
            for logical in (0, 1, 2048, 3040, 3100, 4095):
                frame = PositionFrame(1024)
                physical = logical + 1024 + turn * 4096
                self.assertEqual(frame.observe(physical), logical)
                self.assertEqual(frame.to_physical(logical), physical)


if __name__ == '__main__':
    unittest.main()
