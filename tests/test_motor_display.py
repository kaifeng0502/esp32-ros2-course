"""Render the actual OLED C implementation off-device and check live stop readings."""
import ctypes as c
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Status(c.Structure):
    _fields_ = [('active', c.c_bool), ('target_rpm', c.c_float),
                ('c1_rpm', c.c_float), ('c2_rpm', c.c_float),
                ('pwm_percent', c.c_float), ('elapsed', c.c_float),
                ('seconds', c.c_int), ('encoder_sign', c.c_int), ('reason', c.c_char_p),
                ('requested_rpm',c.c_float),('position_turns',c.c_float),('goal_turns',c.c_float),
                ('p_term',c.c_float),('i_term',c.c_float),('d_term',c.c_float),('ff_term',c.c_float),
                ('kp',c.c_float),('ki',c.c_float),('kd',c.c_float),('max_pwm',c.c_float),
                ('position_mode',c.c_bool),('saturated',c.c_bool),('antiwindup',c.c_bool),('controller',c.c_char_p)]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.controller = b'PI'


class Encoder(c.Structure):
    _fields_ = [('c1_edges', c.c_uint32), ('c2_edges', c.c_uint32),
                ('c1_signed', c.c_int64), ('c2_signed', c.c_int64),
                ('c1_level', c.c_int), ('c2_level', c.c_int)]


def load_renderer(folder):
    lib = Path(folder) / 'display.so'
    subprocess.run([shutil.which('cc'), '-shared', '-fPIC',
                    '-I', str(ROOT / 'main'), str(ROOT / 'main/motor_display.c'),
                    '-o', str(lib)], check=True, capture_output=True)
    renderer = c.CDLL(str(lib)).motor_display_render
    renderer.argtypes = [c.POINTER(c.c_uint8), c.POINTER(Status), c.POINTER(Encoder), c.c_int64]
    return renderer


class DisplayTests(unittest.TestCase):
    def test_render_continues_after_stop_and_stays_in_bounds(self):
        with tempfile.TemporaryDirectory() as tmp:
            render = load_renderer(tmp)
            # Guard the frame on both sides against out-of-bounds writes.
            guarded = (c.c_uint8 * 1026)()
            guarded[0], guarded[-1] = 0xA5, 0x5A
            frame = c.cast(c.byref(guarded, 1), c.POINTER(c.c_uint8))
            status = Status(True, 120, 120, 120, 50.2, 2, 15, 1, b'RUNNING')
            encoder = Encoder()
            for tick in range(21):
                encoder.c1_signed = encoder.c2_signed = tick * 560
                render(frame, c.byref(status), c.byref(encoder), tick * 200000)
            running = bytes(guarded)[1:-1]
            self.assertTrue(any(running))
            # DONE still measures coasting. Then the displayed RPM must decay to zero.
            status.active, status.pwm_percent, status.reason = False, 0, b'DONE'
            for tick in range(21, 61):
                render(frame, c.byref(status), c.byref(encoder), tick * 200000)
            stopped = bytes(guarded)[1:-1]
            self.assertNotEqual(running, stopped)
            # Further no-motion samples leave the rendered zero RPM unchanged.
            render(frame, c.byref(status), c.byref(encoder), 12200000)
            self.assertEqual(stopped, bytes(guarded)[1:-1])
            self.assertEqual((guarded[0], guarded[-1]), (0xA5, 0x5A))
            for reason in (b'NO_FEEDBACK', b'NOT_STOPPED', b'WAITING', b'STOPPED'):
                status.reason = reason
                status.target_rpm = -180
                status.pwm_percent = 80
                status.elapsed = 60
                status.seconds = 60
                render(frame, c.byref(status), c.byref(encoder), 12400000)
                self.assertEqual((guarded[0], guarded[-1]), (0xA5, 0x5A))


if __name__ == '__main__':
    unittest.main()
