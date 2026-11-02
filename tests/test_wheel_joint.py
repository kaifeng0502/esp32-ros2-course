"""Verify signed position, boot/reset handling and the actual C wire formatter."""
import importlib.util
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('wheel_joint_under_test',
    ROOT/'ros2/motor_action_interfaces/scripts/wheel_joint_core.py')
wheel = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wheel)


class WheelTests(unittest.TestCase):
    def test_relative_zero_and_one_turn_in_both_directions(self):
        e = wheel.WheelEstimator()
        first = e.update((1,1000,50,-50),0)
        self.assertEqual(first['position'],0)
        self.assertIsNone(first['velocity'])
        forward = e.update((1,201000,-1350,-1450),.2)
        self.assertAlmostEqual(forward['position'],2*math.pi)
        self.assertAlmostEqual(forward['velocity'],2*math.pi/.2)
        reverse = e.update((1,401000,50,-50),.4)
        self.assertAlmostEqual(reverse['position'],0)
        self.assertLess(reverse['velocity'],0)

    def test_calibration_averages_channels_without_doubling_turns(self):
        e = wheel.WheelEstimator()
        e.update((1,0,0,0),0)
        self.assertAlmostEqual(e.update((1,200000,-1400,0),.2)['position'],math.pi)

    def test_reboot_rezeros_but_no_false_velocity(self):
        e = wheel.WheelEstimator()
        e.update((1,1000000,100,100),0)
        e.update((1,1200000,-1300,-1300),.2)
        row = e.update((2,1500000,0,0),.3)
        self.assertTrue(row['restarted'])
        self.assertEqual(row['position'],0)
        self.assertIsNone(row['velocity'])

    def test_gap_preserves_position_but_not_unknown_velocity(self):
        e = wheel.WheelEstimator()
        e.update((1,0,0,0),0)
        self.assertFalse(e.fresh(1.1))
        row = e.update((1,2000000,-1400,-1400),2)
        self.assertAlmostEqual(row['position'],2*math.pi)
        self.assertIsNone(row['velocity'])
        self.assertTrue(e.fresh(2))

    def test_duplicate_or_out_of_order_does_not_refresh_validity(self):
        e = wheel.WheelEstimator()
        e.update((1,100,0,0),0)
        self.assertIsNone(e.update((1,100,50,50),2))
        self.assertIsNone(e.update((1,99,50,50),2))
        self.assertFalse(e.fresh(2))

    def test_subtracts_large_integer_origin_before_float_conversion(self):
        e = wheel.WheelEstimator(encoder_sign=1)
        e.update((1,0,2**60,2**60),0)
        row = e.update((1,200000,2**60+1,2**60+1),.2)
        self.assertAlmostEqual(row['position'],2*math.pi/1400)

    def test_wire_validation(self):
        self.assertEqual(wheel.parse_encoder_state(b'ENC2 42 1000 -1400 1400'),(42,1000,-1400,1400))
        for line in [b'ENC 1 2 0 1',b'ENC2 1 -1 0 0',b'ENC2 1 2 0',
                     b'ENC2 1 2 nan 0',b'ENC2 4294967296 1 0 0',
                     f'ENC2 1 2 {2**63} 0'.encode()]:
            self.assertIsNone(wheel.parse_encoder_state(line))

    def test_invalid_calibration(self):
        for kwargs in [dict(counts_per_rev=0),dict(counts_per_rev=float('nan')),
                       dict(encoder_sign=0),dict(encoder_sign=True)]:
            with self.assertRaises(ValueError):
                wheel.WheelEstimator(**kwargs)

    def test_c_formatter_to_python_parser(self):
        cc = shutil.which('cc')
        self.assertIsNotNone(cc,'A C compiler is needed to verify the firmware wire formatter')
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "encoder_protocol.h"
int main(void) {
    encoder_snapshot_t counts = {.c1_signed=INT64_MIN+1, .c2_signed=INT64_MAX};
    char out[160], short_out[2];
    int n=encoder_format_state(out,sizeof(out),UINT32_MAX,9000000000000LL,&counts);
    assert(n>0 && n<sizeof(out));
    assert(encoder_format_state(short_out,sizeof(short_out),1,1,&counts)>=sizeof(short_out));
    fputs(out,stdout);
}
'''
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'harness.c';exe=Path(folder)/'harness'
            source.write_text(harness)
            subprocess.run([cc,'-std=c11','-I',str(ROOT/'main'),
                str(ROOT/'main/encoder_protocol.c'),str(source),'-o',str(exe)],check=True,capture_output=True)
            line = subprocess.check_output([str(exe)])
        self.assertEqual(wheel.parse_encoder_state(line),(2**32-1,9000000000000,-2**63+1,2**63-1))


if __name__ == '__main__':
    unittest.main()
