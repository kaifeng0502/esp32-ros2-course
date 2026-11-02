"""Offline analysis preserves timestamps and never fills missing speed with zero."""
import csv
import importlib.util
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location('motor_bag_analysis_test',
    Path(__file__).resolve().parents[1]/'ros2/analyze_motor_bag.py')
bag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bag)


class Reader:
    def __init__(self,rows,types=None):
        self.rows = iter(rows)
        self.types = types or bag.TOPICS
        self.next_row = next(self.rows,None)

    def get_all_topics_and_types(self):
        return [SimpleNamespace(name=k,type=v) for k,v in self.types.items()]

    def has_next(self):
        return self.next_row is not None

    def read_next(self):
        row = self.next_row
        self.next_row = next(self.rows,None)
        return row


class BagAnalysisTests(unittest.TestCase):
    def test_async_rows_sorted_without_fabricating_alignment(self):
        raw = [('/motor/rpm',78.0,2000000010),('/motor/pwm',30.0,1000000000),
               ('/motor/setpoint',80.0,1000000001),('/parameter_events','ignored',0)]
        rows = bag.collect(Reader(raw),lambda data,typ:SimpleNamespace(data=data))
        self.assertEqual(len(rows),3)
        self.assertEqual(rows[0],(1000000000,'/motor/pwm',30.0))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'data.csv'
            bag.write_csv(rows,path)
            with path.open() as stream:
                data = list(csv.DictReader(stream))
            self.assertEqual(data[-1]['timestamp_ns'],'2000000010')
            self.assertEqual(data[-1]['time_s'],'1.000000010')

    def test_gaps_are_broken_and_original_values_preserved(self):
        rows = [(0,'/motor/rpm',120.0),(100000000,'/motor/rpm',118.0),
                (3000000000,'/motor/rpm',0.0)]
        x,y = bag.series(rows,'/motor/rpm')
        self.assertTrue(math.isnan(y[2]))
        self.assertEqual(y[:2],[120,118])
        self.assertEqual(y[-1],0)

    def test_missing_or_wrong_topic_types_fail_explicitly(self):
        for reader in [Reader([],types={'/motor/rpm':'std_msgs/msg/String'}),Reader([])]:
            with self.assertRaises(ValueError):
                bag.collect(reader,lambda data,typ:None)

    def test_nonfinite_sample_is_not_treated_as_valid_zero(self):
        with self.assertRaises(ValueError):
            bag.collect(Reader([('/motor/rpm',float('nan'),0)]),
                        lambda data,typ:SimpleNamespace(data=data))


if __name__ == '__main__':
    unittest.main()
