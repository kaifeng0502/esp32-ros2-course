"""Live-plot data semantics without ROS/GUI/serial or motion."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('motor_live_plot_test',
    Path(__file__).resolve().parents[1]/'ros2/motor_action_interfaces/scripts/motor_plot.py')
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)


class PlotTests(unittest.TestCase):
    def test_subscribers_can_wait_for_late_publishers(self):
        data = plot.PlotBuffer()
        self.assertFalse(data.status(0)[1])
        data.add('target',1,0)
        self.assertFalse(data.status(1)[1])
        data.add('rpm',1.1,0)
        self.assertTrue(data.status(1.1)[1])

    def test_missing_data_never_becomes_a_fake_zero(self):
        data = plot.PlotBuffer()
        data.add('target',0,120)
        data.add('rpm',0,117)
        self.assertFalse(data.status(2)[1])
        self.assertEqual(list(data.samples['rpm']),[(0,117)])
        data.add('rpm',2.1,float('nan'))
        self.assertFalse(data.status(2.1)[1])

    def test_history_is_bounded_and_recovery_is_live(self):
        data = plot.PlotBuffer(5)
        data.add('rpm',0,120)
        data.add('target',0,120)
        data.trim(6)
        self.assertFalse(data.samples['rpm'])
        data.add('rpm',6,0)
        data.add('target',6,0)
        self.assertTrue(data.status(6)[1])


if __name__ == '__main__':
    unittest.main()
