"""Exercise the real Action state machine without ROS or a connected motor."""
import importlib.util
from pathlib import Path
import sys
import unittest

SOURCE = Path(__file__).resolve().parents[1]/'ros2/motor_action_interfaces/scripts/motor_action_core.py'
spec = importlib.util.spec_from_file_location('motor_action_core', SOURCE)
core = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = core
spec.loader.exec_module(core)


def status(**changes):
    row = dict(active=0, requested=60, target=60, c1=0, c2=0, pwm=0,
               elapsed=0, reason='STOPPED', seconds=15, encoder_sign=-1,
               algorithm='PI', position_mode=1, goal_turns=3, position_turns=0,
               p=0, i=0, d=0, ff=0, saturated=0, kp=.15, ki=.3, kd=.005,
               antiwindup=1, max_pwm=80)
    row.update(changes)
    return ('LAB1 '+' '.join(str(row[k]) for k in core.FIELDS)).encode()


class ActionTests(unittest.TestCase):
    def setUp(self):
        self.writes = []
        self.engine = core.ActionEngine(self.writes.append)
        self.engine.receive(status(), 0)

    def start(self, turns=3):
        job, error = self.engine.reserve(turns, 60, 15, .1)
        self.assertFalse(error)
        self.engine.tick(.1)
        return job

    def acknowledge(self):
        self.engine.receive(b'ACK MOTOR', .15)

    def test_startup_and_validation_never_move(self):
        self.assertEqual(self.writes, [])
        for turns, rpm, timeout in [(.001,60,15), (float('nan'),60,15), (3,10,15), (3,60,61)]:
            self.assertIsNone(self.engine.reserve(turns,rpm,timeout,.1)[0])
        self.assertIsNone(self.engine.reserve(3,60,15,2)[0])
        self.assertEqual(self.writes, [])

    def test_normal_goal_and_stale_done(self):
        job = self.start()
        self.assertIn(b'MOTOR LAB 60 15 FWD -1 PI 3 ', self.writes[0])
        self.assertIsNone(self.engine.reserve(3,60,15,.1)[0])
        self.engine.receive(status(reason='DONE', position_turns=3), .12)
        self.assertFalse(job.done.is_set())
        self.acknowledge()
        self.engine.receive(status(active=1,pwm=30,reason='RUNNING',position_turns=1.5), .2)
        self.assertEqual(self.engine.feedback(job)['progress_percent'],50)
        self.engine.receive(status(reason='DONE',position_turns=3.005), .3)
        self.assertEqual(job.outcome,'SUCCEEDED')
        self.assertTrue(job.disabled)

    def test_short_goal_can_finish_between_queries(self):
        job = self.start(.02)
        self.acknowledge()
        self.engine.receive(status(reason='DONE',goal_turns=.02,position_turns=.02), .2)
        self.assertEqual(job.outcome,'SUCCEEDED')

    def test_reverse_goal_has_signed_feedback(self):
        job = self.start(-3)
        self.assertIn(b'REV -1 PI 3 ',self.writes[0])
        self.acknowledge()
        self.engine.receive(status(active=1,pwm=30,requested=-60,goal_turns=-3,
                                   position_turns=-1.5,reason='RUNNING'),.2)
        f = self.engine.feedback(job)
        self.assertEqual(f['progress_percent'],50)
        self.assertEqual(f['remaining_turns'],1.5)

    def test_cancel_requires_new_stop_ack_and_status(self):
        job = self.start()
        self.acknowledge()
        job.cancel = True
        self.engine.tick(.2)
        self.assertEqual(self.writes[-2:], [b'MOTOR STOP\n', b'LAB?\n'])
        self.engine.receive(status(),.21)
        self.assertFalse(job.done.is_set())
        self.engine.receive(b'ACK MOTOR',.22)
        self.assertFalse(job.done.is_set())
        self.engine.receive(status(position_turns=.3),.23)
        self.assertEqual(job.outcome,'CANCELED')
        self.assertTrue(job.disabled)

    def test_cancel_before_start_only_sends_stop(self):
        job, _ = self.engine.reserve(3,60,15,.1)
        job.cancel = True
        self.engine.tick(.1)
        self.assertFalse(any(b'MOTOR LAB' in x for x in self.writes))
        self.acknowledge()
        self.engine.receive(status(),.2)
        self.assertEqual(job.outcome,'CANCELED')
        self.assertEqual(self.engine.feedback(job)['current_turns'],0)

    def test_cancel_wins_over_concurrent_done(self):
        job = self.start()
        self.acknowledge()
        job.cancel = True
        self.engine.receive(status(reason='DONE',position_turns=3),.2)
        self.assertFalse(job.done.is_set())
        self.assertTrue(job.stop_sequence)

    def test_missing_stop_confirmation_latches_fault(self):
        job = self.start()
        job.cancel = True
        self.engine.tick(.2)
        self.engine.tick(1.8)
        self.assertEqual(job.outcome,'ABORTED')
        self.assertFalse(job.disabled)
        self.engine.job = None
        self.engine.receive(status(),1.9)
        self.assertIsNone(self.engine.reserve(3,60,15,2)[0])

    def test_feedback_loss_sends_stop(self):
        job = self.start()
        self.engine.tick(1.1)
        self.assertEqual(job.stop_reason,'FEEDBACK_TIMEOUT')

    def test_parameter_mismatch_sends_stop(self):
        job = self.start()
        self.acknowledge()
        self.engine.receive(status(active=1,pwm=30,goal_turns=2),.2)
        self.assertEqual(job.stop_reason,'PARAMETER_MISMATCH')

    def test_outside_tolerance_or_board_failure_aborts(self):
        for reason, position in [('DONE',3.1),('TIMEOUT',2),('NO_FEEDBACK',.3)]:
            with self.subTest(reason=reason):
                self.setUp()
                job = self.start()
                self.acknowledge()
                self.engine.receive(status(reason=reason,position_turns=position),.2)
                self.assertEqual(job.outcome,'ABORTED')
                self.assertTrue(job.disabled)

    def test_disconnect_wakes_execution_without_claiming_stop(self):
        job = self.start()
        self.engine.disconnect('USB lost')
        self.assertTrue(job.done.is_set())
        self.assertFalse(job.disabled)

    def test_corrupt_telemetry_is_ignored(self):
        for line in [b'LAB1 0', b'random log', status(pwm=float('nan')),status(active=2)]:
            self.assertIsNone(core.parse_status(line))

    def test_live_encoder_speed_reports_coasting_and_stop(self):
        speed = core.EncoderSpeed()
        self.assertIsNone(speed.receive(b'ENC 0 0 1 0',0))
        for n in range(1,7):
            rpm = speed.receive(f'ENC {280*n} {280*n} 1 0'.encode(),n*.2)
            self.assertAlmostEqual(rpm[0],60)
        for n in range(7,14):
            rpm = speed.receive(b'ENC 1680 1680 1 0',n*.2)
        self.assertEqual(rpm,(0,0))
        self.assertIsNone(speed.receive(b'ENC 0 0 0 0',3))

    def test_timed_run_finishes_without_position_tolerance(self):
        job = self.start(0.0)
        self.assertIn(b'FWD -1 PI 0 ',self.writes[0])
        self.acknowledge()
        self.engine.receive(status(active=1,pwm=30,goal_turns=0,position_mode=0,
                                   position_turns=2,elapsed=7.5,reason='RUNNING'),.2)
        self.assertEqual(self.engine.feedback(job)['progress_percent'],50)
        self.engine.receive(status(reason='DONE',goal_turns=0,position_mode=0,
                                   position_turns=14.7,elapsed=15),.3)
        self.assertEqual(job.outcome,'SUCCEEDED')
        self.assertIn('not applicable',job.message)

    def test_controller_snapshot_sent_and_checked(self):
        self.engine.config = core.ControllerConfig(kp=.2,ki=.6,max_pwm=65,antiwindup=False)
        job = self.start()
        self.assertIn(b'0.2 0.6 0.005 0 65 0 0',self.writes[0])
        self.engine.config = core.ControllerConfig(kp=.8,ki=2)
        self.assertEqual(job.config.kp,.2)
        self.acknowledge()
        self.engine.receive(status(active=1,pwm=30,kp=.2,ki=.6,max_pwm=65,antiwindup=0,
                                   reason='RUNNING'),.2)
        self.assertFalse(job.stop_sequence)
        self.engine.receive(status(active=1,pwm=30,kp=.8,ki=2,reason='RUNNING'),.3)
        self.assertEqual(job.stop_reason,'PARAMETER_MISMATCH')

    def test_negative_zero_uses_forward_timed_mode(self):
        job = self.start(-0.0)
        self.assertEqual(str(job.turns),'0.0')
        self.assertIn(b'FWD',self.writes[0])


if __name__ == '__main__':
    unittest.main()
