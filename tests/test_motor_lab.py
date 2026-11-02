import contextlib
import csv
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import test_direction as native
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import motor_lab as lab


def status_line(**changes):
    row = dict.fromkeys(lab.FIELDS, 0)
    row.update(requested=120, target=120, seconds=15, encoder_sign=1,
               algorithm='PI', reason='IDLE', kp=.15, ki=.3, kd=.005,
               antiwindup=1, max_pwm=80)
    row.update(changes)
    return ('LAB1 ' + ' '.join(str(row[k]) for k in lab.FIELDS) + '\n').encode()


class LabHostTests(unittest.TestCase):
    def test_position_command_csv_and_stop(self):
        args = lab.arguments(['--degrees','90','--rpm','60','--seconds','10'])
        port = native.FakePort([status_line(),
            status_line(active=1, requested=60, target=60, seconds=10, position_mode=1,
                        goal_turns=.25, reason='WAITING'),
            status_line(active=1, requested=60, target=25, seconds=10, position_mode=1,
                        goal_turns=.25, position_turns=.24, elapsed=1, reason='DECEL'),
            status_line(requested=60, target=25, seconds=10, position_mode=1,
                        goal_turns=.25, position_turns=.251, elapsed=1.3, reason='DONE')])
        with tempfile.TemporaryDirectory() as tmp, patch.object(lab.time,'sleep'), contextlib.redirect_stdout(io.StringIO()):
            path=Path(tmp)/'pi.csv'
            rows=lab.run_one(port,args,'pi',path)
            self.assertEqual(len(rows),3)
            with path.open() as recorded:
                self.assertEqual(len(list(csv.DictReader(recorded))),3)
        self.assertIn(b'MOTOR LAB 60 10 FWD 1 PI 0.25 0.15 0.3 0.005 1 80 0 0\n',port.writes)
        self.assertEqual(port.writes[-1],b'MOTOR STOP\n')

    def test_failure_saves_partial_csv_and_stops(self):
        args=lab.arguments([])
        port=native.FakePort([status_line(), status_line(reason='NO_FEEDBACK',elapsed=.8)])
        with tempfile.TemporaryDirectory() as tmp, patch.object(lab.time,'sleep'), contextlib.redirect_stdout(io.StringIO()):
            path=Path(tmp)/'pi.csv'
            with self.assertRaises(RuntimeError):lab.run_one(port,args,'pi',path)
            self.assertIn('NO_FEEDBACK',path.read_text())
        self.assertEqual(port.writes[-1],b'MOTOR STOP\n')

    def test_validation_before_serial(self):
        for argv in [['--degrees','nan'], ['--kp','inf'], ['--turns','1','--compare'],
                     ['--turns','1','--algorithm','open'], ['--step-rpm','60'],
                     ['--step-rpm','60','--step-at','20'], ['--max-pwm','81']]:
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):lab.arguments(argv)

    def test_metrics_steady_and_target_step(self):
        rows=[]
        for i in range(151):
            target=120 if i<50 else 60
            row=dict(elapsed=i*.1, requested=target, c1=target, c2=target,
                     reason='RUNNING',saturated=0)
            rows.append(row)
        metrics=lab.phase_metrics(rows)
        self.assertEqual(len(metrics),2)
        for phase in metrics:
            self.assertEqual(phase['tail_mae_rpm'],0)
            self.assertEqual(phase['overshoot_pct'],0)
            self.assertEqual(phase['settling_s'],0)
        rows[-1]['c1']=100
        self.assertIsNone(lab.phase_metrics(rows)[-1]['settling_s'])

    def test_plot_existing_records_never_opens_serial(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            row=dict(zip(lab.FIELDS,status_line().decode().split()[1:]))
            with (folder/'pi.csv').open('w',newline='') as output:
                writer=csv.DictWriter(output,fieldnames=lab.FIELDS)
                writer.writeheader();writer.writerow(row)
            with patch.object(sys,'argv',['motor_lab.py','--plot',tmp]), \
                 patch.object(lab.serial,'Serial') as serial_open, patch.object(lab,'report') as plot:
                lab.main()
            serial_open.assert_not_called()
            self.assertEqual(plot.call_args.args[1],folder)

    def test_plot_generation(self):
        rows=[]
        for i in range(31):
            row={k: (v if k in lab.STRINGS else float(v))
                 for k,v in zip(lab.FIELDS,status_line().decode().split()[1:])}
            row.update(elapsed=i*.1,c1=120,c2=120,pwm=50,reason='RUNNING')
            rows.append(row)
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            lab.report([('pi',rows)],Path(tmp))
            self.assertGreater((Path(tmp)/'comparison.png').stat().st_size,1000)
            self.assertIn('tail_bias_rpm',(Path(tmp)/'metrics.csv').read_text())


class LabFirmwareTests(unittest.TestCase):
    def test_position_deceleration_brake_reverse_and_deadline(self):
        native.FirmwareTests().compile_run('main/motor.c',r'''
static void tick(int edges, int sign, int us){
    enc.c1_edges+=edges;enc.c2_edges+=edges;
    enc.c1_signed+=sign*edges;enc.c2_signed+=sign*edges;
    now+=us;motor_poll();
}
int main(void){
    motor_init();
    assert(motor_command("MOTOR LAB 60 10 FWD 1 PI .25 .15 .3 .005 1 80 0 0"));
    // Coasting before start must not be counted towards the target.
    tick(100,1,100000);tick(0,1,500000);
    assert(!pi_waiting && fabsf(position_turns)<.0001);
    bool decel=false;
    for(int i=0;i<70;i++){
        tick(5,1,10000);
        if(effective_target<60)decel=true;
    }
    assert(decel && braking && duty==0 && levels[7]==1 && levels[5]==1 && levels[6]==1);
    // Count the additional motion during braking, then wait for quiet before DONE.
    tick(3,1,10000);tick(0,1,150000);
    assert(!pi_active && levels[7]==0 && strcmp(pi_reason,"DONE")==0);
    assert(position_turns>.25 && position_turns<.26);
    assert(motor_command("MOTOR LAB 60 10 REV -1 PI .25 .15 .3 .005 1 80 0 0"));
    tick(0,1,500000);
    for(int i=0;i<70;i++)tick(5,1,10000);
    assert(braking && position_turns<0);
    tick(0,1,150000);assert(!pi_active);
    assert(motor_command("MOTOR LAB 60 10 FWD 1 PI 1 .15 .3 .005 1 80 0 0"));
    tick(0,1,500000);tick(100,-1,250000);
    assert(!pi_active && strcmp(pi_reason,"DIRECTION_ERROR")==0);
    assert(motor_command("MOTOR LAB 60 1 FWD 1 PI 100 .15 .3 .005 1 80 0 0"));
    tick(0,1,500000);
    for(int i=0;i<20;i++)tick(1,1,50000);
    assert(!pi_active && strcmp(pi_reason,"TIMEOUT")==0);
    const char *invalid[]={"MOTOR LAB 60 5 FWD 1 PI nan .15 .3 .005 1 80 0 0",
      "MOTOR LAB 60 5 FWD 1 OPEN 1 .15 .3 .005 1 80 0 0",
      "MOTOR LAB 60 5 FWD 1 PI 1 .15 .3 .005 1 80 60 1",
      "MOTOR LAB 60 5 FWD 1 PID 0 inf .3 .005 1 80 0 0",
      "MOTOR LAB 60 5 FWD 1 PI 0 .15 .3 .005 1 80 0 0 extra"};
    for(int i=0;i<5;i++){assert(!motor_command(invalid[i]));assert(!pi_active);}
}
''')

    def test_position_with_a_synthetic_inertial_motor(self):
        # Integration check only: this model does not establish real-world accuracy.
        native.FirmwareTests().compile_run('main/motor.c',r'''
static float speed, wheel_turns;
static int64_t last_ticks;
static void plant_tick(void){
    float desired=0,tau=.15f;
    if(levels[7]){
        if(levels[5] && levels[6])tau=.02f;
        else desired=(levels[5]?1:-1)*(duty*100.0f/1023)*2.4f;
    }
    speed += .01f/tau*(desired-speed);
    wheel_turns+=speed*.01f/60;
    int64_t ticks=(int64_t)(wheel_turns*1400);
    int64_t delta=ticks-last_ticks;last_ticks=ticks;
    enc.c1_edges+=llabs(delta);enc.c2_edges+=llabs(delta);
    enc.c1_signed=ticks;enc.c2_signed=ticks;
    now+=10000;motor_poll();
}
int main(void){
    motor_init();
    const char*commands[]={
        "MOTOR LAB 60 15 FWD 1 PI 3 .15 .3 .005 1 80 0 0",
        "MOTOR LAB 60 15 REV 1 PI .25 .15 .3 .005 1 80 0 0"};
    for(int run=0;run<2;run++){
        assert(motor_command(commands[run]));
        for(int i=0;i<2000 && pi_active;i++)plant_tick();
        assert(!pi_active && strcmp(pi_reason,"DONE")==0);
        float target=run==0?3:-.25;
        assert(fabsf(position_turns-target)<.03f);
        assert(levels[7]==0);
    }
}
''')

    def test_algorithms_antiwindup_and_derivative_no_setpoint_kick(self):
        native.FirmwareTests().compile_run('main/motor.c',r'''
static void sample(int edges){
    enc.c1_edges+=edges;enc.c2_edges+=edges;
    enc.c1_signed+=edges;enc.c2_signed+=edges;now+=50000;motor_poll();
}
static void begin(const char*cmd){assert(motor_command(cmd));now+=500000;motor_poll();}
int main(void){
    motor_init();
    begin("MOTOR LAB 120 10 FWD 1 OPEN 0 .15 .3 .005 1 80 0 0");
    sample(1);assert(fabsf(pi_pwm-50)<.001 && p_term==0 && pi_integral==0 && d_term==0);
    begin("MOTOR LAB 120 10 FWD 1 P 0 .15 .3 .005 1 80 0 0");
    sample(1);assert(p_term>0 && pi_integral==0 && d_term==0);
    begin("MOTOR LAB 120 10 FWD 1 PI 0 .15 .3 .005 1 80 0 0");
    sample(1);assert(p_term>0 && pi_integral>0 && d_term==0);
    begin("MOTOR LAB 120 10 FWD 1 PI 0 .15 .3 .005 1 20 0 0");
    sample(1);assert(saturated && pi_pwm==20 && pi_integral==0);
    begin("MOTOR LAB 120 10 FWD 1 PI 0 .15 .3 .005 0 20 0 0");
    for(int i=0;i<100;i++)sample(1);
    assert(saturated && pi_pwm==20 && pi_integral==25);
    begin("MOTOR LAB 120 10 FWD 1 PID 0 .15 .3 .005 1 80 60 1");
    sample(140);assert(d_term<0);
    now=pi_started+950000;
    // Steady measured speed, target changes from 120 to 60. D must not kick.
    pi_rpm[0]=pi_rpm[1]=previous_speed=120;derivative=0;pi_sampled=now;
    sample(140);assert(pi_target==60 && fabsf(d_term)<.001);
    char reply[512];int n=motor_lab_status(reply,sizeof(reply));
    assert(n>0 && n<512 && strstr(reply,"LAB1 1 60.0") && strstr(reply," PID "));
}
''')


if __name__ == '__main__':unittest.main()
