"""Synthetic/offline validation only. Never opens a real serial port."""
import contextlib
import ctypes
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import test_direction as native

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
import predictive_model as pm
import motor_mpc as host

TRUTH=dict(a1=1.3,a2=-.4,b1=.24,b2=0.,c=-1.,sample_s=.05,move_weight=2.)


def synthetic_identification():
    levels=[30,45,35,55,40,50,30,55,35]
    y=old_y=old_u=0.; held=levels[0]
    rows=[dict(run=1,seq=0,at_us=0,elapsed=0.,dt=0.,target=80.,c1=0.,c2=0.,
               held_pwm=0.,next_pwm=held,solve_us=0,sweeps=0,residual=0.,disturbance=0.,phase='RUNNING')]
    for k in range(1,360):
        nxt=max(0,pm.prediction(TRUTH,y,old_y,held,old_u))
        old_y,y,old_u=y,nxt,held
        next_u=levels[min(int(k*.05/2),8)]
        measured=y+.01*np.sin(k)
        rows.append(dict(run=1,seq=k,at_us=k*50000,elapsed=k*.05,dt=.05,target=80.,
                         c1=measured,c2=measured,held_pwm=held,next_pwm=next_u,
                         solve_us=0,sweeps=0,residual=0.,disturbance=0.,phase='RUNNING'))
        held=next_u
    return rows


class IdentificationTests(unittest.TestCase):
    def test_recovers_dynamics_and_validates_without_teacher_forcing(self):
        model,rows,predicted,split=pm.fit_model(synthetic_identification(),'synthetic-hash',origin='synthetic')
        self.assertTrue(model['validated'])
        self.assertLess(model['validation']['free_run_rmse_rpm'],.1)
        for key in pm.COEFFICIENTS:
            self.assertAlmostEqual(model[key],TRUTH[key],delta=.02)
        # Validation does not use the measured outputs as its own next inputs.
        self.assertAlmostEqual(predicted[split+1],pm.prediction(model,predicted[split],predicted[split-1],
                               rows[split+1]['held_pwm'],rows[split]['held_pwm']))

    def test_rejects_gaps_timestamps_excitation_and_unstable_models(self):
        for mutation in ('sequence','time','excitation','channel'):
            rows=synthetic_identification()
            if mutation=='sequence': rows[100]['seq']+=1
            if mutation=='time': rows[100]['at_us']+=10000
            if mutation=='excitation':
                for row in rows: row['held_pwm']=40
            if mutation=='channel':
                for row in rows: row['c2']+=20
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                pm.fit_model(rows,'bad')
        for changes in (dict(a1=2),dict(b1=-1),dict(sample_s=.5),dict(c=float('nan'))):
            with self.assertRaises(ValueError): pm.validate_model(TRUTH|changes)

    def test_bad_heldout_prediction_cannot_be_uploaded(self):
        rows=synthetic_identification()
        for row in rows[250:]: row['c1']+=20; row['c2']+=20
        model,*_=pm.fit_model(rows,'mismatched')
        self.assertFalse(model['validated'])
        model.update(encoder_sign=-1)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'model.json';host.write_json(path,model)
            with self.assertRaises(ValueError): pm.load_model(path,hardware=True)
            model['validated']=True;model['origin']='synthetic';host.write_json(path,model)
            with self.assertRaises(ValueError): pm.load_model(path,hardware=True)

    def test_metrics_account_for_step_direction_and_insufficient_settling(self):
        rows=[]
        for k in range(100):
            target=80. if k<50 else 120.
            rows.append(dict(elapsed=k*.05,target=target,c1=target,c2=target,next_pwm=40.,phase='RUNNING'))
        result=pm.metrics(rows)
        self.assertEqual(len(result),2)
        self.assertEqual(result[1]['tail_mae_rpm'],0)
        self.assertEqual(result[1]['settling_s'],0)
        rows[-1]['c1']=rows[-1]['c2']=130.
        self.assertIsNone(pm.metrics(rows)[-1]['settling_s'])

    def test_offline_artifacts_keep_synthetic_origin_and_never_connect(self):
        model,rows,simulated,split=pm.fit_model(synthetic_identification(),'synthetic',origin='synthetic')
        with tempfile.TemporaryDirectory() as directory,patch.object(host.serial,'Serial') as serial_open, \
             contextlib.redirect_stdout(io.StringIO()):
            folder=Path(directory)
            host.plot_fit(model,rows,simulated,split,folder/'fit.png')
            for algorithm in ('pid','mpc'):
                path=folder/f'trial_01_{algorithm}.csv'
                with path.open('w',newline='') as output:
                    writer=host.csv.DictWriter(output,fieldnames=host.FIELDS)
                    writer.writeheader();writer.writerows(rows)
                host.write_json(path.with_suffix('.json'),dict(status='DONE',origin='synthetic'))
            host.report(folder)
            self.assertGreater((folder/'fit.png').stat().st_size,1000)
            self.assertGreater((folder/'comparison.png').stat().st_size,1000)
            self.assertIn('solver_max_us',(folder/'metrics.csv').read_text())
            serial_open.assert_not_called()


class SolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory()
        root=Path(cls.temp.name)
        (root/'wrapper.c').write_text('''
#include "predictive.h"
int solve(float *parameters, float *state, float *output){
    predictive_model_t m={parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5],parameters[6]};
    predictive_solver_t solver;
    if(!predictive_init(&solver,&m))return -1;
    float pwm=0;
    int ok=predictive_step(&solver,state[0],state[1],state[2],state[3],state[4],state[5],&pwm);
    for(int k=0;k<MPC_HORIZON;k++)output[k]=solver.plan[k];
    output[10]=solver.residual;output[11]=solver.sweeps;
    return ok;
}
''')
        library=root/'predictive.so'
        subprocess.run(['cc','-shared','-fPIC','-O2','-I',str(ROOT/'main'),str(ROOT/'main/predictive.c'),
                        str(root/'wrapper.c'),'-lm','-o',str(library)],check=True,capture_output=True)
        cls.library=ctypes.CDLL(str(library))
        cls.library.solve.argtypes=[ctypes.POINTER(ctypes.c_float)]*3
        cls.library.solve.restype=ctypes.c_int

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def test_actual_c_matches_reference_and_satisfies_constrained_stationarity(self):
        parameters=(ctypes.c_float*7)(*[TRUTH[k] for k in (*pm.COEFFICIENTS,'sample_s','move_weight')])
        for state in ([0,0,0,0,120,60],[80,80,37.5,0,80,60],[120,120,54,0,80,60],
                      [40,35,30,-.2,120,55],[0,0,0,0,180,60]):
            output=(ctypes.c_float*12)()
            self.assertEqual(self.library.solve(parameters,(ctypes.c_float*6)(*state),output),1)
            expected=pm.ReferenceMPC(TRUTH).step(*state)[0]
            self.assertAlmostEqual(output[0],expected,delta=.05)
            self.assertTrue(all(0<=v<=state[-1] for v in output[:10]))
            self.assertLessEqual(output[10],.03001)
            self.assertLessEqual(output[11],100)
        output=(ctypes.c_float*12)()
        self.assertEqual(self.library.solve(parameters,(ctypes.c_float*6)(float('nan'),0,0,0,80,60),output),0)

    def test_constant_equilibrium_is_unconstrained_optimum(self):
        # This target is exactly an equilibrium; the independent dense optimum is constant PWM.
        ref=pm.ReferenceMPC(TRUTH)
        ref.plan[:]=37.5
        pwm,*_=ref.step(80,80,37.5,0,80,60)
        self.assertAlmostEqual(pwm,37.5,places=5)


class FirmwareIntegrationTests(unittest.TestCase):
    def test_identification_samples_applied_input_and_autonomous_deadline(self):
        output=native.FirmwareTests().compile_run('main/motor.c',r'''
int main(void){
    motor_init();
    assert(!motor_command("MOTOR EXP MPC 80 10 -1 .15 .3 .005 60 120 4"));
    assert(!motor_command("MOTOR MODEL 2 0 .2 0 0 .05 2"));
    assert(motor_command("MOTOR EXP ID 80 18 -1 .15 .3 .005 60 0 0"));
    now+=500000;motor_poll();assert(pi_pwm==30);
    assert(!motor_command("MOTOR MODEL .8 0 .48 0 0 .05 2"));
    float before=pi_pwm;
    for(int k=1;k<=360;k++){
        enc.c1_edges+=140;enc.c2_edges+=140;
        enc.c1_signed-=140;enc.c2_signed-=140;
        now+=50000;motor_poll();
        if(k==40){
            trace_sample_t *sample=&trace_samples[(trace_write-1)%TRACE_CAPACITY];
            assert(fabsf(sample->held_pwm-before)<.1 && sample->next_pwm==45);
        }
        before=pi_pwm;
        char buffer[2048];int n=motor_sample_batch(buffer,sizeof(buffer));
        assert(n>0 && n<sizeof(buffer) && strstr(buffer,"OBSEND 1 0 0"));
        if(k==40)fputs(buffer,stdout);
    }
    assert(!pi_active && duty==0 && levels[7]==0 && strcmp(pi_reason,"DONE")==0);
    assert(trace_dropped==0);
    assert(motor_command("MOTOR MODEL .8 0 .48 0 0 .05 2"));
    char status[512];motor_predictive_status(status,sizeof(status));
    assert(strstr(status,"MPC1 1 1 0"));
    fputs(status,stdout);
}
''')
        lines=output.encode().splitlines()
        record=host.parse_observation(next(line for line in lines if line.startswith(b'OBS1 ')))
        self.assertAlmostEqual(record['elapsed'],2.)
        self.assertLess(record['held_pwm'],30.1)
        self.assertEqual(record['next_pwm'],45)
        cap=host.query_capability(native.FakePort([line for line in lines if line.startswith(b'MPC1 ')]))
        self.assertEqual(cap['reason'],'DONE')
        self.assertAlmostEqual(cap['model']['b1'],.48)

    def test_actual_onboard_mpc_tracks_synthetic_motor_and_stops_on_numerical_failure(self):
        native.FirmwareTests().compile_run('main/motor.c',r'''
int main(void){
    motor_init();
    assert(motor_command("MOTOR MODEL .8 0 .48 0 0 .05 2"));
    assert(motor_command("MOTOR EXP MPC 80 5 1 .15 .3 .005 60 120 2"));
    now+=500000;motor_poll();assert(pi_pwm>0);
    float rpm=0,accumulated=0;
    int previous_ticks=0;
    for(int k=0;k<100;k++){
        rpm+=.25f*(pi_pwm*2.4f-rpm);
        accumulated+=rpm*.05f/60*1400;
        int ticks=(int)accumulated,delta=ticks-previous_ticks;previous_ticks=ticks;
        enc.c1_edges+=delta;enc.c2_edges+=delta;
        enc.c1_signed+=delta;enc.c2_signed+=delta;
        now+=50000;motor_poll();
        if(k==90)assert(fabsf((pi_rpm[0]+pi_rpm[1])/2-120)<6);
        char output[2048];motor_sample_batch(output,sizeof(output));
    }
    assert(!pi_active && duty==0 && strcmp(pi_reason,"DONE")==0);
    assert(motor_command("MOTOR EXP MPC 80 5 1 .15 .3 .005 60 0 0"));
    now+=500000;motor_poll();
    predictive_solver.hessian[0][0]=NAN;
    enc.c1_edges+=1;enc.c2_edges+=1;now+=50000;motor_poll();
    assert(!pi_active && duty==0 && strcmp(pi_reason,"MPC_NOT_CONVERGED")==0);
}
''')


class HostTests(unittest.TestCase):
    def test_invalid_arguments_before_serial_and_offline_fit_never_opens_port(self):
        for argv in (['compare','--model','x','--kp','nan'],['fit','x','--move-weight','inf'],
                     ['compare','--model','x','--seconds','60']):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):host.arguments(argv)
        with patch.object(host,'fit_file') as fit,patch.object(host.serial,'Serial') as serial_open:
            host.main(['fit','/tmp/synthetic.csv'])
            fit.assert_called_once();serial_open.assert_not_called()

    def test_old_firmware_cannot_trigger_motion(self):
        class Port:
            def __init__(self): self.writes=[]
            def write(self,v): self.writes.append(v)
            def readline(self): return b'ENC 0 0 1 0\n'
        port=Port()
        with patch.object(host.time,'monotonic',side_effect=[0,0,2]),self.assertRaises(RuntimeError):
            host.query_capability(port)
        self.assertEqual(port.writes,[b'MPC?\n'])

    def test_sampling_disconnect_requests_stop_and_keeps_partial_metadata(self):
        args=host.arguments(['identify'])
        port=native.FakePort()
        caps=[dict(run=1,active=0),dict(run=2,active=1,algorithm='ID')]
        with tempfile.TemporaryDirectory() as directory,patch.object(host,'query_capability',side_effect=caps), \
             patch.object(host,'read_batch',side_effect=RuntimeError('disconnect')),patch.object(host.time,'sleep'), \
             contextlib.redirect_stdout(io.StringIO()):
            path=Path(directory)/'id.csv'
            with self.assertRaises(RuntimeError):host.run_trial(port,args,'ID',path)
            self.assertEqual(json.loads(path.with_suffix('.json').read_text())['status'],'INCOMPLETE')
            self.assertEqual(port.writes[-1],b'MOTOR STOP\n')


if __name__=='__main__':unittest.main()
