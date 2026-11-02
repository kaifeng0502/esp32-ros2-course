"""Independent numerical checks for the production constrained C optimizer."""
import ctypes
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import contextlib
import io

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
import predictive_model as pm
import motor_mpc as host
import test_direction as native

FITTED=dict(a1=.886839,a2=-.148937,b1=.397151,b2=.194144,c=-.952157,sample_s=.05,move_weight=2.)


class CappedSolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();folder=Path(cls.temp.name)
        (folder/'wrapper.c').write_text('''
#include "predictive.h"
int capped(float *parameters,float *state,float *out){
 predictive_model_t m={parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5],parameters[6]};
 predictive_solver_t s;
 if(!predictive_init(&s,&m))return -1;
 float pwm=-999;
 int ok=predictive_step_capped(&s,state[0],state[1],state[2],state[3],state[4],state[5],state[6],&pwm);
 for(int j=0;j<10;j++)out[j]=s.plan[j];
 out[10]=s.predicted_peak;out[11]=s.prediction_limit;out[12]=s.constraint_violation;
 out[13]=s.sweeps;out[14]=s.residual;out[15]=s.result;out[16]=pwm;
 return ok;
}
''')
        lib=folder/'capped.so'
        subprocess.run(['cc','-std=c11','-O2','-shared','-fPIC','-I',str(ROOT/'main'),
                        str(ROOT/'main/predictive.c'),str(folder/'wrapper.c'),'-lm','-o',str(lib)],
                       check=True,capture_output=True)
        cls.lib=ctypes.CDLL(str(lib));cls.lib.capped.argtypes=[ctypes.POINTER(ctypes.c_float)]*3
        cls.lib.capped.restype=ctypes.c_int

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def solve(self,state,model=FITTED):
        parameters=(ctypes.c_float*7)(*[model[k] for k in (*pm.COEFFICIENTS,'sample_s','move_weight')])
        out=(ctypes.c_float*17)()
        ok=self.lib.capped(parameters,(ctypes.c_float*7)(*state),out)
        return ok,np.array(out,dtype=float)

    @staticmethod
    def problem(model,state):
        y,old_y,old_u,disturbance,target,maximum,margin=state
        previous_control=old_u;free=[]
        for k in range(10):
            nxt=pm.prediction(model,y,old_y,0,old_u,disturbance)
            free.append(nxt);old_y,y,old_u=y,nxt,0
        ref=pm.ReferenceMPC(model)
        free=np.array(free);linear=ref.g.T@(free-target)
        linear[0]-=model['move_weight']*previous_control
        return ref.g,ref.h,free,linear

    def test_fitted_model_plans_meet_every_prediction_bound(self):
        cases=[[0,0,0,0,80,55,1],[64,60,40,.2,80,55,1],
               [80,80,37,0,110,55,1],[95,80,55,0,110,55,1],
               [109,109,50,.1,110,55,1],[80,80,37,0,80,55,0]]
        for state in cases:
            with self.subTest(state=state):
                ok,out=self.solve(state)
                self.assertEqual(ok,1,out)
                g,_,free,_=self.problem(FITTED,state)
                prediction=free+g@out[:10]
                self.assertLessEqual(float(max(prediction)),state[4]-state[6]+.0002)
                self.assertTrue(np.all(out[:10]>=0) and np.all(out[:10]<=state[5]))
                self.assertEqual(out[12],0)
                self.assertLessEqual(out[13],80)

    def test_optimum_agrees_with_independent_scipy_slsqp(self):
        try:
            from scipy.optimize import minimize,LinearConstraint
        except ImportError:
            self.skipTest('Optional scipy required for independent QP oracle')
        for state in ([0,0,0,0,80,55,1],[80,80,37,0,110,55,1],
                      [95,80,55,.3,110,55,1],[79,79,37,.1,80,55,1]):
            for weight in (2.,8.):
                model=FITTED|{'move_weight':weight}
                ok,out=self.solve(state,model);self.assertEqual(ok,1,out)
                g,h,free,linear=self.problem(model,state)
                scale=1+10*state[4]**2
                cost=lambda u:float(u@h@u+2*linear@u)/scale
                jac=lambda u:2*(h@u+linear)/scale
                answer=minimize(cost,np.zeros(10),jac=jac,method='SLSQP',bounds=[(0,state[5])]*10,
                    constraints=LinearConstraint(g,-np.inf,state[4]-state[6]-.005-free),
                    options={'ftol':1e-12,'maxiter':500})
                self.assertTrue(answer.success,answer.message)
                self.assertAlmostEqual(cost(out[:10]),answer.fun,delta=2e-6)

    def test_impossible_limit_and_unsupported_model_are_not_silently_relaxed(self):
        ok,out=self.solve([120,120,55,0,40,55,1])
        self.assertEqual(ok,0);self.assertEqual(out[15],3) # proven infeasible for positive response
        self.assertEqual(out[16],-999) # caller's PWM output is untouched on failure
        # Stable DC gain, but negative later impulse coefficients: unsupported by this variant.
        model=dict(a1=.5,a2=0,b1=1,b2=-.8,c=0,sample_s=.05,move_weight=2)
        ok,out=self.solve([0,0,0,0,80,55,1],model)
        self.assertEqual(ok,0);self.assertEqual(out[15],2)

    def test_closed_loop_predicted_rollout_never_crosses_target_minus_margin(self):
        y=old_y=old_u=0.
        for k in range(200):
            target=80 if k<80 else 110
            state=[y,old_y,old_u,0,target,55,1]
            ok,out=self.solve(state);self.assertEqual(ok,1,(k,out))
            u=out[16]
            nxt=max(0,pm.prediction(FITTED,y,old_y,u,old_u))
            self.assertLessEqual(nxt,target-1+.001)
            old_y,y,old_u=y,nxt,u
        self.assertAlmostEqual(y,109,delta=.02)


class CappedIntegrationTests(unittest.TestCase):
    def test_firmware_cap_configuration_telemetry_and_infeasible_stop(self):
        output=native.FirmwareTests().compile_run('main/motor.c',r'''
int main(void){
 motor_init();
 assert(motor_command("MOTOR MODEL .8 0 .48 0 0 .05 2"));
 assert(!motor_command("MOTOR EXP MPC_CAP 80 5 1 .15 .3 .005 55 110 2"));
 assert(!motor_command("MOTOR CAP nan"));
 assert(motor_command("MOTOR CAP 1"));
 char status[128];motor_constraint_status(status,sizeof(status));fputs(status,stdout);
 assert(motor_command("MOTOR EXP MPC_CAP 80 5 1 .15 .3 .005 55 110 2"));
 now+=500000;motor_poll();assert(pi_active && pi_pwm>0);
 assert(!motor_command("MOTOR CAP 0"));
 for(int k=0;k<8;k++){
  enc.c1_edges+=60;enc.c2_edges+=60;enc.c1_signed+=60;enc.c2_signed+=60;
  now+=50000;motor_poll();
  assert(pi_active && predictive_solver.predicted_peak<=79 && predictive_solver.constraint_violation==0);
 }
 char samples[2048];int count=motor_sample_batch2(samples,sizeof(samples));
 assert(count>0 && count<sizeof(samples));fputs(samples,stdout);
 // A physically inconsistent, very high measured state makes even zero-input prediction infeasible.
 pi_rpm[0]=pi_rpm[1]=200;now+=50000;motor_poll();
 assert(!pi_active && duty==0 && levels[7]==0 && strcmp(pi_reason,"MPC_INFEASIBLE")==0);
}
''')
        records=[host.parse_observation(line) for line in output.encode().splitlines() if line.startswith(b'OBS2 ')]
        self.assertTrue(records)
        for row in records:
            self.assertEqual(row['prediction_limit'],79)
            self.assertLessEqual(row['predicted_peak'],79)
            self.assertEqual(row['constraint_violation'],0)
        status=host.query_constraints(native.FakePort([line for line in output.encode().splitlines() if line.startswith(b'MPC2 ')]))
        self.assertTrue(status['configured']);self.assertEqual(status['margin'],1)

    def test_old_firmware_feature_detection_and_invalid_arguments_cannot_start_motion(self):
        port=native.FakePort([b'MPC1 0 0 0 PI IDLE 0 0 0 0 0 0 0 0 0 0\n'])
        with patch.object(host.time,'monotonic',side_effect=[0,0,2]),self.assertRaises(RuntimeError):
            host.query_constraints(port)
        self.assertEqual(port.writes,[b'MPC2?\n'])
        for extra in (['--controllers','mpc,mpc'],['--controllers','mpc,mpc_cap','--margin-rpm','nan'],
                      ['--controllers','mpc,mpc_cap','--rpm','110','--step-rpm','80']):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                host.arguments(['compare','--model','model.json',*extra])

    def test_legacy_observations_do_not_fabricate_prediction_values(self):
        row=host.parse_observation(b'OBS1 1 0 100 0 0 80 0 0 0 33 0 0 0 0 RUNNING')
        for key in host.PREDICTION_FIELDS:self.assertIsNone(row[key])
        port=native.FakePort([b'OBS1 1 0 100 0 0 80 0 0 0 33 0 0 0 0 RUNNING'])
        with self.assertRaises(RuntimeError):host.read_batch(port,1,version=2)

    def test_report_write_failure_cannot_prevent_stop_request(self):
        args=host.arguments(['identify']);port=native.FakePort()
        caps=[dict(run=1,active=0),dict(run=2,active=1,algorithm='ID')]
        with tempfile.TemporaryDirectory() as directory,patch.object(host,'query_capability',side_effect=caps), \
             patch.object(host,'read_batch',side_effect=RuntimeError('read error')), \
             patch.object(host,'write_json',side_effect=OSError('disk full')),patch.object(host.time,'sleep'), \
             contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(OSError):host.run_trial(port,args,'ID',Path(directory)/'id.csv')
            self.assertEqual(port.writes[-1],b'MOTOR STOP\n')

    def test_constraint_report_distinguishes_prediction_from_measurement(self):
        rows=[]
        for k in range(200):
            target=80. if k<80 else 110.
            y=target-1
            rows.append(dict(run=1,seq=k,at_us=k*50000,elapsed=k*.05,dt=.05,target=target,
                c1=y,c2=y,held_pwm=40.,next_pwm=40.,solve_us=100,sweeps=5,residual=.0001,
                disturbance=0.,phase='RUNNING',prediction_limit=target-1,predicted_peak=target-1,
                constraint_violation=0.))
        # The prediction constraint can be satisfied while the physical feedback exceeds target.
        rows[20]['c1']=rows[20]['c2']=80.3
        result=pm.metrics(rows)
        self.assertEqual(result[0]['prediction_violation_max_rpm'],0)
        self.assertAlmostEqual(result[0]['above_target_rpm'],.3)
        with tempfile.TemporaryDirectory() as directory,contextlib.redirect_stdout(io.StringIO()):
            folder=Path(directory);path=folder/'trial_01_mpc_cap.csv'
            with path.open('w',newline='') as out:
                writer=host.csv.DictWriter(out,fieldnames=host.FIELDS);writer.writeheader();writer.writerows(rows)
            host.write_json(path.with_suffix('.json'),dict(status='DONE',origin='synthetic'))
            host.report(folder)
            self.assertGreater((folder/'constraint_check.png').stat().st_size,1000)


if __name__=='__main__':unittest.main()
