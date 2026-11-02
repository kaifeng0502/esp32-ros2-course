"""Offline motor identification, MPC reference and comparison metrics. No serial I/O."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

COEFFICIENTS = ('a1', 'a2', 'b1', 'b2', 'c')
HORIZON = 10


def validate_model(model):
    values = [float(model[k]) for k in (*COEFFICIENTS, 'sample_s', 'move_weight')]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('模型参数必须为有限数值')
    a1, a2, b1, b2, c, dt, weight = values
    denominator = 1-a1-a2
    if (denominator < .0001 or 1+a1-a2 < .0001 or 1+a2 < .0001 or abs(a2) >= .999
            or abs(a1) > 1.999 or not 0 < b1 <= 20 or abs(b2) > 20 or abs(c) > 100):
        raise ValueError('模型不稳定或系数超出本单电机实验范围；请检查数据并重新辨识')
    gain = (b1+b2)/denominator
    if not .1 <= gain <= 20 or not .04 <= dt <= .08 or not .05 <= weight <= 100:
        raise ValueError('模型增益、采样周期或控制权重不适用')
    return model


def prediction(model, y, old_y, u, old_u, disturbance=0):
    return (model['a1']*y + model['a2']*old_y + model['b1']*u
            + model['b2']*old_u + model['c'] + disturbance)


def dataset(rows):
    """held_pwm in row k is the input applied over (k-1,k], not next_pwm in row k."""
    useful = [r for r in rows if r['phase'] == 'RUNNING']
    if len(useful) < 150:
        raise ValueError('辨识至少需要 150 个连续板上样本')
    if any(b['seq'] != a['seq']+1 for a,b in zip(useful,useful[1:])):
        raise ValueError('采样序号不连续，不能把漏采的数据当成相邻状态拟合')
    y = np.array([(r['c1']+r['c2'])/2 for r in useful], dtype=float)
    u = np.array([r['held_pwm'] for r in useful], dtype=float)
    dt = np.array([r['dt'] for r in useful[1:]], dtype=float)
    if not np.isfinite(np.r_[y,u,dt]).all() or np.any(dt <= 0):
        raise ValueError('无效的采样值或时间戳')
    times = np.array([r['at_us'] for r in useful],dtype=np.int64)
    if np.any(np.diff(times)<=0) or not np.allclose(np.diff(times)/1e6,dt,atol=2e-6):
        raise ValueError('板上时间戳与采样间隔不一致，或设备发生重启')
    period = float(np.median(dt))
    if not .04 <= period <= .08 or np.max(np.abs(dt/period-1)) > .5:
        raise ValueError('采样时间抖动过大，不适合此固定周期模型')
    if len(np.unique(np.round(u[1:],0))) < 4 or np.ptp(u[1:]) < 20:
        raise ValueError('PWM 激励不足，需要多档阶跃，不能只用恒速 PID 数据拟合')
    mismatch = np.array([abs(r['c1']-r['c2']) for r in useful])
    if np.percentile(mismatch,95) > max(3,float(np.max(y))*.05):
        raise ValueError('两路编码器差异过大，先解决反馈问题')
    return useful,y,u,dt,period


def fit_model(rows, source_sha256, origin='hardware_id', move_weight=2.0):
    useful,y,u,dt,period = dataset(rows)
    # Preserve chronology: first 12 seconds for estimation, last 6 for validation.
    split = next((i for i,r in enumerate(useful) if r['elapsed'] >= 12), int(len(y)*2/3))
    if split < 80 or len(y)-split < 60:
        raise ValueError('训练/验证段过短，请完整运行 18 秒辨识')
    x = np.column_stack([y[1:split-1],y[:split-2],u[2:split],u[1:split-1],np.ones(split-2)])
    coefficients,_,rank,singular = np.linalg.lstsq(x,y[2:split],rcond=None)
    if rank != 5 or singular[0]/singular[-1] > 1e7:
        raise ValueError('数据不足以辨识这个模型（秩不足或病态）')
    model = dict(zip(COEFFICIENTS,map(float,coefficients)))
    model.update(sample_s=period,move_weight=move_weight)
    validate_model(model)
    one_step = np.array([prediction(model,y[k-1],y[k-2],u[k],u[k-1])
                         for k in range(split,len(y))])
    # Free-run validation uses only the two initial measured states, never resets
    # predicted y to a measurement during the held-out six seconds.
    simulated = y.copy()
    for k in range(split,len(y)):
        simulated[k] = prediction(model,simulated[k-1],simulated[k-2],u[k],u[k-1])
    error = simulated[split:]-y[split:]
    rmse = float(np.sqrt(np.mean(error**2)))
    one_rmse = float(np.sqrt(np.mean((one_step-y[split:])**2)))
    variance = float(np.sum((y[split:]-np.mean(y[split:]))**2))
    r2 = 1-float(error@error)/variance if variance>1e-8 else -math.inf
    threshold = max(5,float(np.ptp(y[split:]))*.10)
    accepted = rmse <= threshold and r2 >= .8
    denominator=1-model['a1']-model['a2']
    low,high=float(np.min(u[1:])),float(np.max(u[1:]))
    steady=lambda pwm: ((model['b1']+model['b2'])*pwm+model['c'])/denominator
    model.update(schema='motor-arx2-v1',validated=accepted,origin=origin,
                 source_sha256=source_sha256,counts_per_rev=1400,
                 sample_filter_s=.10,horizon=HORIZON,pwm_range=[low,high],
                 rpm_range=[steady(low),steady(high)],
                 validation=dict(free_run_rmse_rpm=rmse,one_step_rmse_rpm=one_rmse,
                                 r2=r2,rmse_limit_rpm=threshold,split_elapsed_s=useful[split]['elapsed'],
                                 dt_p95_s=float(np.percentile(dt,95)),dt_max_s=float(np.max(dt)),
                                 dt_relative_std=float(np.std(dt)/period)))
    return model, useful, simulated, split


class ReferenceMPC:
    """NumPy reference for offline simulation/tests, matching the MCU cost/constraints."""
    def __init__(self, model):
        self.model=validate_model(model)
        self.g=np.zeros((HORIZON,HORIZON))
        for j in range(HORIZON):
            y=old_y=old_u=0.
            for k in range(HORIZON):
                u=float(k==j)
                nxt=model['a1']*y+model['a2']*old_y+model['b1']*u+model['b2']*old_u
                self.g[k,j]=nxt
                old_y,y,old_u=y,nxt,u
        d=np.eye(HORIZON)-np.eye(HORIZON,k=-1)
        self.h=self.g.T@self.g+model['move_weight']*d.T@d
        self.plan=np.zeros(HORIZON)

    def step(self, y, old_y, old_u, disturbance, target, maximum):
        previous_control=old_u
        free=[]
        for k in range(HORIZON):
            nxt=prediction(self.model,y,old_y,0,old_u,disturbance)
            free.append(nxt-target)
            old_y,y,old_u=y,nxt,0
        linear=self.g.T@np.array(free)
        linear[0]-=self.model['move_weight']*previous_control
        self.plan=np.clip(np.r_[self.plan[1:],self.plan[-1]],0,maximum)
        for sweep in range(1,101):
            for j in range(HORIZON):
                gradient=linear[j]+self.h[j]@self.plan
                self.plan[j]=np.clip(self.plan[j]-gradient/self.h[j,j],0,maximum)
            residual=float(np.max(np.abs(np.clip(self.plan-(self.h@self.plan+linear)/self.h.diagonal(),
                                                  0,maximum)-self.plan)))
            if residual <= .03:
                return float(self.plan[0]),sweep,residual
        raise ValueError('离线 MPC 未收敛')


def metrics(rows):
    rows=[r for r in rows if r['phase']=='RUNNING']
    phases=[]
    starts=[0]+[i for i in range(1,len(rows)) if rows[i]['target']!=rows[i-1]['target']]
    for begin,end in zip(starts,starts[1:]+[len(rows)]):
        phase=rows[begin:end]
        if len(phase)<3:
            continue
        t=np.array([r['elapsed'] for r in phase])
        y=np.array([(r['c1']+r['c2'])/2 for r in phase])
        target=phase[0]['target']; error=y-target
        tail=t>=t[-1]-1
        initial=(rows[begin-1]['c1']+rows[begin-1]['c2'])/2 if begin else y[0]
        change=target-initial; band=max(2,abs(target)*.02)
        outside=np.flatnonzero(abs(error)>band)
        settled=(int(outside[-1])+1) if len(outside) else 0
        settling=(float(t[settled]-t[0]) if settled<len(t) and t[-1]-t[settled]>=.5 else None)
        iae=float(np.sum((abs(error[1:])+abs(error[:-1]))*.5*np.diff(t)))
        violation=[r.get('constraint_violation') for r in phase if r.get('constraint_violation') is not None]
        overshoot=max(0,float(np.max(error*np.sign(change))))
        phases.append(dict(target_rpm=target,start_s=float(t[0]),
                           tail_bias_rpm=float(np.mean(error[tail])),tail_mae_rpm=float(np.mean(abs(error[tail]))),
                           iae_rpm_s=iae,settling_s=settling,
                           overshoot_pct=overshoot/max(abs(change),1)*100,overshoot_rpm=overshoot,
                           above_target_rpm=max(0,float(np.max(error))),
                           prediction_violation_max_rpm=max(violation) if violation else None,
                           pwm_total_variation=float(np.sum(abs(np.diff([r['next_pwm'] for r in phase]))))))
    return phases


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_model(path, hardware=False):
    model=json.loads(Path(path).read_text())
    if model.get('schema')!='motor-arx2-v1' or not model.get('validated') or model.get('horizon')!=HORIZON:
        raise ValueError('模型尚未通过验证，不能用于 MPC 实机实验')
    if hardware and model.get('origin')!='hardware_id':
        raise ValueError('演示/仿真模型不能上传实机，请先完成 identify 和 fit')
    return validate_model(model)
