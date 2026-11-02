#!/usr/bin/env python3
"""On-board identification and matched PID/MPC trials. Fit/report/demo never open serial."""
import argparse
import contextlib
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

import numpy as np
import serial

from encoder_monitor import choose_port
from predictive_model import (COEFFICIENTS, ReferenceMPC, file_hash, fit_model,
                              load_model, metrics, prediction)

ROOT=Path(__file__).resolve().parents[1]
BASE_FIELDS=('run seq at_us elapsed dt target c1 c2 held_pwm next_pwm solve_us sweeps residual disturbance phase').split()
PREDICTION_FIELDS=('prediction_limit predicted_peak constraint_violation').split()
FIELDS=BASE_FIELDS+PREDICTION_FIELDS
INTEGERS={'run','seq','at_us','solve_us','sweeps'}


def read_csv(path):
    with Path(path).open(newline='') as source:
        return [{k: v if k=='phase' else None if v=='' else int(v) if k in INTEGERS else float(v)
                 for k,v in row.items()} for row in csv.DictReader(source)]


def write_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def parse_observation(line):
    parts=line.decode('ascii').split()
    if not parts or parts[0] not in ('OBS1','OBS2'):
        raise ValueError('采样协议格式不匹配')
    fields=BASE_FIELDS if parts[0]=='OBS1' else FIELDS
    if len(parts)!=len(fields)+1:raise ValueError('采样协议字段数量不匹配')
    row={k: v if k=='phase' else int(v) if k in INTEGERS else float(v)
         for k,v in zip(fields,parts[1:])}
    if (any(not math.isfinite(v) for k,v in row.items() if k!='phase') or
            not 0<=row['held_pwm']<=80 or not 0<=row['next_pwm']<=80 or row['dt']<0):
        raise ValueError('采样数据无效')
    for key in PREDICTION_FIELDS:
        if key not in row or row[key]==-1:row[key]=None
        elif row[key]<0:raise ValueError('预测轨迹诊断值无效')
    return row


def query_capability(port):
    port.write(b'MPC?\n')
    deadline=time.monotonic()+1.5
    while time.monotonic()<deadline:
        fields=port.readline().split()
        if fields and fields[0]==b'MPC1':
            if len(fields)!=16:
                raise RuntimeError('MPC1 协议版本不匹配')
            return dict(run=int(fields[1]),model_ready=int(fields[2]),active=int(fields[3]),
                        algorithm=fields[4].decode(),reason=fields[5].decode(),samples=int(fields[6]),
                        dropped=int(fields[7]),pending=int(fields[8]),
                        model=dict(zip((*COEFFICIENTS,'sample_s','move_weight'),map(float,fields[9:]))))
    raise RuntimeError('未收到 MPC1 回应：请先烧录本课固件，并关闭其他串口桥接')


def query_constraints(port):
    port.write(b'MPC2?\n')
    deadline=time.monotonic()+1.5
    while time.monotonic()<deadline:
        fields=port.readline().split()
        if fields and fields[0]==b'MPC2':
            if len(fields)!=6 or fields[3]!=b'10' or fields[4]!=b'2':
                raise RuntimeError('约束 MPC 协议版本不匹配')
            return dict(configured=int(fields[1]),margin=float(fields[2]),result=int(fields[5]))
    raise RuntimeError('未收到 MPC2 回应：请先烧录约束版固件；尚未发送运动命令')


def configure_constraints(port,margin):
    query_constraints(port) # Feature detection before configuration or any motion.
    if query_capability(port)['active']:raise RuntimeError('请先停止已有运动任务')
    port.write(f'MOTOR CAP {margin:.8g}\n'.encode())
    state=query_constraints(port)
    if not state['configured'] or not math.isclose(state['margin'],margin,abs_tol=1e-4):
        raise RuntimeError('ESP32 未接受预测转速余量')


def read_batch(port,run,version=1):
    port.write(b'SAMPLES2?\n' if version==2 else b'SAMPLES?\n')
    deadline=time.monotonic()+1.5
    rows=[]
    while time.monotonic()<deadline:
        line=port.readline()
        if line.startswith((b'OBS1 ',b'OBS2 ')):
            if not line.startswith(b'OBS2 ' if version==2 else b'OBS1 '):
                raise RuntimeError('采样协议与请求版本不符，不能验证预测约束')
            row=parse_observation(line)
            if row['run']!=run:
                raise RuntimeError('收到了不同实验的采样，可能有其他程序访问串口')
            rows.append(row)
        elif line.startswith(b'OBSEND '):
            fields=line.split()
            if len(fields)!=4 or int(fields[1])!=run or int(fields[2]):
                raise RuntimeError('板上采样缓冲溢出或实验标识改变，请重新采集')
            return rows,int(fields[3])
    raise RuntimeError('读取采样超时，实验将请求停车')


@contextlib.contextmanager
def connect(name):
    port=serial.Serial(port=None,baudrate=115200,timeout=.10,write_timeout=1,exclusive=True)
    port.dtr=False; port.rts=False; port.port=name or choose_port()
    try:
        port.open()
        yield port
    finally:
        if port.is_open:
            port.close()


def configure_model(port,model):
    cap=query_capability(port)
    if cap['active']:
        raise RuntimeError('电机已有任务运行，请先停止')
    keys=(*COEFFICIENTS,'sample_s','move_weight')
    port.write(('MOTOR MODEL '+' '.join(f'{model[k]:.8g}' for k in keys)+'\n').encode())
    cap=query_capability(port)
    if not cap['model_ready'] or any(not math.isclose(cap['model'][k],model[k],rel_tol=2e-6,abs_tol=2e-6) for k in keys):
        raise RuntimeError('ESP32 未接受或未正确回读模型参数')


def run_trial(port,args,algorithm,path):
    initial=query_capability(port)
    if initial['active']:
        raise RuntimeError('已有任务运行，不能覆盖它')
    seconds=18 if algorithm=='ID' else args.seconds
    step=0 if algorithm=='ID' else args.step_rpm
    step_at=0 if not step else args.step_at
    command=(f'MOTOR EXP {algorithm} {args.rpm} {seconds} {args.encoder_sign} '
             f'{args.kp:.8g} {args.ki:.8g} {args.kd:.8g} {args.max_pwm} {step} {step_at:.8g}\n')
    print(f'{algorithm}: {seconds}s，PWM 上限 {args.max_pwm}%；车轮悬空，电机电源为已调好的 10V。',flush=True)
    for n in (3,2,1):
        print(f'{n}…',flush=True); time.sleep(1)
    rows=[]; owned=False; result=None; last_display=-1.; last_status=0.
    path.parent.mkdir(parents=True,exist_ok=True)
    metadata=dict(command=command.strip(),status='INCOMPLETE',encoder_sign=args.encoder_sign,
                  sample_protocol_version=getattr(args,'sample_version',1),
                  counts_per_rev=1400,voltage_note='User must maintain measured 10V; not measured by firmware.',
                  feedback='same 0.10s low-pass RPM for PID and MPC',
                  sample_source='MCU timestamps, buffered every control update',
                  run_config=dict(rpm=args.rpm,step_rpm=step,step_at=step_at,seconds=seconds,
                                  kp=args.kp,ki=args.ki,kd=args.kd,max_pwm=args.max_pwm))
    if algorithm=='MPC_CAP':
        metadata['prediction_margin_rpm']=args.margin_rpm
        metadata['constraint_scope']='10-step filtered RPM prediction, not a physical speed guarantee'
    with path.open('w',newline='') as output:
        writer=csv.DictWriter(output,fieldnames=FIELDS); writer.writeheader()
        try:
            # Set ownership before write: a timeout can occur after a partial successful write.
            owned=True
            port.write(command.encode())
            cap=query_capability(port)
            if cap['run']!=initial['run']+1 or cap['algorithm']!=algorithm:
                raise RuntimeError('ESP32 未接受实验命令')
            run=cap['run']; metadata['run']=run
            deadline=time.monotonic()+seconds+6
            while time.monotonic()<deadline:
                batch,pending=read_batch(port,run,getattr(args,'sample_version',1))
                for row in batch:
                    if algorithm=='MPC_CAP' and row['phase']=='RUNNING':
                        if (any(row.get(key) is None for key in PREDICTION_FIELDS) or
                            not math.isclose(row['prediction_limit'],row['target']-args.margin_rpm,abs_tol=.001) or
                            row['constraint_violation']>0 or row['predicted_peak']>row['prediction_limit']+.00001):
                            metadata['rejected_sample']=row
                            raise RuntimeError('约束诊断缺失或预测转速越界，停止实验并保留记录')
                    if row['seq']!=(rows[-1]['seq']+1 if rows else 0):
                        raise RuntimeError('采样序号不连续，停止实验并保留已有记录')
                    rows.append(row); writer.writerow(row)
                    if row['elapsed']-last_display>=.2:
                        diagnostic=(f" 预测峰值={row['predicted_peak']:.2f} 上限={row['prediction_limit']:.2f}"
                                    if row['prediction_limit'] is not None and row['predicted_peak'] is not None else '')
                        print(f"{row['elapsed']:5.2f}s {algorithm:3} 目标={row['target']:5.1f} "
                              f"实测={(row['c1']+row['c2'])/2:6.1f} RPM PWM={row['next_pwm']:5.1f}% "
                              f"求解={row['solve_us']}us {row['phase']}{diagnostic}",flush=True)
                        last_display=row['elapsed']
                output.flush()
                now=time.monotonic()
                if now-last_status>=.2 or result:
                    cap=query_capability(port); last_status=now
                    if cap['run']!=run or cap['algorithm']!=algorithm or cap['dropped']:
                        raise RuntimeError('实验改变或丢失样本，当前记录不能用于模型拟合')
                    if not cap['active']:
                        result=cap
                        if cap['pending']==0:
                            break
                if not pending:
                    time.sleep(.025)
            else:
                raise RuntimeError('超过实验等待时间')
            if not result or result['reason']!='DONE':
                raise RuntimeError(f"实验未正常结束：{result['reason'] if result else 'UNKNOWN'}")
            metadata['status']='DONE'
            return rows
        except (Exception,KeyboardInterrupt) as error:
            metadata['error']=str(error) or type(error).__name__
            raise
        finally:
            if owned:
                try:
                    port.write(b'MOTOR STOP\n')
                except serial.SerialException:
                    print('串口不可用，停车指令未确认；固件仍有独立的运行时限。',flush=True)
            # Saving a report can fail (e.g. full disk); it must never prevent the stop request.
            metadata['samples']=len(rows)
            if result:
                metadata['final_status']=result
            write_json(path.with_suffix('.json'),metadata)


def plot_fit(model,rows,simulated,split,destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=np.array([r['elapsed'] for r in rows])
    measured=np.array([(r['c1']+r['c2'])/2 for r in rows])
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
    axes[0].plot(t,measured,label='Measured filtered RPM')
    axes[0].plot(t[split:],simulated[split:],'--',label='Held-out free-run prediction')
    axes[0].axvline(t[split],color='gray',linestyle=':',label='Training / validation split')
    axes[0].set_ylabel('RPM'); axes[0].legend(); axes[0].grid()
    axes[1].step(t,[r['held_pwm'] for r in rows],where='pre',label='Applied PWM')
    axes[1].set_ylabel('PWM (%)'); axes[1].set_xlabel('MCU elapsed time (s)'); axes[1].grid()
    fig.suptitle(('SYNTHETIC DEMO - ' if model['origin']!='hardware_id' else '')+
                 f"Model validation RMSE={model['validation']['free_run_rmse_rpm']:.2f} RPM")
    fig.tight_layout(); fig.savefig(destination,dpi=150); plt.close(fig)


def fit_file(source,output,weight=2.):
    metadata=Path(source).with_suffix('.json')
    if not metadata.exists():
        raise ValueError('缺少辨识运行元数据，不能确认记录完整')
    meta=json.loads(metadata.read_text())
    if meta.get('status')!='DONE' or not meta.get('command','').startswith('MOTOR EXP ID '):
        raise ValueError('只能拟合完整正常结束的 ID 实验记录')
    model,rows,simulated,split=fit_model(read_csv(source),file_hash(source),move_weight=weight)
    model['encoder_sign']=int(meta['encoder_sign'])
    model['identification_metadata']=meta
    output.parent.mkdir(parents=True,exist_ok=True)
    write_json(output,model)
    plot_fit(model,rows,simulated,split,output.with_suffix('.png'))
    print('模型系数：'+', '.join(f'{key}={model[key]:.6g}' for key in COEFFICIENTS))
    v=model['validation']
    print(f"独立验证：自由预测 RMSE={v['free_run_rmse_rpm']:.2f} RPM，"
          f"一步 RMSE={v['one_step_rmse_rpm']:.2f} RPM，R²={v['r2']:.3f}，"
          f"{'通过' if model['validated'] else '未通过'}")
    print(f'模型：{output}\n验证曲线：{output.with_suffix(".png")}')
    if not model['validated']:
        raise ValueError('留出验证未通过，已保存结果用于诊断；禁止上传这个模型')


def report(folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    summary=[]; synthetic=False;cap_trials=[];controller_names=[]
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    for path in sorted(folder.glob('trial_*.csv')):
        meta=json.loads(path.with_suffix('.json').read_text())
        synthetic=synthetic or meta.get('origin')=='synthetic'
        if meta['status']!='DONE':
            raise ValueError(f'{path} 运行不完整，不能作为成功对比')
        rows=read_csv(path); running=[r for r in rows if r['phase']=='RUNNING']
        if not running:
            raise ValueError(f'{path} 无有效控制样本')
        t=[r['elapsed'] for r in running]; rpm=[(r['c1']+r['c2'])/2 for r in running]
        label=path.stem
        controller=path.stem.split('_',2)[-1].upper()
        if controller not in controller_names:controller_names.append(controller)
        if any(r.get('prediction_limit') is not None for r in running):cap_trials.append((label,running))
        axes[0].plot(t,rpm,label=label)
        axes[1].plot(t,[r['next_pwm'] for r in running],label=label)
        if not summary:
            axes[0].step(t,[r['target'] for r in running],where='post',color='black',linestyle='--',label='Target')
        timing=[r['solve_us'] for r in running]
        intervals=[r['dt'] for r in running if r['dt']>0]
        for phase in metrics(rows):
            summary.append(dict(trial=label,**phase,solver_max_us=max(timing),
                                solver_p95_us=float(np.percentile(timing,95)),
                                sample_p95_ms=float(np.percentile(intervals,95))*1000,
                                solver_max_period_percent=max(timing)/(np.median(intervals)*1e6)*100))
    if not summary:
        raise ValueError('目录内没有完整的 trial_*.csv')
    axes[0].set_ylabel('Filtered RPM'); axes[1].set_ylabel('PWM (%)')
    axes[1].set_xlabel('MCU elapsed time (s)')
    for axis in axes:
        axis.grid(); axis.legend()
    fig.suptitle(('SYNTHETIC TEST - ' if synthetic else '')+
                 ' vs '.join(controller_names)+': matched targets / PWM cap / feedback')
    fig.tight_layout(); fig.savefig(folder/'comparison.png',dpi=150); plt.close(fig)
    with (folder/'metrics.csv').open('w',newline='') as out:
        writer=csv.DictWriter(out,fieldnames=list(summary[0])); writer.writeheader(); writer.writerows(summary)
    write_json(folder/'metrics.json',summary)
    if cap_trials:
        limit_fig,limit_axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
        for label,rows in cap_trials:
            t=[r['elapsed'] for r in rows]
            measured=[(r['c1']+r['c2'])/2 for r in rows]
            peak=[r.get('predicted_peak') for r in rows]
            limit_axes[0].plot(t,measured,label=label+' measured')
            limit_axes[0].plot(t,peak,linestyle=':',label=label+' predicted peak')
            limit_axes[1].plot(t,[max(0,y-r['target']) for y,r in zip(measured,rows)],label=label+' measured excess')
            limit_axes[1].plot(t,[r.get('constraint_violation') for r in rows],linestyle=':',label=label+' predicted excess')
        first=cap_trials[0][1];t=[r['elapsed'] for r in first]
        limit_axes[0].step(t,[r['target'] for r in first],where='post',color='black',linestyle='--',label='Requested RPM')
        limit_axes[0].step(t,[r.get('prediction_limit') for r in first],where='post',color='gray',linestyle='--',label='Prediction limit')
        limit_axes[0].set_ylabel('Filtered RPM');limit_axes[1].set_ylabel('Excess RPM')
        limit_axes[1].set_ylim(0,max(.2,limit_axes[1].get_ylim()[1]))
        limit_axes[1].set_xlabel('MCU elapsed time (s)')
        for axis in limit_axes:axis.grid();axis.legend(fontsize=8)
        limit_fig.suptitle(('SYNTHETIC TEST - ' if synthetic else '')+'Prediction constraint vs measured feedback')
        limit_fig.tight_layout();limit_fig.savefig(folder/'constraint_check.png',dpi=150);plt.close(limit_fig)
    for row in summary:
        settling='未在记录内稳定' if row['settling_s'] is None else f"{row['settling_s']:.2f}s"
        print(f"{row['trial']} target={row['target_rpm']:.0f}: "
              f"tail MAE={row['tail_mae_rpm']:.2f} RPM, IAE={row['iae_rpm_s']:.2f} RPM*s, "
              f"overshoot={row['overshoot_pct']:.1f}% ({row['overshoot_rpm']:.2f} RPM), settling={settling}, "
              f"MPC solver max={row['solver_max_us']}us")
        if row.get('prediction_violation_max_rpm') is not None:
            print(f"  预测约束最大违反={row['prediction_violation_max_rpm']:.5f} RPM；"
                  f"实测高于目标的最大值={row['above_target_rpm']:.3f} RPM")
    print(f"曲线：{folder/'comparison.png'}\n指标：{folder/'metrics.csv'}")


def arguments(argv=None):
    parser=argparse.ArgumentParser(description='电机预测模型辨识与板上 PID/MPC 对比')
    commands=parser.add_subparsers(dest='mode',required=True)
    for name in ('identify','compare'):
        p=commands.add_parser(name)
        p.add_argument('--port'); p.add_argument('--out',type=Path)
        p.add_argument('--encoder-sign',type=int,choices=(-1,1),default=-1)
        p.add_argument('--rpm',type=int,default=80)
        p.add_argument('--seconds',type=int,default=10)
        p.add_argument('--step-rpm',type=int,default=120)
        p.add_argument('--step-at',type=float,default=4)
        p.add_argument('--kp',type=float,default=.15)
        p.add_argument('--ki',type=float,default=.30)
        p.add_argument('--kd',type=float,default=.005)
        p.add_argument('--max-pwm',type=int,choices=range(55,61),default=55)
        if name=='compare':
            p.add_argument('--model',type=Path,required=True)
            p.add_argument('--rounds',type=int,choices=(1,2,3),default=2)
            p.add_argument('--controllers',default='pid,mpc',help='Two distinct controllers: pid,mpc,mpc_cap')
            p.add_argument('--margin-rpm',type=float,default=1.,help='Prediction cap = requested RPM minus margin; 0..5')
    p=commands.add_parser('fit')
    p.add_argument('csv',type=Path); p.add_argument('--out',type=Path)
    p.add_argument('--move-weight',type=float,default=2)
    p=commands.add_parser('report'); p.add_argument('folder',type=Path)
    args=parser.parse_args(argv)
    if args.mode in ('identify','compare'):
        if (not 40<=args.rpm<=180 or not 1<=args.seconds<=30 or
            args.step_rpm!=0 and not 40<=args.step_rpm<=180 or
            not math.isfinite(args.step_at) or args.step_rpm and not 1<=args.step_at<args.seconds or
            any(not math.isfinite(v) or not 0<=v<=limit for v,limit in
                [(args.kp,5),(args.ki,10),(args.kd,1)])):
            parser.error('目标转速、运行时间、阶跃时间或 PID 参数超出实验范围')
    if args.mode=='fit' and (not math.isfinite(args.move_weight) or not .05<=args.move_weight<=100):
        parser.error('move-weight 必须在 0.05 到 100 之间')
    if args.mode=='compare':
        args.controllers=[v.strip().upper() for v in args.controllers.split(',')]
        if (len(args.controllers)!=2 or len(set(args.controllers))!=2 or
                any(v not in ('PID','MPC','MPC_CAP') for v in args.controllers)):
            parser.error('controllers 需要两个不同控制器，例如 mpc,mpc_cap')
        if not math.isfinite(args.margin_rpm) or not 0<=args.margin_rpm<=5:
            parser.error('margin-rpm 必须在 0 到 5 RPM 之间')
        if 'MPC_CAP' in args.controllers and args.step_rpm and args.step_rpm<args.rpm:
            parser.error('约束版第一轮仅测试升速；降速可能因惯性导致约束不可行')
    return args


def main(argv=None):
    args=arguments(argv)
    if args.mode=='fit':
        fit_file(args.csv,args.out or args.csv.with_name('model.json'),args.move_weight)
        return
    if args.mode=='report':
        report(args.folder); return
    folder=args.out or ROOT/'.run/mpc'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    if folder.exists() and any(folder.iterdir()):
        raise ValueError('输出目录非空，请换一个目录，避免覆盖实验')
    folder.mkdir(parents=True,exist_ok=True)
    if args.mode=='compare':
        model=load_model(args.model,hardware=True)
        if model.get('encoder_sign')!=args.encoder_sign:
            raise ValueError('编码器方向必须与辨识时一致')
        low,high=model['rpm_range']
        if any(not low<=target<=high for target in [args.rpm,args.step_rpm or args.rpm]):
            raise ValueError(f'目标超出辨识工作区间 [{low:.1f}, {high:.1f}] RPM，先扩展辨识数据')
        if args.max_pwm>model['pwm_range'][1]+.1:
            raise ValueError('PWM 上限超出辨识时的最高激励；本课先使用 --max-pwm 55')
        if 'MPC_CAP' in args.controllers and np.min(ReferenceMPC(model).g)<0:
            raise ValueError('当前约束版要求预测输入响应非负；此模型不支持，尚未连接串口')
        write_json(folder/'model.json',model)
        write_json(folder/'experiment.json',dict(model_sha256=file_hash(args.model),
                   ordering='First round configured order; reverse order on alternate rounds',same_supply_required_v=10,
                   same_load_required=True,settings=vars(args)|{'out':str(folder),'model':str(args.model)}))
    with connect(args.port) as port:
        if args.mode=='identify':
            run_trial(port,args,'ID',folder/'identification.csv')
            print(f'辨识记录：{folder/"identification.csv"}\n下一步：python3 motor_mpc.py fit {folder/"identification.csv"}')
        else:
            configure_model(port,model)
            if 'MPC_CAP' in args.controllers:
                configure_constraints(port,args.margin_rpm)
                args.sample_version=2
            for iteration in range(args.rounds):
                order=args.controllers if iteration%2==0 else list(reversed(args.controllers))
                for algorithm in order:
                    # Allow coast-down/cooling; firmware additionally waits for encoder quiet.
                    time.sleep(3)
                    run_trial(port,args,algorithm,folder/f'trial_{iteration+1:02d}_{algorithm.lower()}.csv')
    if args.mode=='compare':
        report(folder)


if __name__=='__main__':
    try:
        main()
    except (ValueError,RuntimeError,serial.SerialException,OSError) as error:
        raise SystemExit(str(error))
    except KeyboardInterrupt:
        raise SystemExit('用户结束实验；已尝试发送停车命令。')
