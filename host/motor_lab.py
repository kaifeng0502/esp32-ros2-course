"""Position moves and reproducible OPEN/P/PI/PID comparisons. Does not move on import."""
import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import statistics
import time

import serial
from encoder_monitor import choose_port

FIELDS = ('active requested target c1 c2 pwm elapsed reason seconds encoder_sign '
          'algorithm position_mode goal_turns position_turns p i d ff saturated '
          'kp ki kd antiwindup max_pwm').split()
STRINGS = {'reason', 'algorithm'}


def read_status(port):
    port.write(b'LAB?\n')
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        parts = port.readline().decode(errors='replace').strip().split()
        if len(parts) != len(FIELDS) + 1 or parts[0] != 'LAB1':
            continue
        try:
            result = {key: value if key in STRINGS else float(value)
                      for key, value in zip(FIELDS, parts[1:])}
            if not all(math.isfinite(value) for key, value in result.items() if key not in STRINGS):
                continue
            return result
        except ValueError:
            pass
    raise RuntimeError('未收到 LAB 状态，请烧录新版固件并关闭其他串口监控。')


def command(args, algorithm):
    turns = args.turns if args.turns is not None else (args.degrees / 360 if args.degrees is not None else 0)
    direction = 'FWD' if args.direction == 'forward' else 'REV'
    return (f'MOTOR LAB {args.rpm} {args.seconds} {direction} {args.encoder_sign} '
            f'{algorithm.upper()} {turns:.8g} {args.kp:.8g} {args.ki:.8g} {args.kd:.8g} '
            f'{0 if args.no_anti_windup else 1} {args.max_pwm:.8g} '
            f'{args.step_rpm or 0} {args.step_at or 0:.8g}\n').encode()


def show(row):
    actual = (row['c1'] + row['c2']) / 2
    position = (f"  位置={row['position_turns']:.4f}/{row['goal_turns']:.4f}圈"
                if row['position_mode'] else '')
    print(f"{row['elapsed']:5.2f}s {row['algorithm']:4} 目标={row['target']:6.1f}"
          f" 实测={actual:7.1f} RPM PWM={row['pwm']:5.1f}%"
          f" P={row['p']:6.2f} I={row['i']:6.2f} D={row['d']:6.2f}"
          f"{' [限幅]' if row['saturated'] else ''}{position}  {row['reason']}", flush=True)


def run_one(port, args, algorithm, csv_path):
    initial = read_status(port)  # Capability check before any motor command.
    if initial['active']:
        raise RuntimeError('已有任务运行中，请先停止当前任务。')
    print(f'\n{algorithm.upper()}，{args.rpm} RPM，最长 {args.seconds} 秒；车轮须悬空，10V 已接通。', flush=True)
    for remaining in (3, 2, 1):
        print(f'{remaining}…', flush=True)
        time.sleep(1)
    rows = []
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open('w', newline='') as output:
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        writer.writeheader()
        try:
            port.write(command(args, algorithm))
            deadline = time.monotonic() + args.seconds + 6
            goal = (args.turns if args.turns is not None else
                    args.degrees / 360 if args.degrees is not None else 0)
            if args.direction == 'reverse':
                goal = -goal
            while time.monotonic() < deadline:
                row = read_status(port)
                # Refuse stale status or a rejected/misparsed command.
                if (row['algorithm'] != algorithm.upper() or row['seconds'] != args.seconds or
                        row['encoder_sign'] != args.encoder_sign or
                        abs(row['goal_turns'] - goal) > 0.000002 or
                        row['antiwindup'] != (not args.no_anti_windup) or
                        any(abs(row[key] - getattr(args, key)) > 0.00011
                            for key in ('kp', 'ki', 'kd', 'max_pwm'))):
                    raise RuntimeError('开发板返回的实验参数不匹配，已发送停止。')
                sign = 1 if args.direction == 'forward' else -1
                allowed_requests = [sign * args.rpm]
                if args.step_rpm:
                    allowed_requests.append(sign * args.step_rpm)
                if row['requested'] not in allowed_requests:
                    raise RuntimeError('返回的目标转速不匹配，已发送停止。')
                if not rows or (row['elapsed'], row['reason']) != (rows[-1]['elapsed'], rows[-1]['reason']):
                    writer.writerow(row)
                    output.flush()
                    rows.append(row)
                    show(row)
                if not row['active']:
                    if row['reason'] != 'DONE':
                        raise RuntimeError(f"实验停止：{row['reason']}。DIRECTION_ERROR 请检查 --encoder-sign；"
                                           'NO_FEEDBACK 请检查电源和编码器；TIMEOUT 表示未在期限内到达。')
                    if row['position_mode']:
                        error = row['position_turns'] - row['goal_turns']
                        print(f'停止位置={row["position_turns"]:.5f} 圈，目标={row["goal_turns"]:.5f} 圈，'
                              f'误差={error:+.5f} 圈 / {error*360:+.2f}°（编码器推算）。')
                    return rows
                time.sleep(0.1)
            raise RuntimeError('等待完成超时。')
        finally:
            try:
                port.write(b'MOTOR STOP\n')
            except serial.SerialException:
                print('停止指令未送达，板上最长运行时间保护仍有效。', flush=True)
            print(f'记录：{csv_path}', flush=True)


def phase_metrics(rows):
    running = [r for r in rows if r['reason'] in ('RUNNING', 'DECEL')]
    if not running:
        return []
    groups = []
    for row in running:
        if not groups or row['requested'] != groups[-1][0]['requested']:
            groups.append([])
        groups[-1].append(row)
    results = []
    for index, group in enumerate(groups):
        reference = group[0]['requested']
        sign = 1 if reference > 0 else -1
        target = abs(reference)
        # Initial phase starts when the motor actually starts, not first host response.
        start = 0 if index == 0 else group[0]['elapsed']
        tail_start = group[-1]['elapsed'] - max(1, (group[-1]['elapsed']-start)*0.2)
        tail = [sign*(r['c1']+r['c2'])/2 - target for r in group if r['elapsed'] >= tail_start]
        speeds = [sign*(r['c1']+r['c2'])/2 for r in group]
        errors = [abs(v-target) for v in speeds]
        settling = None
        tolerance = 0.02 * target
        for i, row in enumerate(group):
            if group[-1]['elapsed']-row['elapsed'] >= 1 and all(e <= tolerance for e in errors[i:]):
                settling = row['elapsed']-start
                break
        falling = index > 0 and target < abs(groups[index-1][0]['requested'])
        overshoot = max(0, target-min(speeds) if falling else max(speeds)-target) / target*100
        results.append(dict(target_rpm=reference, tail_bias_rpm=statistics.mean(tail),
                            tail_mae_rpm=statistics.mean(abs(x) for x in tail),
                            overshoot_pct=overshoot, settling_s=settling,
                            saturated_samples=sum(bool(r['saturated']) for r in group)))
    return results


def report(runs, folder):
    # Import after the motor is stopped. Never start another run while plotting.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    position_mode = any(rows and rows[0]['position_mode'] for _, rows in runs)
    fig, axes = plt.subplots(4 if position_mode else 3, 1, figsize=(10, 9), sharex=True)
    for name, rows in runs:
        data = [r for r in rows if r['reason'] != 'WAITING']
        if not data:
            continue
        times = [r['elapsed'] for r in data]
        axes[0].plot(times, [(r['c1']+r['c2'])/2 for r in data], label=name.upper())
        axes[0].plot(times, [r['target'] for r in data], '--', color='gray', alpha=0.5,
                     label='Target' if name == runs[0][0] else '_nolegend_')
        axes[1].plot(times, [r['pwm'] for r in data], label=name.upper())
        axes[2].plot(times, [r['i'] for r in data], label=name.upper())
        if position_mode:
            axes[3].plot(times, [r['position_turns'] for r in data], label=name.upper())
            axes[3].axhline(data[-1]['goal_turns'], linestyle='--', color='gray')
        if not position_mode:
            for phase in phase_metrics(data):
                settling = 'not settled' if phase['settling_s'] is None else f"{phase['settling_s']:.2f}s"
                print(f"{name.upper()} target={phase['target_rpm']:.0f}: "
                      f"tail bias={phase['tail_bias_rpm']:+.2f} RPM, "
                      f"tail MAE={phase['tail_mae_rpm']:.2f} RPM, "
                      f"overshoot={phase['overshoot_pct']:.1f}%, settling={settling}")
    labels = ['Speed (RPM)', 'Drive PWM (%)', 'Integral correction (pp)', 'Position (wheel turns)']
    for ax, label in zip(axes, labels):
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.2)
        ax.legend(loc='best')
    axes[-1].set_xlabel('Time since motor start (s)')
    fig.tight_layout()
    for suffix in ('png', 'svg'):
        fig.savefig(folder / f'comparison.{suffix}', dpi=160)
    plt.close(fig)
    metrics = folder / 'metrics.csv'
    with metrics.open('w', newline='') as output:
        keys = ['algorithm', 'target_rpm', 'tail_bias_rpm', 'tail_mae_rpm',
                'overshoot_pct', 'settling_s', 'saturated_samples']
        writer = csv.DictWriter(output, fieldnames=keys)
        writer.writeheader()
        if not position_mode:
            for name, rows in runs:
                for phase in phase_metrics(rows):
                    writer.writerow(dict(algorithm=name.upper(), **phase))
    print(f'曲线：{folder / "comparison.png"}\n指标：{metrics}')



def load_runs(folder):
    runs = []
    for algorithm in ('open', 'p', 'pi', 'pid'):
        path = folder / f'{algorithm}.csv'
        if not path.is_file():
            continue
        with path.open(newline='') as source:
            reader = csv.DictReader(source)
            if reader.fieldnames != FIELDS:
                raise ValueError(f'记录格式不匹配：{path}')
            rows = []
            for row in reader:
                parsed = {key: row[key] if key in STRINGS else float(row[key]) for key in FIELDS}
                if not all(math.isfinite(value) for key, value in parsed.items() if key not in STRINGS):
                    raise ValueError(f'记录包含无效数字：{path}')
                rows.append(parsed)
            if rows:
                runs.append((algorithm, rows))
    if not runs:
        raise ValueError('目录中没有可用的 open/p/pi/pid.csv 记录')
    return runs


def parser():
    p = argparse.ArgumentParser(description='指定圈数/角度，或比较 OPEN/P/PI/PID。默认正转 PI，120 RPM，15 秒。')
    p.add_argument('--port', help='串口路径；默认自动识别')
    p.add_argument('--rpm', type=int, default=120)
    p.add_argument('--seconds', type=int, default=15, help='最长运行时间，1～60 秒；位置任务超时也停机')
    p.add_argument('--direction', choices=('forward', 'reverse'), default='forward')
    p.add_argument('--encoder-sign', type=int, choices=(-1, 1), default=1)
    goal = p.add_mutually_exclusive_group()
    goal.add_argument('--turns', type=float, help='相对本次起点的车轮圈数，5/360～100')
    goal.add_argument('--degrees', type=float, help='相对本次起点的车轮角度，5～36000°')
    p.add_argument('--algorithm', choices=('open', 'p', 'pi', 'pid'), default='pi')
    p.add_argument('--compare', action='store_true', help='依次实际运行 OPEN/P/PI/PID 四次，每次运行 --seconds 秒')
    p.add_argument('--kp', type=float, default=0.15)
    p.add_argument('--ki', type=float, default=0.30)
    p.add_argument('--kd', type=float, default=0.005)
    p.add_argument('--max-pwm', type=float, default=80)
    p.add_argument('--no-anti-windup', action='store_true', help='关闭条件积分抗饱和；仍保留积分 ±25 pp 硬上限')
    p.add_argument('--step-rpm', type=int, help='在本次实验中途切换到这个目标转速')
    p.add_argument('--step-at', type=float, help='启动多少秒后切换目标')
    p.add_argument('--plot', type=Path, help='只读取已有实验目录重新画图，不连接开发板、不转电机')
    p.add_argument('--output', type=Path, help='新建的记录目录，默认项目 .run/lab/时间戳')
    return p


def arguments(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if not 40 <= args.rpm <= 180 or not 1 <= args.seconds <= 60:
        p.error('转速须为 40～180 RPM，时间须为 1～60 秒')
    for key, low, high in [('kp',0,5),('ki',0,10),('kd',0,1),('max_pwm',20,80)]:
        value = getattr(args,key)
        if not math.isfinite(value) or not low <= value <= high:
            p.error(f'--{key.replace("_","-")} 须在 {low}～{high} 之间')
    position = args.turns is not None or args.degrees is not None
    if position:
        turns = args.turns if args.turns is not None else args.degrees/360
        if not math.isfinite(turns) or not 5/360 <= turns <= 100:
            p.error('圈数范围 5/360～100，角度范围 5～36000°')
        if args.algorithm != 'pi' or args.compare or args.step_rpm is not None:
            p.error('位置控制使用 PI，不与算法比较或转速阶跃同时运行')
    if (args.step_rpm is None) != (args.step_at is None):
        p.error('--step-rpm 与 --step-at 需要一起填写')
    if args.step_rpm is not None and (not 40 <= args.step_rpm <= 180 or
            not math.isfinite(args.step_at) or not 0.5 <= args.step_at < args.seconds):
        p.error('阶跃目标须为 40～180 RPM；切换时间须在 0.5 秒至运行结束之前')
    return args


def main():
    args = arguments()
    if args.plot:
        try:
            report(load_runs(args.plot), args.plot)
        except (OSError, ValueError) as exc:
            raise SystemExit(str(exc)) from exc
        return
    folder = args.output or Path(__file__).resolve().parents[1] / '.run/lab' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    folder.mkdir(parents=True, exist_ok=False)
    runs = []
    port = serial.Serial(port=None, baudrate=115200, timeout=0.2, write_timeout=2, exclusive=True)
    port.dtr = port.rts = False
    port.port = choose_port(args.port)
    try:
        port.open()
        port.reset_input_buffer()
        algorithms = ('open', 'p', 'pi', 'pid') if args.compare else (args.algorithm,)
        for index, algorithm in enumerate(algorithms):
            if index:
                time.sleep(2)
            rows = run_one(port, args, algorithm, folder / f'{algorithm}.csv')
            runs.append((algorithm, rows))
    except KeyboardInterrupt:
        print('\n已中断，已尝试停止电机。')
    except (RuntimeError, serial.SerialException) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        port.close()
        if runs:
            report(runs, folder)


if __name__ == '__main__':
    main()
