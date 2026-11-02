#!/usr/bin/env python3
"""Plot published observations; never connects to hardware or invents samples."""
import csv
from pathlib import Path
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    out = ROOT / 'docs/images'
    out.mkdir(parents=True, exist_ok=True)
    rows = list(csv.DictReader((ROOT / 'data/observed/metrics.csv').open()))
    groups = [('baseline_w2', 'mpc', 'MPC w=2'), ('weight_w8', 'mpc', 'MPC w=8'),
              ('capped_margin1', 'mpc', 'MPC reference'),
              ('capped_margin1', 'mpc_cap', 'MPC cap (margin 1)'),
              ('baseline_w2', 'pid', 'PID reference')]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.8))
    for ax, field, title in zip(axes, ['tail_mae_rpm', 'iae_rpm_s', 'overshoot_percent'],
                               ['Tail MAE (RPM)', 'IAE (RPM*s)', 'Overshoot (% of step)']):
        x = np.arange(len(groups))
        for offset, target in [(-.18, '80'), (.18, '110')]:
            values = [np.mean([float(r[field]) for r in rows if
                      (r['experiment'], r['controller'], r['target_rpm']) == (exp, ctrl, target)])
                      for exp, ctrl, _ in groups]
            ax.bar(x + offset, values, width=.35, label=target+' RPM')
        ax.set_xticks(x, [g[2] for g in groups], rotation=35, ha='right')
        ax.set_title(title); ax.grid(axis='y', alpha=.25); ax.legend()
    fig.suptitle('Observed experiment summaries (mean of two trials; console-transcribed metrics)')
    fig.tight_layout(); fig.savefig(out / 'mpc-summary.png', dpi=150); plt.close(fig)

    text = (ROOT/'data/observed/capped_margin1.txt').read_text()
    pattern = re.compile(r'^\s*([\d.]+)s (MPC_CAP|MPC)\s+目标=\s*([\d.]+) 实测=\s*([\d.-]+) RPM PWM=\s*([\d.]+)%', re.M)
    runs = []
    for m in pattern.finditer(text):
        t, ctrl, target, rpm, pwm = m.groups()
        sample = [float(t), float(target), float(rpm), float(pwm)]
        if not runs or ctrl != runs[-1][0] or sample[0] < runs[-1][1][-1][0]:
            runs.append((ctrl, []))
        runs[-1][1].append(sample)
    if not runs:
        raise RuntimeError('No console samples matched; do not produce a fabricated plot')
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for index, (ctrl, samples) in enumerate(runs):
        a = np.array(samples)
        axes[0].plot(a[:,0], a[:,2], label=f'{ctrl} run {index+1}')
        axes[1].plot(a[:,0], a[:,3], label=f'{ctrl} run {index+1}')
    a = np.array(runs[0][1])
    axes[0].step(a[:,0], a[:,1], where='post', color='black', linestyle='--', label='Target')
    axes[0].set_ylabel('Filtered RPM'); axes[1].set_ylabel('PWM (%)')
    axes[1].set_xlabel('Time in each run (s)')
    for ax in axes: ax.legend(); ax.grid(alpha=.25)
    fig.suptitle('Observed console samples (~0.2s display interval), not full-rate telemetry')
    fig.tight_layout(); fig.savefig(out/'mpc-console-comparison.png', dpi=150); plt.close(fig)
    print(f'Generated plots in {out}')


if __name__ == '__main__':
    main()
