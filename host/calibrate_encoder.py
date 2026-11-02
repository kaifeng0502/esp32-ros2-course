#!/usr/bin/env python3
"""Read-only manual wheel calibration. Disconnect motor VM power before use."""
import argparse
import json
from pathlib import Path
import time
import serial
from encoder_monitor import choose_port


def snapshot(port):
    port.reset_input_buffer();port.write(b'ENC?\n');deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        fields=port.readline().split()
        if len(fields)==5 and fields[0]==b'ENC':
            counts=tuple(int(v) for v in fields[1:3])
            if all(0<=v<2**32 for v in counts):return counts
    raise RuntimeError('No valid ENC response; close other monitors and check USB')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port');p.add_argument('--turns',type=int,default=5)
    p.add_argument('--out',type=Path)
    args=p.parse_args()
    if not 1<=args.turns<=20:p.error('turns must be 1..20')
    port=serial.Serial(port=None,baudrate=115200,timeout=.1,write_timeout=1,exclusive=True)
    port.dtr=port.rts=False;port.port=choose_port(args.port)
    try:
        port.open()
        input('电机电源须断开。给轮胎做标记并对齐，然后按 Enter 记录起点。')
        start=snapshot(port)
        input(f'现在同方向慢转准确 {args.turns} 圈，标记对齐后停稳，再按 Enter。')
        finish=snapshot(port)
        delta=[(b-a)&0xffffffff for a,b in zip(start,finish)]
        if any(d==0 or d>1000000 for d in delta):raise RuntimeError('No counts or possible board reset; repeat calibration')
        per_channel=[d/args.turns for d in delta]
        result=dict(turns=args.turns,initial_counts=start,final_counts=finish,
                    edges_per_revolution_per_channel=per_channel,
                    average_edges_per_revolution=sum(per_channel)/2,
                    note='Both edges on each channel; do not sum both channels and divide by one-channel calibration')
        print(json.dumps(result,indent=2,ensure_ascii=False))
        if abs(per_channel[0]-per_channel[1])/max(per_channel)>.05:
            print('两路差异超过5%，先排查接触/供电/共地，再重复校准。')
        if args.out:
            args.out.parent.mkdir(parents=True,exist_ok=True)
            args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    finally:port.close()


if __name__=='__main__':main()
