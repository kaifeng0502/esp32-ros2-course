#!/usr/bin/env python3
"""Send a clock / remaining-percentage / original pixel animation over USB or Wi-Fi."""
import argparse
import json
import math
import time
import zlib
import serial
from display import render,pack
from encoder_monitor import choose_port
from wifi_transport import WiFiTransport


def packet(frame,sequence):
    if len(frame)!=1024:raise ValueError('Expected 1024 display bytes')
    return f'OLED1 {sequence} {zlib.crc32(frame):08x} {frame.hex()}\n'.encode()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--transport',choices=('usb','wifi'),default='usb')
    parser.add_argument('--remaining',type=float,default=73)
    parser.add_argument('--seconds',type=float,default=15)
    parser.add_argument('--text',help='Short ASCII message; Chinese needs a suitable font and custom rasterization')
    parser.add_argument('--snapshot',type=str,help='Optional JSON file containing remaining_percent')
    args=parser.parse_args()
    if not math.isfinite(args.remaining) or not 0<=args.remaining<=100:parser.error('remaining must be 0..100')
    if not math.isfinite(args.seconds) or not 0<args.seconds<=3600:parser.error('seconds must be 0..3600')
    device=None
    try:
        if args.transport=='wifi':device=WiFiTransport.from_config()
        else:
            device=serial.Serial(port=None,baudrate=115200,timeout=.1,write_timeout=1,exclusive=True)
            device.dtr=device.rts=False;device.port=choose_port(args.port);device.open()
            device.reset_input_buffer();device.write(b'DISPLAY STATUS\n')
        end=time.monotonic()+args.seconds;sequence=0
        while time.monotonic()<end:
            remaining=args.remaining
            if args.snapshot:
                value=json.loads(open(args.snapshot,encoding='utf-8').read())['remaining_percent']
                if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<=value<=100:
                    raise ValueError('remaining_percent must be a finite percentage')
                remaining=value
            frame=pack(render({'used_percent':100-remaining},text=args.text))
            if args.transport=='wifi':device.send_frame(frame,sequence)
            else:
                device.write(packet(frame,sequence));deadline=time.monotonic()+2
                while time.monotonic()<deadline:
                    if device.readline().strip()==f'ACK {sequence}'.encode():break
                else:raise RuntimeError('No OLED frame ACK; close other serial clients and check display wiring')
            sequence=(sequence+1)&0xffffffff;time.sleep(.25)
    except KeyboardInterrupt:
        print('Display stopped; serial connection released.')
    finally:
        if device is not None:device.close()


if __name__=='__main__':main()
