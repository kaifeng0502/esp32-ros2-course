"""Send a motor test/demo/stop command over the ESP32-S3 USB serial port."""

import argparse
import time

import serial
from serial.tools import list_ports


def choose_port(explicit):
    if explicit:
        return explicit
    ports = [p.device for p in list_ports.comports()
             if p.vid == 0x303A and p.pid == 0x1001]
    if len(ports) != 1:
        raise SystemExit("请只连接一块 ESP32-S3，或用 --port 指定串口。")
    return ports[0]


def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 电机手动测试")
    parser.add_argument("action", choices=("test", "demo", "stop"),
                        help="test=短转一次；demo=慢->快->慢->定住->反转；stop=停止")
    parser.add_argument("--port", help="默认自动寻找 ESP32-S3 的 USB 串口")
    args = parser.parse_args()
    device = choose_port(args.port)
    port = serial.Serial(port=None, baudrate=115200, timeout=0.2,
                         write_timeout=2, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = device
    try:
        port.open()
        port.reset_input_buffer()
        command = {"test": b"MOTOR TEST\n",
                   "demo": b"MOTOR DEMO\n",
                   "stop": b"MOTOR STOP\n"}[args.action]
        port.write(command)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if port.readline().strip() == b"ACK MOTOR":
                reply = {"test": "ESP32 已收到指令。",
                         "demo": "ESP32 已开始演示（慢->快->慢->定住->反转）。",
                         "stop": "ESP32 已停止电机。"}[args.action]
                print(reply)
                return
        raise SystemExit("ESP32 未回复；确认 USB 已连接且没有打开串口监视器。")
    finally:
        port.close()


if __name__ == "__main__":
    main()
