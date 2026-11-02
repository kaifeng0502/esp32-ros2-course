"""Show live C1/C2 edge counts, optionally during one short motor test."""

import argparse
from collections import deque
import math
import time

import serial
from serial.tools import list_ports


class RpmEstimator:
    """Estimate unsigned wheel RPM over roughly one second of host samples."""

    def __init__(self, counts_per_rev):
        self.counts_per_rev = counts_per_rev
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def update(self, timestamp, counts):
        if self.samples and (timestamp <= self.samples[-1][0]
                             or any(a < b for a, b in zip(counts, self.samples[-1][1]))):
            self.reset()
        self.samples.append((timestamp, counts))
        while len(self.samples) > 2 and self.samples[1][0] <= timestamp - 1.0:
            self.samples.popleft()
        start, initial = self.samples[0]
        elapsed = timestamp - start
        if elapsed <= 0:
            return None
        return tuple((value - baseline) * 60 / (self.counts_per_rev * elapsed)
                     for value, baseline in zip(counts, initial))


def choose_port(explicit=None):
    if explicit:
        return explicit
    ports = [p.device for p in list_ports.comports()
             if p.vid == 0x303A and p.pid == 0x1001]
    if len(ports) != 1:
        raise SystemExit("请通过 USB 连接一块 ESP32-S3。")
    return ports[0]


def main():
    parser = argparse.ArgumentParser(description="实时查看车轮转速 RPM 和编码器计数")
    parser.add_argument("--counts-per-rev", type=float, default=1400,
                        help="单路每圈计数（含上升沿和下降沿），默认 1400")
    parser.add_argument("--seconds", type=int, default=0,
                        help="运行秒数；0 表示一直显示，按 Ctrl+C 结束（默认）")
    parser.add_argument("--spin", action="store_true",
                        help="收到首个编码器读数后，让电机短转一次并继续显示计数")
    parser.add_argument("--port", help="串口路径；默认自动寻找一块 ESP32-S3")
    args = parser.parse_args()
    if not math.isfinite(args.counts_per_rev) or args.counts_per_rev <= 0:
        parser.error("--counts-per-rev 必须是大于 0 的有限数值")
    speed = RpmEstimator(args.counts_per_rev)
    if args.spin and args.seconds == 0:
        args.seconds = 8
    port = serial.Serial(port=None, baudrate=115200, timeout=0.2,
                         write_timeout=2, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = choose_port(args.port)
    try:
        port.open()
        port.reset_input_buffer()
        deadline = time.monotonic() + args.seconds if args.seconds > 0 else float("inf")
        if args.spin:
            print("等待编码器读数，然后让电机短转一次；车轮须悬空。", flush=True)
        else:
            print("等待开发板回应；出现读数后再手转车轮。按 Ctrl+C 结束。", flush=True)
        print("当前固件：C1=GPIO11，C2=GPIO10；标签跟随针脚，不跟随线的颜色。", flush=True)
        print(f"单路每圈 {args.counts_per_rev:g} 次计数；约每 0.25 秒刷新，RPM 按最近约 1 秒计算。"
              "转速为大小，不表示正反转。", flush=True)
        previous = None
        missed = 0
        spin_pending = args.spin
        while time.monotonic() < deadline:
            port.write(b"ENC?\n")
            reply_deadline = time.monotonic() + 1
            received = False
            while time.monotonic() < reply_deadline:
                line = port.readline().decode(errors="replace").strip()
                if not line.startswith("ENC "):
                    continue
                parts = line.split()
                if len(parts) == 5:
                    try:
                        counts = (int(parts[1]), int(parts[2]))
                    except ValueError:
                        continue
                    received = True
                    missed = 0
                    if previous is None:
                        print("已收到读数，现在可以手转车轮。累计数在开发板重启时归零。", flush=True)
                    delta = tuple(str(now - old) if now >= old else "重置"
                                  for now, old in zip(counts, previous)) if previous else ("—", "—")
                    rpm = speed.update(time.monotonic(), counts)
                    rpm_text = tuple(f"{value:.1f}" for value in rpm) if rpm is not None else ("—", "—")
                    print(f"{time.strftime('%H:%M:%S')}  C1(GPIO11): {rpm_text[0]} RPM 计数={counts[0]} 增量={delta[0]} 电平={parts[3]}"
                          f"  |  C2(GPIO10): {rpm_text[1]} RPM 计数={counts[1]} 增量={delta[1]} 电平={parts[4]}", flush=True)
                    previous = counts
                    if spin_pending:
                        port.write(b"MOTOR TEST\n")
                        print("已发送短转指令；请观察车轮与计数。", flush=True)
                        spin_pending = False
                    break
            if not received:
                speed.reset()
                previous = None
                missed += 1
                if missed == 1 or missed % 5 == 0:
                    print("未收到编码器回复：当前 RPM 不可用，请确认开发板正常运行；这不代表转速为零。", flush=True)
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\n已停止读取。")
    except serial.SerialException as exc:
        print(f"串口连接失败：{exc}\n请检查 USB，并关闭 VS Code 串口监视器等占用程序。", flush=True)
        raise SystemExit(1)
    finally:
        if args.spin and port.is_open:
            try:
                port.write(b"MOTOR STOP\n")
            except serial.SerialException:
                pass
        port.close()


if __name__ == "__main__":
    main()
