"""Run the firmware's bounded configurable PI test, or inspect/stop it."""

import argparse
import time

import serial

from encoder_monitor import choose_port


def read_status(port):
    port.write(b"PI?\n")
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        parts = port.readline().decode(errors="replace").strip().split()
        if len(parts) != 10 or parts[0] != "PI3":
            continue
        try:
            return dict(active=int(parts[1]), target=float(parts[2]),
                        c1=float(parts[3]), c2=float(parts[4]), pwm=float(parts[5]),
                        elapsed=float(parts[6]), reason=parts[7], seconds=int(parts[8]), encoder_sign=int(parts[9]))
        except ValueError:
            continue
    raise RuntimeError("未收到 PI 状态。请先烧录新版固件，并关闭其他串口监视器。")


def show(status):
    print(f"{status['elapsed']:5.2f}s  目标={status['target']:.0f} RPM"
          f"  C1={status['c1']:6.1f} RPM  C2={status['c2']:6.1f} RPM"
          f"  PWM={status['pwm']:5.1f}%  {status['reason']}", flush=True)


def run_test(port, rpm=120, seconds=15, direction="forward", encoder_sign=1):
    status = read_status(port)
    if status['active']:
        raise RuntimeError("已有 PI 测试运行中；可先用 --stop 停止。")
    signed_target = rpm if direction == "forward" else -rpm
    direction_token = "FWD" if direction == "forward" else "REV"
    direction_name = "正转" if direction == "forward" else "反转"
    print(f"即将启动{direction_name} {rpm} RPM、{seconds} 秒 PI 测试。车轮须悬空，10V 电源应已接通。", flush=True)
    for remaining in (3, 2, 1):
        print(f"{remaining}…", flush=True)
        time.sleep(1)
    try:
        port.write(f"MOTOR PI {rpm} {seconds} {direction_token} {encoder_sign}\n".encode())
        deadline = time.monotonic() + seconds + 6
        while time.monotonic() < deadline:
            status = read_status(port)
            show(status)
            if status['target'] != signed_target or status['seconds'] != seconds or status['encoder_sign'] != encoder_sign:
                raise RuntimeError("开发板返回的目标或时长不匹配，停止测试。")
            if not status['active']:
                if status['reason'] == "DONE":
                    print(f"{seconds} 秒测试结束，驱动输出已关闭；车轮可能短暂滑行。", flush=True)
                    return
                raise RuntimeError(f"测试提前停止：{status['reason']}。NO_FEEDBACK 请检查电源与编码器；NOT_STOPPED 表示等待停转超时。")
            time.sleep(0.2)
        raise RuntimeError("等待测试结束超时，发送停止指令。")
    finally:
        # Firmware also enforces its own requested deadline when USB disappears.
        try:
            port.write(b"MOTOR STOP\n")
        except serial.SerialException:
            print("停止指令未送达；固件自身的定时停机仍生效。", flush=True)


def main():
    parser = argparse.ArgumentParser(description="正反转 PI 恒速控制，等待停转后启动，到时自动停止")
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument("--status", action="store_true", help="只查看状态，不启动电机")
    actions.add_argument("--stop", action="store_true", help="停止电机")
    parser.add_argument("--rpm", type=int, default=120, help="目标转速 40～180 RPM，默认 120")
    parser.add_argument("--seconds", type=int, default=15, help="运行时间 1～60 秒，默认 15")
    parser.add_argument("--direction", choices=("forward", "reverse"), default="forward",
                        help="forward 正转，reverse 反转；默认正转")
    parser.add_argument("--encoder-sign", type=int, choices=(-1, 1), default=1,
                        help="编码器正负号校正，若正转显示负数则选 -1；两方向保持同一设置")
    parser.add_argument("--port", help="串口路径；默认自动寻找一块 ESP32-S3")
    args = parser.parse_args()
    if not 40 <= args.rpm <= 180:
        parser.error("--rpm 必须是 40～180 的整数")
    if not 1 <= args.seconds <= 60:
        parser.error("--seconds 必须是 1～60 的整数")
    port = serial.Serial(port=None, baudrate=115200, timeout=0.2,
                         write_timeout=2, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = choose_port(args.port)
    try:
        port.open()
        port.reset_input_buffer()
        if args.stop:
            port.write(b"MOTOR STOP\n")
            status = read_status(port)
            show(status)
        elif args.status:
            show(read_status(port))
        else:
            run_test(port, args.rpm, args.seconds, args.direction, args.encoder_sign)
    except KeyboardInterrupt:
        print("\n已中断测试。")
    except (RuntimeError, serial.SerialException) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        port.close()


if __name__ == "__main__":
    main()
