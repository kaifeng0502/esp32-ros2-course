# 06 · 编码器接线与共地排错

[课程首页](../../README.md) · [上一节：电机驱动、万用表与开环PWM](../05-motor-pwm/README.md) · [下一节：每圈计数校准与实时RPM](../07-calibration-rpm/README.md)

## 本节目标

同时读到C1/C2脉冲，用测量缩小供电、接地、端子、GPIO和软件之间的故障范围。

## 供电与准备

电机10V断开，只保留USB和编码器3.3V供电；用手慢转轮子。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [main/encoder.c](../../main/encoder.c)
- [main/encoder.h](../../main/encoder.h)
- [host/encoder_monitor.py](../../host/encoder_monitor.py)
- [docs/hardware.md](../../docs/hardware.md)

## 操作步骤

```bash
python3 lessons/06-encoder-debug/code/run.py --port "$COURSE_PORT" --seconds 30
```

## 预期现象与原理

实例接法：黑VCC→3V3、蓝GND→ESP32 GND、绿C1→GPIO11、黄C2→GPIO10。手转时两路计数都增加，停止后保持。读到瞬时电平1并不代表没有脉冲，事件计数才是关键。

## 我们实际遇到的问题

本次真实故障是编码器地线没有可靠连到ESP32地：蓝线对ESP32 GND曾测到约2.6V，最终修好后断电通断测试持续响，两路才正常。线色仅针对本样机，其他电机以丝印/资料为准。只给VCC和GND不能测速，只接信号没有共同参考也不行。

## 练习与验收

断电后分别检查端到端连续性；上电只测蓝线对ESP32 GND应接近0V，VCC对蓝线应接近3.3V。每次只改一处并记录结果，避免同时调换两根线丢失因果关系。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：电机驱动、万用表与开环PWM](../05-motor-pwm/README.md) · [下一节：每圈计数校准与实时RPM](../07-calibration-rpm/README.md)
