# 08 · PI恒速、目标参数、正反转与OLED反馈

[课程首页](../../README.md) · [上一节：每圈计数校准与实时RPM](../07-calibration-rpm/README.md) · [下一节：指定圈数与角度停止](../09-position/README.md)

## 本节目标

先把120RPM固定实验扩展成可指定目标、时长和方向的闭环任务，观察误差如何改变PWM。

## 供电与准备

车轮悬空、共地可靠、反馈正常后接通10V；保持同一编码器方向校正。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [host/pi_control.py](../../host/pi_control.py)
- [main/motor.c](../../main/motor.c)
- [main/motor_display.c](../../main/motor_display.c)

## 操作步骤

```bash
python3 lessons/08-pi-speed/code/run.py --port "$COURSE_PORT" --rpm 120 --seconds 5 --encoder-sign -1
python3 lessons/08-pi-speed/code/run.py --port "$COURSE_PORT" --rpm 80 --seconds 5 --direction reverse --encoder-sign -1
python3 lessons/08-pi-speed/code/run.py --port "$COURSE_PORT" --status
```

## 预期现象与原理

程序先等轮子停稳，再启动，按时自动关闭驱动。OLED显示目标/实测RPM与PWM。电机任务约10ms检查状态、速度控制名义50ms更新；控制在ESP32上，不靠电脑每次发PWM。PI积分有抗饱和处理。

## 我们实际遇到的问题

正转显示负号时调整 encoder-sign，而不是盲目反接供电；两方向使用同一校正。PWM与RPM是输入/输出关系，积分项通过误差累计补偿负载或前馈不准。断USB时主机停车命令可能送不到，板上的运行时限仍独立执行。

## 练习与验收

找出 e、P、I 和前馈对应的代码与单位。比较启动过程和稳态；说明积分为什么能消除偏差，也可能在限幅时积累过多。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：每圈计数校准与实时RPM](../07-calibration-rpm/README.md) · [下一节：指定圈数与角度停止](../09-position/README.md)
