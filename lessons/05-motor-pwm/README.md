# 05 · 电机驱动、万用表与开环PWM

[课程首页](../../README.md) · [上一节：Wi-Fi 显示与独立供电](../04-wifi-display/README.md) · [下一节：编码器接线与共地排错](../06-encoder-debug/README.md)

## 本节目标

接好TB6612FNG与一个N20电机，完成第一次有限时长的开环转动；理解GPIO不能直接给电机供电。

## 供电与准备

接线时USB与电机电源都断开。按硬件表共地，测量适配器输出并调到本实验的10V，再让车轮悬空测试。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [main/motor.c](../../main/motor.c)
- [host/motor_control.py](../../host/motor_control.py)
- [docs/hardware.md](../../docs/hardware.md)
- [docs/troubleshooting.md](../../docs/troubleshooting.md)

## 操作步骤

```bash
python3 lessons/05-motor-pwm/code/run.py stop --port "$COURSE_PORT"
# 完成低压接线与测量，车轮悬空，接通10V后显式运行：
python3 lessons/05-motor-pwm/code/run.py test --port "$COURSE_PORT"
```

## 预期现象与原理

MOTOR TEST按当前固件固定80% PWM运行约2秒后停止。PWMA=GPIO4、AIN1=5、AIN2=6、STBY=7；VCC=3.3V、VM=电机电源。PWM是控制输入，RPM是测量输出，二者并非始终固定比例。

## 我们实际遇到的问题

万用表黑笔COM、红笔V/Ω，直流电压档测适配器低压输出；不要把电流档并接电源。裸线通过合适端子接杜邦线，不能靠松动接触。接电后不应自行转动。不要测市电插孔或拆适配器。

## 练习与验收

定位 motor_apply 和 motor_stop。解释STBY拉低、PWM清零、方向引脚变化各自的作用；观察驱动关闭后轮子仍可能惯性滑行。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：Wi-Fi 显示与独立供电](../04-wifi-display/README.md) · [下一节：编码器接线与共地排错](../06-encoder-debug/README.md)
