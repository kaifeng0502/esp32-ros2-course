# 09 · 指定圈数与角度停止

[课程首页](../../README.md) · [上一节：PI恒速、目标参数、正反转与OLED反馈](../08-pi-speed/README.md) · [下一节：开环、P/PI/PID与响应速度—稳定性取舍](../10-control-comparison/README.md)

## 本节目标

在速度环外加入累计位置、减速规划与终点制动，理解相对位置和真实机械位置的区别。

## 供电与准备

车轮悬空、10V稳定；先慢速、小角度测试。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [host/motor_lab.py](../../host/motor_lab.py)
- [main/motor.c](../../main/motor.c)
- [main/encoder.c](../../main/encoder.c)

## 操作步骤

```bash
python3 lessons/09-position/code/run.py --port "$COURSE_PORT" --turns 3 --rpm 60 --seconds 15 --encoder-sign -1 --output results/position01
python3 lessons/09-position/code/run.py --port "$COURSE_PORT" --degrees 90 --rpm 60 --seconds 10 --direction reverse --encoder-sign -1 --output results/position02
```

## 预期现象与原理

以每次实际启动位置为零点，接近终点时按剩余距离降低速度，再短路制动，等安静后关闭驱动。seconds是超时上限，不是要求必须转满该时间。真实ROS Action实验曾完成3圈，编码器估算终点约3.0086圈，误差约3.09°。

## 我们实际遇到的问题

DIRECTION_ERROR先核对计数方向。编码器在电机侧，齿隙、惯性和轮胎打滑不能靠它全部观察；编码器估算误差不是独立测得的轮胎位置误差。驱动关闭、轮子停稳和到位精度是不同概念。

原理补充：[对照项目理解](../../docs/control-principles.md)。

## 练习与验收

在轮胎贴标记，用肉眼或外部参考比较正反90°的实际停点。记录惯性、减速距离、齿隙对重复性的影响，不把计数分辨率直接当定位精度。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：PI恒速、目标参数、正反转与OLED反馈](../08-pi-speed/README.md) · [下一节：开环、P/PI/PID与响应速度—稳定性取舍](../10-control-comparison/README.md)
