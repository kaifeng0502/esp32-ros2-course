# 07 · 每圈计数校准与实时RPM

[课程首页](../../README.md) · [上一节：编码器接线与共地排错](../06-encoder-debug/README.md) · [下一节：PI恒速、目标参数、正反转与OLED反馈](../08-pi-speed/README.md)

## 本节目标

用准确圈数确定计数口径，再把时间窗口内的计数增量换算为转速。

## 供电与准备

校准时电机10V断开，手动同方向转动；不要在校准中途重启板子或来回晃轮子。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [host/calibrate_encoder.py](../../host/calibrate_encoder.py)
- [host/encoder_monitor.py](../../host/encoder_monitor.py)
- [main/encoder.h](../../main/encoder.h)

## 操作步骤

```bash
python3 lessons/07-calibration-rpm/code/run.py --port "$COURSE_PORT" --turns 5 --out results/calibration.json
python3 host/encoder_monitor.py --port "$COURSE_PORT" --counts-per-rev 1400 --seconds 30
```

## 预期现象与原理

以轮胎标记对齐为起点，准确转5圈再停在同一标记。每路 Δcount/5 得到每路每圈边沿数。本电机采用每路约1400；RPM=Δcount×60/(每圈计数×Δt秒)。两路平均后仍按每路口径换算，不能直接相加制造两倍转速。

## 我们实际遇到的问题

累计6879等单个数字不能直接证明转了几圈，必须有起点和准确转数。两路若差异很大先修反馈，不要平均掩盖故障。当前ENC?是无方向边沿数，逆转也增加。主机约1秒窗口会带来显示滞后。

原理补充：[对照项目理解](../../docs/control-principles.md)。

## 练习与验收

重复三次校准并比较离散程度。换不同减速比电机后，需一致更新 main/encoder.h、相关主机估算和ROS校准参数，重新辨识模型；脚本不会自动把新数值写进固件。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：编码器接线与共地排错](../06-encoder-debug/README.md) · [下一节：PI恒速、目标参数、正反转与OLED反馈](../08-pi-speed/README.md)
