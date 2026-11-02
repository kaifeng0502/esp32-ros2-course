# 11 · Ubuntu/UTM、ROS 2 Jazzy与USB权限

[课程首页](../../README.md) · [上一节：开环、P/PI/PID与响应速度—稳定性取舍](../10-control-comparison/README.md) · [下一节：Topic：编码器发布、目标订阅与实时曲线](../12-topics/README.md)

## 本节目标

在Ubuntu中确认ROS 2节点可通信，再把ESP32 USB交给虚拟机。

## 供电与准备

ROS入门和权限检查期间电机10V断开。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [lessons/11-ros-ubuntu/code/check.sh](../../lessons/11-ros-ubuntu/code/check.sh)
- [docs/setup.md](../../docs/setup.md)
- [docs/troubleshooting.md](../../docs/troubleshooting.md)

## 操作步骤

```bash
bash lessons/11-ros-ubuntu/code/check.sh
source /opt/ros/jazzy/setup.bash
# 两个独立终端分别运行：
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
# Ubuntu中第一次配置串口组：
sudo usermod -aG dialout "$USER"
newgrp dialout
```

## 预期现象与原理

listener持续收到Hello World。UTM顶部USB菜单选择Espressif设备后，lsusb能看到303a:1001，出现/dev/ttyACM0或ACM1。id -nG包含dialout，并且当前终端可读写该设备。

## 我们实际遇到的问题

安装了ROS不等于当前终端已source；每个终端都需要环境。usermod不会自动更新旧shell的组。No such file先检查USB转发；Permission denied检查组；Resource busy再查占用。串口占用与权限不足不能当成同一种错误。

## 练习与验收

用 ros2 node list、ros2 topic list 与 ros2 topic echo /chatter --once观察原生示例。理解 ros2 是ROS自带CLI，可扩展命令与自己的节点代码是不同层次。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：开环、P/PI/PID与响应速度—稳定性取舍](../10-control-comparison/README.md) · [下一节：Topic：编码器发布、目标订阅与实时曲线](../12-topics/README.md)
