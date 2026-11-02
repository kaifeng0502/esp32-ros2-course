# 15 · Parameter：目标、时长和控制参数如何传到硬件

[课程首页](../../README.md) · [上一节：Action：圈数目标、进度、结果与取消](../14-actions/README.md) · [下一节：Launch：一条命令启动桥接与绘图](../16-launch/README.md)

## 本节目标

读取、修改、校验并保存ROS参数，理解配置与触发运动的区别。

## 供电与准备

只改参数不启动电机；configured客户端会启动任务，执行前车轮悬空、10V接通。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [ros2/motor_action_interfaces/config/motor.yaml](../../ros2/motor_action_interfaces/config/motor.yaml)
- [ros2/motor_action_interfaces/scripts/motor_action_core.py](../../ros2/motor_action_interfaces/scripts/motor_action_core.py)
- [ros2/motor_action_interfaces/scripts/motor_action_client.py](../../ros2/motor_action_interfaces/scripts/motor_action_client.py)

## 操作步骤

```bash
# 保持Action服务器运行
bash lessons/15-parameters/code/set_defaults.sh
bash lessons/14-actions/code/client.sh --configured
```

## 预期现象与原理

设置 target_rpm=80、run_seconds=5、target_turns=0表示一次正向定时测试。--configured读取服务器当前参数，再构造goal；服务端把控制参数转换为MOTOR LAB命令并核对回读。忙碌时拒绝相关参数修改。

## 我们实际遇到的问题

Node not found先确认服务器确实在运行，环境与通信域相同。旧客户端没有 --configured 时需要重新构建并source正确安装目录。参数文件在src修改后要重新构建，不能只改一个没被加载的副本。

## 练习与验收

用 ros2 param list/get/describe/dump 查看参数。跟踪Kp从YAML到Python校验、串口字段再到C结构的路径。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：Action：圈数目标、进度、结果与取消](../14-actions/README.md) · [下一节：Launch：一条命令启动桥接与绘图](../16-launch/README.md)
