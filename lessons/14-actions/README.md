# 14 · Action：圈数目标、进度、结果与取消

[课程首页](../../README.md) · [上一节：Service：请求停止并等待确认](../13-services/README.md) · [下一节：Parameter：目标、时长和控制参数如何传到硬件](../15-parameters/README.md)

## 本节目标

把转3圈这样的长任务做成Action，区分goal、feedback、result与cancel。

## 供电与准备

构建/启动服务器不转电机；发goal前车轮悬空、10V接通。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [ros2/motor_action_interfaces/action/MoveWheel.action](../../ros2/motor_action_interfaces/action/MoveWheel.action)
- [ros2/motor_action_interfaces/scripts/motor_action_server.py](../../ros2/motor_action_interfaces/scripts/motor_action_server.py)
- [ros2/motor_action_interfaces/scripts/motor_action_core.py](../../ros2/motor_action_interfaces/scripts/motor_action_core.py)
- [ros2/motor_action_interfaces/scripts/motor_action_client.py](../../ros2/motor_action_interfaces/scripts/motor_action_client.py)

## 操作步骤

```bash
bash scripts/ros_build.sh
bash lessons/14-actions/code/server.sh --encoder-sign -1
# 另一终端：
bash lessons/14-actions/code/client.sh --turns 3 --rpm 60 --timeout 15
# 再测试中途取消：
bash lessons/14-actions/code/client.sh --turns 10 --rpm 60 --timeout 20 --cancel-after 2
```

## 预期现象与原理

终端持续显示圈数、进度、RPM和PWM，完成后有结果。取消实验先确认板端驱动关闭，再终结为CANCELED；客户端提出取消请求不等于立刻完成取消。

## 我们实际遇到的问题

先source安装后的工作区，否则可能运行旧版接口或找不到包。只保留一个串口服务器。退出客户端本身不自动取消，使用 --cancel-all。最终圈数是编码器估计，非独立位置真值。

## 练习与验收

对照handle_goal、cancel和execute相关回调，找到 ActionEngine 的状态转换、STOP确认及result生成；解释drive_disabled=True与CANCELED的先后关系。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：Service：请求停止并等待确认](../13-services/README.md) · [下一节：Parameter：目标、时长和控制参数如何传到硬件](../15-parameters/README.md)
