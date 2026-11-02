# 12 · Topic：编码器发布、目标订阅与实时曲线

[课程首页](../../README.md) · [上一节：Ubuntu/UTM、ROS 2 Jazzy与USB权限](../11-ros-ubuntu/README.md) · [下一节：Service：请求停止并等待确认](../13-services/README.md)

## 本节目标

先只读发布RPM，再显式启用目标订阅；理解发布者、订阅者、消息类型和字段。

## 供电与准备

只读阶段10V断开，手转验证。启用控制后，车轮悬空再接10V。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [ros2/encoder_publisher.py](../../ros2/encoder_publisher.py)
- [lessons/12-topics/code/run.py](../../lessons/12-topics/code/run.py)
- [ros2/motor_action_interfaces/scripts/motor_plot.py](../../ros2/motor_action_interfaces/scripts/motor_plot.py)

## 操作步骤

```bash
source /opt/ros/jazzy/setup.bash
python3 lessons/12-topics/code/run.py --port /dev/ttyACM0
# 其他终端先source ROS，再观察：
ros2 topic echo /motor/rpm
ros2 run rqt_plot rqt_plot /motor/rpm/data
# 关闭只读桥接后，按需开启控制：
python3 lessons/12-topics/code/run.py --port /dev/ttyACM0 --enable-control --encoder-sign -1
# 车轮悬空、10V接通后在另一终端触发一次：
ros2 topic pub --once /motor/target_rpm std_msgs/msg/Float64 '{data: 60.0}'
```

## 预期现象与原理

默认桥接不启动电机。enable-control模式订阅目标，60表示一次限时5秒任务，0停止并重新使能；同一任务运行中重复正数会被忽略。/motor/rpm是话题，/motor/rpm/data是rqt_plot所选的数值字段。

## 我们实际遇到的问题

每个ROS终端保持相同ROS_DOMAIN_ID。图空时先echo确认发布者与数据，再检查字段和时间窗口。本节早期桥接由ENC?估算无方向RPM大小；后面的LAB反馈、JointState带符号角度有不同语义。

原理补充：[对照项目理解](../../docs/ros-concepts.md)。

## 练习与验收

解释CLI里的 std_msgs/msg/Float64 与 {data:60.0} 如何对应消息定义。画目标和实测两条线，分清状态流与一次控制命令。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：Ubuntu/UTM、ROS 2 Jazzy与USB权限](../11-ros-ubuntu/README.md) · [下一节：Service：请求停止并等待确认](../13-services/README.md)
