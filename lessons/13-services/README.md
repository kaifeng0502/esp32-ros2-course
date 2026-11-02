# 13 · Service：请求停止并等待确认

[课程首页](../../README.md) · [上一节：Topic：编码器发布、目标订阅与实时曲线](../12-topics/README.md) · [下一节：Action：圈数目标、进度、结果与取消](../14-actions/README.md)

## 本节目标

将一次请求/响应与连续Topic区分开，理解收到响应不等于机械瞬间静止。

## 供电与准备

停止服务本身不启动运动；如先启动演示任务，仍需车轮悬空并遵循10V步骤。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [ros2/encoder_publisher.py](../../ros2/encoder_publisher.py)
- [lessons/13-services/code/run.py](../../lessons/13-services/code/run.py)

## 操作步骤

```bash
# 终端1保留上一节enable-control桥接
source /opt/ros/jazzy/setup.bash
ros2 service list
ros2 service type /motor/stop
ros2 service call /motor/stop std_srvs/srv/Trigger '{}'
# 或运行本节Python服务客户端：
python3 lessons/13-services/code/run.py
```

## 预期现象与原理

Trigger请求没有业务字段，响应包含success和message。桥接在等待串口确认和新鲜板上状态后，报告是否确认驱动关闭。服务客户端有等待上限，不会无限挂起。

## 我们实际遇到的问题

不能把所有事情都换成Service：高频传感器数据适合Topic，长任务需要进度和取消时用Action。success描述服务定义的完成条件；轮子可能仍有惯性。

原理补充：[对照项目理解](../../docs/ros-concepts.md)。

## 练习与验收

读 on_stop 回调，找到请求、板上确认与响应的顺序。分别说明Topic、Service、Action在电机系统中的合适场景。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：Topic：编码器发布、目标订阅与实时曲线](../12-topics/README.md) · [下一节：Action：圈数目标、进度、结果与取消](../14-actions/README.md)
