# 16 · Launch：一条命令启动桥接与绘图

[课程首页](../../README.md) · [上一节：Parameter：目标、时长和控制参数如何传到硬件](../15-parameters/README.md) · [下一节：rosbag：录制、离线回放与分析](../17-rosbag/README.md)

## 本节目标

统一启动节点、加载配置和管理退出，让实验不依赖手工开许多程序。

## 供电与准备

Launch本身不自动发送goal；启动时可保持10V断开，准备运动后再接。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [ros2/motor_action_interfaces/launch/motor_lab.launch.py](../../ros2/motor_action_interfaces/launch/motor_lab.launch.py)
- [ros2/motor_action_interfaces/scripts/motor_plot.py](../../ros2/motor_action_interfaces/scripts/motor_plot.py)
- [ros2/motor_action_interfaces/config/motor.yaml](../../ros2/motor_action_interfaces/config/motor.yaml)

## 操作步骤

```bash
bash lessons/16-launch/code/run.sh
# 另一终端显式启动：
bash lessons/14-actions/code/client.sh --configured
# 如不需要绘图：
bash lessons/16-launch/code/run.sh plot:=false
```

## 预期现象与原理

出现服务器READY与实时曲线窗口；未发指令时目标和RPM在0附近是正常现象。当前使用项目自己的订阅绘图程序，能明确显示等待、有效和数据过期状态。桥接退出时Launch统一关闭子进程。

## 我们实际遇到的问题

串口不存在时桥接启动失败，绘图随后被关闭的KeyboardInterrupt通常是退出链的一部分。窗口有了却没线，先核对话题与发布频率；不把旧曲线当新鲜反馈。

原理补充：[对照项目理解](../../docs/ros-concepts.md)。

## 练习与验收

阅读Launch中的Node、parameters、LaunchArgument与OnProcessExit。用自定义config文件改变目标，不通过修改绘图器来改变硬件行为。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：Parameter：目标、时长和控制参数如何传到硬件](../15-parameters/README.md) · [下一节：rosbag：录制、离线回放与分析](../17-rosbag/README.md)
