# 17 · rosbag：录制、离线回放与分析

[课程首页](../../README.md) · [上一节：Launch：一条命令启动桥接与绘图](../16-launch/README.md) · [下一节：JointState、URDF、TF与RViz虚拟轮子](../18-jointstate-rviz/README.md)

## 本节目标

保存目标、实测、PWM和有效性，离线重现一次实验，理解消息时间戳与采样口径。

## 供电与准备

录包不启动运动。回放时关闭硬件桥接并使用独立ROS_DOMAIN_ID，不连接电机电源。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [ros2/analyze_motor_bag.py](../../ros2/analyze_motor_bag.py)
- [lessons/17-rosbag/code/record.sh](../../lessons/17-rosbag/code/record.sh)
- [lessons/17-rosbag/code/replay.sh](../../lessons/17-rosbag/code/replay.sh)

## 操作步骤

```bash
# 终端1：保留第16节Launch，确认 /motor/rpm 有数据
# 终端2录包：
bash lessons/17-rosbag/code/record.sh results/bags/rpm80_5s
# 终端3运行configured任务；完成后在录包终端Ctrl+C
ros2 bag info results/bags/rpm80_5s
# 独立通信域离线回放：
bash lessons/17-rosbag/code/replay.sh results/bags/rpm80_5s
```

## 预期现象与原理

bag info有非零消息数、合理时间范围和所需话题；记录中很长的静止段与较短运动段正常，录包时长不等于任务时长。分析工具可导出telemetry.csv和RPM/PWM图。

## 我们实际遇到的问题

曾出现0消息、2262年的奇怪起止时间，这是空记录的元数据占位，不代表真实实验发生在未来。先echo确认话题有数据，再录新目录，不覆盖旧数据。参数快照须在节点存在时保存。

## 练习与验收

在已source的Ubuntu终端运行本节 code/analyze.py --help，并按参数导出分析。说明接收时间戳与板上采样时间的差异，丢失数据时为什么不应连成一条平滑线。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：Launch：一条命令启动桥接与绘图](../16-launch/README.md) · [下一节：JointState、URDF、TF与RViz虚拟轮子](../18-jointstate-rviz/README.md)
