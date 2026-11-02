# 18 · JointState、URDF、TF与RViz虚拟轮子

[课程首页](../../README.md) · [上一节：rosbag：录制、离线回放与分析](../17-rosbag/README.md) · [下一节：真实电机的预测模型辨识与留出验证](../19-identification/README.md)

## 本节目标

用带方向累计编码器计数发布关节角度，通过模型生成坐标变换，让虚拟轮子跟随实物。

## 供电与准备

先保持10V断开，手转验证；启动可视化不自动转电机。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [ros2/motor_action_interfaces/scripts/wheel_joint_state.py](../../ros2/motor_action_interfaces/scripts/wheel_joint_state.py)
- [ros2/motor_action_interfaces/scripts/wheel_joint_core.py](../../ros2/motor_action_interfaces/scripts/wheel_joint_core.py)
- [ros2/motor_action_interfaces/urdf/single_wheel.urdf](../../ros2/motor_action_interfaces/urdf/single_wheel.urdf)
- [ros2/motor_action_interfaces/launch/wheel_rviz.launch.py](../../ros2/motor_action_interfaces/launch/wheel_rviz.launch.py)

## 操作步骤

```bash
bash lessons/18-jointstate-rviz/code/run.sh
# 另一个已source ROS和工作区的终端：
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo base_link wheel_link
# 准备让实物主动转动时，可另发已有Action goal
```

## 预期现象与原理

手转四分之一圈，橙色标记约转90°；反向转回，虚拟姿态恢复。JointState给角度，URDF给安装位置与转轴，robot_state_publisher算TF。位置单位rad、速度rad/s；effort没有传感器测量就留空。

## 我们实际遇到的问题

ENC2?区别于ENC?和任务内LAB位置：它有带方向累计计数、设备时间与boot_id，空闲和重启也能处理。TF的[0,0,0.055]是模型安装位置，当前旋转是另一部分。旧RViz多visual材质问题用固定子link解决；TF太大调整Marker Scale。

## 练习与验收

在RViz添加TF，拖动鼠标改变观察视角，区分视角变化与关节变化。转一整圈时外观回原位，但累计JointState角度仍增加约2π。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：rosbag：录制、离线回放与分析](../17-rosbag/README.md) · [下一节：真实电机的预测模型辨识与留出验证](../19-identification/README.md)
