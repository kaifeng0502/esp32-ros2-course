# 对照本项目理解ROS 2

对应第11–18节。ROS不是让电机转动的另一种PID算法，而是把不同程序之间的数据、任务和配置连接起来的工具。

## 节点、话题与命令行

节点是ROS参与通信的程序对象。一个节点可以发布多个话题，也能订阅、提供Service和Action；一个`.py`文件不等于只能有一个话题。

`ros2 topic pub`是ROS 2提供的命令行工具，会临时创建发布者。我们决定话题名、类型及节点如何处理数据，不需要修改CLI自身：

```bash
ros2 topic pub --once /motor/target_rpm std_msgs/msg/Float64 '{data: 60.0}'
```

`/motor/target_rpm`是名，`std_msgs/msg/Float64`是消息类型，最后是符合消息字段的YAML内容。可用`ros2 interface show std_msgs/msg/Float64`查看字段。只有对应订阅者在线且明确启用控制时，消息才会变成串口任务。

第12节早期桥接在`ros2/encoder_publisher.py`创建这些接口；第14节起Action服务器是另一套入口。两者不能同时占用一个串口。

## 为什么不全用Service？

| 接口 | 本项目例子 | 更适合的语义 |
|---|---|---|
| Topic | RPM、PWM、连续编码器反馈 | 持续数据流，允许多个订阅者同时接收 |
| Service | `/motor/stop` | 短请求与响应，获得此次调用结果 |
| Action | `/motor/move_wheel` | 持续几秒的目标任务，有进度、结果、取消 |
| Parameter | `target_rpm`、`run_seconds`、增益 | 节点配置；修改配置本身通常不启动运动 |

每50ms用Service询问一次RPM会把连续状态人为做成轮询，增加调用管理，也让多个观察者重复请求。同样，把10秒运动放进一个长Service，会缺少标准的持续反馈与取消协议。选接口主要看数据和任务的时间语义，而不只是有没有返回值。

本例Service确认关闭驱动后返回；物理轮子仍可能滑行。响应字段不能被解释成电机立刻静止的独立传感证明。

## Action取消的先后顺序

1. 客户端请求取消；服务器的`cancel()`检查并接受请求。
2. `ActionEngine`发出`MOTOR STOP`，继续等待板端状态；客户端看到请求已接受还不代表停车完成。
3. 收到固件驱动关闭确认后，状态机才确认`drive_disabled`。
4. `execute()`调用`goal_handle.canceled()`，返回最终Result，客户端显示CANCELED。

如果确认缺失/超时，不能伪报已经安全取消。阅读`motor_action_server.py`的`goal()`、`cancel()`、`execute()`，再看`motor_action_core.py`的状态机与测试。客户端退出进程和明确发送取消也不是同一个操作。

## Parameter与Launch分别负责什么？

`motor.yaml`保存默认配置。Launch启动进程并把文件作为参数载入，节点检查类型和范围，接收任务时才把有效配置编码成`MOTOR LAB`命令。固件再验证并回报实际配置。

配置目标80RPM，和立刻转到80RPM是两个动作；本项目不会仅因为启动Launch或设置参数就自动运动。`--configured`客户端显式使用当前配置发起一次任务。

Launch还管理绘图、JointState等多个进程及其退出。它不创造“新控制算法”，也不能让不存在的USB设备自动出现。

## rosbag记录了什么？

它记录ROS消息和时间戳，不是把真实电机动作录成了可直接播放的硬件程序。`/motor/requested_rpm`是请求，`/motor/setpoint`是板端确认目标，`/motor/rpm`是测量反馈；三者分别记录才能发现通信、拒绝或超时问题。

空bag不是一次有效实验。先确认话题在线，录制后用`ros2 bag info`检查消息数、持续时间和类型。回放使用单独`ROS_DOMAIN_ID=77`，避免与控制节点混在一起；本课程只回放观察话题。

## JointState、URDF、TF与RViz

- `wheel_joint_state.py`把持续编码器计数换成关节角度rad和角速度，发布`/joint_states`。
- URDF描述`base_link`、轮子、关节位置和转轴，不负责读硬件。
- `robot_state_publisher`结合URDF与JointState计算TF坐标变换。
- RViz订阅并绘制模型；旋转鼠标视角只改变观察方式，不会控制实体轮子。

每圈2π rad，RPM换rad/s乘`2π/60`。TF姿态是周期旋转，可能显示成主值角；累计转了多少圈要看连续关节位置，不能只看单个四元数。直接启动RViz不会主动转动实物，需要显式发运动目标或手转。

## micro-ROS讨论到了哪里？

本课程使用自定义USB协议加Ubuntu桥接，未部署micro-ROS。micro-ROS可以把ROS消息接口带到ESP32，减少多板、多传感器各自维护格式的成本，但仍需Agent、资源与实时任务设计；它不会自动改善编码器接线、PI参数或电机动力学。一块板增加第二个电机并不必然需要它。
