# 代码地图与协议

## 共用实现

| 路径 | 职责 |
|---|---|
| `main/codex_oled.c` | 应用入口、SH1106、USB命令路由；文件名沿用实验早期命名 |
| `main/encoder.c` | 中断计数、方向、原子快照 |
| `main/motor.c` | PWM/方向、速度环、位置规划、定时保护、实验采样 |
| `main/predictive.c` | 普通MPC与预测转速约束QP |
| `main/motor_display.c` | 板上电机状态页 |
| `main/network.c` | 配网、NVS、签名UDP画面 |
| `host/` | 电脑端显示、标定、控制、辨识、比较 |
| `ros2/encoder_publisher.py` | 初期Topic/Service桥接，默认只读 |
| `ros2/motor_action_interfaces/` | 自定义消息/Action、服务器、客户端、参数、Launch、URDF、RViz |
| `lessons/*/code/` | 各节可执行入口或独立工程，参数传到以上共用实现 |

## 自定义串口协议

以下都是本项目设计的应用层命令，通过同一USB Serial/JTAG通道按行发送，不是ROS原生串口标准。

| 命令 | 返回/作用 | 启动电机？ |
|---|---|---|
| `ENC?` | `ENC c1_edges c2_edges c1_level c2_level`，无方向累计边沿 |
| `ENC2?` | `ENC2 boot_id uptime_us c1_signed c2_signed`，连续方向反馈 |
| `PI?` | PI3兼容状态 |
| `LAB?` | LAB1任务状态、目标/实测/PWM、位置和控制项 |
| `MPC?` | MPC1实验状态与模型回读 |
| `MPC2?` | 约束能力与配置回读 |
| `SAMPLES?` / `SAMPLES2?` | 取OBS1/OBS2板上采样 |
| `MOTOR STOP` | 关闭驱动输出 |
| `MOTOR PI ...` / `MOTOR LAB ...` | 显式速度/位置任务 | 是 |
| `MOTOR MODEL ...` | RAM中设置模型，重启后需重新上传 | 否 |
| `MOTOR CAP margin` | 设置预测余量 | 否 |
| `MOTOR EXP ID/PID/MPC/MPC_CAP ...` | 显式运行实验 | 是 |
| `DISPLAY STATUS` / `OLED1 ...` | 切换/传输显示画面 | 否 |

表中空白的“启动电机”均为否；停止驱动不保证轮子瞬间停稳。格式的解析与范围以生产C和对应测试为准，不能绕过主机验证随意拼接运动命令。

## ROS 2 阅读路线

1. 看`encoder_publisher.py`的create_publisher、create_subscription与create_service，认识消息接口。
2. 看`action/MoveWheel.action`三段：goal、result、feedback。
3. 看`motor_action_server.py`的goal/cancel/execute回调和`motor_action_core.py`状态机。
4. 看`config/motor.yaml`如何进入Launch、参数校验、MOTOR LAB命令与板端回读。
5. 看`wheel_joint_state.py`把连续ENC2计数换算为rad，再由robot_state_publisher结合URDF发布TF。

取消请求、串口STOP确认、驱动关闭确认与Action最终CANCELED是有顺序的。客户端退出与取消也是不同动作。

## 公开版相对原实验的调整

公开库是课程整理后的新仓库，不伪造各节发生时的Git提交。共用控制核心保留，安装路径/工作区名改为可配置；早期账号用量屏用可注入百分比数据源替代私人账户连接；像素图改为代码绘制，字体采用Pillow默认或用户明确指定的字体。新增两个独立入门工程与每节入口。

协议保留`DISPLAY CODEX`旧别名以便兼容，但公开教程使用`DISPLAY STATUS`。公开固件默认显示等待页，收到运动命令后切到电机页。参考实验的实际成绩与这些公开版适配调整的验证范围分别说明。
