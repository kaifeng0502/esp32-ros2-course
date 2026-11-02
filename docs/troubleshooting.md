# 从真实故障整理的排查表

| 现象 | 优先检查 | 不要混淆 |
|---|---|---|
| Resource busy | 关闭VS Code监控、旧Python脚本、旧ROS桥接；Linux用`fuser -v /dev/ttyACM0`查看 | 端口被占用不等于驱动没装 |
| No such file or directory | 实际端口、USB数据线、UTM USB转发，重新`lsusb`和列串口 | `chmod`不能创建不存在的设备 |
| Permission denied | `id -nG`是否包含dialout；新shell/`newgrp dialout` | 不是一律杀进程 |
| stub_flasher_32s3.json缺失 | 使用本项目固件包的`--no-stub` ROM烧录 | 芯片已被识别，问题在主机工具资源 |
| 烧录后无应用输出 | 松BOOT、只按RST，确认正确USB口并重新交给UTM | 下载模式与正常运行模式不同 |
| OLED偏移/花屏 | SH1106与SSD1306、2列偏移、扫描方向 | 地址0x3C不能唯一识别型号 |
| 两路编码器始终0 | VCC、GND、插头、端子、实际GPIO；逐项测量 | 单次读到电平1不说明从未出现脉冲 |
| 一路有计数另一路没反应 | 逐路交换已知可靠通路，记录每次只改什么 | 不要同时改供电/信号/软件失去定位依据 |
| 蓝地线对ESP32 GND约2.6V | 断电确认端到端持续导通，再上电复测 | 接线端子上的VCC读数不证明电机端正确供电 |
| 两路计数明显不一致 | 松动、共地、接线、误接同组面包板孔 | 不能简单平均掩盖硬件问题 |
| DIRECTION_ERROR | 校准`--encoder-sign`和真实电机方向 | 正转/反转都应使用同一个编码器校正 |
| Node not found | 先启动节点、source环境、核对ROS_DOMAIN_ID | 参数命令本身不会启动服务器 |
| `--configured`不认识 | 确认新源码、重新构建、source对应install | 修改src不一定更新正在运行的旧副本 |
| 绘图窗口没有线 | 先`ros2 topic echo ... --once`确认数据，再检查字段与时间窗口 | 空数据不是转速0 |
| bag 0消息、起止时间奇怪 | 先确认话题有数据再录新目录，Ctrl+C正常结束 | 空bag的占位时间不是实际日期 |
| RViz轮子全黑 | 使用当前URDF，每个彩色部件独立固定子link | 有轮廓说明模型可能已加载 |
| RViz TF轴太大 | TF显示项里减小Marker Scale | 缩放标记不会改变物理模型尺寸 |
| 输出目录非空 | 换新实验目录保留旧数据 | 不需要删掉旧结果来重试 |
| MPC模型被拒绝 | 检查完整采样、激励、稳定性与留出验证 | 不应手工改validated绕过检查 |
| MPC_CAP不可行 | 保存错误与采样；检查当前速度、目标变化、模型 | 不能悄悄放宽约束当作已满足 |

## 处理串口的最短路线

先停止旧程序，再检查设备与权限；不要先执行全局杀进程或永久放宽所有设备权限。只关闭已经确认属于本实验的进程。Linux例：

```bash
lsusb
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
id -nG
fuser -v /dev/ttyACM0
```

设备名ACM0只是例子。macOS看`/dev/cu.usbmodem*`；UTM转接后macOS可能暂时看不到该设备，这是USB所有权变化。

## 网络下载不再依赖某个人的IP

原学习中曾用Mac临时HTTP服务器传包，Mac地址改变或服务停止后下载超时。公开课程以GitHub克隆/发布附件为主，不把聊天中的临时端口、Token或局域网IP写成安装依赖。
