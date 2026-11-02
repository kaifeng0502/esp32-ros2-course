# 04 · Wi-Fi 显示与独立供电

[课程首页](../../README.md) · [上一节：USB 实时显示：时间、剩余百分比与像素动画](../03-usb-display/README.md) · [下一节：电机驱动、万用表与开环PWM](../05-motor-pwm/README.md)

## 本节目标

将画面传输从USB换成局域网UDP，同时理解无线通信与供电是两回事。

## 供电与准备

配置时USB接电脑。成功后可把USB接合规5V充电器或充电宝；电机10V绝不能直接接ESP32的3V3或USB供电引脚。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [host/setup_wifi.py](../../host/setup_wifi.py)
- [host/wifi_transport.py](../../host/wifi_transport.py)
- [main/network.c](../../main/network.c)
- [host/oled_demo.py](../../host/oled_demo.py)

## 操作步骤

```bash
python3 lessons/04-wifi-display/code/run.py
# 在程序打印的本机网页中配置2.4GHz网络；配置完成后运行：
python3 host/oled_demo.py --transport wifi --remaining 65 --seconds 30
```

## 预期现象与原理

配置页只在127.0.0.1运行，密码经USB写入板端NVS；电脑的 .run/wifi.json 保存配对信息，不应提交。UDP画面带配对签名和确认。电脑和ESP32需可互通，Wi-Fi并不替代供电。

## 我们实际遇到的问题

只支持本板2.4GHz Wi-Fi。本节的无线功能传画面，不通过UDP远程控制电机。曾进入电机显示页时可先复位，或经USB发送 DISPLAY STATUS。访客网络隔离、广播过滤和IP变化会影响发现。

## 练习与验收

用独立USB电源给板供电，确认电脑断开USB后画面仍更新。结束电脑程序后观察离线等待画面，区分断网、断数据与断电。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：USB 实时显示：时间、剩余百分比与像素动画](../03-usb-display/README.md) · [下一节：电机驱动、万用表与开环PWM](../05-motor-pwm/README.md)
