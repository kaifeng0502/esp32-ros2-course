# 03 · USB 实时显示：时间、剩余百分比与像素动画

[课程首页](../../README.md) · [上一节：OLED：I²C 接线、SH1106 与文字位图](../02-oled-sh1106/README.md) · [下一节：Wi-Fi 显示与独立供电](../04-wifi-display/README.md)

## 本节目标

让电脑生成画面，ESP32只接收并显示；分清数据获取、排版、传输与硬件刷新。

## 供电与准备

USB供电，电机10V断开。完成本节前先切换到仓库根目录的共用固件。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [host/oled_demo.py](../../host/oled_demo.py)
- [host/display.py](../../host/display.py)
- [main/codex_oled.c](../../main/codex_oled.c)

## 操作步骤

```bash
cp config/sdkconfig.verified sdkconfig
./idf.sh build
./idf.sh -p "$COURSE_PORT" flash
python3 lessons/03-usb-display/code/run.py --port "$COURSE_PORT" --remaining 73 --seconds 15
```

## 预期现象与原理

屏幕显示电脑时间、73%剩余量和像素动画。OLED1序号、CRC32和ACK把一帧的边界与完整性表达清楚；Ctrl+C结束后释放串口。余量口径是100减已用百分比，未知值不能假装成0。

## 我们实际遇到的问题

同一个串口只由一个程序持有。该功能最初以 Codex 周额度为例；公开课程用 --remaining 或 --snapshot JSON 数据源重现，不需要账户、登录凭据或专有素材。像素猫由代码绘制，动画不代表后台任务状态。

## 练习与验收

创建 results/status.json，内容为 {"remaining_percent":60}，通过 --snapshot 读取，再修改文件观察显示。短文本使用 --text；读懂1024字节的页式位图布局。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：OLED：I²C 接线、SH1106 与文字位图](../02-oled-sh1106/README.md) · [下一节：Wi-Fi 显示与独立供电](../04-wifi-display/README.md)
