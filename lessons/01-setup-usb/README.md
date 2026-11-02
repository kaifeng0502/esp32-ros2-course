# 01 · 开发环境、USB 与 Hello World

[课程首页](../../README.md) · [下一节：OLED：I²C 接线、SH1106 与文字位图](../02-oled-sh1106/README.md)

## 本节目标

从完全没有串口输出开始，完成 ESP-IDF 编译、烧录、复位和监控，理解 IDE 与命令行使用的是同一套工具链。

## 供电与准备

只接 ESP32 的 USB；本节不接电机电源。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [lessons/01-setup-usb/code/firmware/main/hello_world_main.c](../../lessons/01-setup-usb/code/firmware/main/hello_world_main.c)
- [idf.sh](../../idf.sh)
- [docs/setup.md](../../docs/setup.md)

## 操作步骤

```bash
# 先按 docs/setup.md 安装并激活 ESP-IDF 5.5.6
idf.py -C lessons/01-setup-usb/code/firmware set-target esp32s3
idf.py -C lessons/01-setup-usb/code/firmware build
idf.py -C lessons/01-setup-usb/code/firmware -p "$COURSE_PORT" flash monitor
```

## 预期现象与原理

出现 Hello world、芯片和 Flash 信息。示例倒计时后重启，这是示例代码的行为，不是不断崩溃。监控退出用 Ctrl+]。VS Code 中选择同一 ESP-IDF、芯片和串口后，Build/Flash/Monitor 对应这些步骤。

## 我们实际遇到的问题

Resource busy 表示先关闭占用该串口的监控或脚本。No such file 表示设备路径不存在，先检查连接，不要反复改权限。需要下载模式时按住 BOOT、点 RST、松 BOOT；烧录完成后松开 BOOT，只点 RST。Ubuntu 的 USB 权限和 UTM 转发在第11节详细处理。

## 练习与验收

把打印文字改为自己的问候并重新烧录。分别观察启动日志、应用日志和周期输出，找到对应源码。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [下一节：OLED：I²C 接线、SH1106 与文字位图](../02-oled-sh1106/README.md)
