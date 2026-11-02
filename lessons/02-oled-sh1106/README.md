# 02 · OLED：I²C 接线、SH1106 与文字位图

[课程首页](../../README.md) · [上一节：开发环境、USB 与 Hello World](../01-setup-usb/README.md) · [下一节：USB 实时显示：时间、剩余百分比与像素动画](../03-usb-display/README.md)

## 本节目标

点亮 1.3 英寸 SH1106 128×64 屏，理解供电、I²C 地址、显示控制器和字体栅格化。

## 供电与准备

先断开 USB 再接线；本节只用 USB 供电，不接电机10V。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [lessons/02-oled-sh1106/code/firmware/main/oled_test.c](../../lessons/02-oled-sh1106/code/firmware/main/oled_test.c)
- [lessons/02-oled-sh1106/code/make_bitmap.py](../../lessons/02-oled-sh1106/code/make_bitmap.py)
- [docs/hardware.md](../../docs/hardware.md)

## 操作步骤

```bash
python3 lessons/02-oled-sh1106/code/make_bitmap.py --text 'Hello OLED'
idf.py -C lessons/02-oled-sh1106/code/firmware set-target esp32s3
idf.py -C lessons/02-oled-sh1106/code/firmware build
idf.py -C lessons/02-oled-sh1106/code/firmware -p "$COURSE_PORT" flash monitor
```

## 预期现象与原理

VCC→3V3、GND→GND、SDA→GPIO8、SCL→GPIO9，屏幕出现 Hello OLED。代码探测0x3C/0x3D，SH1106写入有2列偏移。文字先在电脑上变成1024字节单色位图，屏幕并不直接理解中文或 emoji。

## 我们实际遇到的问题

I²C 地址不能可靠区分 SH1106 与 SSD1306。花屏、偏移和上下方向要检查驱动与扫描配置。母对母线适用于两端都有排针；裸线需要可靠的端子或焊接。面包板电源排可能中间断开。

## 练习与验收

修改正文和副标题。中文可用 --font 指向你自己安装、包含相应字形的字体，先查看 results/oled-message.png，再编译；不在库中分发系统字体。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：开发环境、USB 与 Hello World](../01-setup-usb/README.md) · [下一节：USB 实时显示：时间、剩余百分比与像素动画](../03-usb-display/README.md)
