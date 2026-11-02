# 第三方依赖与参考

- `lessons/01-setup-usb/code/firmware/main/hello_world_main.c`来自Espressif ESP-IDF Hello World示例，文件保留Espressif版权声明与`SPDX-License-Identifier: CC0-1.0`，不由本库MIT许可覆盖其原声明。[上游示例](https://github.com/espressif/esp-idf/tree/v5.5.6/examples/get-started/hello_world)。
- ESP-IDF、FreeRTOS、ROS 2、Pillow、NumPy、Matplotlib、pyserial、SciPy等作为外部依赖使用，各自遵循上游许可。本库不内嵌它们的安装目录、私钥、固件Flash备份或系统字体。
- SH1106初始化与列偏移参考[Adafruit SH110x](https://github.com/adafruit/Adafruit_SH110x)，电机驱动状态表参考[Toshiba TB6612FNG](https://toshiba.semicon-storage.com/ap-en/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TB6612FNG.html)。这些链接用于说明硬件资料来源，不代表厂家背书。
- MPC的有限时域目标与约束可对照[OSQP官方MPC示例](https://osqp.org/docs/examples/mpc.html)。本库板上求解器为独立小规模C实现，不复制或移植OSQP求解器。
- 本公开版的像素角色由`host/display.py`几何绘制，不分发原学习中使用的专有应用精灵图或系统字体。用户输入中文时应自行选择有适当使用权限的字体。
