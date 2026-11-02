# 10 · 开环、P/PI/PID与响应速度—稳定性取舍

[课程首页](../../README.md) · [上一节：指定圈数与角度停止](../09-position/README.md) · [下一节：Ubuntu/UTM、ROS 2 Jazzy与USB权限](../11-ros-ubuntu/README.md)

## 本节目标

在相同目标和输入限幅下比较四种控制，区分稳态精度、超调、响应和抗扰。

## 供电与准备

车轮悬空、10V稳定，先不加人为负载。扰动比较应使用可重复的负载装置，不用手卡住转动的轮胎。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [host/motor_lab.py](../../host/motor_lab.py)
- [main/motor.c](../../main/motor.c)
- [data/observed/README.md](../../data/observed/README.md)

## 操作步骤

```bash
python3 lessons/10-control-comparison/code/run.py --port "$COURSE_PORT" --compare --rpm 80 --seconds 10 --max-pwm 55 --encoder-sign -1 --output results/pid01
python3 host/motor_lab.py --plot results/pid01
```

## 预期现象与原理

得到每种算法CSV、comparison.png和指标。前馈估得准、负载恒定时，开环也可能表现很好；这不证明反馈没价值。P反应快但可能留偏差，积分能补偏差，D对测速噪声敏感。抗积分饱和与硬限幅不能混为一谈。

## 我们实际遇到的问题

控制器已有数学模型与优化目标仍需要实验，因为摩擦、死区、供电、测量滤波与参数误差都会影响实际系统。只看最终接近目标不够，还要比较IAE、超调和调节时间。不同运行的手动摩擦不能视为完全相同扰动。

原理补充：[对照项目理解](../../docs/control-principles.md)。

## 练习与验收

分别改变一个增益，保持其他条件不变。解释为什么PID不一定比PI好；说明“PWM=0”之后主机平滑RPM仍逐步降到0的原因。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：指定圈数与角度停止](../09-position/README.md) · [下一节：Ubuntu/UTM、ROS 2 Jazzy与USB权限](../11-ros-ubuntu/README.md)
