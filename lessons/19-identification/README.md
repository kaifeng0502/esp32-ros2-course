# 19 · 真实电机的预测模型辨识与留出验证

[课程首页](../../README.md) · [上一节：JointState、URDF、TF与RViz虚拟轮子](../18-jointstate-rviz/README.md) · [下一节：板上MPC与PID：公平比较与权重实验](../20-mpc-comparison/README.md)

## 本节目标

用多档PWM实验拟合二阶ARX模型，区分训练拟合、一步预测与连续自由预测。

## 供电与准备

车轮悬空、供电10V且负载不变；这是显式18秒开环实验，结束后自动停机。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [host/motor_mpc.py](../../host/motor_mpc.py)
- [host/predictive_model.py](../../host/predictive_model.py)
- [main/motor.c](../../main/motor.c)
- [data/observed/model-summary.json](../../data/observed/model-summary.json)

## 操作步骤

```bash
python3 lessons/19-identification/code/run.py --port "$COURSE_PORT" --out results/id01
python3 host/motor_mpc.py fit results/id01/identification.csv
```

## 预期现象与原理

9档PWM各2秒。前12秒训练，后约6秒连续预测验证，不在每一步借用真实RPM纠正预测。输出model.json与验证图；不稳定、激励不足、采样丢失、验证不合格会被拒绝。真实参考实验自由预测RMSE约0.40RPM。

## 我们实际遇到的问题

模型是 y[k+1]=a1*y[k]+a2*y[k-1]+b1*u[k]+b2*u[k-1]+c；u用百分数50而不是0.5。采样中的held_pwm是造成当前转速的上一间隔输入，不能错用next_pwm。RMSE是均方根误差，不是最坏误差界。

## 练习与验收

把动态模型在稳态下化简，得到本参考实验 RPM≈2.256×PWM百分比−3.633。解释为什么它只能给最终稳态，而MPC需要动态模型来预测过渡过程。公开转录参数仅作阅读，实机必须自己采集并验证。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：JointState、URDF、TF与RViz虚拟轮子](../18-jointstate-rviz/README.md) · [下一节：板上MPC与PID：公平比较与权重实验](../20-mpc-comparison/README.md)
