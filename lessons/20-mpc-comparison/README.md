# 20 · 板上MPC与PID：公平比较与权重实验

[课程首页](../../README.md) · [上一节：真实电机的预测模型辨识与留出验证](../19-identification/README.md) · [下一节：约束MPC：限制预测转速、误差余量与代价](../21-constrained-mpc/README.md)

## 本节目标

将10步预测的小规模优化放到ESP32，在相同反馈、目标和PWM限幅下比较PID与MPC。

## 供电与准备

模型通过验证后，车轮悬空、10V与负载保持不变；每次10秒，默认交换顺序重复两轮。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [main/predictive.c](../../main/predictive.c)
- [host/motor_mpc.py](../../host/motor_mpc.py)
- [host/predictive_model.py](../../host/predictive_model.py)
- [data/observed/metrics.csv](../../data/observed/metrics.csv)
- [docs/results.md](../../docs/results.md)

## 操作步骤

```bash
python3 lessons/20-mpc-comparison/code/run.py --port "$COURSE_PORT" --model results/id01/model.json --rpm 80 --step-rpm 110 --out results/compare_w2
# 只改变控制变化惩罚，模型数据不变：
python3 host/motor_mpc.py fit results/id01/identification.csv --move-weight 8 --out results/id01/model_w8.json
python3 host/motor_mpc.py compare --port "$COURSE_PORT" --model results/id01/model_w8.json --rpm 80 --step-rpm 110 --out results/compare_w8
```

## 预期现象与原理

电脑负责拟合和画图；PID/MPC都在ESP32运行，避免把虚拟机/串口延迟混进控制比较。MPC每次优化未来10个PWM，只执行第一个再重算。参考实测权重2最大570µs、权重8最大1098µs；不能把耗时列0的PID解释为零计算开销。

## 我们实际遇到的问题

当前代价是转速误差平方加lambda×PWM变化平方。增大lambda表达动作更平缓，不是禁止超调。真实实验起步超调约3.65%降至1.70%，但IAE增加约40%；进入误差带的调节时间不等于整个加速过程更快。

原理补充：[对照项目理解](../../docs/control-principles.md)。

## 练习与验收

读实际控制模型而非仅稳态直线，找出预测矩阵、求解迭代、输入限幅、扰动估计和10ms预算。比较调节时间、IAE、超调与末段误差，不据两轮数据宣称MPC普遍优于PID。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：真实电机的预测模型辨识与留出验证](../19-identification/README.md) · [下一节：约束MPC：限制预测转速、误差余量与代价](../21-constrained-mpc/README.md)
