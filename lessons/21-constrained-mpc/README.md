# 21 · 约束MPC：限制预测转速、误差余量与代价

[课程首页](../../README.md) · [上一节：板上MPC与PID：公平比较与权重实验](../20-mpc-comparison/README.md)

## 本节目标

把“尽量不超调”的偏好改成明确的预测不越界约束，并用实测区分模型保证与真实结果。

## 供电与准备

车轮悬空、10V稳定；本版仅做升速。使用同一模型/权重比较普通MPC与MPC_CAP。

所有命令从仓库根目录运行。电脑工具须先安装 [依赖](../../docs/setup.md)。`COURSE_PORT`是你当前设备路径，例：Ubuntu `/dev/ttyACM0`，macOS `/dev/cu.usbmodemXXXX`。换USB口后重新检查。

## 本节代码

[本节独立示例或运行入口](code/)。后续入口调用共用固件/库；实现只有一份，避免复制出多个行为不同的控制版本。

- [main/predictive.c](../../main/predictive.c)
- [main/predictive.h](../../main/predictive.h)
- [host/motor_mpc.py](../../host/motor_mpc.py)
- [tests/test_predictive_constraints.py](../../tests/test_predictive_constraints.py)
- [docs/results.md](../../docs/results.md)

## 操作步骤

```bash
python3 lessons/21-constrained-mpc/code/run.py --port "$COURSE_PORT" --model results/id01/model.json --margin-rpm 1 --rpm 80 --step-rpm 110 --out results/compare_cap1
# 若另做0.5RPM余量实验，必须换新输出目录：
python3 lessons/21-constrained-mpc/code/run.py --port "$COURSE_PORT" --model results/id01/model.json --margin-rpm 0.5 --rpm 80 --step-rpm 110 --out results/compare_cap05
```

## 预期现象与原理

增加全部10步预测 RPM≤目标−余量，默认余量1RPM。参考两轮实测对80/110RPM均未观察到滤波反馈超调，代价是稳定在约79/109RPM，末段MAE约1.05RPM；最大求解2716µs，约占名义50ms周期5.4%。

## 我们实际遇到的问题

constraint_check.png分别显示预测违反量与真实反馈越界量。预测满足不代表所有负载下的连续物理转速都不超调，1RPM不是已证明的最坏误差界。当前主动集求解器对不支持模型、不可行、数值失败或10ms超时停驱，不偷偷放宽约束。降速可能因惯性不可行，不能保证降PWM能立即刹住轮子。

原理补充：[对照项目理解](../../docs/control-principles.md)。

## 练习与验收

解释预测峰值为何贴着79/109上限，以及实测109.1与预测109为何不是同一个量。记录减小余量后是否重新出现超调；如果要求既精确到目标又严格不越界，需要进一步建模不确定性，而不是只把权重调得更大。

完成标准：能解释本节输入、处理过程与输出，并能按预期复现实验或清楚记录失败原因。示例中的参考成绩不是你换硬件后的保证值。

[课程首页](../../README.md) · [上一节：板上MPC与PID：公平比较与权重实验](../20-mpc-comparison/README.md)
