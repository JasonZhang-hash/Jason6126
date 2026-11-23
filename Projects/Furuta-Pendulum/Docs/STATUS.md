# 当前状态与下一步

## 代码与旧说明的差异

本次以实际文件为准，原始算法保持原样，以下为读取源码及冒烟检查所得。

- `furuta_sim_baseline.py` 当前 MAX_TORQUE=0.15 N·m、MOTOR_TAU=0.025 s、SWING_TORQUE=0.30 N·m；旧记录使用约 0.074 N·m 与较慢执行器。当前仿真比真实电机估计更强、更快。
- `_swing_drive()` 保留 DRIVE/BRAKE/SETTLE 代码，但 controller() 直接返回 cmd，没有调用互锁。该函数引用的 SWING_RELEASE_SPEED、SWING_BRAKE_TORQUE、SWING_SETTLE_TORQUE 也未定义。不能仅把函数接上便宣称换向保护有效。
- `accel_to_torque()` 引用未定义 ENERGY_ACCEL_LIMIT；目前属于未使用的遗留路径。
- 当前顶部进入/退出窗口是 25°/40°，速度门槛为 10 rad/s，不等于旧实验约 ±6° 的可捕获区。
- K_WALL=D_WALL=0，alpha 每步包角；alpha_unwrapped 虽有记录却未用于全程限位。±195° 的包角硬限位不可作为实机行程保护。
- `nn_furuta.py` 导入 main，但 main 已变为 EnergySwingController GUI，没有所需 SwingAndBalanceController，原样导入失败。可继续研究 `nn_furuta_measured.py`，后者直接依赖 measured core。
- `headless_diag.py` 写死旧电脑绝对路径，并读取旧版接口；仅保留作历史调试记录。
- `furuta_swingup_stm32.c` 依赖 HAL/GPIO/定时器句柄，不是可独立构建的完整固件工程。

## 历史结果的适用范围

旧记录中的 ±6° Near Top 约十秒收敛、±10° 失败以及起摆无法进入捕获区，均来自当时的模型参数。本次不把这些历史结果作为当前文件的复现结果；当前复现输出见 VALIDATION.md。

## 建议顺序

1. 测真实横臂在已知电机命令下的角加速度，识别等效惯量和执行器响应。
2. 固定参数版本，统一 torque 模型和 PWM 速度模型，分别记录其适用条件。
3. 修复并单独验证换向互锁、未包角横臂行程限制、传感器异常停机。
4. 固定真实执行器上限后测 Near Top 捕获区域，再让起摆以低角速度进入该区域。
5. 先通过传感器与电机方向开环检查，再移植控制器；记录时间、角度、角速度、输出和保护事件。

本次整理未修改这些控制算法，避免把未验证的修复混入历史快照。
