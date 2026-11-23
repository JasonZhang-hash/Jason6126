# 代码地图

| 文件 | 用途与接手提示 |
| --- | --- |
| Simulation/furuta_sim_baseline.py | 耦合非线性动力学与顶部五状态反馈；原始参数已偏离硬件估计，先读 STATUS |
| Simulation/furuta_sim.py | 较早的单级界面与模型实验；不要按名称当成最新可靠基线 |
| Simulation/furuta_measured_core.py | 带实测 PWM/转速曲线的模型、轨迹跟随、起摆与捕获实验；提供 --headless |
| Simulation/furuta_measured_core_initial.py | 实测 core 的早期快照 |
| Simulation/furuta_energy_swingup.py | 另一份独立测量模型/策略搜索入口；与 GUI 文件同名近似但实现不同 |
| Simulation/furuta_energy_swingup_gui.py | 使用 measured core 的能量起摆与 PD 捕获 GUI；提供 --headless --seconds |
| Simulation/main.py | 上一行的原文件名，同字节兼容副本；不是 nn_furuta 所期待的早期 core |
| Simulation/furuta_energy_pd_180.py | 独立能量起摆、15°进入/35°退出纯 PD、横臂限幅实验 |
| Simulation/nn_furuta_measured.py | TinyNN 与 CEM 参数搜索；神经网络不是大模型 API；依赖 measured core |
| Simulation/nn_furuta.py | 早期 TinyNN；main 模块接口已失配，原样不能运行 |
| Simulation/double_furuta.py | 双级模型、频率扫描及训练实验；几何与质量参数不等于当前单级原型 |
| Simulation/direct_drive_pendulum.py | 直驱单杆历史路线，可作基础控制对照 |
| Simulation/headless_diag.py | 旧版横臂漂移诊断脚本；含机器路径和过时接口 |
| STM32/diagnostic-project | main.c、启动汇编、链接脚本、Makefile；裸机诊断工程 |
| STM32/furuta_diag_stm32.c | 独立诊断固件源文件快照 |
| STM32/furuta_swingup_stm32.c | HAL 风格起摆草稿，需接入初始化和周期调度 |
| Experiments/CEM | best/refined/sweep 参数 JSON 与 history CSV；需对应模型与种子再复现 |

`History/furuta_measured_core_original.py` 与 `Simulation/furuta_measured_core_initial.py` 字节相同，保留原来源供比对；较新的 measured core 与它不同，不按修改时间擅自覆盖。
