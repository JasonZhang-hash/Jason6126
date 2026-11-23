# 新设备运行

推荐 Python 3.12，桌面可视化需要带 Tk 的 Python。仿真主体仅使用标准库，历史 headless_diag 的数值分析额外使用 numpy。

在仓库根目录执行：

```powershell
cd Projects/Furuta-Pendulum
python Tools/smoke_check.py
python Simulation/furuta_measured_core.py --headless
python Simulation/furuta_energy_swingup_gui.py --headless --seconds 2
python Simulation/furuta_sim_baseline.py
```

最后一条打开 GUI；运行成功不表示控制已通过实机验收。使用 baseline 前核对 STATUS 中力矩、限位和未接通互锁的问题。

## STM32 诊断工程

需要 ARM GNU Embedded GCC、Make；烧录还需要 OpenOCD 与 ST-Link。
原 Makefile 的 CC、OBJCOPY 和 flash 命令硬编码了旧电脑 F:/Tools 路径。跨设备构建可覆盖编译器：

```powershell
cd STM32/diagnostic-project
make CC=arm-none-eabi-gcc OBJCOPY=arm-none-eabi-objcopy
```

必须先按本机安装位置调整 flash 目标。此处只描述构建入口，本次未烧录或连接硬件。独立的 furuta_swingup_stm32.c 需要补齐 HAL 工程、GPIO/定时器初始化及 1 kHz 调度，不能直接用诊断工程的 Makefile 编译。

## 机械

使用 OpenSCAD 打开 Mechanical 内 .scad，再导出 STL。early-rod 是早期外形估计，尺寸、公差及安装孔需要依据实际装配复核。arm 目录保留多个设计版本，没有按文件名选定最终制造版本。
