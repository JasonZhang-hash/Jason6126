# 本次验证

检查日期：2026-11-23。

```text
Syntax passed: 13 Python sources
furuta_measured_core.py exit= 0 mode=SWING_UP capture=None phi=-140.16 deg theta_dot=-9.66 rad/s alpha=4.81 deg max_alpha=60.24 deg hard_limit=False 
furuta_energy_swingup_gui.py exit= 0 mode=SWING_UP phi=27.09deg theta_dot=-4.06rad/s alpha=80.8deg max_pwm=40% hard_limit=False
transitions=[] 
Smoke checks passed; this is not proof of successful swing-up or hardware stability.

```

退出码：0。只验证语法和所列无界面入口，不表示所有历史脚本可运行，也不表示起摆或硬件闭环成功。复制源码已按FILE_MANIFEST中的SHA256核对。未执行长时间搜索、GUI和固件构建/烧录。
