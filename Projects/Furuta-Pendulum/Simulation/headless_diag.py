"""headless_diag.py — 跟踪 near top 模式下臂位置漂移"""
import math, sys
sys.path.insert(0, r'C:\Users\44162\Documents\Codex\2026-07-10\selina-chen-3\work\direct-drive-pendulum-sim')

# 用力导入仿真参数和方法
exec(open(r'C:\Users\44162\Documents\Codex\2026-07-10\selina-chen-3\work\direct-drive-pendulum-sim\furuta_sim.py', encoding='utf-8').read().split('class App')[0])

PI = math.pi
sim = FurutaPendulum()
sim.set_near_top()

# 记录 alpha 位置
log_alpha = []
log_theta = []
log_torque = []
log_t = []

for i in range(5000):  # 5 秒
    sim.step()
    if i % 10 == 0:
        log_alpha.append(sim.alpha)
        log_theta.append(sim.theta)
        log_torque.append(sim.motor_torque)
        log_t.append(sim.time)

# 分析
import numpy as np
alphas = np.array(log_alpha)
thetas = np.array(log_theta)
torques = np.array(log_torque)

print(f"DT={sim.DT:.4f}  MOTOR_TAU={MOTOR_TAU:.3f}  BAL_P={BALANCE_P:.2f}  BAL_D={BALANCE_D:.2f}")
print(f"ARM_DAMPING={ARM_DAMPING:.4f}")
print(f"ARM_CENTER_KP={ARM_CENTER_KP}, KD={ARM_CENTER_KD}")
print()
print(f"alpha  range: {np.degrees(alphas.min()):.2f}° ~ {np.degrees(alphas.max()):.2f}°")
print(f"alpha  drift: {np.degrees(alphas[-1] - alphas[0]):.2f}° over {sim.time:.2f}s")
print(f"alpha  slope: {np.degrees((alphas[-1]-alphas[0])/sim.time):.2f}°/s")
print(f"theta  range: {np.degrees(thetas.min()):.2f}° ~ {np.degrees(thetas.max()):.2f}° (upright=180°)")
print(f"torque mean:  {torques.mean():.4f} N·m  max: {torques.max():.4f}")
print(f"mode: {sim.mode}  centering: {sim.centering_active}")

# 检查 alpha_dot 在最后 1 秒是否仍在增
alpha_dot_last = (alphas[-1] - alphas[-100]) / (log_t[-1] - log_t[-100])
print(f"alpha_dot (last 1s): {np.degrees(alpha_dot_last):.2f}°/s")
