"""furuta_sim.py — 一阶 Furuta 倒立摆仿真 (实测参数 + 能量起摆 + PD 平衡 + 目标追踪)"""

import math
import tkinter as tk

PI = math.pi
TWO_PI = 2.0 * PI


# ============================================================
#  >> 实测物理参数(来自硬件标定) <<
# ============================================================

ARM_LENGTH      = 0.160     # m    实测横臂约 16 cm
ARM_INERTIA     = 0.0030    # kg·m² 横臂+电机输出端等效惯量
ARM_DAMPING     = 0.0015    # N·m·s/rad

PEND_MASS       = 0.016     # kg    约 6g 摆杆 + 10g 配重
PEND_LENGTH     = 0.180     # m     机械总长约 18 cm
PEND_COM        = 0.151     # m     自由衰减测得的等效质心距
PEND_DAMPING    = 0.00030   # N·m·s/rad, zeta≈0.05

GRAVITY         = 9.80665
PEND_OMEGA_N    = math.sqrt(GRAVITY / PEND_COM)  # 自然频率

MAX_TORQUE      = 0.15   # N·m   simulation limit; real hardware ~0.074
MOTOR_TAU       = 0.025     # s     电机响应（仿直驱）
DT              = 0.001     # s

# ============================================================
#  >> 控制器参数 <<
# ============================================================

# 起摆：简单能量泵 + 静止踢
# 不对横臂位置/速度做回中控制；横臂只作为向摆杆注入能量的执行器。
SWING_TORQUE         = 0.30   # N·m, max pumping torque
STATIC_THRESHOLD     = 0.10   # rad/s, below this velocity the pendulum needs a kick start
ENERGY_BAND          = 0.20   # taper torque as energy approaches target

# 顶部平衡：full-state feedback
# x = [alpha error, alpha_dot, phi, theta_dot, motor_torque]
# theta=pi is upright.  The gain signs follow this file's alpha/theta convention.
# These values are tuned against the physical estimates below and the 0.074 N*m limit.
BALANCE_K_ALPHA      = -0.447  # N*m/rad
BALANCE_K_ALPHA_DOT  = -0.513  # N*m/(rad/s)
BALANCE_K_PHI        = 12.052  # N*m/rad
BALANCE_K_PHI_DOT    = 1.616   # N*m/(rad/s)
BALANCE_K_MOTOR_TAU  = 7.058   # dimensionless actuator-lag damping

# 切换条件
TOP_ENTER_DEG    = 25.0
TOP_EXIT_DEG     = 40.0
SPEED_ENTER      = 10.0
CATCH_ALPHA      = math.radians(360.0)
CATCH_ALPHA_DOT  = 10.0

TARGET_SLEW      = 0.5

# 臂角限位(仅平衡模式)
ALPHA_SOFT_DEG   = 180       # 到边界时只刹车，不主动回推
ALPHA_HARD_DEG   = 195       # 防穿透保护
K_WALL           = 0.0
D_WALL           = 0.0

STEPS_PER_FRAME  = 12

# ============================================================
#  工具函数
# ============================================================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def sign(x):
    return 1.0 if x >= 0.0 else -1.0

def wrap_pi(x):
    while x > PI:       x -= TWO_PI
    while x < -PI:      x += TWO_PI
    return x

def wrap_0_2pi(x):
    while x < 0:        x += TWO_PI
    while x >= TWO_PI:  x -= TWO_PI
    return x

def accel_to_torque(desired_accel):
    """加速度 → 力矩: tau = J_eff * alpha_ddot"""
    J_eff = ARM_INERTIA + PEND_MASS * ARM_LENGTH * ARM_LENGTH
    accel = clamp(desired_accel, -ENERGY_ACCEL_LIMIT, ENERGY_ACCEL_LIMIT)
    return J_eff * accel

# ============================================================
#  动力学
# ============================================================

class FurutaPendulum:
    def __init__(self):
        self.reset()

    def reset(self):
        self.alpha     = 0.0
        self.alpha_dot = 0.0
        self.theta     = math.radians(2.0)
        self.theta_dot = 0.0
        self.motor_torque = 0.0
        self.mode     = "SWING_UP"
        self.time     = 0.0
        self.alpha_target_user = 0.0
        self.alpha_target_ref  = 0.0
        self.last_desired_accel = 0.0
        self.last_energy_error = 0.0
        self.swing_direction = 1.0
        self.swing_drive_direction = 1.0
        self.swing_target_direction = 1.0
        self.swing_braking = False
        self.swing_action = "SEED"
        self.limit_hit = False
        self.balance_frames = 0
        self.centering_active = False
        self.alpha_unwrapped = 0.0       # 展开角(不包裹, 累计全转数)

    def set_near_top(self):
        self.theta     = PI + math.radians(6.0)
        self.theta_dot = 0.0
        self.alpha     = 0.0
        self.alpha_dot = 0.0
        self.motor_torque = 0.0
        self.mode     = "BALANCE"
        self.time     = 0.0
        self.alpha_target_user = 0.0
        self.alpha_target_ref  = 0.0

    def pendulum_energy(self):
        m, g, L = PEND_MASS, GRAVITY, PEND_COM
        # Only pendulum vertical energy belongs in the swing-up target.
        # Arm kinetic energy must not make the controller think it is near top.
        T = 0.5 * m * L*L * self.theta_dot**2
        U = m * g * L * (1.0 - math.cos(self.theta))
        return T + U

    def target_energy(self):
        return 2.0 * PEND_MASS * GRAVITY * PEND_COM

    def _swing_drive(self, desired_cmd):
        """Brake to rest before applying a newly requested drive direction."""
        if abs(desired_cmd) < 1e-9:
            self.swing_action = "COAST"
            return 0.0

        self.swing_target_direction = sign(desired_cmd)

        if self.swing_braking:
            if abs(self.alpha_dot) > SWING_RELEASE_SPEED:
                self.swing_action = "BRAKE"
                return -sign(self.alpha_dot) * SWING_BRAKE_TORQUE
            if abs(self.motor_torque) > SWING_SETTLE_TORQUE:
                self.swing_action = "SETTLE"
                return 0.0

            self.swing_braking = False
            self.swing_drive_direction = self.swing_target_direction

        if self.swing_target_direction != self.swing_drive_direction:
            if abs(self.alpha_dot) > SWING_RELEASE_SPEED:
                self.swing_braking = True
                self.swing_action = "BRAKE"
                return -sign(self.alpha_dot) * SWING_BRAKE_TORQUE
            if abs(self.motor_torque) > SWING_SETTLE_TORQUE:
                self.swing_braking = True
                self.swing_action = "SETTLE"
                return 0.0
            self.swing_drive_direction = self.swing_target_direction

        self.swing_action = "DRIVE"
        return desired_cmd

    # ---- 控制器(实测策略) ----

    def _avoid_180_slew(self, current, target):
        """目标角滑移: 绕远路(经过0, 避开±180°)"""
        if sign(current) == sign(target) or target == 0 or current == 0:
            # 同侧: 直走
            err = wrap_pi(target - current)
        else:
            # 异侧: 必须穿过0, 不能抄近路走180
            if current > 0 and target < 0:
                err = -(current - target)  # CW, 经过0
            else:
                err = target - current      # CCW, 经过0
        return current + clamp(err, -TARGET_SLEW * DT, TARGET_SLEW * DT)

    def controller(self):
        phi = wrap_pi(self.theta - PI)
        top_in  = math.radians(TOP_ENTER_DEG)
        top_out = math.radians(TOP_EXIT_DEG)

        # 目标滑移(避180)
        self.alpha_target_ref = self._avoid_180_slew(
            self.alpha_target_ref, self.alpha_target_user)

        # ---- 模式切换(带回滞) ----
        if self.mode == "SWING_UP":
            if (abs(phi) <= top_in and abs(self.theta_dot) <= SPEED_ENTER
                and abs(self.alpha) <= CATCH_ALPHA
                and abs(self.alpha_dot) <= CATCH_ALPHA_DOT):
                self.mode = "BALANCE"
        elif abs(phi) >= top_out:
            self.mode = "SWING_UP"

        # ---- 平衡: [alpha, alpha_dot, phi, theta_dot, motor torque] ----
        if self.mode == "BALANCE":
            alpha_err = wrap_pi(self.alpha - self.alpha_target_ref)

            # A Furuta pendulum has one motor but two angles.  Theta-only PD can
            # hold a small lean only by continuously accelerating the arm.  This
            # state feedback damps the arm from the first balance sample.
            cmd = -(
                BALANCE_K_ALPHA * alpha_err
                + BALANCE_K_ALPHA_DOT * self.alpha_dot
                + BALANCE_K_PHI * phi
                + BALANCE_K_PHI_DOT * self.theta_dot
                + BALANCE_K_MOTOR_TAU * self.motor_torque
            )
            self.last_desired_accel = cmd
            return self._alpha_wall(cmd)

        # ---- Swing-up: energy pump (sign method, full authority even at low speed) ----
        # Pendulum energy ignores arm dynamics; target is upright potential.
        E = self.pendulum_energy()
        E_err = self.target_energy() - E
        self.last_energy_error = E_err

        # ---- Swing-up: static kick + energy pump ----
        E = self.pendulum_energy()
        E_err = self.target_energy() - E
        self.last_energy_error = E_err

        if abs(self.theta_dot) < STATIC_THRESHOLD:
            cmd = -SWING_TORQUE * sign(math.sin(self.theta))
        else:
            taper = clamp(abs(E_err) / (ENERGY_BAND * self.target_energy()), 0.0, 1.0)
            cmd = -SWING_TORQUE * taper * sign(self.theta_dot * math.cos(self.theta)) * sign(E_err)

        self.last_desired_accel = cmd
        return cmd

    def _alpha_wall(self, base_cmd):
        if self.mode == "SWING_UP":
            return base_cmd

        soft = math.radians(ALPHA_SOFT_DEG)
        hard = math.radians(ALPHA_HARD_DEG)

        if abs(self.alpha) > soft:
            self.limit_hit = True
            penetration = min(abs(self.alpha) - soft, hard - soft)
            wall = K_WALL * penetration + D_WALL * abs(self.alpha_dot)
            wall_torque = wall * (-sign(self.alpha))
            if sign(base_cmd) == sign(self.alpha):
                return wall_torque
            return base_cmd + wall_torque
        return base_cmd

    # ---- RK4 积分 ----

    def step(self):
        cmd = clamp(self.controller(), -MAX_TORQUE, MAX_TORQUE)
        self.motor_torque += (cmd - self.motor_torque) * DT / MOTOR_TAU
        tau = clamp(self.motor_torque, -MAX_TORQUE, MAX_TORQUE)

        def rhs(a, ad, th, thd, tq):
            m, g, Lp, La, Ja = PEND_MASS, GRAVITY, PEND_COM, ARM_LENGTH, ARM_INERTIA
            c, s = math.cos(th), math.sin(th)
            M11 = Ja + m*La*La + m*Lp*Lp*s*s
            M12 = m*La*Lp*c
            M22 = m*Lp*Lp
            C1 = (tq - ARM_DAMPING*ad
                  - 2.0*m*Lp*Lp*s*c*ad*thd
                  + m*La*Lp*s*thd*thd)
            C2 = (m*Lp*Lp*s*c*ad*ad
                  - m*g*Lp*s
                  - PEND_DAMPING*thd)
            det = M11*M22 - M12*M12
            return ad, (C1*M22-M12*C2)/det, thd, (M11*C2-M12*C1)/det

        h = DT
        a, ad, th, thd = self.alpha, self.alpha_dot, self.theta, self.theta_dot
        k1 = rhs(a,         ad,         th,         thd,         tau)
        k2 = rhs(a+0.5*h*k1[0], ad+0.5*h*k1[1], th+0.5*h*k1[2], thd+0.5*h*k1[3], tau)
        k3 = rhs(a+0.5*h*k2[0], ad+0.5*h*k2[1], th+0.5*h*k2[2], thd+0.5*h*k2[3], tau)
        k4 = rhs(a+    h*k3[0], ad+    h*k3[1], th+    h*k3[2], thd+    h*k3[3], tau)

        self.alpha     += h/6*(k1[0]+2*k2[0]+2*k3[0]+k4[0])
        self.alpha_dot += h/6*(k1[1]+2*k2[1]+2*k3[1]+k4[1])
        self.theta     += h/6*(k1[2]+2*k2[2]+2*k3[2]+k4[2])
        self.theta_dot += h/6*(k1[3]+2*k2[3]+2*k3[3]+k4[3])

        self.alpha_unwrapped += h/6*(k1[0]+2*k2[0]+2*k3[0]+k4[0])  # 展开角(累计全转数)
        self.alpha = wrap_pi(self.alpha)
        self.theta = wrap_0_2pi(self.theta)
        self.time += h


# ============================================================
#  GUI
# ============================================================

class App:
    def __init__(self, root):
        self.sim = FurutaPendulum()
        self.root = root
        self.root.title("Furuta — Energy Swing-up + PD Balance")
        self.running = True

        self.canvas = tk.Canvas(root, width=880, height=620, bg="#fafaf5")
        self.canvas.pack(padx=10, pady=10)

        bar = tk.Frame(root)
        bar.pack(fill="x", padx=10, pady=(0,4))
        tk.Button(bar, text="Pause / Run",  command=self.toggle).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Reset bottom", command=self.sim.reset).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Near top",     command=self.sim.set_near_top).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Zero",         command=self.zero_target).pack(side=tk.LEFT, padx=3)
        tk.Label(bar, text="  Target α (°):").pack(side=tk.LEFT)
        self.tgt_var = tk.StringVar(value="0")
        self.tgt_entry = tk.Entry(bar, textvariable=self.tgt_var, width=6,
                                  font=("Consolas", 11), justify="center")
        self.tgt_entry.pack(side=tk.LEFT, padx=2)
        self.tgt_entry.bind("<Return>", self.set_target)
        tk.Button(bar, text="Go", command=self.set_target).pack(side=tk.LEFT, padx=3)

        self.info = tk.StringVar()
        tk.Label(root, textvariable=self.info, font=("Consolas", 11), justify="left").pack(
            anchor="w", padx=14, pady=6)
        self.root.after(16, self.update)

    def set_target(self, e=None):
        try: self.sim.alpha_target_user = math.radians(float(self.tgt_var.get()))
        except: pass

    def zero_target(self):
        self.tgt_var.set("0"); self.set_target()

    def toggle(self): self.running = not self.running

    def update(self):
        if self.running:
            for _ in range(STEPS_PER_FRAME):
                self.sim.step()
        self.draw()
        self.root.after(16, self.update)

    def draw(self):
        self.canvas.delete("all")
        w, h = 880, 620
        s = self.sim
        sc = 450.0

        # ===== 俯视图 =====
        cx, cy = w // 4, h // 2
        arm_px   = ARM_LENGTH * sc
        bob_rad = PEND_COM * math.sin(s.theta) * sc

        tip_x = cx + arm_px * math.cos(s.alpha)
        tip_y = cy - arm_px * math.sin(s.alpha)
        bob_x = tip_x + bob_rad * math.sin(s.alpha)
        bob_y = tip_y + bob_rad * math.cos(s.alpha)

        tgt_len = arm_px + 25
        tgt_x = cx + tgt_len * math.cos(s.alpha_target_user)
        tgt_y = cy - tgt_len * math.sin(s.alpha_target_user)
        self.canvas.create_line(cx, cy, tgt_x, tgt_y, fill="#27ae60", width=2, dash=(6,4))
        self.canvas.create_oval(tgt_x-5, tgt_y-5, tgt_x+5, tgt_y+5, fill="#27ae60", outline="")

        rr = arm_px + max(bob_rad, 25) + 40
        self.canvas.create_oval(cx-rr, cy-rr, cx+rr, cy+rr, outline="#e0dcc8")

        # 限位线
        limit_px = arm_px + 35
        for ang_deg in [ALPHA_SOFT_DEG, -ALPHA_SOFT_DEG, ALPHA_HARD_DEG, -ALPHA_HARD_DEG]:
            a = math.radians(ang_deg)
            lx = cx + limit_px * math.cos(a)
            ly = cy - limit_px * math.sin(a)
            color = "#e74c3c" if abs(ang_deg) > ALPHA_SOFT_DEG else "#f39c12"
            dash = (6,4) if abs(ang_deg) == ALPHA_HARD_DEG else (4,2)
            self.canvas.create_line(cx, cy, lx, ly, fill=color, width=2, dash=dash)

        self.canvas.create_oval(cx-20, cy-20, cx+20, cy+20, fill="#333", outline="#111")
        self.canvas.create_line(cx, cy, tip_x, tip_y, fill="#c0392b", width=8, capstyle=tk.ROUND)
        self.canvas.create_oval(tip_x-6, tip_y-6, tip_x+6, tip_y+6, fill="#666")
        self.canvas.create_oval(bob_x-9, bob_y-9, bob_x+9, bob_y+9, fill="#2980b9")
        self.canvas.create_text(cx, cy-rr-5, text="top-down (green=target, yellow=150°, red=180°)",
                                fill="#999", font=("Consolas", 10))

        # ===== 侧视图 =====
        sx, sy = w*3//4, h//2+20
        pen_px = PEND_COM * 400.0
        self.canvas.create_line(sx, sy, sx, sy+pen_px, fill="#ccc", width=2, dash=(6,4))
        self.canvas.create_line(sx-60, sy, sx+60, sy, fill="#e0dcc8", width=2)
        ex = sx + pen_px*math.sin(s.theta)
        ey = sy + pen_px*math.cos(s.theta)
        self.canvas.create_line(sx, sy, ex, ey, fill="#27ae60", width=10, capstyle=tk.ROUND)
        self.canvas.create_oval(ex-10, ey-10, ex+10, ey+10, fill="#e67e22")
        self.canvas.create_text(sx, sy-pen_px-15, text="side view", fill="#999",
                                font=("Consolas", 10))

        # ===== 模式徽章 =====
        bc = "#e67e22" if s.mode == "SWING_UP" else "#27ae60"
        self.canvas.create_rectangle(w-110, 8, w-10, 30, fill=bc, outline="")
        self.canvas.create_text(w-60, 19, text=s.mode, fill="white",
                                font=("Consolas", 10, "bold"))

        # ===== 信息 =====
        phi = wrap_pi(s.theta - PI)
        wall = "WALL" if s.limit_hit else ""
        feedback = (
            "full-state feedback"
            if s.mode == "BALANCE"
            else f"energy swing-up: {s.swing_action}"
        )
        s.limit_hit = False
        self.info.set(
            f"mode = {s.mode}    t = {s.time:5.2f}s    {wall} {feedback}\n"
            f"alpha = {math.degrees(s.alpha):7.2f}deg  (tgt {math.degrees(s.alpha_target_user):.0f}"
            f", ref {math.degrees(s.alpha_target_ref):.0f})    "
            f"alpha_dot = {s.alpha_dot:7.2f} rad/s\n"
            f"theta = {math.degrees(s.theta):7.2f}deg    theta_dot = {s.theta_dot:7.2f} rad/s    "
            f"phi = {math.degrees(phi):+6.2f}deg\n"
            f"torque = {s.motor_torque:+.3f} N*m    "
            f"E = {s.pendulum_energy():.4f}/{s.target_energy():.4f}J    "
            f"dE = {s.last_energy_error:+.4f}J    "
            f"cmd = {s.last_desired_accel:+.3f} N*m"
        )


def main():                                                                                                                                                                                                                                                                                                                                                                     
    root = tk.Tk()
    App(root)
    root.mainloop()

if __name__ == "__main__":
    main()
8