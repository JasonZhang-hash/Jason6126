"""Single-link Furuta simulation: energy swing-up, then upright PD.

This file is intentionally independent from the earlier simulations.
There is no alpha/arm-centering controller.  The arm may travel freely until
the +/-180 degree soft limit, where outward drive is removed and the motor
brakes to zero speed.  theta=0 is hanging down; theta=pi is upright.
"""

import argparse
import math
import tkinter as tk


PI = math.pi
TWO_PI = 2.0 * PI
G = 9.80665
DT = 0.001
STEPS_PER_FRAME = 16

# Measured / estimated hardware model.
ARM_LENGTH = 0.160
ARM_INERTIA = 0.0030
PEND_MASS = 0.016
PEND_DAMPED_HZ = 1.282
PEND_ZETA = 0.050
PEND_OMEGA_D = TWO_PI * PEND_DAMPED_HZ
PEND_OMEGA_N = PEND_OMEGA_D / math.sqrt(1.0 - PEND_ZETA * PEND_ZETA)
PEND_LENGTH = G / (PEND_OMEGA_N * PEND_OMEGA_N)
PEND_INERTIA = PEND_MASS * PEND_LENGTH * PEND_LENGTH
PEND_DAMPING = 2.0 * PEND_ZETA * PEND_OMEGA_N * PEND_INERTIA

# Measured output-shaft speed curve. PWM is a fraction, not a percentage.
PWM_SPEED_CURVE = (
    (0.060, 0.00),
    (0.085, 0.00),
    (0.100, 4.43),
    (0.140, 7.56),
    (0.180, 8.93),
    (0.220, 14.01),
    (0.260, 18.27),
)
MAX_PWM = 0.260
MOTOR_DRIVE_TAU = 0.120
MOTOR_BRAKE_TAU = 0.090
MAX_ARM_ACCEL = 180.0

# Travel protection only. Neither limit contains a return-to-centre command.
ARM_SOFT_LIMIT = math.radians(180.0)
ARM_HARD_LIMIT = math.radians(195.0)

# Requested top-mode hysteresis, measured from the upright pose.
TOP_ENTER = math.radians(15.0)
TOP_EXIT = math.radians(35.0)

# Edit these two values during PD tuning.
KP = 160.0
KD = 32.0

# Energy-mode commands. The short seed breaks the lower-equilibrium deadlock;
# after that, the arm alternates with the measured pendulum velocity.
SWING_PWM = 0.260
SEED_PWM = 0.140
SEED_SPEED = 0.25


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_to_pi(angle):
    return (angle + PI) % TWO_PI - PI


def interpolate(x, points):
    if x <= points[0][0]:
        return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return points[-1][1]


def pwm_to_speed(pwm):
    magnitude = abs(pwm)
    if magnitude <= PWM_SPEED_CURVE[1][0]:
        return 0.0
    return math.copysign(interpolate(magnitude, PWM_SPEED_CURVE), pwm)


def speed_to_pwm(speed):
    magnitude = abs(speed)
    if magnitude < 0.20:
        return 0.0
    reverse = tuple((omega, pwm) for pwm, omega in PWM_SPEED_CURVE[1:])
    return math.copysign(clamp(interpolate(magnitude, reverse), 0.0, MAX_PWM), speed)


class FurutaPlant:
    def __init__(self, dt=DT):
        self.dt = dt
        self.reset()

    def reset(self, theta_deg=2.0):
        self.alpha = 0.0
        self.alpha_dot = 0.0
        self.theta = math.radians(theta_deg)
        self.theta_dot = 0.0
        self.pwm = 0.0
        self.requested_pwm = 0.0
        self.time = 0.0
        self.limit_active = False
        self.hard_limit_hit = False

    def set_near_top(self):
        self.reset(theta_deg=172.0)

    def energy(self):
        return (
            0.5 * PEND_INERTIA * self.theta_dot * self.theta_dot
            + PEND_MASS * G * PEND_LENGTH * (1.0 - math.cos(self.theta))
        )

    @staticmethod
    def target_energy():
        return 2.0 * PEND_MASS * G * PEND_LENGTH

    def _safe_pwm(self, command):
        command = clamp(command, -MAX_PWM, MAX_PWM)
        self.limit_active = False

        # The brake distance model removes drive before the arm reaches 180.
        # It deliberately does not request an inward return command.
        stopping_angle = abs(self.alpha_dot) * MOTOR_BRAKE_TAU
        moving_outward = self.alpha * self.alpha_dot > 0.0
        if moving_outward and abs(self.alpha) + stopping_angle >= ARM_SOFT_LIMIT:
            command = 0.0
            self.limit_active = True

        # A geared motor should shed speed before a powered reversal.
        if command * self.alpha_dot < 0.0 and abs(self.alpha_dot) > 1.0:
            command = 0.0

        if abs(self.alpha) >= ARM_HARD_LIMIT:
            command = 0.0
            self.limit_active = True
            self.hard_limit_hit = True
        return command

    @staticmethod
    def _accelerations(alpha, alpha_dot, theta, theta_dot, pwm):
        target_speed = pwm_to_speed(pwm)
        accelerating = target_speed * alpha_dot >= 0.0 and abs(target_speed) > abs(alpha_dot)
        tau = MOTOR_DRIVE_TAU if accelerating else MOTOR_BRAKE_TAU
        free_accel = clamp(
            (target_speed - alpha_dot) / tau,
            -MAX_ARM_ACCEL,
            MAX_ARM_ACCEL,
        )

        s = math.sin(theta)
        c = math.cos(theta)
        m = PEND_MASS
        la = ARM_LENGTH
        lp = PEND_LENGTH

        # Coupled Furuta mass matrix: alpha and theta react on each other.
        m11 = ARM_INERTIA + m * la * la + m * lp * lp * s * s
        m12 = m * la * lp * c
        m22 = PEND_INERTIA
        motor_torque = m11 * free_accel
        rhs1 = (
            motor_torque
            - 2.0 * m * lp * lp * s * c * alpha_dot * theta_dot
            + m * la * lp * s * theta_dot * theta_dot
        )
        rhs2 = (
            m * lp * lp * s * c * alpha_dot * alpha_dot
            - m * G * lp * s
            - PEND_DAMPING * theta_dot
        )
        determinant = m11 * m22 - m12 * m12
        alpha_ddot = (rhs1 * m22 - m12 * rhs2) / determinant
        theta_ddot = (m11 * rhs2 - m12 * rhs1) / determinant
        return clamp(alpha_ddot, -MAX_ARM_ACCEL, MAX_ARM_ACCEL), theta_ddot

    def step(self, command):
        self.requested_pwm = command
        self.pwm = self._safe_pwm(command)
        h = self.dt
        a0, ad0, t0, td0 = self.alpha, self.alpha_dot, self.theta, self.theta_dot

        def derivative(a, ad, th, thd):
            add, thdd = self._accelerations(a, ad, th, thd, self.pwm)
            return ad, add, thd, thdd

        k1 = derivative(a0, ad0, t0, td0)
        k2 = derivative(a0 + h*k1[0]/2, ad0 + h*k1[1]/2, t0 + h*k1[2]/2, td0 + h*k1[3]/2)
        k3 = derivative(a0 + h*k2[0]/2, ad0 + h*k2[1]/2, t0 + h*k2[2]/2, td0 + h*k2[3]/2)
        k4 = derivative(a0 + h*k3[0], ad0 + h*k3[1], t0 + h*k3[2], td0 + h*k3[3])

        self.alpha += h * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]) / 6
        self.alpha_dot += h * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]) / 6
        self.theta += h * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]) / 6
        self.theta_dot += h * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]) / 6
        self.theta %= TWO_PI
        self.time += h

        if abs(self.alpha) > ARM_HARD_LIMIT:
            self.alpha = math.copysign(ARM_HARD_LIMIT, self.alpha)
            self.alpha_dot = 0.0
            self.hard_limit_hit = True


class EnergyThenPD:
    def __init__(self):
        self.reset()

    def reset(self):
        self.mode = "SWING_UP"
        self.capture_time = None
        self.energy_error = 0.0
        self.accel_command = 0.0
        self.pump_command = 0.0

    def _energy_pwm(self, sim):
        self.energy_error = sim.target_energy() - sim.energy()

        # At the lower equilibrium theta_dot is zero, so any pure energy law
        # would command zero forever.  Give one small, deterministic kick.
        if (
            abs(sim.theta_dot) < SEED_SPEED
            and sim.energy() < 0.12 * sim.target_energy()
        ):
            self.pump_command = SEED_PWM * (1.0 if math.cos(sim.theta) >= 0.0 else -1.0)
            return self.pump_command

        # Measured-model sign.  theta=0 is down, and positive motor command
        # produces the opposite pendulum-energy sign to the strategy notes'
        # convention, hence the leading minus sign.
        direction = 1.0 if self.energy_error * sim.theta_dot >= 0.0 else -1.0
        self.pump_command = -SWING_PWM * direction
        return self.pump_command

    def _pd_pwm(self, sim):
        phi = wrap_to_pi(sim.theta - PI)
        # theta_ddot ~= (g/L)*phi + (L_arm/L)*alpha_ddot near upright.
        # This is deliberately theta-only PD: no alpha position/velocity term.
        self.accel_command = (PEND_LENGTH / ARM_LENGTH) * (
            -(PEND_OMEGA_N * PEND_OMEGA_N + KP) * phi - KD * sim.theta_dot
        )
        target_speed = sim.alpha_dot + MOTOR_DRIVE_TAU * clamp(
            self.accel_command, -MAX_ARM_ACCEL, MAX_ARM_ACCEL
        )
        target_speed = clamp(target_speed, -PWM_SPEED_CURVE[-1][1], PWM_SPEED_CURVE[-1][1])
        self.pump_command = speed_to_pwm(target_speed)
        return self.pump_command

    def command(self, sim):
        phi = wrap_to_pi(sim.theta - PI)
        if self.mode == "SWING_UP" and abs(phi) <= TOP_ENTER:
            self.mode = "PD"
            self.capture_time = sim.time
        elif self.mode == "PD" and abs(phi) >= TOP_EXIT:
            self.mode = "SWING_UP"

        if self.mode == "PD":
            return self._pd_pwm(sim)
        return self._energy_pwm(sim)


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Furuta: Energy Swing-up + PD, +/-180 deg")
        self.sim = FurutaPlant()
        self.control = EnergyThenPD()
        self.running = True

        self.canvas = tk.Canvas(root, width=980, height=650, bg="#f8f8f5")
        self.canvas.pack(padx=10, pady=10)
        bar = tk.Frame(root)
        bar.pack(fill="x", padx=10)
        tk.Button(bar, text="Pause / Run", command=self.toggle).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Reset bottom", command=self.reset).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Near top", command=self.near_top).pack(side=tk.LEFT, padx=3)
        self.info = tk.StringVar()
        tk.Label(root, textvariable=self.info, font=("Consolas", 10), justify="left").pack(
            anchor="w", padx=14, pady=8
        )
        self.root.after(16, self.update)

    def toggle(self):
        self.running = not self.running

    def reset(self):
        self.sim.reset()
        self.control.reset()

    def near_top(self):
        self.sim.set_near_top()
        self.control.reset()
        self.control.mode = "PD"

    def update(self):
        if self.running:
            for _ in range(STEPS_PER_FRAME):
                self.sim.step(self.control.command(self.sim))
        self.draw()
        self.root.after(16, self.update)

    def draw(self):
        self.canvas.delete("all")
        sim = self.sim
        width, height = 980, 650

        # Top view: base -> arm tip -> horizontal projection of pendulum mass.
        cx, cy = 260, 310
        scale = 640.0
        arm_px = ARM_LENGTH * scale
        tip_x = cx + arm_px * math.cos(sim.alpha)
        tip_y = cy - arm_px * math.sin(sim.alpha)
        projection = PEND_LENGTH * math.sin(sim.theta) * scale
        mass_x = tip_x + projection * math.sin(sim.alpha)
        mass_y = tip_y + projection * math.cos(sim.alpha)
        radius = ARM_LENGTH * scale + 20
        self.canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, outline="#d5d5cf")
        self.canvas.create_oval(cx-18, cy-18, cx+18, cy+18, fill="#30343b", outline="")
        self.canvas.create_line(cx, cy, tip_x, tip_y, fill="#b84c3d", width=9)
        self.canvas.create_oval(tip_x-7, tip_y-7, tip_x+7, tip_y+7, fill="#555", outline="")
        self.canvas.create_line(tip_x, tip_y, mass_x, mass_y, fill="#2f7d58", width=7)
        self.canvas.create_oval(mass_x-11, mass_y-11, mass_x+11, mass_y+11, fill="#e29a31", outline="")
        self.canvas.create_text(cx, 28, text="TOP VIEW: ARM TIP + PENDULUM MASS", fill="#555")

        # Side view: true pendulum angle in its vertical plane.
        sx, sy = 735, 350
        pend_px = PEND_LENGTH * 700.0
        end_x = sx + pend_px * math.sin(sim.theta)
        end_y = sy + pend_px * math.cos(sim.theta)
        self.canvas.create_line(sx-100, sy, sx+100, sy, fill="#bcbcb7", width=2)
        self.canvas.create_oval(sx-9, sy-9, sx+9, sy+9, fill="#555", outline="")
        self.canvas.create_line(sx, sy, end_x, end_y, fill="#2f7d58", width=9)
        self.canvas.create_oval(end_x-12, end_y-12, end_x+12, end_y+12, fill="#e29a31", outline="")
        self.canvas.create_text(sx, 28, text="SIDE VIEW: PENDULUM", fill="#555")

        color = "#2f8f5b" if self.control.mode == "PD" else "#c47a22"
        self.canvas.create_rectangle(width-145, 12, width-18, 42, fill=color, outline="")
        self.canvas.create_text(width-82, 27, text=self.control.mode, fill="white", font=("Consolas", 11, "bold"))

        phi = wrap_to_pi(sim.theta - PI)
        self.info.set(
            f"t={sim.time:6.2f}s  mode={self.control.mode}  E={sim.energy():.4f}/{sim.target_energy():.4f}J\n"
            f"theta={math.degrees(sim.theta):7.2f}deg  top_error={math.degrees(phi):7.2f}deg  theta_dot={sim.theta_dot:7.2f}rad/s\n"
            f"alpha={math.degrees(sim.alpha):7.2f}deg  alpha_dot={sim.alpha_dot:7.2f}rad/s  pwm={100*sim.pwm:5.1f}%\n"
            f"PD Kp={KP:.0f}  Kd={KD:.0f}  enter=15deg exit=35deg  soft_limit=+/-180deg  limit={sim.limit_active}"
        )


def run_headless(seconds):
    sim = FurutaPlant()
    control = EnergyThenPD()
    transitions = []
    last = control.mode
    for _ in range(int(seconds / DT)):
        sim.step(control.command(sim))
        if control.mode != last:
            transitions.append((round(sim.time, 3), control.mode))
            last = control.mode
    phi = math.degrees(wrap_to_pi(sim.theta - PI))
    print(f"mode={control.mode} top_error={phi:.2f}deg alpha={math.degrees(sim.alpha):.1f}deg hard={sim.hard_limit_hit}")
    print(f"transitions={transitions}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=12.0)
    args = parser.parse_args()
    if args.headless:
        run_headless(args.seconds)
        return
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
