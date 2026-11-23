import argparse
import itertools
import math
import tkinter as tk
from dataclasses import dataclass


PI = math.pi
TWO_PI = 2.0 * PI
GRAVITY = 9.80665


# ============================================================
# IDENTIFIED HARDWARE MODEL - edit these values first
# ============================================================

# Geometry and inertia.
ARM_LENGTH = 0.16                 # m, measured horizontal arm length
ARM_INERTIA = 0.0030              # kg*m^2, arm + reflected gearbox estimate
PEND_MASS = 0.016                 # kg, 6 g rod + provisional 10 g tip weight

# Weighted-pendulum identification, 2026-08-05.
PEND_DAMPED_FREQ_HZ = 1.282       # median measured free-decay frequency
PEND_DAMPING_RATIO = 0.050        # measured range approximately 0.045-0.060
PEND_OMEGA_D = TWO_PI * PEND_DAMPED_FREQ_HZ
PEND_OMEGA_N = PEND_OMEGA_D / math.sqrt(1.0 - PEND_DAMPING_RATIO ** 2)
PEND_EFFECTIVE_LENGTH = GRAVITY / (PEND_OMEGA_N ** 2)  # about 0.151 m
PEND_INERTIA = PEND_MASS * PEND_EFFECTIVE_LENGTH ** 2
PEND_VISCOUS_DAMPING = (
    2.0 * PEND_DAMPING_RATIO * PEND_OMEGA_N * PEND_INERTIA
)

# Measured PWM -> output-shaft speed. PWM is a fraction: 0.10 means 10%.
PWM_SPEED_CURVE = (
    (0.060, 0.00),
    (0.085, 0.00),                # measured practical breakaway threshold
    (0.100, 4.43),
    (0.140, 7.56),
    (0.180, 8.93),
    (0.220, 14.01),
    (0.260, 18.27),
)
MAX_PWM = 0.260                   # do not extrapolate beyond measured data
RETURN_PWM = 0.140
MOTOR_DRIVE_TAU = 0.120           # s, provisional loaded speed time constant
MOTOR_BRAKE_TAU = 0.090           # s, from short-brake stopping distance
MAX_ARM_ACCEL = 180.0             # rad/s^2, numerical and hardware guard
REVERSE_RELEASE_SPEED = 1.0       # rad/s; brake before powered reversal

# Cable-safe unwrapped arm limits.
ARM_SOFT_LIMIT = math.radians(90.0)
ARM_HARD_LIMIT = math.radians(135.0)

# Capture and balance.
TOP_ENTER = math.radians(10.0)
TOP_EXIT = math.radians(25.0)
CATCH_SPEED = 2.0                 # rad/s
BALANCE_OMEGA = 12.0              # desired local closed-loop bandwidth
BALANCE_ZETA = 0.85
ARM_CENTER_KP = 1.5               # desired alpha acceleration gains
ARM_CENTER_KD = 2.0

DT = 0.001
STEPS_PER_FRAME = 16


def clamp(value, low, high):
    return max(low, min(high, value))


def sign(value):
    return 1.0 if value >= 0.0 else -1.0


def wrap_to_pi(value):
    return (value + PI) % TWO_PI - PI


def interpolate(value, points):
    if value <= points[0][0]:
        return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if value <= x1:
            ratio = (value - x0) / (x1 - x0)
            return y0 + ratio * (y1 - y0)
    return points[-1][1]


def pwm_to_speed(pwm):
    magnitude = abs(pwm)
    if magnitude <= PWM_SPEED_CURVE[1][0]:
        return 0.0
    speed = interpolate(magnitude, PWM_SPEED_CURVE)
    return math.copysign(speed, pwm)


def speed_to_pwm(speed):
    magnitude = abs(speed)
    if magnitude < 0.20:
        return 0.0
    reverse_curve = tuple((omega, pwm) for pwm, omega in PWM_SPEED_CURVE[1:])
    pwm = interpolate(magnitude, reverse_curve)
    return math.copysign(clamp(pwm, 0.0, MAX_PWM), speed)


@dataclass(frozen=True)
class SwingStrategy:
    start_hz: float = 1.25
    end_hz: float = 1.00
    chirp_seconds: float = 7.0
    arm_amplitude_deg: float = 45.0
    tracking_gain: float = 5.0
    phase_deg: float = 270.0


@dataclass
class SearchResult:
    strategy: SwingStrategy
    score: float
    best_top_deg: float
    best_top_speed: float
    best_arm_deg: float
    best_arm_speed: float
    best_time: float
    max_arm_deg: float
    caught: bool


DEFAULT_STRATEGY = SwingStrategy()


class FurutaPendulum:
    """Coupled Furuta model with an empirical motor-speed actuator."""

    def __init__(self, dt=DT):
        self.dt = dt
        self.lenient = False
        self.reset()

    def reset(self, theta_deg=2.0):
        self.alpha = 0.0
        self.alpha_dot = 0.0
        self.theta = math.radians(theta_deg)
        self.theta_dot = 0.0
        self.time = 0.0
        self.pwm = 0.0
        self.requested_pwm = 0.0
        self.limit_active = False
        self.hard_limit_hit = False

    def set_near_top(self):
        self.reset(theta_deg=172.0)

    def pendulum_energy(self):
        return (
            0.5 * PEND_INERTIA * self.theta_dot ** 2
            + PEND_MASS * GRAVITY * PEND_EFFECTIVE_LENGTH
            * (1.0 - math.cos(self.theta))
        )

    def target_energy(self):
        return 2.0 * PEND_MASS * GRAVITY * PEND_EFFECTIVE_LENGTH

    def _safe_pwm(self, requested):
        requested = clamp(requested, -MAX_PWM, MAX_PWM)
        self.limit_active = False

        if self.lenient:
            return requested

        # A first-order short-brake model stops in approximately omega*tau.
        stop_angle = abs(self.alpha_dot) * MOTOR_BRAKE_TAU
        moving_outward = self.alpha * self.alpha_dot > 0.0
        if moving_outward and abs(self.alpha) + stop_angle >= ARM_SOFT_LIMIT:
            requested = 0.0
            self.limit_active = True

        if abs(self.alpha) >= ARM_SOFT_LIMIT and abs(self.alpha_dot) < 1.0:
            requested = -math.copysign(RETURN_PWM, self.alpha)
            self.limit_active = True

        # The geared motor must brake before changing powered direction.
        if requested * self.alpha_dot < 0.0 and abs(self.alpha_dot) > REVERSE_RELEASE_SPEED:
            requested = 0.0

        if abs(self.alpha) >= ARM_HARD_LIMIT:
            requested = 0.0
            self.hard_limit_hit = True
            self.limit_active = True

        return requested

    @staticmethod
    def _accelerations(alpha, alpha_dot, theta, theta_dot, pwm):
        target_speed = pwm_to_speed(pwm)
        accelerating = (
            target_speed * alpha_dot >= 0.0
            and abs(target_speed) > abs(alpha_dot)
        )
        tau = MOTOR_DRIVE_TAU if accelerating else MOTOR_BRAKE_TAU
        free_accel = clamp(
            (target_speed - alpha_dot) / tau,
            -MAX_ARM_ACCEL,
            MAX_ARM_ACCEL,
        )

        m = PEND_MASS
        la = ARM_LENGTH
        lp = PEND_EFFECTIVE_LENGTH
        s = math.sin(theta)
        c = math.cos(theta)

        # Full two-way mass matrix. The empirical motor curve supplies the
        # acceleration the arm would have without pendulum reaction.
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
            - m * GRAVITY * lp * s
            - PEND_VISCOUS_DAMPING * theta_dot
        )

        determinant = m11 * m22 - m12 * m12
        alpha_ddot = (rhs1 * m22 - m12 * rhs2) / determinant
        theta_ddot = (m11 * rhs2 - m12 * rhs1) / determinant
        alpha_ddot = clamp(alpha_ddot, -MAX_ARM_ACCEL, MAX_ARM_ACCEL)
        return alpha_ddot, theta_ddot

    def step(self, requested_pwm):
        self.requested_pwm = requested_pwm
        self.pwm = self._safe_pwm(requested_pwm)
        h = self.dt
        a0, ad0, t0, td0 = self.alpha, self.alpha_dot, self.theta, self.theta_dot

        def derivative(a, ad, th, thd):
            add, thdd = self._accelerations(a, ad, th, thd, self.pwm)
            return ad, add, thd, thdd

        k1 = derivative(a0, ad0, t0, td0)
        k2 = derivative(
            a0 + 0.5 * h * k1[0], ad0 + 0.5 * h * k1[1],
            t0 + 0.5 * h * k1[2], td0 + 0.5 * h * k1[3],
        )
        k3 = derivative(
            a0 + 0.5 * h * k2[0], ad0 + 0.5 * h * k2[1],
            t0 + 0.5 * h * k2[2], td0 + 0.5 * h * k2[3],
        )
        k4 = derivative(
            a0 + h * k3[0], ad0 + h * k3[1],
            t0 + h * k3[2], td0 + h * k3[3],
        )

        self.alpha += h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
        self.alpha_dot += h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        self.theta += h * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0
        self.theta_dot += h * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]) / 6.0
        self.theta %= TWO_PI
        self.time += h

        if abs(self.alpha) > ARM_HARD_LIMIT:
            self.alpha = math.copysign(ARM_HARD_LIMIT, self.alpha)
            self.alpha_dot = 0.0
            self.hard_limit_hit = True


class SwingAndBalanceController:
    def __init__(self, strategy=DEFAULT_STRATEGY, enable_balance=True):
        self.strategy = strategy
        self.enable_balance = enable_balance
        self.reset()

    def reset(self):
        self.mode = "SWING_UP"
        self.phase = math.radians(self.strategy.phase_deg)
        self.capture_time = None
        self.frequency = self.strategy.start_hz
        self.alpha_reference = 0.0

    def _swing_pwm(self, sim):
        s = self.strategy
        ratio = clamp(sim.time / s.chirp_seconds, 0.0, 1.0)
        self.frequency = s.start_hz + ratio * (s.end_hz - s.start_hz)
        self.phase += TWO_PI * self.frequency * sim.dt

        amplitude = math.radians(s.arm_amplitude_deg)
        self.alpha_reference = amplitude * math.sin(self.phase)
        reference_speed = amplitude * TWO_PI * self.frequency * math.cos(self.phase)
        desired_speed = reference_speed + s.tracking_gain * (
            self.alpha_reference - sim.alpha
        )
        desired_speed = clamp(
            desired_speed,
            -PWM_SPEED_CURVE[-1][1],
            PWM_SPEED_CURVE[-1][1],
        )
        return speed_to_pwm(desired_speed)

    @staticmethod
    def _balance_pwm(sim):
        phi = wrap_to_pi(sim.theta - PI)
        desired_accel = -(
            PEND_EFFECTIVE_LENGTH / ARM_LENGTH
        ) * (
            (PEND_OMEGA_N ** 2 + BALANCE_OMEGA ** 2) * phi
            + (
                2.0 * PEND_DAMPING_RATIO * PEND_OMEGA_N
                + 2.0 * BALANCE_ZETA * BALANCE_OMEGA
            ) * sim.theta_dot
        )
        desired_accel -= ARM_CENTER_KP * sim.alpha + ARM_CENTER_KD * sim.alpha_dot
        desired_accel = clamp(desired_accel, -MAX_ARM_ACCEL, MAX_ARM_ACCEL)
        desired_speed = sim.alpha_dot + MOTOR_DRIVE_TAU * desired_accel
        return speed_to_pwm(clamp(
            desired_speed,
            -PWM_SPEED_CURVE[-1][1],
            PWM_SPEED_CURVE[-1][1],
        ))

    def command(self, sim):
        phi = wrap_to_pi(sim.theta - PI)
        if self.mode == "SWING_UP":
            if (
                self.enable_balance
                and abs(phi) <= TOP_ENTER
                and abs(sim.theta_dot) <= CATCH_SPEED
                and abs(sim.alpha) <= math.radians(45.0)
                and abs(sim.alpha_dot) <= 3.0
            ):
                self.mode = "BALANCE"
                self.capture_time = sim.time
        elif abs(phi) >= TOP_EXIT:
            self.mode = "SWING_UP"

        if self.mode == "BALANCE":
            return self._balance_pwm(sim)
        return self._swing_pwm(sim)


def evaluate_strategy(strategy, duration=12.0, dt=0.001):
    sim = FurutaPendulum(dt=dt)
    controller = SwingAndBalanceController(strategy, enable_balance=False)
    best_metric = float("inf")
    best_top = PI
    best_speed = float("inf")
    best_arm = 0.0
    best_arm_speed = 0.0
    best_time = 0.0
    max_arm = 0.0
    caught = False

    for _ in range(int(duration / dt)):
        phi = abs(wrap_to_pi(sim.theta - PI))
        metric = (
            phi
            + 0.04 * abs(sim.theta_dot)
            + 0.20 * abs(sim.alpha)
            + 0.02 * abs(sim.alpha_dot)
        )
        if metric < best_metric:
            best_metric = metric
            best_top = phi
            best_speed = abs(sim.theta_dot)
            best_arm = sim.alpha
            best_arm_speed = sim.alpha_dot
            best_time = sim.time
        if (
            phi <= TOP_ENTER
            and abs(sim.theta_dot) <= CATCH_SPEED
            and abs(sim.alpha) <= math.radians(45.0)
            and abs(sim.alpha_dot) <= 3.0
        ):
            caught = True

        sim.step(controller.command(sim))
        max_arm = max(max_arm, abs(sim.alpha))

    limit_penalty = 20.0 if sim.hard_limit_hit else 0.0
    travel_penalty = max(0.0, max_arm - ARM_SOFT_LIMIT) * 4.0
    time_penalty = 0.01 * best_time
    catch_penalty = 2.0 if not caught else 0.0
    score = best_metric + limit_penalty + travel_penalty + time_penalty + catch_penalty
    return SearchResult(
        strategy=strategy,
        score=score,
        best_top_deg=math.degrees(best_top),
        best_top_speed=best_speed,
        best_arm_deg=math.degrees(best_arm),
        best_arm_speed=best_arm_speed,
        best_time=best_time,
        max_arm_deg=math.degrees(max_arm),
        caught=caught,
    )


def find_best_strategy():
    candidates = (
        SwingStrategy(start, end, seconds, amplitude, gain, phase)
        for start, end, seconds, amplitude, gain, phase in itertools.product(
            (1.15, 1.25, 1.35, 1.45),
            (0.70, 0.85, 1.00, 1.15),
            (5.0, 7.0, 9.0),
            (45.0, 60.0),
            (5.0, 8.0),
            (0.0, 90.0, 180.0, 270.0),
        )
        if end <= start
    )
    results = [evaluate_strategy(strategy) for strategy in candidates]
    results.sort(key=lambda result: result.score)
    return results


def run_headless(strategy=DEFAULT_STRATEGY, duration=12.0):
    sim = FurutaPendulum()
    controller = SwingAndBalanceController(strategy, enable_balance=True)
    max_arm = 0.0
    for _ in range(int(duration / sim.dt)):
        sim.step(controller.command(sim))
        max_arm = max(max_arm, abs(sim.alpha))
    phi = wrap_to_pi(sim.theta - PI)
    print(
        f"mode={controller.mode} capture={controller.capture_time} "
        f"phi={math.degrees(phi):.2f} deg theta_dot={sim.theta_dot:.2f} rad/s "
        f"alpha={math.degrees(sim.alpha):.2f} deg "
        f"max_alpha={math.degrees(max_arm):.2f} deg hard_limit={sim.hard_limit_hit}"
    )
    return sim, controller


class App:
    def __init__(self, root, strategy=DEFAULT_STRATEGY):
        self.root = root
        self.root.title("Measured Furuta Pendulum Simulation")
        self.sim = FurutaPendulum()
        self.controller = SwingAndBalanceController(strategy)
        self.running = True

        self.canvas = tk.Canvas(root, width=920, height=620, bg="#f7f7f4")
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
        self.controller.reset()

    def near_top(self):
        self.sim.set_near_top()
        self.controller.reset()
        self.controller.mode = "BALANCE"

    def update(self):
        if self.running:
            for _ in range(STEPS_PER_FRAME):
                self.sim.step(self.controller.command(self.sim))
        self.draw()
        self.root.after(16, self.update)

    def draw(self):
        self.canvas.delete("all")
        sim = self.sim
        width, height = 920, 620
        scale = 600.0

        cx, cy = width // 4, height // 2
        arm_px = ARM_LENGTH * scale
        tip_x = cx + arm_px * math.cos(sim.alpha)
        tip_y = cy - arm_px * math.sin(sim.alpha)
        pendulum_projection = (
            PEND_EFFECTIVE_LENGTH * math.sin(sim.theta) * scale
        )
        weight_x = tip_x + pendulum_projection * math.sin(sim.alpha)
        weight_y = tip_y + pendulum_projection * math.cos(sim.alpha)
        self.canvas.create_oval(cx - 150, cy - 150, cx + 150, cy + 150, outline="#d5d5cf")
        self.canvas.create_oval(cx - 18, cy - 18, cx + 18, cy + 18, fill="#30343b", outline="")
        self.canvas.create_line(cx, cy, tip_x, tip_y, fill="#b84c3d", width=9)
        self.canvas.create_oval(tip_x - 7, tip_y - 7, tip_x + 7, tip_y + 7, fill="#555")
        self.canvas.create_line(
            tip_x, tip_y, weight_x, weight_y,
            fill="#2f7d58", width=7, capstyle=tk.ROUND,
        )
        self.canvas.create_oval(
            weight_x - 10, weight_y - 10, weight_x + 10, weight_y + 10,
            fill="#e29a31", outline="",
        )
        self.canvas.create_text(
            cx, 24, text="TOP VIEW: ARM TIP + PENDULUM WEIGHT", fill="#555"
        )

        sx, sy = width * 3 // 4, height // 2 + 40
        pend_px = PEND_EFFECTIVE_LENGTH * 650.0
        end_x = sx + pend_px * math.sin(sim.theta)
        end_y = sy + pend_px * math.cos(sim.theta)
        self.canvas.create_line(sx - 80, sy, sx + 80, sy, fill="#bbb", width=2)
        self.canvas.create_line(sx, sy, end_x, end_y, fill="#2f7d58", width=9)
        self.canvas.create_oval(end_x - 11, end_y - 11, end_x + 11, end_y + 11,
                                fill="#e29a31", outline="")
        self.canvas.create_text(sx, 24, text="SIDE VIEW: WEIGHTED PENDULUM", fill="#555")

        mode_color = "#2f8f5b" if self.controller.mode == "BALANCE" else "#c47a22"
        self.canvas.create_rectangle(width - 130, 10, width - 15, 36,
                                     fill=mode_color, outline="")
        self.canvas.create_text(width - 72, 23, text=self.controller.mode,
                                fill="white", font=("Consolas", 10, "bold"))

        phi = wrap_to_pi(sim.theta - PI)
        self.info.set(
            f"t={sim.time:6.2f}s  mode={self.controller.mode}  "
            f"pump_f={self.controller.frequency:4.2f}Hz\n"
            f"theta={math.degrees(sim.theta):7.2f}deg  phi={math.degrees(phi):7.2f}deg  "
            f"theta_dot={sim.theta_dot:7.2f}rad/s\n"
            f"alpha={math.degrees(sim.alpha):7.2f}deg  "
            f"alpha_dot={sim.alpha_dot:7.2f}rad/s  pwm={100.0 * sim.pwm:5.1f}%  "
            f"limit={sim.limit_active}\n"
            f"identified: fd={PEND_DAMPED_FREQ_HZ:.3f}Hz  zeta={PEND_DAMPING_RATIO:.3f}  "
            f"L_eff={PEND_EFFECTIVE_LENGTH:.3f}m  arm={ARM_LENGTH:.2f}m"
        )


def main():
    parser = argparse.ArgumentParser(description="Measured Furuta pendulum model")
    parser.add_argument("--search", action="store_true", help="grid-search swing trajectory")
    parser.add_argument("--headless", action="store_true", help="run the default strategy without GUI")
    args = parser.parse_args()

    if args.search:
        results = find_best_strategy()
        for index, result in enumerate(results[:10], 1):
            s = result.strategy
            print(
                f"{index:2d}: score={result.score:.4f} caught={result.caught} "
                f"top={result.best_top_deg:.2f}deg speed={result.best_top_speed:.2f}rad/s "
                f"alpha={result.best_arm_deg:.1f}deg alpha_dot={result.best_arm_speed:.2f}rad/s "
                f"t={result.best_time:.2f}s arm={result.max_arm_deg:.1f}deg "
                f"f={s.start_hz:.2f}->{s.end_hz:.2f}Hz/{s.chirp_seconds:.0f}s "
                f"A={s.arm_amplitude_deg:.0f}deg K={s.tracking_gain:.1f} "
                f"phase={s.phase_deg:.0f}deg"
            )
        return

    if args.headless:
        run_headless()
        return

    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
