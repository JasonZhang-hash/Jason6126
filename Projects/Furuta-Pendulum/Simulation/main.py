
"""Measured single-link Furuta model with energy swing-up and PD capture.

Angles follow furuta_measured_core.py:
  theta = 0 at the hanging-down equilibrium, theta = pi at upright.
  alpha = horizontal arm angle.
"""

import argparse
import math
import tkinter as tk

import furuta_measured_core as core


# Energy-mode tuning. These produce a desired arm acceleration, then use the
# measured PWM-to-speed curve in the shared actuator model.
ENERGY_ACCEL_GAIN = 2100.0
ENERGY_ACCEL_LIMIT = 125.0
SWING_ARM_CENTER_KP = 0.0
SWING_ARM_CENTER_KD = 0.0

# Local upright PD. phi is the signed error from theta = pi.
BALANCE_P = 160.0
BALANCE_D = 32.0
BALANCE_ARM_CENTER_KP = 1.5
BALANCE_ARM_CENTER_KD = 2.0

CATCH_SPEED = 3.0
CATCH_ALPHA = math.radians(180.0)
CATCH_ALPHA_DOT = 4.0

# The requested hysteresis: enter PD inside 15 deg; leave it only after 35 deg.
TOP_ENTER = math.radians(15.0)
TOP_EXIT = math.radians(35.0)


def accel_to_pwm(sim, desired_accel):
    """Translate a desired arm acceleration to the measured motor command."""
    desired_accel = core.clamp(
        desired_accel, -ENERGY_ACCEL_LIMIT, ENERGY_ACCEL_LIMIT
    )
    desired_speed = sim.alpha_dot + core.MOTOR_DRIVE_TAU * desired_accel
    desired_speed = core.clamp(
        desired_speed,
        -core.PWM_SPEED_CURVE[-1][1],
        core.PWM_SPEED_CURVE[-1][1],
    )
    return core.speed_to_pwm(desired_speed)


class EnergySwingController:
    def __init__(self):
        self.reset()

    def reset(self):
        self.mode = "SWING_UP"
        self.capture_time = None
        self.frequency = 0.0  # kept for the shared display
        self.alpha_reference = 0.0
        self.last_energy_error = 0.0
        self.last_desired_accel = 0.0

    def _swing_pwm(self, sim):
        energy_error = sim.pendulum_energy() - sim.target_energy()

        # dE/dt ~= -m * L_arm * L_p * theta_dot * cos(theta) * alpha_ddot.
        pump = ENERGY_ACCEL_GAIN * energy_error * sim.theta_dot * math.cos(sim.theta)
        desired_accel = pump

        self.last_energy_error = energy_error
        self.last_desired_accel = desired_accel
        return accel_to_pwm(sim, desired_accel)

    def _balance_pwm(self, sim):
        phi = core.wrap_to_pi(sim.theta - core.PI)

        # Near upright, theta_ddot ~= (g/L) phi + (L_arm/L) alpha_ddot.
        # The arm acceleration cancels the unstable gravity term and adds PD.
        desired_accel = (core.PEND_EFFECTIVE_LENGTH / core.ARM_LENGTH) * (
            -(core.PEND_OMEGA_N ** 2 + BALANCE_P) * phi
            - BALANCE_D * sim.theta_dot
        )
        desired_accel += (
            -BALANCE_ARM_CENTER_KP * sim.alpha
            -BALANCE_ARM_CENTER_KD * sim.alpha_dot
        )
        self.last_desired_accel = desired_accel
        return accel_to_pwm(sim, desired_accel)

    def command(self, sim):
        phi = core.wrap_to_pi(sim.theta - core.PI)

        if (
            self.mode == "SWING_UP"
            and abs(phi) <= TOP_ENTER
            and abs(sim.theta_dot) <= CATCH_SPEED
            and abs(sim.alpha) <= CATCH_ALPHA
            and abs(sim.alpha_dot) <= CATCH_ALPHA_DOT
        ):
            self.mode = "BALANCE"
            sim.lenient = True
            self.capture_time = sim.time
        elif self.mode == "BALANCE" and abs(phi) >= TOP_EXIT:
            self.mode = "SWING_UP"
            sim.lenient = False

        if self.mode == "BALANCE":
            return self._balance_pwm(sim)
        return self._swing_pwm(sim)


class EnergyApp(core.App):
    def __init__(self, root):
        self.root = root
        self.root.title("Furuta: Energy Swing-up + PD Capture")
        self.sim = core.FurutaPendulum()
        self.controller = EnergySwingController()
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

    def draw(self):
        super().draw()
        sim = self.sim
        phi = core.wrap_to_pi(sim.theta - core.PI)
        self.info.set(
            f"t={sim.time:6.2f}s  mode={self.controller.mode}  "
            f"E={sim.pendulum_energy():.4f}/{sim.target_energy():.4f}J  "
            f"dE={self.controller.last_energy_error:+.4f}J\n"
            f"theta={math.degrees(sim.theta):7.2f}deg  phi={math.degrees(phi):7.2f}deg  "
            f"theta_dot={sim.theta_dot:7.2f}rad/s\n"
            f"alpha={math.degrees(sim.alpha):7.2f}deg  "
            f"alpha_dot={sim.alpha_dot:7.2f}rad/s  pwm={100.0 * sim.pwm:5.1f}%  "
            f"a_cmd={self.controller.last_desired_accel:7.1f}rad/s^2\n"
            f"PD: enter {math.degrees(TOP_ENTER):.0f}deg / exit {math.degrees(TOP_EXIT):.0f}deg  "
            f"limit={sim.limit_active}"
        )


def run_headless(seconds):
    sim = core.FurutaPendulum()
    controller = EnergySwingController()
    transitions = []
    prior_mode = controller.mode
    for _ in range(int(seconds / sim.dt)):
        sim.step(controller.command(sim))
        if controller.mode != prior_mode:
            transitions.append((sim.time, controller.mode))
            prior_mode = controller.mode

    phi = core.wrap_to_pi(sim.theta - core.PI)
    print(
        f"mode={controller.mode} phi={math.degrees(phi):.2f}deg "
        f"theta_dot={sim.theta_dot:.2f}rad/s alpha={math.degrees(sim.alpha):.1f}deg "
        f"max_pwm={100.0 * core.MAX_PWM:.0f}% hard_limit={sim.hard_limit_hit}"
    )
    print(f"transitions={transitions}")


def main():
    parser = argparse.ArgumentParser(description="Energy swing-up + PD Furuta simulation")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=12.0)
    args = parser.parse_args()
    if args.headless:
        run_headless(args.seconds)
        return
    root = tk.Tk()
    EnergyApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
