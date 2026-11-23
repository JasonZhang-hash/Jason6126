import math
import tkinter as tk
from dataclasses import dataclass


PI = math.pi
TWO_PI = 2.0 * math.pi


def clamp(x, low, high):
    return max(low, min(high, x))


def sign_nonzero(x):
    return 1.0 if x >= 0.0 else -1.0


def wrap_to_pi(x):
    while x > PI:
        x -= TWO_PI
    while x < -PI:
        x += TWO_PI
    return x


@dataclass
class PendulumParams:
    mass: float = 0.10
    length: float = 0.24
    damping: float = 0.002
    gravity: float = 9.81
    max_torque: float = 0.25
    motor_tau: float = 0.025
    dt: float = 0.001

    @property
    def com(self):
        return self.length / 2.0

    @property
    def inertia(self):
        return (1.0 / 3.0) * self.mass * self.length * self.length


@dataclass
class ControllerParams:
    top_enter_deg: float = 15.0
    top_exit_deg: float = 35.0
    speed_enter: float = 8.0
    start_torque: float = 0.11
    swing_torque: float = 0.16
    kp: float = 0.95
    kd: float = 0.060
    ki: float = 0.01
    integral_limit: float = 0.3


class DirectDrivePendulum:
    def __init__(self):
        self.p = PendulumParams()
        self.c = ControllerParams()
        self.reset()

    def reset(self):
        self.theta = math.radians(2.0)
        self.theta_dot = 0.0
        self.motor_torque = 0.0
        self.integral = 0.0
        self.mode = "SWING_UP"
        self.time = 0.0

    def set_near_top(self):
        self.theta = PI + math.radians(8.0)
        self.theta_dot = 0.0
        self.motor_torque = 0.0
        self.integral = 0.0
        self.mode = "BALANCE"
        self.time = 0.0

    def energy(self):
        return (
            0.5 * self.p.inertia * self.theta_dot * self.theta_dot
            + self.p.mass * self.p.gravity * self.p.com * (1.0 - math.cos(self.theta))
        )

    def target_energy(self):
        return 2.0 * self.p.mass * self.p.gravity * self.p.com

    def controller(self):
        phi = wrap_to_pi(self.theta - PI)
        top_enter = math.radians(self.c.top_enter_deg)
        top_exit = math.radians(self.c.top_exit_deg)

        if self.mode == "SWING_UP":
            if abs(phi) < top_enter and abs(self.theta_dot) < self.c.speed_enter:
                self.mode = "BALANCE"
                self.integral = 0.0
        else:
            if abs(phi) > top_exit:
                self.mode = "SWING_UP"
                self.integral = 0.0

        if self.mode == "SWING_UP":
            if abs(self.theta_dot) < 0.15 and abs(self.theta) < math.radians(8.0):
                return self.c.start_torque

            # Pump energy into the pendulum.
            # For this direct-drive model, dE/dt = motor_torque * theta_dot.
            # If energy is below target, torque should have the same sign as
            # theta_dot; if energy is above target, it should oppose theta_dot.
            energy_error = self.target_energy() - self.energy()
            return self.c.swing_torque * sign_nonzero(self.theta_dot) * sign_nonzero(energy_error)

        self.integral += phi * self.p.dt
        self.integral = clamp(self.integral, -self.c.integral_limit, self.c.integral_limit)
        return -self.c.kp * phi - self.c.kd * self.theta_dot - self.c.ki * self.integral

    def step(self):
        command = clamp(self.controller(), -self.p.max_torque, self.p.max_torque)

        # First-order motor response. Real motor torque does not jump instantly.
        self.motor_torque += (command - self.motor_torque) * self.p.dt / self.p.motor_tau
        self.motor_torque = clamp(self.motor_torque, -self.p.max_torque, self.p.max_torque)

        gravity_torque = -self.p.mass * self.p.gravity * self.p.com * math.sin(self.theta)
        damping_torque = -self.p.damping * self.theta_dot
        theta_ddot = (self.motor_torque + gravity_torque + damping_torque) / self.p.inertia

        self.theta_dot += theta_ddot * self.p.dt
        self.theta += self.theta_dot * self.p.dt
        self.theta = wrap_to_pi(self.theta)
        if self.theta < 0:
            self.theta += TWO_PI
        self.time += self.p.dt


class App:
    def __init__(self, root):
        self.sim = DirectDrivePendulum()
        self.root = root
        self.root.title("Direct-drive inverted pendulum simulation")
        self.running = True

        self.canvas = tk.Canvas(root, width=760, height=520, bg="#fbfaf7")
        self.canvas.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

        tk.Button(root, text="Pause / Run", command=self.toggle).grid(row=1, column=0, sticky="ew", padx=6)
        tk.Button(root, text="Reset bottom", command=self.sim.reset).grid(row=1, column=1, sticky="ew", padx=6)
        tk.Button(root, text="Start near top", command=self.sim.set_near_top).grid(row=1, column=2, sticky="ew", padx=6)
        tk.Button(root, text="More swing", command=self.more_swing).grid(row=1, column=3, sticky="ew", padx=6)

        self.info = tk.StringVar()
        tk.Label(root, textvariable=self.info, font=("Consolas", 11), justify="left").grid(
            row=2, column=0, columnspan=4, sticky="w", padx=10, pady=8
        )
        self.update()

    def toggle(self):
        self.running = not self.running

    def more_swing(self):
        self.sim.c.swing_torque = min(self.sim.c.swing_torque + 0.02, self.sim.p.max_torque)

    def update(self):
        if self.running:
            for _ in range(8):
                self.sim.step()
        self.draw()
        self.root.after(16, self.update)

    def draw(self):
        self.canvas.delete("all")
        w, h = 760, 520
        pivot = (w // 2, 320)
        scale = 760.0
        length_px = self.sim.p.length * scale

        theta = self.sim.theta
        x = pivot[0] + length_px * math.sin(theta)
        y = pivot[1] + length_px * math.cos(theta)

        self.canvas.create_line(70, pivot[1], w - 70, pivot[1], fill="#d8d0c3", width=2)
        self.canvas.create_oval(pivot[0] - 22, pivot[1] - 22, pivot[0] + 22, pivot[1] + 22, fill="#262626")
        self.canvas.create_line(pivot[0], pivot[1], x, y, fill="#b23a2e", width=12, capstyle=tk.ROUND)
        self.canvas.create_oval(x - 10, y - 10, x + 10, y + 10, fill="#4a6fa5", outline="")

        self.canvas.create_arc(
            pivot[0] - 70,
            pivot[1] - 70,
            pivot[0] + 70,
            pivot[1] + 70,
            start=90,
            extent=-math.degrees(self.sim.theta),
            outline="#777",
            width=2,
            style=tk.ARC,
        )

        phi = wrap_to_pi(self.sim.theta - PI)
        self.info.set(
            f"mode={self.sim.mode}   t={self.sim.time:5.2f}s\n"
            f"theta_from_down={math.degrees(self.sim.theta):7.2f} deg   "
            f"phi_from_top={math.degrees(phi):7.2f} deg   "
            f"theta_dot={self.sim.theta_dot:7.2f} rad/s\n"
            f"torque={self.sim.motor_torque: .3f} N*m   "
            f"E={self.sim.energy():.3f} J   E_target={self.sim.target_energy():.3f} J   "
            f"swing_torque={self.sim.c.swing_torque:.2f}"
        )


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
