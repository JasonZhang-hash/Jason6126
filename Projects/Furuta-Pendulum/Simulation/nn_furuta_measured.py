import argparse
import math
import random
import tkinter as tk

import furuta_measured_core as core


NN_INPUT = 4
NN_HIDDEN = 12
NN_LEARNING_RATE = 0.002
NN_BATCH = 32
NN_BUFFER = 5000
NN_WARMUP = 256
NN_FULL_BLEND = 2000


def clamp(value, low, high):
    return max(low, min(high, value))


class TinyNN:
    """Small online policy that learns the model-based balance command."""

    def __init__(self, seed=42):
        self.random = random.Random(seed)
        self.reset()

    def reset(self):
        scale1 = math.sqrt(2.0 / NN_INPUT)
        scale2 = math.sqrt(2.0 / NN_HIDDEN)
        self.w1 = [
            self.random.uniform(-scale1, scale1)
            for _ in range(NN_INPUT * NN_HIDDEN)
        ]
        self.b1 = [0.0] * NN_HIDDEN
        self.w2 = [self.random.uniform(-scale2, scale2) for _ in range(NN_HIDDEN)]
        self.b2 = 0.0
        self.states = []
        self.targets = []
        self.loss = 0.0
        self.train_steps = 0

    def forward_normalized(self, state):
        hidden = []
        for i in range(NN_HIDDEN):
            value = self.b1[i]
            offset = i * NN_INPUT
            for j in range(NN_INPUT):
                value += self.w1[offset + j] * state[j]
            hidden.append(math.tanh(value))
        output_linear = self.b2 + sum(
            self.w2[i] * hidden[i] for i in range(NN_HIDDEN)
        )
        return math.tanh(output_linear), hidden

    def command(self, state):
        normalized, _ = self.forward_normalized(state)
        return core.MAX_PWM * normalized

    def store(self, state, target_pwm):
        self.states.append(state[:])
        self.targets.append(clamp(target_pwm / core.MAX_PWM, -1.0, 1.0))
        if len(self.states) > NN_BUFFER:
            self.states.pop(0)
            self.targets.pop(0)

    def train_batch(self):
        if len(self.states) < NN_BATCH:
            return 0.0
        indices = self.random.sample(range(len(self.states)), NN_BATCH)
        total_loss = 0.0
        for index in indices:
            state = self.states[index]
            target = self.targets[index]
            output, hidden = self.forward_normalized(state)
            error = output - target
            output_grad = error * (1.0 - output * output)
            old_w2 = self.w2[:]

            for i in range(NN_HIDDEN):
                self.w2[i] -= NN_LEARNING_RATE * output_grad * hidden[i]
            self.b2 -= NN_LEARNING_RATE * output_grad

            for i in range(NN_HIDDEN):
                hidden_grad = output_grad * old_w2[i] * (1.0 - hidden[i] ** 2)
                offset = i * NN_INPUT
                for j in range(NN_INPUT):
                    self.w1[offset + j] -= (
                        NN_LEARNING_RATE * hidden_grad * state[j]
                    )
                self.b1[i] -= NN_LEARNING_RATE * hidden_grad
            total_loss += error * error

        self.train_steps += 1
        self.loss = total_loss / NN_BATCH
        return self.loss


class NeuralSwingController(core.SwingAndBalanceController):
    def __init__(self, strategy=core.DEFAULT_STRATEGY):
        self.nn = TinyNN()
        self.use_nn = True
        self.nn_blend = 0.0
        self.last_teacher_pwm = 0.0
        self.last_nn_pwm = 0.0
        self.control_steps = 0
        super().__init__(strategy=strategy, enable_balance=True)

    @staticmethod
    def normalized_state(sim):
        phi = core.wrap_to_pi(sim.theta - core.PI)
        return [
            clamp(phi / core.TOP_EXIT, -2.0, 2.0),
            clamp(sim.theta_dot / 8.0, -2.0, 2.0),
            clamp(sim.alpha / core.ARM_SOFT_LIMIT, -2.0, 2.0),
            clamp(sim.alpha_dot / core.PWM_SPEED_CURVE[-1][1], -2.0, 2.0),
        ]

    def command(self, sim):
        teacher_pwm = super().command(sim)
        if self.mode != "BALANCE":
            self.nn_blend = 0.0
            return teacher_pwm

        state = self.normalized_state(sim)
        self.last_teacher_pwm = teacher_pwm
        self.nn.store(state, teacher_pwm)
        self.control_steps += 1
        if self.control_steps % 10 == 0:
            self.nn.train_batch()

        self.last_nn_pwm = self.nn.command(state)
        if not self.use_nn or len(self.nn.states) < NN_WARMUP:
            self.nn_blend = 0.0
        else:
            learned = len(self.nn.states) - NN_WARMUP
            self.nn_blend = 0.80 * clamp(
                learned / (NN_FULL_BLEND - NN_WARMUP), 0.0, 1.0
            )
        return (
            (1.0 - self.nn_blend) * teacher_pwm
            + self.nn_blend * self.last_nn_pwm
        )

    def reset_network(self):
        self.nn.reset()
        self.nn_blend = 0.0
        self.control_steps = 0


class NeuralApp(core.App):
    def __init__(self, root, strategy=core.DEFAULT_STRATEGY):
        super().__init__(root, strategy=strategy)
        self.root.title("Measured Furuta - Neural Balance")
        self.controller = NeuralSwingController(strategy)

        neural_bar = tk.Frame(root)
        neural_bar.pack(fill="x", padx=10, pady=(4, 0))
        tk.Button(neural_bar, text="NN ON / OFF", command=self.toggle_nn).pack(
            side=tk.LEFT, padx=3
        )
        tk.Button(neural_bar, text="Reset NN", command=self.reset_nn).pack(
            side=tk.LEFT, padx=3
        )

    def reset(self):
        self.sim.reset()
        self.controller.reset()

    def near_top(self):
        self.sim.set_near_top()
        self.controller.reset()
        self.controller.mode = "BALANCE"

    def toggle_nn(self):
        self.controller.use_nn = not self.controller.use_nn

    def reset_nn(self):
        self.controller.reset_network()

    def draw(self):
        super().draw()
        network = self.controller.nn
        self.info.set(
            self.info.get()
            + f"\nNN={'ON' if self.controller.use_nn else 'OFF'} "
            + f"blend={self.controller.nn_blend:.2f} samples={len(network.states)} "
            + f"loss={network.loss:.5f} teacher={100*self.controller.last_teacher_pwm:.1f}% "
            + f"nn={100*self.controller.last_nn_pwm:.1f}%"
        )


def run_headless(strategy=core.DEFAULT_STRATEGY, duration=12.0):
    sim = core.FurutaPendulum()
    controller = NeuralSwingController(strategy)
    max_arm = 0.0
    for _ in range(int(duration / sim.dt)):
        sim.step(controller.command(sim))
        max_arm = max(max_arm, abs(sim.alpha))
    phi = core.wrap_to_pi(sim.theta - core.PI)
    print(
        f"mode={controller.mode} capture={controller.capture_time} "
        f"phi={math.degrees(phi):.2f}deg theta_dot={sim.theta_dot:.2f}rad/s "
        f"alpha={math.degrees(sim.alpha):.2f}deg "
        f"max_alpha={math.degrees(max_arm):.2f}deg "
        f"hard_limit={sim.hard_limit_hit} nn_samples={len(controller.nn.states)} "
        f"nn_loss={controller.nn.loss:.6f}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Measured Furuta model with neural balance imitation"
    )
    parser.add_argument("--search", action="store_true")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    if args.search:
        results = core.find_best_strategy()
        for index, result in enumerate(results[:10], 1):
            strategy = result.strategy
            print(
                f"{index:2d}: score={result.score:.4f} caught={result.caught} "
                f"top={result.best_top_deg:.2f}deg "
                f"theta_dot={result.best_top_speed:.2f}rad/s "
                f"alpha={result.best_arm_deg:.1f}deg "
                f"alpha_dot={result.best_arm_speed:.2f}rad/s "
                f"max_arm={result.max_arm_deg:.1f}deg "
                f"f={strategy.start_hz:.2f}->{strategy.end_hz:.2f}Hz "
                f"A={strategy.arm_amplitude_deg:.0f}deg "
                f"phase={strategy.phase_deg:.0f}deg"
            )
        return

    if args.headless:
        run_headless()
        return

    root = tk.Tk()
    NeuralApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
