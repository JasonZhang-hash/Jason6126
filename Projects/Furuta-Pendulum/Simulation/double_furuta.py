import math
import tkinter as tk

PI = math.pi
TWO_PI = 2.0 * PI

try:
    import numpy as np
except Exception:  # pragma: no cover - fallback for very small Python envs
    np = None

try:
    from scipy.linalg import solve_continuous_are
except Exception:  # pragma: no cover
    solve_continuous_are = None


# ============================================================
#  Tunable parameters
# ============================================================

# Geometry and mass.  theta1/theta2 are absolute angles from downward.
ARM_L = 0.15
J_ARM = 0.0020
B_ARM = 0.010

# First (12 cm) hollow aluminium pendulum rod: estimated 6 g.
M1 = 0.006
L1 = 0.12
B1 = 0.0007

M2 = 0.010
L2 = 0.20
B2 = 0.0007

G = 9.81
KGCM_TO_NM = 0.0980665

# Selected motor from Jason's 12 V JGA25-370 table: 35:1 encoder version.
# Peak values describe the voltage-limited torque-speed curve; rated values
# are the continuous-duty reference, not another hard torque clip.
MOTOR_NAME = "JGA25-370 12 V, 35:1 encoder"
MOTOR_VOLTAGE = 12.0
MOTOR_GEAR_RATIO = 35.0
MOTOR_NOLOAD_RPM = 170.0
MOTOR_NOLOAD_CURRENT = 0.06
MOTOR_RATED_RPM = 130.0
MOTOR_RATED_CURRENT = 0.45
MOTOR_RATED_TORQUE = 0.8 * KGCM_TO_NM
MOTOR_STALL_CURRENT = 1.3
MOTOR_STALL_TORQUE = 2.8 * KGCM_TO_NM
MOTOR_NOLOAD_SPEED = MOTOR_NOLOAD_RPM * TWO_PI / 60.0
# Approximate electrical/current response.  The seller table does not give L/R.
MOTOR_ELECTRICAL_TAU = 0.004
T_MAX = MOTOR_STALL_TORQUE
DT = 0.001

L1C = L1 / 2.0
L2C = L2 / 2.0
I1 = M1 * L1 * L1 / 12.0
I2 = M2 * L2 * L2 / 12.0

# Mode switching.
TOP_IN = 22.0
TOP_OUT = 45.0
SPD_IN = 14.0
CATCH_IN = 90.0
CATCH_OUT = 165.0
CATCH_TIME = 0.12
CATCH_TIMEOUT = 2.0
BALANCE_ENTRY_ANGLE = 0.8
BALANCE_ENTRY_SPEED = 1.2
SAFE_ENERGY_MULT = 9.0
SAFE_SPEED = 45.0

# Swing-up and frequency identification.
T_SWING = T_MAX
ENERGY_GAIN = 10.0
ENERGY_DEADBAND = 0.025
SWING_ALPHA_MIN = 0.35
SWING_ALPHA_MAX = 1.35

# Before adding energy, sweep the arm near the low passive mode.  This is a
# small-signal experiment, so it intentionally uses a modest arm amplitude.
FREQ_SCAN_WIDTH_HZ = 0.45
FREQ_SCAN_PERIOD_S = 14.0
FREQ_SCAN_ALPHA_AMP = 0.20

USE_LEARNED_SWINGUP = False
LEARNED_SWINGUP_WEIGHTS = [
    1.4125249128061532,
    0.14606836675618196,
    0.21455840145146848,
    0.19286309233959298,
    0.6540537987628147,
    1.299979625970118,
    0.34908582016173917,
    0.5277146714765004,
    -1.1221262722676155,
]

# Used only if SciPy is not available.
K_FALLBACK = (0.7, 0.20, 42.0, 5.0, 58.0, 5.8)

KP_A_SWING = 0.06
KD_A_SWING = 0.02

TARGET_SLEW = 0.8
STEPS_PER_FRAME = 8


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def sign_nonzero(x):
    return 1.0 if x >= 0.0 else -1.0


def wrap_to_pi(x):
    while x > PI:
        x -= TWO_PI
    while x < -PI:
        x += TWO_PI
    return x


def wrap_to_2pi(x):
    while x < 0.0:
        x += TWO_PI
    while x >= TWO_PI:
        x -= TWO_PI
    return x


def mat3_solve(a, b):
    if np is not None:
        return np.linalg.solve(np.array(a, dtype=float), np.array(b, dtype=float)).tolist()

    # Small Gaussian elimination fallback.
    m = [[float(a[i][j]) for j in range(3)] + [float(b[i])] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        div = m[col][col]
        if abs(div) < 1e-12:
            div = 1e-12 if div >= 0.0 else -1e-12
        for j in range(col, 4):
            m[col][j] /= div
        for r in range(3):
            if r == col:
                continue
            fac = m[r][col]
            for j in range(col, 4):
                m[r][j] -= fac * m[col][j]
    return [m[i][3] for i in range(3)]


# ============================================================
#  Double Furuta physics
# ============================================================

def mass_matrix(theta1, theta2):
    s1 = math.sin(theta1)
    s2 = math.sin(theta2)
    c1 = math.cos(theta1)
    c2 = math.cos(theta2)

    # The pendulums swing in the vertical plane tangent to the arm tip.
    # This creates the alpha-theta inertia terms a Furuta pendulum needs.
    y1 = L1C * s1
    y2 = L1 * s1 + L2C * s2

    m00 = J_ARM + M1 * (ARM_L * ARM_L + y1 * y1) + M2 * (ARM_L * ARM_L + y2 * y2)
    m01 = M1 * ARM_L * L1C * c1 + M2 * ARM_L * L1 * c1
    m02 = M2 * ARM_L * L2C * c2
    m11 = M1 * L1C * L1C + I1 + M2 * L1 * L1
    m22 = M2 * L2C * L2C + I2
    m12 = M2 * L1 * L2C * math.cos(theta1 - theta2)

    return [
        [m00, m01, m02],
        [m01, m11, m12],
        [m02, m12, m22],
    ]


def potential_grad(theta1, theta2):
    # dU/dtheta for U = -m*g*l*cos(theta).  Downward is stable.
    g1 = G * (M1 * L1C + M2 * L1) * math.sin(theta1)
    g2 = G * M2 * L2C * math.sin(theta2)
    return [0.0, g1, g2]


def coriolis_vector(q, qd):
    theta1 = q[1]
    theta2 = q[2]
    s1 = math.sin(theta1)
    c1 = math.cos(theta1)
    s2 = math.sin(theta2)
    c2 = math.cos(theta2)
    sd = math.sin(theta1 - theta2)

    y1 = L1C * s1
    y2 = L1 * s1 + L2C * s2
    a1 = M1 * ARM_L * L1C + M2 * ARM_L * L1
    a2 = M2 * ARM_L * L2C
    c12 = M2 * L1 * L2C

    d_m = [
        [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ]

    d_m[1][0][0] = 2.0 * M1 * y1 * L1C * c1 + 2.0 * M2 * y2 * L1 * c1
    d_m[1][0][1] = d_m[1][1][0] = -a1 * s1
    d_m[1][1][2] = d_m[1][2][1] = -c12 * sd

    d_m[2][0][0] = 2.0 * M2 * y2 * L2C * c2
    d_m[2][0][2] = d_m[2][2][0] = -a2 * s2
    d_m[2][1][2] = d_m[2][2][1] = c12 * sd

    c = [0.0, 0.0, 0.0]
    for i in range(3):
        total = 0.0
        for j in range(3):
            for k in range(3):
                gamma = 0.5 * (d_m[k][i][j] + d_m[j][i][k] - d_m[i][j][k])
                total += gamma * qd[j] * qd[k]
        c[i] = total
    return c


def rhs_state(state, tau):
    alpha, alpha_dot, p1, theta1_dot, p2, theta2_dot = state
    theta1 = PI + p1
    theta2 = PI + p2

    q = [alpha, theta1, theta2]
    qd = [alpha_dot, theta1_dot, theta2_dot]
    m = mass_matrix(theta1, theta2)
    c = coriolis_vector(q, qd)
    gu = potential_grad(theta1, theta2)

    forces = [
        tau - B_ARM * alpha_dot,
        -B1 * theta1_dot,
        -B2 * theta2_dot,
    ]
    right = [forces[i] - c[i] - gu[i] for i in range(3)]
    qdd = mat3_solve(m, right)

    return [
        alpha_dot,
        qdd[0],
        theta1_dot,
        qdd[1],
        theta2_dot,
        qdd[2],
    ]


def rk4_step(state, tau, dt):
    k1 = rhs_state(state, tau)
    k2 = rhs_state([state[i] + 0.5 * dt * k1[i] for i in range(6)], tau)
    k3 = rhs_state([state[i] + 0.5 * dt * k2[i] for i in range(6)], tau)
    k4 = rhs_state([state[i] + dt * k3[i] for i in range(6)], tau)
    return [state[i] + dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) for i in range(6)]


def mechanical_energy(alpha_dot, theta1, theta1_dot, theta2, theta2_dot):
    qd = [alpha_dot, theta1_dot, theta2_dot]
    m = mass_matrix(theta1, theta2)
    kinetic = 0.0
    for i in range(3):
        for j in range(3):
            kinetic += 0.5 * qd[i] * m[i][j] * qd[j]

    potential = G * (
        (M1 * L1C + M2 * L1) * (1.0 - math.cos(theta1))
        + M2 * L2C * (1.0 - math.cos(theta2))
    )
    return kinetic + potential


def target_energy():
    return 2.0 * G * (M1 * L1C + M2 * L1 + M2 * L2C)


def finite_difference_lqr():
    if np is None or solve_continuous_are is None:
        return None

    x0 = np.zeros(6)
    u0 = 0.0
    eps = 1e-5

    def f(x, u):
        return np.array(rhs_state(x.tolist(), float(u)), dtype=float)

    a = np.zeros((6, 6))
    for i in range(6):
        xp = x0.copy()
        xm = x0.copy()
        xp[i] += eps
        xm[i] -= eps
        a[:, i] = (f(xp, u0) - f(xm, u0)) / (2.0 * eps)

    b = ((f(x0, u0 + eps) - f(x0, u0 - eps)) / (2.0 * eps)).reshape((6, 1))

    q = np.diag([0.4, 0.12, 110.0, 3.0, 135.0, 3.5])
    r = np.array([[2.2]])

    try:
        p = solve_continuous_are(a, b, q, r)
        k = np.linalg.solve(r, b.T @ p)
        return k.reshape(6).tolist()
    except Exception:
        return None


K_BALANCE = finite_difference_lqr() or list(K_FALLBACK)


def passive_mode_frequencies():
    """Return non-zero downward-equilibrium modes of the free Furuta system."""
    # alpha has no restoring stiffness, hence one rigid-body zero mode.
    # The two remaining eigenvalues describe the coupled pendulum modes.
    if np is None:
        return (1.13, 3.18)

    m = np.array(mass_matrix(0.0, 0.0), dtype=float)
    k = np.zeros((3, 3), dtype=float)
    k[1, 1] = G * (M1 * L1C + M2 * L1)
    k[2, 2] = G * M2 * L2C
    values = np.linalg.eigvals(np.linalg.solve(m, k))
    frequencies = sorted(
        math.sqrt(value.real) / TWO_PI
        for value in values
        if abs(value.imag) < 1e-7 and value.real > 1e-8
    )
    return tuple(frequencies[:2]) if len(frequencies) >= 2 else (1.13, 3.18)


LOW_MODE_HZ, HIGH_MODE_HZ = passive_mode_frequencies()


# ============================================================
#  Simulation object
# ============================================================

class DoubleFuruta:
    def __init__(self):
        self.reset()

    def reset(self):
        self.a = 0.0
        self.ad = 0.0
        self.t1 = math.radians(2.0)
        self.t1d = 0.0
        self.t2 = math.radians(2.0)
        self.t2d = 0.0
        self.tq = 0.0
        self.motor_duty = 0.0
        self.motor_current = 0.0
        self.mode = "FREQ_SCAN"
        self.time = 0.0
        self.catch_time = 0.0
        self.catch_started_at = None
        self.stop_reason = ""
        self.a_tgt = 0.0
        self.a_ref = 0.0
        self.last_cmd = 0.0
        self.frequency_scan = True
        self.drive_freq_hz = LOW_MODE_HZ
        self.swing_phase = 0.0
        self.best_scan_response = 0.0
        self.best_scan_freq_hz = LOW_MODE_HZ

    def set_near_top(self):
        self.a = 0.0
        self.ad = 0.0
        # With the present 0.0735 N*m motor limit, balance is only local.
        self.t1 = PI + math.radians(0.5)
        self.t1d = 0.0
        self.t2 = PI - math.radians(0.4)
        self.t2d = 0.0
        self.tq = 0.0
        self.motor_duty = 0.0
        self.motor_current = 0.0
        self.mode = "BALANCE"
        self.time = 0.0
        self.catch_time = 0.0
        self.catch_started_at = None
        self.stop_reason = ""
        self.a_tgt = 0.0
        self.a_ref = 0.0
        self.last_cmd = 0.0
        self.frequency_scan = False
        self.drive_freq_hz = LOW_MODE_HZ
        self.swing_phase = 0.0
        self.swing_phase = 0.0

    def energy(self):
        return mechanical_energy(self.ad, self.t1, self.t1d, self.t2, self.t2d)

    def target_energy(self):
        return target_energy()

    def state_error(self):
        return [
            wrap_to_pi(self.a - self.a_ref),
            self.ad,
            wrap_to_pi(self.t1 - PI),
            self.t1d,
            wrap_to_pi(self.t2 - PI),
            self.t2d,
        ]

    def update_mode(self):
        if self.frequency_scan:
            self.mode = "FREQ_SCAN"
            return

        p1 = wrap_to_pi(self.t1 - PI)
        p2 = wrap_to_pi(self.t2 - PI)
        enter = math.radians(TOP_IN)
        exit_ = math.radians(TOP_OUT)
        catch_in = math.radians(CATCH_IN)
        catch_out = math.radians(CATCH_OUT)
        x = self.state_error()
        lqr_cmd = -sum(K_BALANCE[i] * x[i] for i in range(6))
        max_angle = max(abs(p1), abs(p2))
        max_speed = max(abs(self.t1d), abs(self.t2d))

        if self.energy() > SAFE_ENERGY_MULT * self.target_energy() or max_speed > SAFE_SPEED:
            self.mode = "SWING_UP"
            self.catch_time = 0.0
            self.catch_started_at = None
            self.stop_reason = "over-energy / over-speed"
            return

        if self.mode == "SWING_UP":
            self.catch_time = 0.0
            self.catch_started_at = None
            if max_angle > math.radians(150.0) and max_speed < 2.0:
                self.stop_reason = ""
            if max_angle < catch_in:
                self.mode = "CATCH"
                self.catch_started_at = self.time
        elif self.mode == "CATCH":
            if self.catch_started_at is not None and self.time - self.catch_started_at > CATCH_TIMEOUT:
                self.mode = "SWING_UP"
                self.catch_time = 0.0
                self.catch_started_at = None
                self.stop_reason = "catch timeout"
            elif max_angle > catch_out:
                self.mode = "SWING_UP"
                self.catch_time = 0.0
                self.catch_started_at = None
                self.stop_reason = "catch failed"
            elif (
                max_angle < math.radians(BALANCE_ENTRY_ANGLE)
                and max_speed < BALANCE_ENTRY_SPEED
                and abs(lqr_cmd) < 2.8 * T_MAX
            ):
                self.catch_time += DT
                if self.catch_time > CATCH_TIME:
                    self.mode = "BALANCE"
                    self.catch_started_at = None
            else:
                self.catch_time = max(0.0, self.catch_time - 0.5 * DT)
        elif max_angle > exit_:
            self.mode = "SWING_UP"
            self.catch_time = 0.0
            self.catch_started_at = None

    def balance_cmd(self):
        x = self.state_error()
        return -sum(K_BALANCE[i] * x[i] for i in range(6))

    def active_drive_frequency(self):
        if not self.frequency_scan:
            return self.drive_freq_hz

        # Triangle scan: low -> high -> low, without a frequency jump.
        progress = (self.time / FREQ_SCAN_PERIOD_S) % 1.0
        scan = 1.0 - 2.0 * abs(2.0 * progress - 1.0)
        return max(0.20, LOW_MODE_HZ + FREQ_SCAN_WIDTH_HZ * scan)

    def toggle_frequency_scan(self):
        if self.frequency_scan:
            self.drive_freq_hz = self.active_drive_frequency()
            self.frequency_scan = False
            self.mode = "SWING_UP"
        else:
            self.frequency_scan = True
            self.mode = "FREQ_SCAN"
            self.swing_phase = 0.0
            self.best_scan_response = 0.0
            self.best_scan_freq_hz = LOW_MODE_HZ

    def lock_best_scan_frequency(self):
        self.drive_freq_hz = self.best_scan_freq_hz
        self.frequency_scan = False
        self.mode = "SWING_UP"

    def change_drive_frequency(self, delta_hz):
        self.frequency_scan = False
        self.drive_freq_hz = clamp(self.drive_freq_hz + delta_hz, 0.20, 5.00)
        self.mode = "SWING_UP"

    def swingup_cmd(self):
        energy_err = self.target_energy() - self.energy()
        phase = math.sin(self.t1) * self.t1d + 0.65 * math.sin(self.t2) * self.t2d
        if abs(phase) < 0.02:
            phase = math.sin(self.t1) + 0.65 * math.sin(self.t2)

        if USE_LEARNED_SWINGUP:
            target = max(self.target_energy(), 1e-6)
            energy_feature = clamp(energy_err / target, -6.0, 6.0)
            phase_feature = clamp(phase / 25.0, -3.0, 3.0)
            features = [
                clamp(energy_feature * phase_feature, -6.0, 6.0),
                energy_feature,
                phase_feature,
                math.sin(self.t1),
                math.sin(self.t2),
                clamp(self.t1d / 25.0, -3.0, 3.0),
                clamp(self.t2d / 25.0, -3.0, 3.0),
                clamp(self.ad / 25.0, -3.0, 3.0),
                1.0,
            ]
            raw = sum(w * x for w, x in zip(LEARNED_SWINGUP_WEIGHTS, features))
            arm_center = -KP_A_SWING * wrap_to_pi(self.a - self.a_ref) - KD_A_SWING * self.ad
            return T_MAX * math.tanh(raw) + arm_center

        target = max(self.target_energy(), 1e-6)
        energy_ratio = clamp(self.energy() / target, 0.0, 2.2)
        energy_deficit = clamp(1.0 - energy_ratio, -1.2, 1.0)

        # Stage 1: find the low coupled mode with a clean, bounded sine sweep.
        # Stage 2: lock the observed resonance, then let energy feedback raise
        # the amplitude.  The two rods are not independently driven; the arm
        # excites collective modes of the complete mechanism.
        freq = self.active_drive_frequency()
        self.swing_phase = wrap_to_2pi(self.swing_phase + TWO_PI * freq * DT)
        if self.frequency_scan:
            amp = FREQ_SCAN_ALPHA_AMP
        else:
            amp = SWING_ALPHA_MIN + (SWING_ALPHA_MAX - SWING_ALPHA_MIN) * clamp(
                0.25 + abs(energy_deficit), 0.0, 1.0
            )

        alpha_ref = amp * math.sin(self.swing_phase)
        alpha_ref_dot = amp * TWO_PI * freq * math.cos(self.swing_phase)

        cmd = (
            0.75 * wrap_to_pi(alpha_ref - self.a)
            + 0.085 * (alpha_ref_dot - self.ad)
        )

        if not self.frequency_scan:
            # Add/dissipate energy only after the resonance is selected.
            phase1 = self.t1d * math.cos(self.t1)
            phase2 = self.t2d * math.cos(self.t2)
            cmd += 0.018 * energy_deficit * (0.65 * phase1 + 0.35 * phase2)

        p1 = abs(wrap_to_pi(self.t1 - PI))
        p2 = abs(wrap_to_pi(self.t2 - PI))
        max_angle = max(p1, p2)
        max_speed = max(abs(self.t1d), abs(self.t2d))
        near_top = max_angle < math.radians(70.0)
        if near_top:
            cmd += -0.018 * self.ad
            if max_speed > 9.0 or energy_err < 0.0:
                cmd *= 0.45

        return clamp(cmd, -T_MAX, T_MAX)

    def controller(self):
        err_ref = wrap_to_pi(self.a_tgt - self.a_ref)
        self.a_ref += clamp(err_ref, -TARGET_SLEW * DT, TARGET_SLEW * DT)
        self.update_mode()

        if self.mode == "BALANCE":
            cmd = self.balance_cmd()
            self.last_cmd = cmd
            return cmd

        if self.mode == "CATCH":
            cmd = self.balance_cmd()
            cmd += -0.22 * self.ad
            cmd = clamp(cmd, -T_MAX, T_MAX)
            self.last_cmd = cmd
            return cmd

        cmd = self.swingup_cmd()
        self.last_cmd = cmd
        return cmd

    def step(self):
        cmd = clamp(self.controller(), -T_MAX, T_MAX)
        self.motor_duty = clamp(cmd / MOTOR_STALL_TORQUE, -1.0, 1.0)

        # Ideal H-bridge / DC motor relation at the geared output shaft.
        # This intentionally does not clamp speed: back EMF reduces torque at
        # high speed and reverses it when the mechanism back-drives the motor.
        back_emf_fraction = self.ad / MOTOR_NOLOAD_SPEED
        torque_target = MOTOR_STALL_TORQUE * (self.motor_duty - back_emf_fraction)
        torque_target = clamp(torque_target, -MOTOR_STALL_TORQUE, MOTOR_STALL_TORQUE)
        self.tq += (torque_target - self.tq) * DT / MOTOR_ELECTRICAL_TAU
        tau = self.tq

        current_fraction = abs(tau) / MOTOR_STALL_TORQUE
        if current_fraction < 1e-4:
            self.motor_current = 0.0
        else:
            self.motor_current = sign_nonzero(tau) * (
                MOTOR_NOLOAD_CURRENT
                + (MOTOR_STALL_CURRENT - MOTOR_NOLOAD_CURRENT) * current_fraction
            )

        state = [
            self.a,
            self.ad,
            wrap_to_pi(self.t1 - PI),
            self.t1d,
            wrap_to_pi(self.t2 - PI),
            self.t2d,
        ]
        state = rk4_step(state, tau, DT)

        self.a = wrap_to_pi(state[0])
        self.ad = state[1]
        self.t1 = wrap_to_2pi(PI + state[2])
        self.t1d = state[3]
        self.t2 = wrap_to_2pi(PI + state[4])
        self.t2d = state[5]
        self.time += DT

        if self.frequency_scan:
            response = max(
                abs(wrap_to_pi(self.t1)),
                abs(wrap_to_pi(self.t2)),
            )
            if response > self.best_scan_response:
                self.best_scan_response = response
                self.best_scan_freq_hz = self.active_drive_frequency()


# ============================================================
#  GUI
# ============================================================

class App:
    def __init__(self, root):
        self.sim = DoubleFuruta()
        self.root = root
        self.root.title("Double Furuta Pendulum")
        self.running = True

        self.canvas = tk.Canvas(root, width=840, height=600, bg="#fbfaf7")
        self.canvas.grid(row=0, column=0, columnspan=7, padx=10, pady=10)

        bar = tk.Frame(root)
        bar.grid(row=1, column=0, columnspan=7, sticky="ew", padx=6, pady=(0, 4))
        tk.Button(bar, text="Pause / Run", command=self.toggle).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Reset bottom", command=self.sim.reset).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Near top", command=self.sim.set_near_top).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Stop", command=self.stop_motion).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="+Swing", command=self.more_swing).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="-Swing", command=self.less_swing).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Freq -", command=lambda: self.sim.change_drive_frequency(-0.05)).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Freq +", command=lambda: self.sim.change_drive_frequency(0.05)).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Scan / Lock", command=self.sim.toggle_frequency_scan).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Lock best", command=self.sim.lock_best_scan_frequency).pack(side=tk.LEFT, padx=3)
        tk.Button(bar, text="Zero", command=self.zero_target).pack(side=tk.LEFT, padx=3)

        tk.Label(bar, text="  Target alpha (deg):").pack(side=tk.LEFT)
        self.tgt_var = tk.StringVar(value="0")
        self.tgt_entry = tk.Entry(bar, textvariable=self.tgt_var, width=6, font=("Consolas", 11), justify="center")
        self.tgt_entry.pack(side=tk.LEFT, padx=2)
        self.tgt_entry.bind("<Return>", self.set_target)
        tk.Button(bar, text="Go", command=self.set_target).pack(side=tk.LEFT, padx=3)

        self.info = tk.StringVar()
        tk.Label(root, textvariable=self.info, font=("Consolas", 11), justify="left").grid(
            row=2, column=0, columnspan=7, sticky="w", padx=10, pady=6
        )
        self.root.after(16, self.update)

    def set_target(self, event=None):
        try:
            self.sim.a_tgt = math.radians(float(self.tgt_var.get()))
        except ValueError:
            pass

    def zero_target(self):
        self.tgt_var.set("0")
        self.set_target()

    def toggle(self):
        self.running = not self.running

    def stop_motion(self):
        self.running = False
        self.sim.tq = 0.0
        self.sim.last_cmd = 0.0
        self.sim.stop_reason = "manual pause"

    def more_swing(self):
        global T_SWING
        T_SWING = min(T_SWING + 0.03, T_MAX)

    def less_swing(self):
        global T_SWING
        T_SWING = max(T_SWING - 0.01, 0.01)

    def update(self):
        if self.running:
            for _ in range(STEPS_PER_FRAME):
                self.sim.step()
        self.draw()
        self.root.after(16, self.update)

    def draw(self):
        self.canvas.delete("all")
        w, h = 840, 600
        s = self.sim
        sc = 520.0

        # Left: serial pendulum side view.
        sx, sy = w // 4, h // 2 + 20
        l1_px = L1 * sc
        l2_px = L2 * sc
        self.canvas.create_line(sx, sy - 155, sx, sy + 40, fill="#d0c8b8", width=2, dash=(5, 4))
        self.canvas.create_line(sx - 75, sy, sx + 75, sy, fill="#e0d8cc", width=2)

        e1x = sx + l1_px * math.sin(s.t1)
        e1y = sy - l1_px * math.cos(s.t1)
        e2x = e1x + l2_px * math.sin(s.t2)
        e2y = e1y - l2_px * math.cos(s.t2)

        self.canvas.create_line(sx, sy, e1x, e1y, fill="#2e8b57", width=8, capstyle=tk.ROUND)
        self.canvas.create_line(e1x, e1y, e2x, e2y, fill="#3498db", width=6, capstyle=tk.ROUND)
        self.canvas.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill="#333", outline="")
        self.canvas.create_oval(e1x - 7, e1y - 7, e1x + 7, e1y + 7, fill="#e67e22", outline="")
        self.canvas.create_oval(e2x - 6, e2y - 6, e2x + 6, e2y + 6, fill="#e74c3c", outline="")
        self.canvas.create_text(sx, 25, text="side view: double pendulum", fill="#888", font=("Consolas", 10))

        # Right: arm top-down view.
        cx, cy = w * 3 // 4, h // 2
        arm_px = ARM_L * sc
        tip_x = cx + arm_px * math.cos(s.a)
        tip_y = cy - arm_px * math.sin(s.a)
        tgt_x = cx + (arm_px + 30) * math.cos(s.a_tgt)
        tgt_y = cy - (arm_px + 30) * math.sin(s.a_tgt)

        self.canvas.create_line(cx, cy, tgt_x, tgt_y, fill="#2ecc71", width=2, dash=(6, 4))
        self.canvas.create_oval(cx - arm_px - 35, cy - arm_px - 35, cx + arm_px + 35, cy + arm_px + 35, outline="#e0d8cc")
        self.canvas.create_oval(cx - 18, cy - 18, cx + 18, cy + 18, fill="#333", outline="#111")
        self.canvas.create_line(cx, cy, tip_x, tip_y, fill="#b23a2e", width=8, capstyle=tk.ROUND)
        self.canvas.create_oval(tip_x - 6, tip_y - 6, tip_x + 6, tip_y + 6, fill="#666", outline="")
        self.canvas.create_text(cx, 25, text="top-down: driven arm", fill="#888", font=("Consolas", 10))

        if s.mode == "BALANCE":
            badge_color = "#27ae60"
        elif s.mode == "CATCH":
            badge_color = "#8e44ad"
        elif s.mode == "FREQ_SCAN":
            badge_color = "#2874a6"
        else:
            badge_color = "#e67e22"
        self.canvas.create_rectangle(w - 120, 8, w - 10, 32, fill=badge_color, outline="")
        self.canvas.create_text(w - 65, 20, text=s.mode, fill="white", font=("Consolas", 10, "bold"))

        p1 = wrap_to_pi(s.t1 - PI)
        p2 = wrap_to_pi(s.t2 - PI)
        drive_freq = s.active_drive_frequency()
        freq_state = "scan" if s.frequency_scan else "locked"
        self.info.set(
            f"mode={s.mode}   t={s.time:6.2f}s   torque={s.tq: .3f}   raw_cmd={s.last_cmd: .3f}\n"
            f"motor=35:1  pwm={s.motor_duty:+.2f}  current={s.motor_current:+.2f} A  "
            f"rated={MOTOR_RATED_TORQUE / KGCM_TO_NM:.2f} kgcm  peak={MOTOR_STALL_TORQUE / KGCM_TO_NM:.2f} kgcm\n"
            f"alpha={math.degrees(s.a):7.2f} deg   alpha_dot={s.ad:7.2f}   target={math.degrees(s.a_tgt):.1f} deg\n"
            f"theta1={math.degrees(s.t1):7.2f} deg   phi1={math.degrees(p1):7.2f} deg   theta1_dot={s.t1d:7.2f}\n"
            f"theta2={math.degrees(s.t2):7.2f} deg   phi2={math.degrees(p2):7.2f} deg   theta2_dot={s.t2d:7.2f}\n"
            f"f_drive={drive_freq:.3f} Hz ({freq_state})   f_mode=[{LOW_MODE_HZ:.3f}, {HIGH_MODE_HZ:.3f}] Hz\n"
            f"scan_peak={math.degrees(s.best_scan_response):.1f} deg @ {s.best_scan_freq_hz:.3f} Hz\n"
            f"E={s.energy():.4f}   E_target={s.target_energy():.4f}   swing={T_SWING:.2f}   "
            f"stop={s.stop_reason or '-'}   K={[round(v, 2) for v in K_BALANCE]}"
        )


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
