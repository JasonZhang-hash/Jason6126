/**
 * furuta_swingup_stm32.c — Furuta pendulum energy swing-up for STM32
 *
 * Hardware:
 *   AS5600 (I2C 0x36, PB10/PB11) — pendulum angle
 *   Motor encoder TIM4 CH1/CH2 (PB6/PB7) — arm angle, 844.8 counts/rev (4x)
 *   TB6612 motor driver — PA0 PWM, PA5/PA6 dir, PA7 enable
 *
 * Usage: call furuta_swingup_init() once, then furuta_swingup_step() at 1 kHz.
 *        Target: swing pendulum to within 15° of upright.
 */

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ── Hardware pin definitions ──────────────────────────────── */
#define AS5600_I2C_ADDR        0x36
#define AS5600_REG_HIGH        0x0E
#define AS5600_REG_LOW         0x0F

/* Software I2C on PB10 (SDA) / PB11 (SCL) */
#define I2C_SDA_PORT           GPIOB
#define I2C_SDA_PIN            GPIO_PIN_10
#define I2C_SCL_PORT           GPIOB
#define I2C_SCL_PIN            GPIO_PIN_11

#define DIR_PORT               GPIOA
#define DIR_PIN1               GPIO_PIN_5    // AIN1
#define DIR_PIN2               GPIO_PIN_6    // AIN2
#define EN_PORT                GPIOA
#define EN_PIN                 GPIO_PIN_7    // STBY

#define PWM_TIM                &htim2
#define PWM_CHANNEL            TIM_CHANNEL_1

#define ENCODER_TIM            &htim4

/* ── Physical constants (verified 2026-08-06) ───────────────── */
#define PEND_MASS              0.016f        // kg, 6g rod + 10g weight
#define PEND_COM               0.151f        // m, effective CoM from free-decay
#define ARM_LENGTH             0.160f        // m, horizontal arm
#define ARM_INERTIA            0.0030f       // kg·m², arm + reflected motor
#define ARM_DAMPING            0.0015f       // N·m·s/rad
#define PEND_DAMPING           0.00030f      // N·m·s/rad, ζ≈0.05
#define GRAVITY                9.80665f

#define MAX_TORQUE             0.074f        // N·m, stall torque (0.75 kg·cm)
#define MOTOR_TAU              0.120f        // s, motor response lag

#define DT                     0.001f        // s, 1 kHz control loop

#define PI_F                   3.14159265f
#define TWO_PI_F               6.2831853f
#define DEG_TO_RAD             0.017453292f
#define RAD_TO_DEG             57.29578f

/* ── AS5600 calibration ────────────────────────────────────── */
#define AS5600_RESOLUTION      4096          // 12-bit
#define AS5600_ZERO_RAW        1242          // raw count at vertical-down (calibrated 2026-08-06) (calibrated 2026-08-06)
#define AS5600_UPRIGHT_RAW     ((AS5600_ZERO_RAW + 2048) % AS5600_RESOLUTION)  // ≈1202

/* ── Motor / encoder ───────────────────────────────────────── */
#define ENCODER_CPR            844.8f        // counts per rev after 4x quad
#define PWM_DEADBAND           60            // below 6% duty the motor won't move
#define PWM_MAX                260           // 26% = max measured operating point

/* ── Swing-up control ──────────────────────────────────────── */
#define STATIC_THRESHOLD       0.10f         // rad/s, below this velocity use static kick
#define SWING_PWM              260           // max pumping PWM duty (26%)
#define ENERGY_BAND            0.20f         // taper PWM as energy nears target
#define TOP_ENTER_DEG          15.0f         // target: reach within this of upright

/* ── Derived ───────────────────────────────────────────────── */
#define E_TARGET   (2.0f * PEND_MASS * GRAVITY * PEND_COM)

/* ── State ─────────────────────────────────────────────────── */
static float g_theta;           // pendulum angle, rad, 0=down, PI=up
static float g_theta_prev;
static float g_theta_dot;       // rad/s, filtered
static float g_alpha;           // arm angle, rad  (monitor only during swing-up)
static float g_energy;          // pendulum mechanical energy, J
static float g_energy_err;      // E_target - E
static int32_t g_encoder_count; // live TIM4 counter

static float g_theta_dot_filt;  // simple low-pass filtered theta_dot
static float g_alpha_dot_filt;

static bool  g_caught;          // true when phi <= 15° (upright captured)
static float g_best_phi_deg;

/* ── Forward declarations (hardware-specific, implement per your HAL) ── */
static uint16_t as5600_read_raw(void);
static void motor_set_raw(int16_t pwm);  // negative = reverse direction via AIN1/AIN2
static float wrap_to_pi(float rad);
static float wrap_0_2pi(float rad);

/* ============================================================
 *  Public API
 * ============================================================ */

void furuta_swingup_init(void) {
    g_theta          = 0.0f;
    g_theta_prev     = 0.0f;
    g_theta_dot      = 0.0f;
    g_theta_dot_filt = 0.0f;
    g_alpha          = 0.0f;
    g_alpha_dot_filt = 0.0f;
    g_energy         = 0.0f;
    g_energy_err     = 0.0f;
    g_encoder_count  = 0;
    g_caught         = false;
    g_best_phi_deg   = 180.0f;

    /* ── Read first AS5600 sample ── */
    uint16_t raw = as5600_read_raw();
    float raw_rad = (float)(int32_t)(raw - AS5600_ZERO_RAW) * TWO_PI_F / (float)AS5600_RESOLUTION;
    g_theta_prev = wrap_0_2pi(raw_rad);
    g_theta = g_theta_prev;

    /* ── Reset encoder ── */
    __HAL_TIM_SET_COUNTER(ENCODER_TIM, 0);

    /* ── Enable motor driver ── */
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_SET);
    /* ── Start PWM ── */
    HAL_TIM_PWM_Start(PWM_TIM, PWM_CHANNEL);

    /* ── Start encoder ── */
    HAL_TIM_Encoder_Start(ENCODER_TIM, TIM_CHANNEL_ALL);
}

/**
 * Run one control step. Call at 1 kHz.
 * Returns: 0 = swinging, 1 = captured (phi <= 15°).
 */
int furuta_swingup_step(void) {
    /* ── Read pendulum angle (AS5600) ── */
    uint16_t raw = as5600_read_raw();
    float theta_raw_rad = (float)(int32_t)(raw - AS5600_ZERO_RAW)
                          * TWO_PI_F / (float)AS5600_RESOLUTION;
    g_theta = wrap_0_2pi(theta_raw_rad);

    /* ── Pendulum velocity (differentiate + simple LPF) ── */
    float diff = (g_theta - g_theta_prev) / DT;
    /* handle wrap-around in angle difference */
    if (diff > PI_F)       diff -= TWO_PI_F;
    else if (diff < -PI_F) diff += TWO_PI_F;
    g_theta_dot_filt = 0.9f * g_theta_dot_filt + 0.1f * diff;
    g_theta_dot = g_theta_dot_filt;
    g_theta_prev = g_theta;

    /* ── Arm angle (encoder, monitor only) ── */
    int32_t enc = (int32_t)__HAL_TIM_GET_COUNTER(ENCODER_TIM);
    g_alpha = (float)enc * TWO_PI_F / ENCODER_CPR;

    /* ── Energy computation ── */
    float td = g_theta_dot;
    float cth = cosf(g_theta);
    float v = 1.0f - cth;               /* 1 - cos(theta) */
    g_energy    = 0.5f * PEND_MASS * PEND_COM * PEND_COM * td * td
                + PEND_MASS * GRAVITY * PEND_COM * v;
    g_energy_err = E_TARGET - g_energy;

    /* ── Check capture condition ── */
    float phi = wrap_to_pi(g_theta - PI_F);
    float phi_deg = fabsf(phi) * RAD_TO_DEG;
    if (phi_deg < g_best_phi_deg) g_best_phi_deg = phi_deg;

    if (phi_deg <= TOP_ENTER_DEG && g_caught == false) {
        g_caught = true;
        motor_set_raw(0);   /* captured — stop motor, user adds balance later */
        return 1;
    }

    if (g_caught) {
        motor_set_raw(0);
        return 1;
    }

    /* ── Swing-up control ── */
    int16_t pwm;
    float td_abs = fabsf(td);

    if (td_abs < STATIC_THRESHOLD) {
        /* stationary → static kick in the direction that starts a swing */
        float kick_dir = sinf(g_theta);
        pwm = (int16_t)(SWING_PWM * (kick_dir >= 0.0f ? 1.0f : -1.0f));
    } else {
        /* moving → energy pump: sign(td) * sign(E_err) * SWING_PWM * taper */
        float taper = fabsf(g_energy_err) / (ENERGY_BAND * E_TARGET);
        if (taper > 1.0f) taper = 1.0f;

        float pump_dir  = (td >= 0.0f ? 1.0f : -1.0f);       /* sign(td) */
        float err_dir   = (g_energy_err >= 0.0f ? 1.0f : -1.0f); /* sign(E_err) */
        float mag = SWING_PWM * taper;

        pwm = (int16_t)(mag * pump_dir * err_dir);
    }

    motor_set_raw(pwm);
    return 0;
}

/* ============================================================
 *  Hardware helpers (implement these for your HAL / board)
 * ============================================================ */

/**
 * Read AS5600 raw angle via software I2C on PB10 (SDA) / PB11 (SCL).
 * Returns 12-bit value 0..4095.
 *
 * Pin setup: PB10 and PB11 should be configured as GPIO_Output_OD (open-drain)
 * with external pull-ups to 3.3V (typically 4.7kΩ).  No internal pull-up needed.
 */
static void i2c_delay(void) {
    for (volatile int i = 0; i < 5; i++) { __NOP(); }  /* ~2.5 µs @ 72 MHz */
}

static void i2c_start(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    i2c_delay();
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    i2c_delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
}

static void i2c_stop(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    i2c_delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    i2c_delay();
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, (byte >> i) & 1);
        i2c_delay();
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        i2c_delay();
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    }
    /* Read ACK — release SDA, then read it during 9th clock */
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    i2c_delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    i2c_delay();
    uint8_t ack = HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) == GPIO_PIN_RESET;
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    return ack;
}

static uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t byte = 0;
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);  /* release SDA */
    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        i2c_delay();
        if (HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) == GPIO_PIN_SET)
            byte |= (1 << i);
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        i2c_delay();
    }
    /* Send ACK / NACK */
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
    i2c_delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    i2c_delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    return byte;
}

static uint16_t as5600_read_raw(void) {
    uint8_t addr_w = (AS5600_I2C_ADDR << 1) | 0;   /* write */
    uint8_t addr_r = (AS5600_I2C_ADDR << 1) | 1;   /* read  */

    i2c_start();
    i2c_write_byte(addr_w);
    i2c_write_byte(AS5600_REG_HIGH);
    i2c_start();                                    /* repeated start */
    i2c_write_byte(addr_r);
    uint8_t hi = i2c_read_byte(1);                  /* ACK after hi */
    uint8_t lo = i2c_read_byte(0);                  /* NACK after lo */
    i2c_stop();

    return ((uint16_t)hi << 8) | lo;
}

/**
 * Set motor PWM and direction.
 *  pwm > 0: CW  (AIN1=1, AIN2=0)
 *  pwm < 0: CCW (AIN1=0, AIN2=1)
 *  pwm = 0: coast or brake
 * PWM duty = |pwm| / 1000  (TIM2 ARR = 999 for 8 kHz with 8 MHz clock)
 */
static void motor_set_raw(int16_t pwm) {
    if (pwm > PWM_MAX)  pwm = PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    if (pwm == 0) {
        /* coast */
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CHANNEL, 0);
    } else if (pwm > 0) {
        /* CW */
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CHANNEL, (uint32_t)pwm);
    } else {
        /* CCW */
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CHANNEL, (uint32_t)(-pwm));
    }
}

/* ── Angle wrapping ────────────────────────────────────────── */
static float wrap_to_pi(float rad) {
    while (rad >  PI_F)   rad -= TWO_PI_F;
    while (rad < -PI_F)   rad += TWO_PI_F;
    return rad;
}

static float wrap_0_2pi(float rad) {
    while (rad < 0.0f)        rad += TWO_PI_F;
    while (rad >= TWO_PI_F)   rad -= TWO_PI_F;
    return rad;
}

/* ============================================================
 *  UART Telemetry (call from main loop or timer callback)
 * ============================================================ */

/* buffer for sprintf via UART — configure UART2 (PA2 TX / PA3 RX) at 115200 in CubeMX */
#define TELEM_UART_HANDLE  &huart2
#define TELEM_INTERVAL_MS  100     /* log every 100 ms */

static uint32_t g_telem_tick = 0;  /* incremented in 1 kHz ISR */

void furuta_telem_print(void) {
    float phi = wrap_to_pi(g_theta - PI_F);
    float phi_deg = phi * RAD_TO_DEG;
    float theta_deg = g_theta * RAD_TO_DEG;
    float alpha_deg = g_alpha * RAD_TO_DEG;
    float pwm_pct = 0.0f;
    /* read live PWM compare to get current duty */
    uint32_t comp = __HAL_TIM_GET_COMPARE(PWM_TIM, PWM_CHANNEL);
    if (comp > 0) pwm_pct = (float)comp / 10.0f;  /* ARR=999 → 100%=100.0 */

    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "t=%d phi=%.1f td=%.2f E=%.4f/%.4f pwm=%.0f%% best=%.1f a=%.1f\r\n",
        (int)g_telem_tick,
        phi_deg, g_theta_dot,
        g_energy, E_TARGET,
        pwm_pct, g_best_phi_deg,
        alpha_deg);
    HAL_UART_Transmit(TELEM_UART_HANDLE, (uint8_t *)buf, len, 10);
}

/* ============================================================
 *  Main loop example
 * ============================================================
 *
  // In main.c, set up a 1 kHz timer (TIM3) calling:
  void furuta_tick(void) {
      int done = furuta_swingup_step();
      g_telem_tick++;

      if (done) {
          furuta_telem_print();  /* final state */
          // pendulum within 15° of upright — stop motor, switch to balance
      }

      /* periodic telemetry */
      if (g_telem_tick % (TELEM_INTERVAL_MS) == 0) {
          furuta_telem_print();
      }
  }

  int main(void) {
      HAL_Init();
      SystemClock_Config();
      MX_GPIO_Init();
      MX_TIM2_Init();        // PWM on PA0, ARR=999, PSC=7 → 8 kHz
      MX_TIM4_Init();        // encoder mode on PB6/PB7
      MX_TIM3_Init();        // 1 kHz control timer
      MX_USART2_UART_Init(); // 115200 baud, PA2 TX / PA3 RX

      furuta_swingup_init();
      HAL_TIM_Base_Start_IT(&htim3);

      while (1) {
          /* telemetry runs inside TIM3 ISR, nothing to do here */
      }
  }
 */
