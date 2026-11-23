/**
 * furuta_diag_stm32.c — 30-second sensor diagnostic
 * Reads AS5600 (pendulum angle) and motor encoder (arm angle).
 * Green LED (PC13) on while running.  Logs every 100 ms via UART2.
 *
 * Wiring:
 *   AS5600: PB10 (SDA) / PB11 (SCL), software I2C
 *   Encoder: PB6 (TIM4_CH1) / PB7 (TIM4_CH2)
 *   LED:     PC13 (on-board, active-low)
 *   UART:    PA2 (TX) @ 115200 bps
 */

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ── Hardware pins ─────────────────────────────────────────── */
#define I2C_SDA_PORT           GPIOB
#define I2C_SDA_PIN            GPIO_PIN_10
#define I2C_SCL_PORT           GPIOB
#define I2C_SCL_PIN            GPIO_PIN_11

#define AS5600_I2C_ADDR        0x36
#define AS5600_REG_HIGH        0x0E
#define AS5600_REG_LOW         0x0F
#define AS5600_RESOLUTION      4096        /* 12-bit */
#define AS5600_ZERO_RAW        3250        /* calibrate: raw count at vertical-down */

#define ENCODER_TIM            &htim4
#define ENCODER_CPR            844.8f      /* counts/rev after 4x quadrature */

#define LED_PORT               GPIOC
#define LED_PIN                GPIO_PIN_13  /* on-board LED, active low */

#define TELEM_UART_HANDLE      &huart2
#define TELEM_INTERVAL_MS      100

#define TEST_DURATION_S        30
#define DT                     0.001f      /* 1 kHz loop */

#define PI_F                   3.14159265f
#define TWO_PI_F               6.2831853f
#define DEG_TO_RAD             0.017453292f
#define RAD_TO_DEG             57.29578f

/* ── State ─────────────────────────────────────────────────── */
static uint32_t g_tick;                    /* loop counter */
static float    g_theta;                   /* pendulum angle, rad (0=down, PI=up) */
static float    g_theta_dot;               /* rad/s */
static float    g_theta_prev;
static float    g_alpha;                   /* arm angle, rad */
static float    g_alpha_dot;               /* rad/s (from encoder) */
static int32_t  g_alpha_prev_enc;
static bool     g_running;

/* ============================================================
 *  Software I2C (PB10=SDA, PB11=SCL, open-drain + external pullup)
 * ============================================================ */

static void i2c_delay(void) {
    for (volatile int i = 0; i < 5; i++) { __NOP(); }
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
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        i2c_delay();
        if (HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) == GPIO_PIN_SET)
            byte |= (1 << i);
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        i2c_delay();
    }
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
    i2c_delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    i2c_delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    return byte;
}

static uint16_t as5600_read_raw(void) {
    uint8_t addr_w = (AS5600_I2C_ADDR << 1) | 0;
    uint8_t addr_r = (AS5600_I2C_ADDR << 1) | 1;
    i2c_start();
    i2c_write_byte(addr_w);
    i2c_write_byte(AS5600_REG_HIGH);
    i2c_start();
    i2c_write_byte(addr_r);
    uint8_t hi = i2c_read_byte(1);
    uint8_t lo = i2c_read_byte(0);
    i2c_stop();
    return ((uint16_t)hi << 8) | lo;
}

/* ============================================================
 *  Angle conversion
 * ============================================================ */

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

static float as5600_to_theta(uint16_t raw) {
    /* raw 0..4095 → theta 0..2π, where zero at AS5600_ZERO_RAW (vertical down) */
    float theta = (float)(int32_t)(raw - AS5600_ZERO_RAW) * TWO_PI_F / (float)AS5600_RESOLUTION;
    return wrap_0_2pi(theta);
}

/* ============================================================
 *  Public API
 * ============================================================ */

void furuta_diag_init(void) {
    g_tick          = 0;
    g_theta         = 0.0f;
    g_theta_prev    = 0.0f;
    g_theta_dot     = 0.0f;
    g_alpha         = 0.0f;
    g_alpha_dot     = 0.0f;
    g_alpha_prev_enc = 0;
    g_running       = true;

    /* ── Green LED on ── */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);  /* active-low */

    /* ── Prime AS5600 ── */
    uint16_t raw = as5600_read_raw();
    g_theta_prev = as5600_to_theta(raw);

    /* ── Reset encoder ── */
    __HAL_TIM_SET_COUNTER(ENCODER_TIM, 0);

    /* ── Start encoder ── */
    HAL_TIM_Encoder_Start(ENCODER_TIM, TIM_CHANNEL_ALL);

    /* ── Header ── */
    char header[] = "=== SENSOR DIAGNOSTIC ===\r\n"
                    "t(s)  theta_deg  td_rad/s  alpha_deg  ad_rad/s  as5600_raw\r\n";
    HAL_UART_Transmit(TELEM_UART_HANDLE, (uint8_t *)header, sizeof(header) - 1, 100);
}

void furuta_diag_step(void) {
    if (!g_running) return;
    g_tick++;

    /* ── AS5600 pendulum angle ── */
    uint16_t raw = as5600_read_raw();
    g_theta = as5600_to_theta(raw);

    /* theta_dot (differentiate + simple low-pass) */
    float diff = g_theta - g_theta_prev;
    if (diff > PI_F)       diff -= TWO_PI_F;
    else if (diff < -PI_F) diff += TWO_PI_F;
    g_theta_dot = 0.9f * g_theta_dot + 0.1f * (diff / DT);
    g_theta_prev = g_theta;

    /* ── Encoder arm angle ── */
    int32_t enc = (int32_t)__HAL_TIM_GET_COUNTER(ENCODER_TIM);
    g_alpha = (float)enc * TWO_PI_F / ENCODER_CPR;
    float alpha_diff = (float)(enc - g_alpha_prev_enc) * TWO_PI_F / ENCODER_CPR;
    g_alpha_dot = 0.9f * g_alpha_dot + 0.1f * (alpha_diff / DT);
    g_alpha_prev_enc = enc;

    /* ── Periodic log ── */
    if (g_tick % TELEM_INTERVAL_MS == 0) {
        float t_s = (float)g_tick * DT;
        float theta_deg = g_theta * RAD_TO_DEG;
        float alpha_deg = g_alpha * RAD_TO_DEG;
        char buf[100];
        int len = snprintf(buf, sizeof(buf),
            "%5.1f  %8.1f  %8.2f  %8.1f  %8.2f  %5u\r\n",
            t_s, theta_deg, g_theta_dot, alpha_deg, g_alpha_dot, raw);
        HAL_UART_Transmit(TELEM_UART_HANDLE, (uint8_t *)buf, len, 100);
    }

    /* ── Stop after 30 s ── */
    if (g_tick >= (uint32_t)(TEST_DURATION_S / DT)) {
        g_running = false;
        /* Green LED off */
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

        char footer[] = "=== DONE ===\r\n";
        HAL_UART_Transmit(TELEM_UART_HANDLE, (uint8_t *)footer, sizeof(footer) - 1, 100);
    }
}

/* ============================================================
 *  Main example
 * ============================================================
 *
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *       if (htim->Instance == TIM3) {
 *           furuta_diag_step();
 *       }
 *   }
 *
 *   int main(void) {
 *       HAL_Init();
 *       SystemClock_Config();
 *       MX_GPIO_Init();
 *       MX_TIM3_Init();          // 1 kHz control timer
 *       MX_TIM4_Init();          // encoder mode on PB6/PB7
 *       MX_USART2_UART_Init();   // 115200 baud, PA2 TX
 *
 *       furuta_diag_init();
 *       HAL_TIM_Base_Start_IT(&htim3);
 *
 *       while (1) { __WFI(); }
 *   }
 */
