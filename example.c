#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <mpu6050_i2c_lib.h>

// -----------------------------
// User wiring
// -----------------------------
#define ENCODER_A_PIN 18   // GP18
#define ENCODER_B_PIN 19   // GP19

// -----------------------------
// Tuning / constants
// -----------------------------
#define LOOP_HZ 100
#define LOOP_PERIOD_US (1000000u / LOOP_HZ)

#define ALPHA 0.05f
#define G 9.80665f
#define DEG2RAD 0.01745329252f
#define RAD2DEG 57.2957795f
#define PI 3.14159265359f

// Encoder mechanical conversion:
// If it's a wheel, velocity = omega * radius.
// If it’s just a knob, you can still print counts/sec instead.
#define COUNTS_PER_REV 24.0f     // typical when decoding all edges; adjust to YOUR encoder
#define WHEEL_RADIUS_M 0.01f     // meters; adjust or set 1.0f if not a wheel

// Which accel axis is "forward" for velocity from IMU (0=x,1=y,2=z)
#define FORWARD_AXIS 1

// Velocity fusion:
// v_est = (1-beta)*(v_est + a*dt) + beta*v_enc
// Higher beta trusts encoder more (more stable, less responsive)
#define VEL_FUSION_BETA 0.15f

// "Stopped" detection for ZUPT + bias learning
#define STOP_VEL_THRESH_MPS   0.02f
#define STOP_ACCEL_THRESH_MS2 0.25f
#define BIAS_LEARN_RATE       0.01f

// -----------------------------
// Encoder quadrature decoding
// -----------------------------
static volatile int32_t g_enc_count = 0;
static volatile uint8_t g_enc_state = 0;

// old_state<<2 | new_state (state is 2-bit: A<<1 | B)
static const int8_t QDEC_TABLE[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static inline uint8_t read_ab_state(void) {
    uint8_t a = gpio_get(ENCODER_A_PIN) ? 1u : 0u;
    uint8_t b = gpio_get(ENCODER_B_PIN) ? 1u : 0u;
    return (uint8_t)((a << 1) | b);
}

static void encoder_isr(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;

    uint8_t new_state = read_ab_state();
    uint8_t idx = (uint8_t)((g_enc_state << 2) | new_state);
    g_enc_count += QDEC_TABLE[idx];
    g_enc_state = new_state;
}

static void encoder_init(void) {
    gpio_init(ENCODER_A_PIN);
    gpio_init(ENCODER_B_PIN);

    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);

    // Safe default: encoder contacts usually short to GND -> pull-up needed
    gpio_pull_up(ENCODER_A_PIN);
    gpio_pull_up(ENCODER_B_PIN);

    g_enc_state = read_ab_state();

    // Install callback on A; enable interrupts on both pins
    gpio_set_irq_enabled_with_callback(
        ENCODER_A_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_isr
    );
    gpio_set_irq_enabled(
        ENCODER_B_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );
}

// -----------------------------
// IMU helpers
// -----------------------------
// raw accel -> m/s^2 assuming ±2g => 16384 LSB/g
static inline void accel_raw_to_ms2(const int16_t acc_raw[3], float acc_ms2[3]) {
    acc_ms2[0] = G * (float)acc_raw[0] / 16384.0f;
    acc_ms2[1] = G * (float)acc_raw[1] / 16384.0f;
    acc_ms2[2] = G * (float)acc_raw[2] / 16384.0f;
}

// Gravity vector in sensor frame from roll/pitch (common convention)
// If your axes differ, you might need to swap signs/axes or change FORWARD_AXIS.
static inline void gravity_from_roll_pitch(float roll_deg, float pitch_deg, float g_out[3]) {
    float roll  = roll_deg  * DEG2RAD;
    float pitch = pitch_deg * DEG2RAD;

    g_out[0] = -G * sinf(pitch);
    g_out[1] =  G * sinf(roll) * cosf(pitch);
    g_out[2] =  G * cosf(roll) * cosf(pitch);
}

// -----------------------------
// Main
// -----------------------------
int main(void) {
    stdio_init_all();
    sleep_ms(1500);

    encoder_init();

    // MPU init (as in your code)
    mpu6050_initialise(16, 17, 0, 0);
    mpu6050_reset();

    float roll_offset = 0.0f, pitch_offset = 0.0f;
    mpu6050_get_gyro_offset(10000, &roll_offset, &pitch_offset);

    // Initial roll/pitch from accelerometer
    int16_t acc_raw[3], gyro_raw[3], temp;
    mpu6050_read_raw(acc_raw, gyro_raw, &temp);

    float ax0 = (float)acc_raw[0];
    float ay0 = (float)acc_raw[1];
    float az0 = (float)acc_raw[2];

    float roll  = atan2f(ay0, az0) * RAD2DEG;
    float pitch = atan2f(-ax0, sqrtf(ay0*ay0 + az0*az0)) * RAD2DEG;

    // Velocity estimation state
    float v_est = 0.0f;       // fused velocity (m/s)
    float v_imu = 0.0f;       // IMU-only integrated velocity (m/s)
    float prev_a_fwd = 0.0f;  // previous forward linear accel (m/s^2)
    float a_bias = 0.0f;      // learned accel bias (m/s^2)

    // Encoder state
    int32_t prev_count = g_enc_count;

    // Timing via time_us_64 (robust across SDK versions)
    uint64_t last_us = time_us_64();
    uint64_t next_tick = last_us + LOOP_PERIOD_US;

    while (true) {
        // keep loop rate stable
        uint64_t now_us = time_us_64();
        int64_t sleep_for = (int64_t)(next_tick - now_us);
        if (sleep_for > 0) {
            sleep_us((uint32_t)sleep_for);
        }
        now_us = time_us_64();

        float dt = (float)(now_us - last_us) / 1e6f;
        if (dt <= 0.0f) dt = 1.0f / (float)LOOP_HZ;
        last_us = now_us;
        next_tick += LOOP_PERIOD_US;

        // --- Encoder velocity ---
        int32_t count = g_enc_count;
        int32_t dcount = count - prev_count;
        prev_count = count;

        float counts_per_sec = (dt > 0.0f) ? ((float)dcount / dt) : 0.0f;
        float omega = counts_per_sec * (2.0f * PI / COUNTS_PER_REV); // rad/s
        float v_enc = omega * WHEEL_RADIUS_M;                         // m/s

        // --- Orientation fusion (roll/pitch) ---
        int dt_ms = (int)(dt * 1000.0f + 0.5f);
        if (dt_ms < 1) dt_ms = 1;
        if (dt_ms > 50) dt_ms = 50; // sanity clamp
        mpu6050_fusion_output(roll_offset, pitch_offset, ALPHA, dt_ms, &roll, &pitch);

        // --- Accel read + gravity removal ---
        mpu6050_read_raw(acc_raw, gyro_raw, &temp);

        float acc_ms2[3];
        accel_raw_to_ms2(acc_raw, acc_ms2);

        float g_vec[3];
        gravity_from_roll_pitch(roll, pitch, g_vec);

        float a_lin[3] = {
            acc_ms2[0] - g_vec[0],
            acc_ms2[1] - g_vec[1],
            acc_ms2[2] - g_vec[2]
        };

        float a_fwd = a_lin[FORWARD_AXIS] - a_bias;

        // --- IMU integrate (trapezoid) ---
        v_imu += 0.5f * (a_fwd + prev_a_fwd) * dt;
        prev_a_fwd = a_fwd;

        // --- Fuse IMU + encoder velocity (complementary) ---
        float v_pred = v_est + a_fwd * dt;
        v_est = (1.0f - VEL_FUSION_BETA) * v_pred + VEL_FUSION_BETA * v_enc;

        // --- ZUPT + bias learning when stopped ---
        if (fabsf(v_enc) < STOP_VEL_THRESH_MPS && fabsf(a_fwd) < STOP_ACCEL_THRESH_MS2) {
            // hard pull velocity toward 0 when we believe we're stationary
            v_est *= 0.1f;
            v_imu *= 0.1f;

            // learn bias so linear accel trends to 0 when stopped
            a_bias = (1.0f - BIAS_LEARN_RATE) * a_bias + (BIAS_LEARN_RATE) * a_lin[FORWARD_AXIS];
        }

        // --- Output ---
        //printf("cnt=%ld d=%ld cps=%.1f v_enc=%.3f | a_fwd=%.3f bias=%.3f | v_imu=%.3f v_est=%.3f | roll=%.1f pitch=%.1f\n",
        //       (long)count, (long)dcount, counts_per_sec, v_enc,
        //       a_fwd, a_bias, v_imu, v_est, roll, pitch);
        
        static uint64_t t0_us = 0;
        if (t0_us == 0) t0_us = time_us_64();
        float t = (float)(time_us_64() - t0_us) / 1e6f;

        printf("%.3f,%.6f\n", t, v_est);
    }
}
