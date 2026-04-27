/**
 * @file roll_pitch.c
 * @brief Triển khai tính toán góc Roll/Pitch bằng Complementary Filter
 */

#include "roll_pitch.h"
#include "mpu6050_constants.h"
#include <math.h>
#include "esp_timer.h"

static float roll = 0.0f;   // Góc nghiêng trái/phải (X)
static float pitch = 0.0f;  // Góc nghiêng tiến/lùi (Y)
static int64_t prev_timestamp_us = 0;  // Timestamp trước đó (tính dt)

void roll_pitch_init(void) {
    roll = 0.0f;
    pitch = 0.0f;
    prev_timestamp_us = 0;
}

void roll_pitch_update(float accel_x, float accel_y, float accel_z,
                       float gyro_x, float gyro_y, float gyro_z) {
    // ========== Tính dt (delta time) ==========
    int64_t current_timestamp_us = esp_timer_get_time();
    float dt;
    if (prev_timestamp_us == 0) {
        dt = MPU6050_DT;  // Lần đầu: dùng 10ms
    } else {
        dt = (current_timestamp_us - prev_timestamp_us) / 1000000.0f;
        if (dt < 0.001f) dt = 0.001f;  // Min 1ms
        if (dt > 0.050f) dt = 0.050f;  // Max 50ms
    }
    prev_timestamp_us = current_timestamp_us;

    // ========== Bước 1: Tính góc từ Accelerometer ==========
    // Roll = atan2(ay, az) → nghiêng trái/phải
    float accel_roll = atan2f(-accel_y, accel_z) * 180.0f / M_PI - 90.0f;
    // Pitch = atan2(-ax, sqrt(ay²+az²)) → nghiêng tiến/lùi
    float accel_pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;

    // ========== Bước 2: Tích phân Gyroscope ==========
    roll += gyro_x * dt;    // roll += gyro_X * dt
    pitch += gyro_y * dt;   // pitch += gyro_Y * dt

    // ========== Bước 3: Complementary Filter ==========
    // Kết hợp gyro (ổn định) + accel (tuyệt đối)
    roll = ROLL_PITCH_ALPHA * roll + (1.0f - ROLL_PITCH_ALPHA) * accel_roll;
    pitch = ROLL_PITCH_ALPHA * pitch + (1.0f - ROLL_PITCH_ALPHA) * accel_pitch;
}

float get_roll(void) {
    return roll;
}

float get_pitch(void) {
    return pitch;
}
