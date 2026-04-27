/**
 * @file roll_pitch.h
 * @brief Tính toán góc Roll và Pitch từ MPU6050
 *
 * Sử dụng Complementary Filter kết hợp:
 * - Accelerometer: cho góc tuyệt đối (nhiễu khi rung)
 * - Gyroscope: cho giá trị ổn định (drift theo thời gian)
 *
 * Công thức: angle = α * (angle_prev + gyro*dt) + (1-α) * accel_angle
 * với α = 0.80 (80% gyro, 20% accel)
 */

#ifndef ROLL_PITCH_H
#define ROLL_PITCH_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "mpu6050_constants.h"

// Khởi tạo góc về 0
void roll_pitch_init(void);

// Cập nhật góc với dữ liệu cảm biến mới
void roll_pitch_update(float accel_x, float accel_y, float accel_z,
                      float gyro_x, float gyro_y, float gyro_z);

// Lấy giá trị góc
float get_roll(void);  // Góc nghiêng trái/phải (quanh trục X)
float get_pitch(void); // Góc nghiêng tiến/lùi (quanh trục Y)

#endif
