/**
 * @file mpu6050.h
 * @brief Driver cho cảm biến MPU6050 (Accelerometer + Gyroscope 6-axis)
 *
 * Kết nối I2C với ESP32. Đọc dữ liệu:
 * - Accelerometer: gia tốc 3 trục (m/s²)
 * - Gyroscope: tốc độ góc 3 trục (deg/s)
 *
 * Thông số cấu hình:
 * - Accelerometer: ±8g (4096 LSB/g)
 * - Gyroscope: ±2000 deg/s (16.4 LSB/deg/s)
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "mpu6050_constants.h"

// Địa chỉ I2C của MPU6050
#define MPU6050_ADDR             0x68

// Các thanh ghi quan trọng
#define MPU6050_REG_PWR_MGMT_1   0x6B    // Quản lý nguồn
#define MPU6050_REG_ACCEL_XOUT_H 0x3B    // Dữ liệu accelerometer
#define MPU6050_REG_GYRO_XOUT_H  0x43    // Dữ liệu gyroscope

// ========== KHỞI TẠO ==========
// Khởi tạo MPU6050: wake up, cấu hình ±8g và ±2000 deg/s
esp_err_t mpu6050_init(i2c_port_t i2c_num);

// ========== ĐỌC DỮ LIỆU ==========
// Đọc dữ liệu thô (raw) từ cảm biến
esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_num,
                                int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                                int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

// Chuyển đổi raw → m/s² (trừ bias)
void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *accel_x, float *accel_y, float *accel_z);

// Chuyển đổi raw → deg/s (trừ bias)
void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *gyro_x, float *gyro_y, float *gyro_z);

// ========== HIỆU CHUẨN ==========
// Phương thức 1: Blocking calibration (500 mẫu)
void mpu6050_calibrate(i2c_port_t i2c_num, float *accel_bias, float *gyro_bias);

// Phương thức 2: Two-phase calibration (gọi từng mẫu)
// Bước 1: Gọi calibrate_sample() N lần trong vòng lặp (LED blink)
// Bước 2: Gọi calibrate_finish() để hoàn thành
void mpu6050_calibrate_sample(i2c_port_t i2c_num);
void mpu6050_calibrate_finish(float *accel_bias_out, float *gyro_bias_out);

// ========== TÍNH TỔNG ==========
// Tổng độ lớn vector: A = √(x² + y² + z²)
float mpu6050_get_total_accel(float accel_x, float accel_y, float accel_z, float *accel_g);
float mpu6050_get_total_gyro(float gyro_x, float gyro_y, float gyro_z);

#endif
