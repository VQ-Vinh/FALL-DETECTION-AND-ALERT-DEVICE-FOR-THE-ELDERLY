/*
 * Header công khai của driver MPU6050 - định nghĩa API cho các module khác.
 * Chi tiết triển khai được giấu trong .c (tính đóng gói).
 *
 * Sơ đồ kết nối:
 *   ESP32: GPIO 21 (SCL), GPIO 22 (SDA), 3.3V, GND
 *   MPU6050: AD0 = GND → địa chỉ I2C 0x68
 *
 * Cách sử dụng:
 *   1. Khởi tạo I2C, gọi mpu6050_init()
 *   2. mpu6050_calibrate_sample() × N → mpu6050_calibrate_finish()
 *   3. Vòng lặp: read_raw → convert → roll_pitch_update()
 *
 * Cấu hình: ±8g (accel), ±2000 deg/s (gyro), 100Hz, I2C 0x68, 3.3V
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "mpu6050_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Địa chỉ I2C 7-bit (0x68). AD0 = GND. Nếu kéo AD0 lên HIGH sẽ thành 0x69. */
#define MPU6050_ADDR             0x68

/* Các thanh ghi quan trọng */
#define MPU6050_REG_PWR_MGMT_1   0x6B    /* Quản lý nguồn: bit 6 = SLEEP */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B    /* Byte cao đầu tiên của dữ liệu accel */
#define MPU6050_REG_GYRO_XOUT_H  0x43    /* Byte cao đầu tiên của dữ liệu gyro */

/* ======================================================================== *
 *                      KHỞI TẠO                                           *
 * ======================================================================== */

/*
 * Khởi tạo MPU6050: wake up (thoát SLEEP), cấu hình ±8g, ±2000 deg/s.
 * Bắt buộc phải gọi trước khi đọc dữ liệu.
 * Trả về ESP_OK nếu thành công.
 */
esp_err_t mpu6050_init(i2c_port_t i2c_num);

/* ======================================================================== *
 *                      ĐỌC & CHUYỂN ĐỔI                                   *
 * ======================================================================== */

/*
 * Đọc 14 byte raw từ cảm biến (accel X/Y/Z + temp + gyro X/Y/Z).
 * Dùng combined write-read với repeated start. Dữ liệu big-endian.
 * Trả về ESP_OK nếu thành công.
 */
esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_num,
                                int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                                int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

/*
 * Chuyển raw accel (int16) → m/s²:
 *   m/s² = (raw / ACC_SCALE) × GRAVITY - accel_bias
 * ACC_SCALE = 4096 (LSB/g), GRAVITY = 9.81.
 */
void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *accel_x, float *accel_y, float *accel_z);

/*
 * Chuyển raw gyro (int16) → deg/s:
 *   deg/s = (raw / GYRO_SCALE) - gyro_bias
 * GYRO_SCALE = 16.4 (LSB/deg/s).
 */
void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *gyro_x, float *gyro_y, float *gyro_z);

/* ======================================================================== *
 *                      HIỆU CHUẨN (CALIBRATION)                            *
 * ======================================================================== */

/*
 * Hiệu chuẩn gyro bias hai pha.
 *
 * Vấn đề: MEMS gyro có offset chế tạo. Khi đứng yên, lý tưởng = 0,
 * thực tế ±(1-10) deg/s. Không hiệu chuẩn → drift lớn khi tích phân.
 *
 * Accel bias luôn = 0 vì không thể tách bias khỏi gravity khi chỉ có accel.
 *
 * Pha 1: Gọi mpu6050_calibrate_sample() nhiều lần (vd 200 lần, delay 10ms).
 * Pha 2: Gọi mpu6050_calibrate_finish() để tính bias từ các mẫu đã thu.
 */
void mpu6050_calibrate_sample(i2c_port_t i2c_num);
void mpu6050_calibrate_finish(float *accel_bias_out, float *gyro_bias_out);

/* ======================================================================== *
 *                      TÍNH TỔNG VECTOR                                   *
 * ======================================================================== */

/*
 * Tổng độ lớn vector gia tốc: |A| = √(Ax² + Ay² + Az²)
 * Đứng yên: ≈ 9.81. Rơi tự do: ≈ 0. Va chạm: >> 9.81.
 * Nếu accel_g != NULL, lưu thêm giá trị theo đơn vị g.
 */
float mpu6050_get_total_accel(float accel_x, float accel_y, float accel_z, float *accel_g);

/*
 * Tổng độ lớn vector tốc độ góc: |G| = √(Gx² + Gy² + Gz²)
 * Đánh giá cường độ xoay. Đứng yên: ≈ 0. Xoay nhanh: 100-300 deg/s.
 */
float mpu6050_get_total_gyro(float gyro_x, float gyro_y, float gyro_z);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
