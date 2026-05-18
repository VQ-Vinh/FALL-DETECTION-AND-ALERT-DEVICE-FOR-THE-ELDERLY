/**
 * @file mpu6050.c
 * @brief Driver MPU6050 - Đọc và chuyển đổi dữ liệu cảm biến
 */

#include "mpu6050.h"
#include "mpu6050_constants.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// Các thanh ghi cấu hình
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C

// Bias sau hiệu chuẩn
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float gyro_bias[3]  = {0.0f, 0.0f, 0.0f};

// ========== KHỞI TẠO ==========
esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    esp_err_t ret;

    // Bước 1: Wake up - ghi 0x00 vào thanh ghi PWR_MGMT_1
    uint8_t pwr_data[] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, pwr_data, 2, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to wake up: %s", esp_err_to_name(ret));
        return ret;
    }

    // Bước 2: Cấu hình Accelerometer ±8g (0x10)
    uint8_t accel_cfg[] = {MPU6050_REG_ACCEL_CONFIG, 0x10};
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, accel_cfg, 2, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to configure accel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Bước 3: Cấu hình Gyroscope ±2000 deg/s (0x18)
    uint8_t gyro_cfg[] = {MPU6050_REG_GYRO_CONFIG, 0x18};
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, gyro_cfg, 2, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to configure gyro: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("MPU6050", "Initialized: Accel ±8g, Gyro ±2000 deg/s");
    return ESP_OK;
}

// ========== ĐỌC DỮ LIỆU THÔ ==========
esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_num,
                                int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                                int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t data[14];
    uint8_t reg_addr = MPU6050_REG_ACCEL_XOUT_H;

    // I2C combined write-read
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, &reg_addr, 1, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return ret;

    // Big-endian: [MSB][LSB] → combine thành int16
    *accel_x = (int16_t)((data[0] << 8) | data[1]);
    *accel_y = (int16_t)((data[2] << 8) | data[3]);
    *accel_z = (int16_t)((data[4] << 8) | data[5]);
    // data[6-7] = temperature (bỏ qua)
    *gyro_x  = (int16_t)((data[8] << 8) | data[9]);
    *gyro_y  = (int16_t)((data[10] << 8) | data[11]);
    *gyro_z  = (int16_t)((data[12] << 8) | data[13]);

    return ESP_OK;
}

// ========== CHUYỂN ĐỔI ==========
void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *accel_x, float *accel_y, float *accel_z) {
    // raw / 4096 * 9.81 = m/s², sau đó trừ bias
    *accel_x = (raw_x / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias[0];
    *accel_y = (raw_y / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias[1];
    *accel_z = (raw_z / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias[2];
}

void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *gyro_x, float *gyro_y, float *gyro_z) {
    // raw / 16.4 = deg/s, sau đó trừ bias
    *gyro_x = (raw_x / MPU6050_GYRO_SCALE) - gyro_bias[0];
    *gyro_y = (raw_y / MPU6050_GYRO_SCALE) - gyro_bias[1];
    *gyro_z = (raw_z / MPU6050_GYRO_SCALE) - gyro_bias[2];
}

// ========== HIỆU CHUẨN ==========
// Biến tích lũy cho two-phase calibration
static float calib_gyro_sum[3] = {0};
static int calib_sample_count = 0;

// Two-phase: thu thập 1 mẫu
void mpu6050_calibrate_sample(i2c_port_t i2c_num) {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;

    mpu6050_read_raw_data(i2c_num, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

    gyro_x_dps = (gyro_x / MPU6050_GYRO_SCALE);
    gyro_y_dps = (gyro_y / MPU6050_GYRO_SCALE);
    gyro_z_dps = (gyro_z / MPU6050_GYRO_SCALE);

    calib_gyro_sum[0] += gyro_x_dps;
    calib_gyro_sum[1] += gyro_y_dps;
    calib_gyro_sum[2] += gyro_z_dps;
    calib_sample_count++;
}

// Two-phase: hoàn thành calibration
void mpu6050_calibrate_finish(float *accel_bias_out, float *gyro_bias_out) {
    if (calib_sample_count == 0) return;

    // Gyro bias = trung bình (gyro = 0 khi đứng yên)
    gyro_bias[0] = calib_gyro_sum[0] / calib_sample_count;
    gyro_bias[1] = calib_gyro_sum[1] / calib_sample_count;
    gyro_bias[2] = calib_gyro_sum[2] / calib_sample_count;

    if (gyro_bias_out) {
        gyro_bias_out[0] = gyro_bias[0];
        gyro_bias_out[1] = gyro_bias[1];
        gyro_bias_out[2] = gyro_bias[2];
    }

    // Accel bias = 0 (gravity luôn hiện diện, không thể tách riêng)
    accel_bias[0] = accel_bias[1] = accel_bias[2] = 0.0f;
    if (accel_bias_out) {
        accel_bias_out[0] = accel_bias_out[1] = accel_bias_out[2] = 0.0f;
    }

    // Reset counters cho lần hiệu chuẩn sau
    calib_gyro_sum[0] = calib_gyro_sum[1] = calib_gyro_sum[2] = 0;
    calib_sample_count = 0;

    ESP_LOGI("MPU6050", "Calibration done: gyro_bias=[%.2f, %.2f, %.2f] deg/s",
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

// ========== TÍNH TỔNG ==========
float mpu6050_get_total_accel(float accel_x, float accel_y, float accel_z, float *accel_g) {
    // A = √(x² + y² + z²)
    float total_mps2 = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

    if (accel_g != NULL) {
        *accel_g = total_mps2 / MPU6050_GRAVITY;  // Chuyển sang g
    }
    return total_mps2;
}

float mpu6050_get_total_gyro(float gyro_x, float gyro_y, float gyro_z) {
    return sqrtf(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
}
