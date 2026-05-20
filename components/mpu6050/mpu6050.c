/*
 * Triển khai driver MPU6050 - giao tiếp I2C, hiệu chuẩn, chuyển đổi dữ liệu.
 *
 * Chức năng: khởi tạo, đọc raw, chuyển đổi sang m/s² và deg/s,
 * hiệu chuẩn gyro bias hai pha, tính tổng vector.
 * Thiết kế tách biệt rõ ràng: đọc → chuyển đổi → hiệu chuẩn là các hàm riêng.
 */

#include "mpu6050.h"
#include "mpu6050_constants.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

/* ======================================================================== *
 *                 THANH GHI CON CỦA MPU6050 (PRIVATE)                      *
 * ======================================================================== */

/*
 * Các thanh ghi cấu hình dải đo. Khai báo trong .c (private) vì là chi tiết
 * triển khai, module khác không cần biết.
 * GYRO_CONFIG (0x1B): FS_SEL=11 → ±2000 deg/s (16.4 LSB/deg/s)
 * ACCEL_CONFIG (0x1C): AFS_SEL=10 → ±8g (4096 LSB/g)
 */
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C

/* ======================================================================== *
 *        BIẾN STATIC - BIAS SAU HIỆU CHUẨN                                  *
 * ======================================================================== */

/*
 * Bias gyro (deg/s) - tính bằng trung bình N mẫu khi đứng yên.
 * Nếu không trừ, tích phân góc sẽ sai ±(10-100)° chỉ sau 10 giây.
 * Bias accel luôn = 0 vì không thể tách bias khỏi trọng lực khi chỉ có accel.
 */
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float gyro_bias[3]  = {0.0f, 0.0f, 0.0f};

/* ======================================================================== *
 *                      KHỞI TẠO CẢM BIẾN                                  *
 * ======================================================================== */

esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    esp_err_t ret;

    /* Bước 1: Wake up - xóa bit SLEEP trong PWR_MGMT_1.
       Khi mới cấp nguồn, MPU6050 ở sleep mode, mọi đọc đều trả về 0. */
    uint8_t pwr_data[] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, pwr_data, 2, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "LỖI: Không thể wake up MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Bước 2: Cấu hình accel ±8g.
       Chọn ±8g vì: đứng yên ~1g, đi bộ ~2-3g, té ngã ~6-8g.
       ±8g đủ rộng để đo té ngã, đủ hẹp để độ phân giải tốt (4096 LSB/g). */
    uint8_t accel_cfg[] = {MPU6050_REG_ACCEL_CONFIG, 0x10};
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, accel_cfg, 2, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "LỖI: Không thể cấu hình accel: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Bước 3: Cấu hình gyro ±2000 deg/s.
       Tốc độ quay khi té ngã có thể đạt 1500 deg/s, ±2000 là dải an toàn
       không bị bão hòa. */
    uint8_t gyro_cfg[] = {MPU6050_REG_GYRO_CONFIG, 0x18};
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, gyro_cfg, 2, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "LỖI: Không thể cấu hình gyro: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("MPU6050", "Khởi tạo thành công: Accel ±8g, Gyro ±2000 deg/s");
    return ESP_OK;
}

/* ======================================================================== *
 *                      ĐỌC DỮ LIỆU THÔ (RAW)                              *
 * ======================================================================== */

esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_num,
                                int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                                int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t data[14];
    uint8_t reg_addr = MPU6050_REG_ACCEL_XOUT_H;

    /*
     * Giao thức I2C combined write-read với repeated start:
     *   START → GHI 0xD0 + REG_ADDR → REPEATED START → ĐỌC 0xD1 + 14 BYTE → STOP
     * Dùng repeated start thay vì STOP-START để nhanh hơn và tránh chip khác
     * can thiệp bus giữa hai giao dịch.
     *
     * 14 byte: [0-1] ACCEL_X, [2-3] ACCEL_Y, [4-5] ACCEL_Z,
     *          [6-7] TEMP (bỏ qua), [8-9] GYRO_X, [10-11] GYRO_Y, [12-13] GYRO_Z
     */
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

    /*
     * Giải mã big-endian: MPU6050 lưu MSB trước, LSB sau.
     * Ghép: value = (MSB << 8) | LSB. Ví dụ [0x12, 0x34] → 0x1234
     */
    *accel_x = (int16_t)((data[0] << 8) | data[1]);
    *accel_y = (int16_t)((data[2] << 8) | data[3]);
    *accel_z = (int16_t)((data[4] << 8) | data[5]);
    /* data[6-7] = TEMPERATURE - không dùng */
    *gyro_x  = (int16_t)((data[8] << 8) | data[9]);
    *gyro_y  = (int16_t)((data[10] << 8) | data[11]);
    *gyro_z  = (int16_t)((data[12] << 8) | data[13]);

    return ESP_OK;
}

/* ======================================================================== *
 *                  CHUYỂN ĐỔI RAW SANG ĐƠN VỊ VẬT LÝ                      *
 * ======================================================================== */

/* raw / ACC_SCALE × GRAVITY = m/s², sau đó trừ bias (hiện tại = 0) */
void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *accel_x, float *accel_y, float *accel_z) {
    *accel_x = (raw_x / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias[0];
    *accel_y = (raw_y / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias[1];
    *accel_z = (raw_z / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias[2];
}

/* raw / GYRO_SCALE = deg/s, sau đó trừ bias. Bias 1 deg/s → sau 60s tích phân sai 60°! */
void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *gyro_x, float *gyro_y, float *gyro_z) {
    *gyro_x = (raw_x / MPU6050_GYRO_SCALE) - gyro_bias[0];
    *gyro_y = (raw_y / MPU6050_GYRO_SCALE) - gyro_bias[1];
    *gyro_z = (raw_z / MPU6050_GYRO_SCALE) - gyro_bias[2];
}

/* ======================================================================== *
 *                      HIỆU CHUẨN (CALIBRATION)                            *
 * ======================================================================== */

/* Biến tích lũy cho hiệu chuẩn hai pha */
static float calib_gyro_sum[3] = {0.0f, 0.0f, 0.0f};
static int calib_sample_count = 0;

/* Pha 1: Thu thập mẫu. Đọc raw, chuyển gyro sang deg/s, cộng dồn.
   Gọi nhiều lần (100-500) khi cảm biến đứng yên. Nhiều mẫu → giảm nhiễu. */
void mpu6050_calibrate_sample(i2c_port_t i2c_num) {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;

    mpu6050_read_raw_data(i2c_num, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

    /* Không trừ bias vì đây chính là lúc đo bias */
    gyro_x_dps = (gyro_x / MPU6050_GYRO_SCALE);
    gyro_y_dps = (gyro_y / MPU6050_GYRO_SCALE);
    gyro_z_dps = (gyro_z / MPU6050_GYRO_SCALE);

    calib_gyro_sum[0] += gyro_x_dps;
    calib_gyro_sum[1] += gyro_y_dps;
    calib_gyro_sum[2] += gyro_z_dps;
    calib_sample_count++;
}

/* Pha 2: Tính bias = tổng / số_mẫu.
   Khi đứng yên, gyro lý tưởng = 0, giá trị đọc = "nhiễu + bias".
   Trung bình N mẫu loại nhiễu ngẫu nhiên.
   Accel bias = 0 vì không thể tách bias khỏi gravity. */
void mpu6050_calibrate_finish(float *accel_bias_out, float *gyro_bias_out) {
    if (calib_sample_count == 0) return;

    gyro_bias[0] = calib_gyro_sum[0] / calib_sample_count;
    gyro_bias[1] = calib_gyro_sum[1] / calib_sample_count;
    gyro_bias[2] = calib_gyro_sum[2] / calib_sample_count;

    if (gyro_bias_out) {
        gyro_bias_out[0] = gyro_bias[0];
        gyro_bias_out[1] = gyro_bias[1];
        gyro_bias_out[2] = gyro_bias[2];
    }

    accel_bias[0] = accel_bias[1] = accel_bias[2] = 0.0f;
    if (accel_bias_out) {
        accel_bias_out[0] = accel_bias_out[1] = accel_bias_out[2] = 0.0f;
    }

    calib_gyro_sum[0] = calib_gyro_sum[1] = calib_gyro_sum[2] = 0.0f;
    calib_sample_count = 0;

    ESP_LOGI("MPU6050", "Hiệu chuẩn hoàn tất: gyro_bias=[%.2f, %.2f, %.2f] deg/s",
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

/* ======================================================================== *
 *                    TÍNH TỔNG ĐỘ LỚN VECTOR                              *
 * ======================================================================== */

/*
 * |A| = √(Ax² + Ay² + Az²) - tổng độ lớn vector gia tốc.
 * Đứng yên: ~9.81 (1g). Rơi tự do: ~0. Va chạm: >> 9.81.
 * accel_g (tùy chọn) nhận giá trị theo đơn vị g.
 */
float mpu6050_get_total_accel(float accel_x, float accel_y, float accel_z, float *accel_g) {
    float total_mps2 = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

    if (accel_g != NULL) {
        *accel_g = total_mps2 / MPU6050_GRAVITY;
    }

    return total_mps2;
}

/*
 * |G| = √(Gx² + Gy² + Gz²) - tổng độ lớn vector tốc độ góc.
 * Đứng yên: ~0. Xoay nhẹ: 30-80. Té ngã lăn: >500 deg/s.
 */
float mpu6050_get_total_gyro(float gyro_x, float gyro_y, float gyro_z) {
    return sqrtf(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
}
