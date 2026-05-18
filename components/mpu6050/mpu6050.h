/**
 * @file mpu6050.h
 * @brief Header file cho driver MPU6050 (6-axis: Accelerometer + Gyroscope)
 *
 * Đây là header công khai (public API) của module MPU6050, định nghĩa tất cả
 * các hàm mà module khác có thể gọi để tương tác với cảm biến. Các chi tiết
 * triển khai nội bộ (internal implementation) được giấu trong file .c để
 * đảm bảo tính đóng gói (encapsulation).
 *
 == SƠ ĐỒ KẾT NỐI PHẦN CỨNG ==
 *
 *   ESP32 (I2C Master)          MPU6050 (I2C Slave)
 *   ─────────────────────────────────────────────
 *   GPIO 21 (SCL)  ───────────── SCL (chân 23)
 *   GPIO 22 (SDA)  ───────────── SDA (chân 24)
 *   3.3V           ───────────── VCC (chân 8)
 *   GND            ───────────── GND (chân 1)
 *                                 AD0  → GND (địa chỉ 0x68)
 *
 * == CÁC BƯỚC SỬ DỤNG ==
 *   1. Khởi tạo I2C (trong main/project code)
 *   2. mpu6050_init()         → wake up & cấu hình ±8g, ±2000 deg/s
 *   3. mpu6050_calibrate_sample() × N  → thu thập mẫu hiệu chuẩn (LED blink)
 *   4. mpu6050_calibrate_finish()      → tính bias gyroscope
 *   5. Vòng lặp chính:
 *      a. mpu6050_read_raw_data()   → đọc raw (int16)
 *      b. mpu6050_convert_accel()   → raw → m/s^2 (trừ bias)
 *      c. mpu6050_convert_gyro()    → raw → deg/s (trừ bias)
 *      d. roll_pitch_update()       → tính góc Roll/Pitch
 *
 * == THÔNG SỐ CẤU HÌNH ==
 *   - Accelerometer:       ±8g          (độ nhạy 4096 LSB/g)
 *   - Gyroscope:           ±2000 deg/s   (độ nhạy 16.4 LSB/deg/s)
 *   - Tần số lấy mẫu:      100 Hz       (mỗi 10ms một mẫu)
 *   - Giao tiếp:           I2C           (địa chỉ 0x68)
 *   - Điện áp hoạt động:   3.3V
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"       /* Cấu trúc i2c_port_t, i2c_cmd_handle_t */
#include "esp_err.h"          /* Kiểu esp_err_t cho xử lý lỗi */
#include "mpu6050_constants.h" /* Hằng số scale factor, gravity, dt, alpha */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Địa chỉ I2C của MPU6050 (7-bit)
 *
 * Địa chỉ mặc định là 0x68. Có thể thay đổi thành 0x69 bằng cách kéo chân
 * AD0 lên mức HIGH (nếu muốn kết nối 2 MPU6050 trên cùng bus I2C).
 *
 * Trong giao thức I2C, địa chỉ 7-bit được dịch trái 1 bit để ghép với bit
 * R/W (0 = ghi, 1 = đọc):
 *   - Ghi:  0x68 << 1 | 0 = 0xD0
 *   - Đọc:  0x68 << 1 | 1 = 0xD1
 */
#define MPU6050_ADDR             0x68    /* 7-bit I2C address (AD0 = GND) */

/**
 * @brief Địa chỉ các thanh ghi quan trọng của MPU6050
 *
 * == MPU6050_REG_PWR_MGMT_1 (0x6B) - Quản lý nguồn ==
 *   Bit 6: SLEEP (1 = sleep mode, 0 = normal)
 *   Bit 3: CYCLE (1 = cycle mode để tiết kiệm năng lượng)
 *   Bit [2:0]: CLKSEL (chọn nguồn clock)
 *   Khi cấp nguồn, MPU6050 ở chế độ SLEEP. Cần ghi 0x00 để wake up.
 *
 * == MPU6050_REG_ACCEL_XOUT_H (0x3B) - Dữ liệu Accelerometer ==
 *   Từ địa chỉ này, đọc 6 byte liên tiếp cho 3 trục (cao + thấp):
 *     0x3B: ACCEL_XOUT[15:8]  (MSB - Most Significant Byte)
 *     0x3C: ACCEL_XOUT[7:0]   (LSB - Least Significant Byte)
 *     0x3D: ACCEL_YOUT[15:8]
 *     0x3E: ACCEL_YOUT[7:0]
 *     0x3F: ACCEL_ZOUT[15:8]
 *     0x40: ACCEL_ZOUT[7:0]
 *
 * == MPU6050_REG_GYRO_XOUT_H (0x43) - Dữ liệu Gyroscope ==
 *   Từ địa chỉ này, đọc 6 byte liên tiếp cho 3 trục:
 *     0x43: GYRO_XOUT[15:8]
 *     0x44: GYRO_XOUT[7:0]
 *     ...  (tương tự)
 *
 *   Để đọc cả accel + gyro + nhiệt độ trong một lần, bắt đầu từ 0x3B
 *   và đọc 14 byte (6 accel + 2 temp + 6 gyro).
 */
#define MPU6050_REG_PWR_MGMT_1   0x6B    /* Thanh ghi quản lý nguồn (bit SLEEP) */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B    /* Byte cao đầu tiên của dữ liệu accel */
#define MPU6050_REG_GYRO_XOUT_H  0x43    /* Byte cao đầu tiên của dữ liệu gyro */

/* ======================================================================== *
 *                      KHỞI TẠO CẢM BIẾN                                  *
 * ======================================================================== */

/**
 * @brief Khởi tạo MPU6050
 *
 * Thực hiện ba bước theo thứ tự bắt buộc:
 *   1. WAKE UP: Ghi 0x00 vào PWR_MGMT_1 để thoát chế độ SLEEP.
 *      Khi mới cấp nguồn, MPU6050 ở sleep mode. Phải wake up thì mới
 *      đọc được dữ liệu. Nếu bỏ qua bước này, mọi lần đọc đều trả về 0.
 *
 *   2. CẤU HÌNH ACCEL: Ghi 0x10 vào ACCEL_CONFIG để chọn dải ±8g.
 *      Giá trị 0x10 = 0b00010000, tức bit 3 = 1:
 *        - FS_SEL[1:0] = 10 → ±8g
 *        - Thang đo: 4096 LSB/g
 *      Các giá trị khác: 00=±2g (16384), 01=±4g (8192), 11=±16g (2048)
 *
 *   3. CẤU HÌNH GYRO: Ghi 0x18 vào GYRO_CONFIG để chọn dải ±2000 deg/s.
 *      Giá trị 0x18 = 0b00011000, tức bit 3 và 4 = 1:
 *        - FS_SEL[1:0] = 11 → ±2000 deg/s
 *        - Thang đo: 16.4 LSB/deg/s
 *      Các giá trị khác: 00=±250 (131), 01=±500 (65.5), 10=±1000 (32.8)
 *
 * @param i2c_num  Cổng I2C của ESP32 (I2C_NUM_0 hoặc I2C_NUM_1)
 * @return ESP_OK  nếu thành công, mã lỗi nếu thất bại
 */
esp_err_t mpu6050_init(i2c_port_t i2c_num);

/* ======================================================================== *
 *                      ĐỌC DỮ LIỆU                                        *
 * ======================================================================== */

/**
 * @brief Đọc dữ liệu thô (raw) từ cảm biến
 *
 * Sử dụng giao thức I2C "combined write-then-read" với một điều kiện khởi
 * tạo lại (repeated start) ở giữa để không giải phóng bus:
 *
 *   START → GHI 0xD0 + REG_ADDR → REPEATED START → ĐỌC 0xD1 + 14 BYTE → STOP
 *
 * Dữ liệu được lưu dạng big-endian (MSB trước, LSB sau) như quy ước của
 * MPU6050. Cần ghép (MSB << 8 | LSB) để được giá trị int16.
 *
 * 14 byte đọc được:
 *   [0-1]  ACCEL_X       [2-3]  ACCEL_Y        [4-5]  ACCEL_Z
 *   [6-7]  TEMPERATURE   [8-9]  GYRO_X         [10-11] GYRO_Y
 *   [12-13] GYRO_Z
 *
 * @param i2c_num   Cổng I2C
 * @param accel_x   Con trỏ lưu giá trị raw accel X (trả về)
 * @param accel_y   Con trỏ lưu giá trị raw accel Y (trả về)
 * @param accel_z   Con trỏ lưu giá trị raw accel Z (trả về)
 * @param gyro_x    Con trỏ lưu giá trị raw gyro X (trả về)
 * @param gyro_y    Con trỏ lưu giá trị raw gyro Y (trả về)
 * @param gyro_z    Con trỏ lưu giá trị raw gyro Z (trả về)
 * @return ESP_OK  nếu thành công, ESP_FAIL nếu lỗi I2C
 */
esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_num,
                                int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                                int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

/**
 * @brief Chuyển đổi giá trị raw của accelerometer sang m/s^2
 *
 * Công thức chuyển đổi:
 *   m/s² = (raw_value / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias
 *
 * Giải thích từng bước:
 *   1. raw_value / 4096        → đổi từ LSB sang "g" (gia tốc trọng trường)
 *   2. × 9.81                 → đổi từ "g" sang m/s^2
 *   3. - accel_bias[i]        → trừ độ lệch đo được khi hiệu chuẩn
 *
 * Ví dụ: raw = 16384, accel_bias = 0
 *   → (16384 / 4096) * 9.81 = 4 * 9.81 = 39.24 m/s^2 (≈ 4g)
 *
 * @param raw_x   Giá trị raw từ cảm biến (trục X)
 * @param raw_y   Giá trị raw từ cảm biến (trục Y)
 * @param raw_z   Giá trị raw từ cảm biến (trục Z)
 * @param accel_x Con trỏ lưu kết quả m/s^2 (trả về)
 * @param accel_y Con trỏ lưu kết quả m/s^2 (trả về)
 * @param accel_z Con trỏ lưu kết quả m/s^2 (trả về)
 */
void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *accel_x, float *accel_y, float *accel_z);

/**
 * @brief Chuyển đổi giá trị raw của gyroscope sang deg/s
 *
 * Công thức chuyển đổi:
 *   deg/s = (raw_value / MPU6050_GYRO_SCALE) - gyro_bias
 *
 * Giải thích:
 *   1. raw_value / 16.4   → đổi từ LSB sang deg/s
 *   2. - gyro_bias[i]     → trừ độ lệch (khi đứng yên, gyro lý tưởng = 0)
 *
 * Ví dụ: raw = 164, gyro_bias = 0
 *   → 164 / 16.4 = 10 deg/s
 *
 * @param raw_x   Giá trị raw từ cảm biến (trục X)
 * @param raw_y   Giá trị raw từ cảm biến (trục Y)
 * @param raw_z   Giá trị raw từ cảm biến (trục Z)
 * @param gyro_x  Con trỏ lưu kết quả deg/s (trả về)
 * @param gyro_y  Con trỏ lưu kết quả deg/s (trả về)
 * @param gyro_z  Con trỏ lưu kết quả deg/s (trả về)
 */
void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *gyro_x, float *gyro_y, float *gyro_z);

/* ======================================================================== *
 *                      HIỆU CHUẨN (CALIBRATION)                            *
 * ======================================================================== */

/**
 * @brief Hiệu chuẩn hai pha (Two-phase calibration)
 *
 * == VẤN ĐỀ ==
 * Cảm biến MPU6050 (như hầu hết MEMS sensors) có sai số chế tạo (offset).
 * Khi đứng yên:
 *   - Gyroscope lý tưởng:  0 deg/s (cả 3 trục)
 *   - Gyroscope thực tế:   ≠ 0 (có thể lệch ±10 deg/s hoặc hơn)
 * Nếu không hiệu chuẩn, sai số này tích lũy qua tích phân gây "drift" góc.
 *
 * == GIẢI PHÁP: Hiệu chuẩn gyro bias ==
 * Lấy N mẫu khi cảm biến đứng yên, tính trung bình. Kết quả chính là
 * độ lệch (bias) cần trừ đi trong các lần đọc sau.
 *
 * == LƯU Ý VỀ ACCELEROMETER ==
 * Accel bias KHÔNG được tính vì gia tốc trọng trường luôn hiện diện.
 * Không thể tách biệt đâu là bias và đâu là gravity khi chỉ có dữ liệu
 * accel. Trong project này, accel_bias được đặt = 0.
 *
 * == GIAO THỨC HAI PHA (TWO-PHASE) ==
 *   Pha 1: Gọi mpu6050_calibrate_sample() nhiều lần (ví dụ 200 lần).
 *           Giữa các lần gọi nên có delay 10ms (bằng dt).
 *           Có thể nhấp nháy LED để báo hiệu "đang hiệu chuẩn".
 *
 *   Pha 2: Gọi mpu6050_calibrate_finish() để tính bias từ các mẫu đã thu.
 *           Kết quả lưu vào biến static trong .c và có thể lấy qua tham số.
 */
void mpu6050_calibrate_sample(i2c_port_t i2c_num);   /* Thu thập một mẫu */
void mpu6050_calibrate_finish(float *accel_bias_out,  /* Kết thúc & tính bias */
                              float *gyro_bias_out);

/* ======================================================================== *
 *                      TÍNH TỔNG VECTOR                                    *
 * ======================================================================== */

/**
 * @brief Tính tổng độ lớn vector gia tốc
 *
 * Sử dụng định lý Pythagoras trong không gian 3 chiều:
 *   |A| = √(Ax² + Ay² + Az²)
 *
 * Khi đứng yên trên mặt đất, |A| ≈ 9.81 m/s^2 (≈ 1g).
 * Khi té ngã, |A| có thể giảm đột ngột (rơi tự do) hoặc tăng đột biến (va chạm).
 *
 * @param accel_x   Gia tốc trục X (m/s^2)
 * @param accel_y   Gia tốc trục Y (m/s^2)
 * @param accel_z   Gia tốc trục Z (m/s^2)
 * @param accel_g   Con trỏ (tùy chọn) để nhận giá trị theo đơn vị g
 *                  (tổng vector / 9.81). Có thể NULL nếu không cần.
 * @return float    Tổng độ lớn vector (m/s^2)
 */
float mpu6050_get_total_accel(float accel_x, float accel_y, float accel_z, float *accel_g);

/**
 * @brief Tính tổng độ lớn vector tốc độ góc
 *
 *   |G| = √(Gx² + Gy² + Gz²)
 *
 * Dùng để đánh giá cường độ chuyển động xoay. Khi đứng yên, |G| ≈ 0.
 * Khi xoay người nhanh, |G| có thể đạt 100-300 deg/s.
 *
 * @param gyro_x  Tốc độ góc trục X (deg/s)
 * @param gyro_y  Tốc độ góc trục Y (deg/s)
 * @param gyro_z  Tốc độ góc trục Z (deg/s)
 * @return float  Tổng độ lớn vector (deg/s)
 */
float mpu6050_get_total_gyro(float gyro_x, float gyro_y, float gyro_z);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
