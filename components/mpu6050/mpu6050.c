/**
 * @file mpu6050.c
 * @brief Triển khai (implementation) driver MPU6050 - Đọc và chuyển đổi dữ liệu
 *
 * File này chứa toàn bộ logic giao tiếp I2C với cảm biến MPU6050, bao gồm:
 *   - Khởi tạo (wake up, cấu hình dải đo)
 *   - Đọc dữ liệu thô (raw) qua I2C
 *   - Chuyển đổi raw → đơn vị vật lý (m/s^2, deg/s)
 *   - Hiệu chuẩn gyro bias (two-phase calibration)
 *   - Tính tổng vector gia tốc/tốc độ góc
 *
 * == KIẾN TRÚC ==
 * Module này được thiết kế theo mô hình "tách biệt rõ ràng" (separation of
 * concerns): đọc raw data, chuyển đổi, và hiệu chuẩn là các hàm riêng biệt.
 * Điều này giúp dễ dàng kiểm tra (testing) và tái sử dụng.
 */

/* ======================================================================== *
 *                            THƯ VIỆN                                     *
 * ======================================================================== */

#include "mpu6050.h"                   /* Định nghĩa API công khai */
#include "mpu6050_constants.h"         /* Scale factor, gravity, dt, alpha */
#include "driver/i2c.h"                /* I2C driver của ESP-IDF */
#include "esp_log.h"                   /* Log system (ESP_LOGI, ESP_LOGE) */
#include "freertos/FreeRTOS.h"         /* FreeRTOS (portTICK_PERIOD_MS) */
#include "freertos/task.h"             /* Task delay (vTaskDelay) */
#include <math.h>                      /* sqrtf cho tính tổng vector */

/* ======================================================================== *
 *                 ĐỊNH NGHĨA THANH GHI CON TRÊN MPU6050                   *
 * ======================================================================== */

/**
 * @brief Địa chỉ thanh ghi cấu hình Accelerometer và Gyroscope
 *
 * == MPU6050_REG_GYRO_CONFIG (0x1B) ==
 *   Bit [7:6]: XG_ST (self-test), [5:4]: YG_ST, [3:2]: ZG_ST
 *   Bit [1:0]: FS_SEL - chọn dải đo:
 *     00 = ±250  deg/s (scale: 131.0 LSB/deg/s)
 *     01 = ±500  deg/s (scale: 65.5  LSB/deg/s)
 *     10 = ±1000 deg/s (scale: 32.8  LSB/deg/s)
 *     11 = ±2000 deg/s (scale: 16.4  LSB/deg/s)
 *   → Giá trị 0x18 (= 0b00011000) đặt FS_SEL = 11 (dải ±2000)
 *
 * == MPU6050_REG_ACCEL_CONFIG (0x1C) ==
 *   Bit [7:6]: XA_ST (self-test), [5:4]: YA_ST, [3:2]: ZA_ST
 *   Bit [1:0]: AFS_SEL - chọn dải đo:
 *     00 = ±2g  (scale: 16384 LSB/g)
 *     01 = ±4g  (scale: 8192  LSB/g)
 *     10 = ±8g  (scale: 4096  LSB/g)
 *     11 = ±16g (scale: 2048  LSB/g)
 *   → Giá trị 0x10 (= 0b00010000) đặt AFS_SEL = 10 (dải ±8)
 *
 * ⚠ LƯU Ý: Các hằng số này khai báo trong .c (private), KHÔNG phải .h
 *   vì chúng là chi tiết triển khai, không cần thiết cho module khác.
 */
#define MPU6050_REG_GYRO_CONFIG   0x1B   /* Thanh ghi cấu hình Gyroscope */
#define MPU6050_REG_ACCEL_CONFIG  0x1C   /* Thanh ghi cấu hình Accelerometer */

/* ======================================================================== *
 *        BIẾN STATIC (NỘI BỘ MODULE) - BIAS SAU HIỆU CHUẨN                *
 * ======================================================================== */

/**
 * @brief Giá trị bias (độ lệch) của Accelerometer và Gyroscope
 *
 * Các biến này được khai báo static → chỉ file .c này truy cập được
 * (tính đóng gói / encapsulation). Các module khác muốn lấy bias
 * phải qua tham số của hàm mpu6050_calibrate_finish().
 *
 * == accel_bias[3] ==
 *   Lưu độ lệch accelerometer. Hiện tại luôn = 0 vì không thể tách
 *   bias khỏi gravity khi chỉ có dữ liệu accel (xem giải thích ở
 *   mpu6050_calibrate_finish).
 *
 * == gyro_bias[3] ==
 *   Lưu độ lệch gyroscope (deg/s). Được tính bằng cách lấy trung bình
 *   N mẫu khi cảm biến đứng yên. Lý tưởng: 0, thực tế: ±(1-10) deg/s.
 *   Nếu không trừ bias, sau 10 giây tích phân góc sẽ sai ±(10-100) độ!
 */
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};   /* Bias accel (m/s^2) - luôn 0 */
static float gyro_bias[3]  = {0.0f, 0.0f, 0.0f};   /* Bias gyro (deg/s) - cần hiệu chuẩn */

/* ======================================================================== *
 *                      KHỞI TẠO CẢM BIẾN                                 *
 * ======================================================================== */

esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    esp_err_t ret;  /* Mã lỗi trả về từ các hàm I2C */

    /* ===== BƯỚC 1: WAKE UP =====
     *
     * Khi mới cấp nguồn, MPU6050 ở chế độ SLEEP (bit 6 của PWR_MGMT_1 = 1).
     * Trong chế độ này, mạch cảm biến và ADC tắt để tiết kiệm năng lượng.
     * Cần ghi 0x00 để xóa bit SLEEP và chọn nguồn clock mặc định (nội bộ).
     *
     * Tại sao phải wake up?
     *   - Nếu đọc dữ liệu khi đang sleep, I2C vẫn ACK nhưng dữ liệu trả về
     *     toàn 0 hoặc giá trị rác.
     *   - Không wake up = không có dữ liệu = phát hiện té ngã thất bại.
     *
     * Giao thức I2C:
     *   START → Địa chỉ 0xD0 (0x68 << 1 + WRITE) → REG_ADDR (0x6B) → DATA (0x00) → STOP
     */
    uint8_t pwr_data[] = {MPU6050_REG_PWR_MGMT_1, 0x00};  /* [REG_ADDR, VALUE] */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();          /* Tạo I2C command link */
    i2c_master_start(cmd);                                 /* Điều kiện START */
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true); /* Ghi */
    i2c_master_write(cmd, pwr_data, 2, true);              /* Ghi 2 byte: reg + data */
    i2c_master_stop(cmd);                                  /* Điều kiện STOP */
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);                              /* Giải phóng bộ nhớ */
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "LỖI: Không thể wake up MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ===== BƯỚC 2: CẤU HÌNH ACCELEROMETER ±8g =====
     *
     * Ghi 0x10 vào thanh ghi ACCEL_CONFIG (0x1C):
     *   0x10 = 0b00010000 → AFS_SEL[1:0] = 10 → ±8g
     *
     * Tại sao chọn ±8g?
     *   - Gia tốc khi đứng yên: 1g (9.81 m/s^2)
     *   - Gia tốc khi đi bộ:    2-3g
     *   - Gia tốc khi té ngã:   6-8g (va chạm mạnh)
     *   - ±8g là dải đo phù hợp: đủ rộng để đo té ngã, đủ hẹp để có độ
     *     phân giải tốt (4096 LSB/g, so với 2048 LSB/g của ±16g).
     *
     * Công thức chuyển đổi (xem mpu6050_constants.h):
     *   m/s^2 = raw / 4096 * 9.81
     */
    uint8_t accel_cfg[] = {MPU6050_REG_ACCEL_CONFIG, 0x10}; /* [REG, ±8g] */
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

    /* ===== BƯỚC 3: CẤU HÌNH GYROSCOPE ±2000 deg/s =====
     *
     * Ghi 0x18 vào thanh ghi GYRO_CONFIG (0x1B):
     *   0x18 = 0b00011000 → FS_SEL[1:0] = 11 → ±2000 deg/s
     *
     * Tại sao chọn ±2000 deg/s?
     *   - Tốc độ quay khi xoay người bình thường: 50-150 deg/s
     *   - Tốc độ quay khi té ngã, xoay người đột ngột: 300-1500 deg/s
     *   - Dải ±2000 cho phép đo mọi tình huống mà không bị bão hòa.
     *
     * Công thức chuyển đổi:
     *   deg/s = raw / 16.4
     */
    uint8_t gyro_cfg[] = {MPU6050_REG_GYRO_CONFIG, 0x18}; /* [REG, ±2000 deg/s] */
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
    uint8_t data[14];          /* Bộ đệm 14 byte cho dữ liệu thô */
    uint8_t reg_addr = MPU6050_REG_ACCEL_XOUT_H;  /* Địa chỉ bắt đầu đọc */

    /* ===== GIAO THỨC I2C: COMBINED WRITE-READ =====
     *
     * Đây là giao thức "ghi địa chỉ thanh ghi, sau đó đọc dữ liệu" mà
     * không giải phóng bus I2C ở giữa (dùng REPEATED START).
     *
     * Sơ đồ tín hiệu:
     *
     *   S   0xD0  A  [REG_ADDR]  A    Sr   0xD1  A  [14 BYTES]  NACK  P
     *   │    │     │     │       │    │     │    │      │         │     │
     *   │    │     │     │       │    │     │    │      │         │     │
     *   START     ACK          ACK   RSTART      ACK          NACK  STOP
     *        GHI              GHI              ĐỌC           (chủ)
     *
     * Chi tiết:
     *   1. START:            Tạo điều kiện bắt đầu trên bus I2C
     *   2. GHI 0xD0:         Gửi địa chỉ MPU6050 (0x68 << 1 = 0xD0) + bit WRITE
     *   3. GHI REG_ADDR:     Gửi địa chỉ thanh ghi cần đọc (0x3B)
     *   4. REPEATED START:   Tạo lại điều kiện START (không STOP)
     *   5. GHI 0xD1:         Gửi địa chỉ MPU6050 + bit READ
     *   6. ĐỌC 14 BYTE:      Đọc 13 byte đầu có ACK, byte cuối NACK
     *   7. STOP:             Kết thúc truyền thông
     *
     * Tại sao dùng REPEATED START thay vì STOP-START?
     *   - Ngăn chip khác can thiệp vào bus giữa hai giao dịch
     *   - Nhanh hơn (bỏ qua 1 điều kiện STOP và 1 START)
     *   - Là quy ước chuẩn trong giao tiếp I2C với cảm biến
     *
     * 14 BYTE DỮ LIỆU:
     *   [0-1]   ACCEL_XOUT_H/L    (0x3B-0x3C) - Gia tốc trục X
     *   [2-3]   ACCEL_YOUT_H/L    (0x3D-0x3E) - Gia tốc trục Y
     *   [4-5]   ACCEL_ZOUT_H/L    (0x3F-0x40) - Gia tốc trục Z
     *   [6-7]   TEMP_OUT_H/L      (0x41-0x42) - Nhiệt độ (bỏ qua trong project này)
     *   [8-9]   GYRO_XOUT_H/L     (0x43-0x44) - Tốc độ góc trục X
     *   [10-11] GYRO_YOUT_H/L     (0x45-0x46) - Tốc độ góc trục Y
     *   [12-13] GYRO_ZOUT_H/L     (0x47-0x48) - Tốc độ góc trục Z
     */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                           /* 1. START */
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true); /* 2. GHI */
    i2c_master_write(cmd, &reg_addr, 1, true);                      /* 3. GHI REG_ADDR */
    i2c_master_start(cmd);                                           /* 4. REPEATED START */
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);  /* 5. GHI READ */
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);           /* 6. ĐỌC 14 BYTE */
    i2c_master_stop(cmd);                                            /* 7. STOP */

    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);  /* Giải phóng bộ nhớ của command link */

    if (ret != ESP_OK) return ret;

    /* ===== GIẢI MÃ BIG-ENDIAN: [MSB][LSB] → int16 =====
     *
     * MPU6050 lưu trữ dữ liệu dạng big-endian (MSB trước, LSB sau).
     * Đây là quy ước chung của hầu hết cảm biến MEMS.
     *
     * Ví dụ với giá trị 0x1234:
     *   data[0] = 0x12 (MSB - Most Significant Byte)
     *   data[1] = 0x34 (LSB - Least Significant Byte)
     *
     * Công thức ghép:
     *   int16 value = (data[0] << 8) | data[1]
     *               = (0x12 << 8) | 0x34
     *               = 0x1200 | 0x34
     *               = 0x1234 (= 4660 trong thập phân)
     *
     * Tại sao phải << 8?
     *   data[0] là uint8 (8 bit), chứa 8 bit cao.
     *   data[1] là uint8 (8 bit), chứa 8 bit thấp.
     *   Muốn có int16 (16 bit): (MSB << 8) + LSB
     *
     * Chú ý: data[6-7] là TEMPERATURE, không dùng trong project này.
     */
    *accel_x = (int16_t)((data[0] << 8) | data[1]);   /* ACCEL_X: data[0-1] */
    *accel_y = (int16_t)((data[2] << 8) | data[3]);   /* ACCEL_Y: data[2-3] */
    *accel_z = (int16_t)((data[4] << 8) | data[5]);   /* ACCEL_Z: data[4-5] */
    /* data[6-7] = TEMPERATURE (bỏ qua) */
    *gyro_x  = (int16_t)((data[8] << 8) | data[9]);   /* GYRO_X:  data[8-9] */
    *gyro_y  = (int16_t)((data[10] << 8) | data[11]); /* GYRO_Y:  data[10-11] */
    *gyro_z  = (int16_t)((data[12] << 8) | data[13]); /* GYRO_Z:  data[12-13] */

    return ESP_OK;
}

/* ======================================================================== *
 *                  CHUYỂN ĐỔI RAW SANG ĐƠN VỊ VẬT LÝ                      *
 * ======================================================================== */

/**
 * @brief Chuyển đổi raw accelerometer (int16) sang m/s^2
 *
 * Công thức đầy đủ:
 *   m/s² = (raw_value / ACC_SCALE) × GRAVITY - accel_bias
 *
 * Phân tích từng thành phần:
 *   1. raw_value / 4096.0:
 *      - Chuyển từ LSB (đơn vị raw) sang "g" (gia tốc trọng trường)
 *      - 4096 LSB = 1g (vì dải ±8g với độ phân giải 16-bit)
 *
 *   2. × 9.81:
 *      - Chuyển từ "g" sang "m/s^2"
 *      - 1g = 9.81 m/s^2 theo định nghĩa quốc tế
 *
 *   3. - accel_bias[i]:
 *      - Trừ độ lệch của cảm biến (hiện = 0, xem giải thích ở calibration)
 *
 * Ví dụ: raw = 20480 (khoảng 5g)
 *   → (20480 / 4096) × 9.81 - 0
 *   = 5 × 9.81
 *   = 49.05 m/s^2
 *
 * @note Các tham số đều là int16, nhưng chia cho MPU6050_ACC_SCALE (float)
 *       nên kết quả là float (ép kiểu tự động).
 */
void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *accel_x, float *accel_y, float *accel_z) {
    /* raw / 4096.0 * 9.81 = m/s^2, sau đó trừ bias */
    *accel_x = (raw_x / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias[0];
    *accel_y = (raw_y / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias[1];
    *accel_z = (raw_z / MPU6050_ACC_SCALE) * MPU6050_GRAVITY - accel_bias[2];
}

/**
 * @brief Chuyển đổi raw gyroscope (int16) sang deg/s
 *
 * Công thức đầy đủ:
 *   deg/s = (raw_value / GYRO_SCALE) - gyro_bias
 *
 * Phân tích:
 *   1. raw_value / 16.4:
 *      - Chuyển từ LSB sang deg/s
 *      - 16.4 LSB = 1 deg/s (vì dải ±2000 deg/s)
 *
 *   2. - gyro_bias[i]:
 *      - Trừ độ lệch đo được khi hiệu chuẩn
 *      - Rất quan trọng: bias 1 deg/s sau 60s tích phân gây sai 60 độ!
 *
 * Ví dụ: raw = 328, gyro_bias = 2.5 deg/s
 *   → (328 / 16.4) - 2.5
 *   = 20 - 2.5
 *   = 17.5 deg/s
 */
void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *gyro_x, float *gyro_y, float *gyro_z) {
    /* raw / 16.4 = deg/s, sau đó trừ bias */
    *gyro_x = (raw_x / MPU6050_GYRO_SCALE) - gyro_bias[0];
    *gyro_y = (raw_y / MPU6050_GYRO_SCALE) - gyro_bias[1];
    *gyro_z = (raw_z / MPU6050_GYRO_SCALE) - gyro_bias[2];
}

/* ======================================================================== *
 *                      HIỆU CHUẨN (CALIBRATION)                           *
 * ======================================================================== */

/**
 * @brief Biến tĩnh dùng cho quy trình hiệu chuẩn hai pha (two-phase)
 *
 * == calib_gyro_sum[3] ==
 *   Tổng tích lũy các giá trị gyro (đã đổi sang deg/s) trong pha thu thập.
 *   Mỗi lần gọi calibrate_sample, giá trị gyro được cộng dồn vào đây.
 *
 * == calib_sample_count ==
 *   Số mẫu đã thu thập. Dùng để tính trung bình: bias = sum / count.
 *
 * Ví dụ: thu 200 mẫu, mỗi mẫu gyro_x ≈ 1.5 deg/s
 *   → calib_gyro_sum[0] = 200 × 1.5 = 300
 *   → calib_sample_count = 200
 *   → gyro_bias[0] = 300 / 200 = 1.5 deg/s
 */
static float calib_gyro_sum[3] = {0.0f, 0.0f, 0.0f};  /* Tổng tích lũy gyro (deg/s) */
static int calib_sample_count = 0;                     /* Số mẫu đã thu */

/**
 * @brief Pha 1 của hiệu chuẩn: thu thập một mẫu dữ liệu
 *
 * Hàm này đọc raw data, chuyển gyro sang deg/s, và cộng dồn vào biến
 * tích lũy. Được gọi nhiều lần (thường 100-500 lần) khi cảm biến đứng yên.
 *
 * Lưu ý: Không cần dùng bias ở bước này vì đây chính là lúc đo bias.
 * Giá trị gyro đọc được chính là "bias + nhiễu".
 *
 * Tại sao cần nhiều mẫu?
 *   - Một mẫu đơn lẻ có nhiễu (noise) ±(0.5-2) deg/s
 *   - Lấy trung bình 200 mẫu: noise giảm √200 ≈ 14 lần
 *   - Kết quả chính xác đến ±0.1 deg/s
 *
 * @param i2c_num Cổng I2C để đọc dữ liệu
 */
void mpu6050_calibrate_sample(i2c_port_t i2c_num) {
    int16_t accel_x, accel_y, accel_z;  /* Raw accel (không dùng trong hiệu chuẩn) */
    int16_t gyro_x, gyro_y, gyro_z;     /* Raw gyro */
    float gyro_x_dps, gyro_y_dps, gyro_z_dps; /* Gyro deg/s tạm thời */

    /* Đọc raw data từ cảm biến */
    mpu6050_read_raw_data(i2c_num, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

    /* Chuyển raw → deg/s (chưa trừ bias vì đây là lúc đo bias) */
    gyro_x_dps = (gyro_x / MPU6050_GYRO_SCALE);
    gyro_y_dps = (gyro_y / MPU6050_GYRO_SCALE);
    gyro_z_dps = (gyro_z / MPU6050_GYRO_SCALE);

    /* Cộng dồn vào tổng tích lũy */
    calib_gyro_sum[0] += gyro_x_dps;   /* Tích lũy gyro X */
    calib_gyro_sum[1] += gyro_y_dps;   /* Tích lũy gyro Y */
    calib_gyro_sum[2] += gyro_z_dps;   /* Tích lũy gyro Z */
    calib_sample_count++;               /* Tăng số mẫu */
}

/**
 * @brief Pha 2 của hiệu chuẩn: hoàn thành và tính bias
 *
 * Hàm này được gọi SAU KHI đã thu thập đủ mẫu (thường 200 mẫu với
 * mỗi mẫu cách nhau 10ms, tổng cộng 2 giây).
 *
 * == CÁCH TÍNH GYRO BIAS ==
 *   bias = tổng / số_mẫu
 *
 * Giả sử cảm biến đứng yên, gyro lý tưởng = 0.
 * Nếu gyro thực tế = 2.5 deg/s, đó là do lỗi chế tạo.
 * Lấy trung bình N mẫu để loại bỏ nhiễu ngẫu nhiên.
 *
 * == TẠI SAO ACCEL BIAS = 0? ==
 *   Accelerometer đo GIA TỐC, bao gồm:
 *     - Gia tốc trọng trường (gravity) ≈ 9.81 m/s^2 (LUÔN TỒN TẠI)
 *     - Gia tốc chuyển động (do té ngã, đi lại...)
 *     - Bias (lỗi chế tạo)
 *
 *   Khi cảm biến đứng yên:
 *     |A| = √(Ax² + Ay² + Az²) ≈ 9.81 m/s^2 (trọng trường + bias)
 *
 *   Không thể biết 9.81 này có bao nhiêu phần trăm là bias.
 *   → Cách an toàn nhất: coi accel_bias = 0.
 *
 *   Hậu quả: góc tính từ accel có thể lệch ±(1-3) độ, nhưng bộ lọc
 *   complementary filter sẽ bù lại bằng gyroscope.
 *
 * @param accel_bias_out Con trỏ mảng 3 phần tử để nhận accel bias (có thể NULL)
 * @param gyro_bias_out  Con trỏ mảng 3 phần tử để nhận gyro bias (có thể NULL)
 */
void mpu6050_calibrate_finish(float *accel_bias_out, float *gyro_bias_out) {
    if (calib_sample_count == 0) return;  /* Không có mẫu nào → không làm gì */

    /* ===== TÍNH GYRO BIAS = TRUNG BÌNH CỘNG =====
     *
     * Công thức:  bias[0] = sum(gyro_X) / N
     *
     * Lý do: Khi đứng yên, gyro lý tưởng = 0.
     * Giá trị đọc được là "nhiễu + bias".
     * Trung bình N mẫu sẽ loại nhiễu (vì nhiễu là ngẫu nhiên, sum tiến về 0).
     */
    gyro_bias[0] = calib_gyro_sum[0] / calib_sample_count;  /* Bias gyro X (deg/s) */
    gyro_bias[1] = calib_gyro_sum[1] / calib_sample_count;  /* Bias gyro Y (deg/s) */
    gyro_bias[2] = calib_gyro_sum[2] / calib_sample_count;  /* Bias gyro Z (deg/s) */

    /* Trả kết quả qua tham số nếu con trỏ không NULL */
    if (gyro_bias_out) {
        gyro_bias_out[0] = gyro_bias[0];
        gyro_bias_out[1] = gyro_bias[1];
        gyro_bias_out[2] = gyro_bias[2];
    }

    /* ===== ACCEL BIAS = 0 (lý do đã giải thích ở trên) ===== */
    accel_bias[0] = accel_bias[1] = accel_bias[2] = 0.0f;
    if (accel_bias_out) {
        accel_bias_out[0] = accel_bias_out[1] = accel_bias_out[2] = 0.0f;
    }

    /* ===== RESET BIẾN TÍCH LŨY CHO LẦN HIỆU CHUẨN SAU ===== */
    calib_gyro_sum[0] = calib_gyro_sum[1] = calib_gyro_sum[2] = 0.0f;
    calib_sample_count = 0;

    /* Log kết quả hiệu chuẩn để debug */
    ESP_LOGI("MPU6050", "Hiệu chuẩn hoàn tất: gyro_bias=[%.2f, %.2f, %.2f] deg/s",
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

/* ======================================================================== *
 *                    TÍNH TỔNG ĐỘ LỚN VECTOR                              *
 * ======================================================================== */

/**
 * @brief Tính tổng độ lớn vector gia tốc (magnitude)
 *
 * Công thức toán học (chuẩn Euclidean):
 *   |A| = √(Ax² + Ay² + Az²)
 *
 * == GIẢI THÍCH VẬT LÝ ==
 *   Vector gia tốc tổng hợp cho biết "cường độ" gia tốc mà thiết bị
 *   đang chịu, không phụ thuộc vào hướng.
 *
 *   Khi đứng yên:        |A| ≈ 9.81 m/s^2 (= 1g, chỉ có trọng lực)
 *   Khi rơi tự do:       |A| ≈ 0 m/s^2    (= 0g, không trọng lượng)
 *   Khi va chạm:         |A| > 9.81 m/s^2 (> 1g, lực tác động thêm)
 *   Khi té ngã:          |A| giảm → 0 (rơi), sau đó tăng đột biến (va chạm)
 *
 * == ỨNG DỤNG ==
 *   - Phát hiện rơi tự do: |A| < 3 m/s^2 (0.3g)
 *   - Phát hiện va chạm:   |A| > 20 m/s^2 (2g)
 *   - Kiểm tra trạng thái: |A| ∈ [9, 11] → đứng yên hoặc chuyển động đều
 *
 * @param accel_x  Gia tốc trục X (m/s^2)
 * @param accel_y  Gia tốc trục Y (m/s^2)
 * @param accel_z  Gia tốc trục Z (m/s^2)
 * @param accel_g  Con trỏ (tùy chọn) để nhận giá trị theo đơn vị g
 *                 (nếu NULL thì bỏ qua). Hữu ích khi muốn hiển thị
 *                 dưới dạng "1.2g" thay vì "11.8 m/s^2".
 * @return float   Tổng độ lớn vector (m/s^2)
 */
float mpu6050_get_total_accel(float accel_x, float accel_y, float accel_z, float *accel_g) {
    /* Định lý Pythagoras 3 chiều: sqrt(Ax² + Ay² + Az²) */
    float total_mps2 = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

    /* Chuyển sang đơn vị "g" nếu người dùng yêu cầu */
    if (accel_g != NULL) {
        *accel_g = total_mps2 / MPU6050_GRAVITY;  /* m/s^2 → g */
    }

    return total_mps2;  /* Trả về m/s^2 */
}

/**
 * @brief Tính tổng độ lớn vector tốc độ góc (magnitude)
 *
 * Công thức:
 *   |G| = √(Gx² + Gy² + Gz²)
 *
 * == GIẢI THÍCH ==
 *   Tổng tốc độ góc cho biết thiết bị đang xoay "mạnh" như thế nào,
 *   không phụ thuộc vào hướng xoay.
 *
 *   Khi đứng yên:           |G| ≈ 0 deg/s
 *   Khi xoay người nhẹ:     |G| ≈ 30-80 deg/s
 *   Khi xoay người nhanh:   |G| ≈ 150-300 deg/s
 *   Khi té ngã lăn:         |G| có thể > 500 deg/s
 *
 * @param gyro_x  Tốc độ góc trục X (deg/s)
 * @param gyro_y  Tốc độ góc trục Y (deg/s)
 * @param gyro_z  Tốc độ góc trục Z (deg/s)
 * @return float  Tổng độ lớn vector (deg/s)
 */
float mpu6050_get_total_gyro(float gyro_x, float gyro_y, float gyro_z) {
    /* Tính magnitude theo Pythagorean chuẩn */
    return sqrtf(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
}
