/*
 * Triển khai tính góc Roll/Pitch bằng Complementary Filter.
 *
 * Vấn đề:
 *   - Accelerometer: góc tuyệt đối nhưng nhiễu khi rung
 *   - Gyroscope: mượt nhưng drift (sai số tích lũy qua tích phân)
 *
 * Complementary Filter kết hợp ưu điểm cả hai:
 *   góc = α × (góc_cũ + gyro × dt) + (1-α) × góc_accel
 * với α = 0.80.
 *
 * Cách hình dung: gyro là "bước đi" (nhanh, mượt nhưng lệch dần),
 * accel là "GPS" (chính xác nhưng nhiễu). Bộ lọc cho phép mở mắt
 * 20% thời gian để nhìn GPS và chỉnh lại hướng đi.
 * Tần số cắt ~3.2Hz - phù hợp với chuyển động của con người.
 */

#include "roll_pitch.h"
#include "mpu6050_constants.h"
#include <math.h>
#include "esp_timer.h"

/* ======================================================================== *
 *                      BIẾN STATIC (NỘI BỘ MODULE)                         *
 * ======================================================================== */

/* Giá trị góc hiện tại (độ). Bắt đầu từ 0, hội tụ dần về giá trị thực sau vài lần update. */
static float roll = 0.0f;   /* Nghiêng trái/phải (quanh X) */
static float pitch = 0.0f;  /* Nghiêng trước/sau (quanh Y) */

/* Timestamp lần trước (µs), dùng tính dt chính xác thay vì giả định 10ms cố định.
   Quan trọng vì task có thể bị preempt gây jitter, dt sai → tích phân sai. */
static int64_t prev_timestamp_us = 0;

/* ======================================================================== *
 *                      KHỞI TẠO                                           *
 * ======================================================================== */

void roll_pitch_init(void) {
    roll = 0.0f;
    pitch = 0.0f;
    prev_timestamp_us = 0;
}

/* ======================================================================== *
 *                      CẬP NHẬT GÓC (CORE ALGORITHM)                       *
 * ======================================================================== */

void roll_pitch_update(float accel_x, float accel_y, float accel_z,
                       float gyro_x, float gyro_y, float gyro_z) {

    /* Bước 1: Tính dt thực tế (giây).
       Giới hạn trong [1ms, 50ms] để tránh tích phân sai nếu task bị treo
       hoặc preempt quá lâu. Lần đầu dùng dt mặc định 10ms. */
    int64_t current_timestamp_us = esp_timer_get_time();
    float dt;

    if (prev_timestamp_us == 0) {
        dt = MPU6050_DT;
    } else {
        dt = (current_timestamp_us - prev_timestamp_us) / 1000000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.050f) dt = 0.050f;
    }
    prev_timestamp_us = current_timestamp_us;

    /* Bước 2: Góc tham chiếu từ accel.
       Khi đứng yên, vector đo được chính là trọng lực. Từ hướng của nó
       so với các trục, tính được góc nghiêng tuyệt đối bằng atan2.
       Chỉ chính xác khi không có gia tốc ngoài - đây là lý do cần lọc. */
    float accel_roll = atan2f(-accel_y, accel_z) * 180.0f / M_PI - 90.0f;
    float accel_pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;

    /* Bước 3: Tích phân gyro.
       Gyro đo tốc độ (deg/s), cần tích phân để có góc: góc += tốc_độ × dt.
       Chỉ dùng gyro_x cho roll (quanh X), gyro_y cho pitch (quanh Y). */
    roll  += gyro_x * dt;
    pitch += gyro_y * dt;

    /* Bước 4: Complementary filter.
       Kết hợp gyro (mượt, nhanh) và accel (tuyệt đối, chống drift):
         góc = α × góc_gyro + (1-α) × góc_accel
       Hoạt động như low-pass cho accel (chặn nhiễu rung) và high-pass
       cho gyro (chặn drift). Tần số cắt ~3.2Hz với α=0.80, dt=10ms. */
    roll  = ROLL_PITCH_ALPHA * roll  + (1.0f - ROLL_PITCH_ALPHA) * accel_roll;
    pitch = ROLL_PITCH_ALPHA * pitch + (1.0f - ROLL_PITCH_ALPHA) * accel_pitch;
}

/* ======================================================================== *
 *                      LẤY GIÁ TRỊ GÓC                                    *
 * ======================================================================== */

float get_roll(void) {
    return roll;
}

float get_pitch(void) {
    return pitch;
}
