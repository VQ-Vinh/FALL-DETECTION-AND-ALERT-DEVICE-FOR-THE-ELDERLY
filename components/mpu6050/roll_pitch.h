/*
 * Header module ước lượng góc Roll/Pitch bằng Complementary Filter.
 *
 * Tại sao cần lọc? Mỗi loại cảm biến có điểm yếu riêng:
 *   - Accelerometer: góc tuyệt đối nhưng nhiễu khi rung
 *   - Gyroscope: mượt, nhanh nhưng bị drift (trôi) theo thời gian
 * Complementary Filter kết hợp ưu điểm cả hai, nhẹ hơn Kalman filter nhiều
 * (chỉ vài phép toán, không cần ma trận).
 *
 * Cách dùng: init() một lần → update() mỗi ~100Hz → get_roll()/get_pitch()
 */

#ifndef ROLL_PITCH_H
#define ROLL_PITCH_H

#include "mpu6050_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Khởi tạo góc về 0, reset timestamp. Hội tụ dần về giá trị thực sau 3-5 lần update. */
void roll_pitch_init(void);

/*
 * Cập nhật góc Roll/Pitch - hàm cốt lõi.
 *   - Tính dt thực tế, giới hạn 1-50ms để tránh bất thường
 *   - Tính góc tham chiếu từ accel (atan2)
 *   - Tích phân gyro: góc += tốc_độ × dt
 *   - Complementary filter: α × gyro + (1-α) × accel
 *
 * Các tham số đầu vào: accel_x/y/z (m/s²), gyro_x/y/z (deg/s)
 *   - Dữ liệu phải được chuyển đổi từ raw và trừ bias trước khi gọi
 *   - Gọi đều đặn ~100Hz (dt dao động càng ít càng chính xác)
 */
void roll_pitch_update(float accel_x, float accel_y, float accel_z,
                      float gyro_x, float gyro_y, float gyro_z);

/* Roll hiện tại (độ). Dương = nghiêng phải, âm = nghiêng trái. */
float get_roll(void);

/* Pitch hiện tại (độ). Dương = cúi xuống, âm = ngửa lên. */
float get_pitch(void);

#ifdef __cplusplus
}
#endif

#endif /* ROLL_PITCH_H */
