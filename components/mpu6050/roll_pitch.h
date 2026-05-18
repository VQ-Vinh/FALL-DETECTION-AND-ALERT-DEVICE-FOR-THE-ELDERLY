/**
 * @file roll_pitch.h
 * @brief Header cho module ước lượng góc Roll và Pitch từ MPU6050
 *
 * Module này cung cấp giải pháp ước lượng góc nghiêng của thiết bị
 * bằng Complementary Filter - một thuật toán nhẹ, hiệu quả cho hệ thống
 * nhúng (không cần ma trận hay Kalman filter phức tạp).
 *
 * == TẠI SAO CẦN BỘ LỌC? ==
 *
 *   Chỉ dùng Accelerometer:
 *     + Góc tuyệt đối (luôn biết chính xác tư thế khi đứng yên)
 *     - NHIỄU KHI RUNG (do accel đo tổng gia tốc, không chỉ trọng lực)
 *     - Không phân biệt được trọng lực và gia tốc chuyển động
 *
 *   Chỉ dùng Gyroscope:
 *     + Mượt, ổn định, đáp ứng nhanh với chuyển động
 *     - DRIFT THEO THỜI GIAN (sai số tích phân tích lũy dần)
 *     - Cần dt chính xác
 *
 *   Kết hợp cả hai (Complementary Filter):
 *     + Góc mượt (nhờ gyro)
 *     + Không drift (nhờ accel kéo về giá trị thực)
 *     + Ít tốn CPU (chỉ 1 phép nhân, 1 phép cộng cho mỗi trục)
 *
 * == CÔNG THỨC ==
 *   angle = α × (angle_prev + gyro × dt) + (1-α) × accel_angle
 *
 *   Với α = 0.80: 80% tin vào gyro, 20% tin vào accel
 *
 * == SO SÁNH VỚI KALMAN FILTER ==
 *   Complementary filter:  đơn giản, ít RAM, nhanh (vài μs)
 *   Kalman filter:         phức tạp, cần ma trận covariance, chậm (ms)
 *   → Complementary filter là lựa chọn phù hợp cho ESP32 + phát hiện té ngã
 *
 * == THAM KHẢO ==
 *   - MPU6050 datasheet: https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/
 *   - Complementary filter tutorial: https://www.pieter-jan.com/node/11
 */

#ifndef ROLL_PITCH_H
#define ROLL_PITCH_H

#include "mpu6050_constants.h"  /* MPU6050_DT, ROLL_PITCH_ALPHA */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo module roll_pitch
 *
 * Thiết lập góc roll và pitch về 0, reset timestamp nội bộ.
 * Hàm này phải được gọi MỘT LẦN trước khi sử dụng module, thường
 * là trong giai đoạn khởi tạo hệ thống.
 *
 * Lưu ý quan trọng:
 *   - Sau init, góc bắt đầu từ 0 và sẽ hội tụ về giá trị thực sau
 *     3-5 lần gọi roll_pitch_update() (do α = 0.80, mỗi lần chỉ thay
 *     đổi 20% để hướng về góc thực từ accel).
 *   - Nếu muốn góc chính xác ngay lập tức, cần gọi roll_pitch_update()
 *     vài lần trước khi sử dụng kết quả.
 */
void roll_pitch_init(void);

/**
 * @brief Cập nhật góc Roll và Pitch với dữ liệu cảm biến mới
 *
 * Đây là hàm CỐT LÕI của module, thực hiện 4 bước:
 *   1. Tính dt thực tế từ timestamp (giây)
 *   2. Tính góc tuyệt đối từ accelerometer (atan2)
 *   3. Tích phân tốc độ góc từ gyroscope
 *   4. Complementary filter: kết hợp gyro + accel
 *
 * Hàm này nên được gọi ĐỀU ĐẶN ở tần số cố định (lý tưởng ~100Hz).
 * Nếu gọi không đều, dt sẽ dao động và ảnh hưởng đến độ chính xác.
 *
 * @param accel_x  Gia tốc trục X từ MPU6050 (m/s^2, đã convert & trừ bias)
 * @param accel_y  Gia tốc trục Y từ MPU6050 (m/s^2, đã convert & trừ bias)
 * @param accel_z  Gia tốc trục Z từ MPU6050 (m/s^2, đã convert & trừ bias)
 * @param gyro_x   Tốc độ góc trục X từ MPU6050 (deg/s, đã convert & trừ bias)
 * @param gyro_y   Tốc độ góc trục Y từ MPU6050 (deg/s, đã convert & trừ bias)
 * @param gyro_z   Tốc độ góc trục Z từ MPU6050 (deg/s, đã convert & trừ bias)
 *
 * Yêu cầu dữ liệu đầu vào:
 *   - Dữ liệu phải được chuyển đổi từ raw và đã trừ bias (bởi
 *     mpu6050_convert_accel() và mpu6050_convert_gyro())
 *   - Gyro bias phải được hiệu chuẩn (bởi mpu6050_calibrate_finish())
 *     nếu không drift sẽ rất lớn
 */
void roll_pitch_update(float accel_x, float accel_y, float accel_z,
                      float gyro_x, float gyro_y, float gyro_z);

/**
 * @brief Lấy góc Roll hiện tại
 *
 * Roll là góc nghiêng của thiết bị quanh trục X (trục dọc).
 *
 * == QUY ƯỚC DẤU ==
 *   Roll > 0: Nghiêng sang PHẢI (bên phải thấp hơn)
 *   Roll < 0: Nghiêng sang TRÁI (bên trái thấp hơn)
 *   Roll = 0: Nằm ngang (cân bằng trái-phải)
 *
 * == GIÁ TRỊ ĐIỂN HÌNH ==
 *   Đứng thẳng, nhìn thẳng:       ≈ 0°
 *   Nghiêng người sang phải 30°:   ≈ 30°
 *   Nghiêng người sang trái 30°:   ≈ -30°
 *   Nằm nghiêng bên phải:          ≈ 90°
 *   Nằm nghiêng bên trái:          ≈ -90°
 *
 * @return float Góc Roll hiện tại (độ)
 */
float get_roll(void);

/**
 * @brief Lấy góc Pitch hiện tại
 *
 * Pitch là góc nghiêng của thiết bị quanh trục Y (trục ngang).
 *
 * == QUY ƯỚC DẤU ==
 *   Pitch > 0: CÚI XUỐNG (mặt trước hướng xuống đất)
 *   Pitch < 0: NGỬA LÊN (mặt trước hướng lên trời)
 *   Pitch = 0: Nằm ngang (cân bằng trước-sau)
 *
 * == GIÁ TRỊ ĐIỂN HÌNH ==
 *   Đứng thẳng:                     ≈ 0° (hoặc 90° tùy cách gắn)
 *   Cúi người xuống 45°:            ≈ 45°
 *   Nằm sấp (mặt hướng xuống):      ≈ 90°
 *   Ngửa mặt lên 30°:               ≈ -30°
 *
 * @return float Góc Pitch hiện tại (độ)
 */
float get_pitch(void);

#ifdef __cplusplus
}
#endif

#endif /* ROLL_PITCH_H */
