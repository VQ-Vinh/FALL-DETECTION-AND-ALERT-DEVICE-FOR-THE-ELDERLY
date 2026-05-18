/**
 * @file mpu6050_constants.h
 * @brief Các hằng số dùng chung cho toàn bộ project MPU6050
 *
 * File này đóng vai trò là "nguồn chân lý duy nhất" (single source of truth)
 * cho tất cả thông số cấu hình cảm biến. Bất kỳ thay đổi nào về thang đo,
 * tần số lấy mẫu hay bộ lọc đều chỉ cần sửa tại đây, các module khác
 * (mpu6050.c, roll_pitch.c) sẽ tự động đồng bộ qua #include.
 *
 * Các module sử dụng:
 *   - mpu6050.c:     đọc raw data, chuyển đổi sang đơn vị vật lý (m/s^2, deg/s)
 *   - roll_pitch.c:  tính góc nghiêng Roll/Pitch từ dữ liệu accel và gyro
 */

#ifndef MPU6050_CONSTANTS_H
#define MPU6050_CONSTANTS_H

/**
 * @brief Hệ số chuyển đổi từ giá trị raw sang đơn vị vật lý
 *
 * == GIẢI THÍCH CHI TIẾT ==
 *
 * MPU6050 trả về giá trị raw dạng int16 (số nguyên có dấu 16-bit).
 * Để chuyển sang đơn vị vật lý, ta cần biết "độ nhạy" (scale factor)
 * ứng với dải đo đã cấu hình.
 *
 * ACC_SCALE = 4096 (LSB/g)  Ứng với dải ±8g:
 *   Cảm biến MPU6050 có các dải đo: ±2g, ±4g, ±8g, ±16g.
 *   Với dải ±8g, độ nhạy là 4096 LSB/g (theo datasheet).
 *   Công thức: Gia tốc (g) = raw_value / 4096
 *   → Nếu raw = 4096 thì gia tốc = 1g = 9.81 m/s^2
 *   → Giá trị raw tối đa: ±32768 / 4096 ≈ ±8g (đúng với dải đã chọn)
 *
 * GYRO_SCALE = 16.4 (LSB/deg/s)  Ứng với dải ±2000 deg/s:
 *   Các dải đo: ±250, ±500, ±1000, ±2000 deg/s.
 *   Với dải ±2000 deg/s, độ nhạy là 16.4 LSB/deg/s.
 *   Công thức: Tốc độ góc (deg/s) = raw_value / 16.4
 *   → Nếu raw = 16.4 thì tốc độ góc = 1 deg/s
 *   → Giá trị raw tối đa: ±32768 / 16.4 ≈ ±2000 deg/s
 *
 * LƯU Ý QUAN TRỌNG: Các hằng số này PHẢI khớp với cấu hình phần cứng
 * được thiết lập trong mpu6050_init() (thanh ghi ACCEL_CONFIG và GYRO_CONFIG).
 * Nếu thay đổi dải đo, phải cập nhật cả scale factor ở ĐÂY và giá trị
 * ghi vào thanh ghi trong file .c.
 */
#define MPU6050_ACC_SCALE   4096.0f   /* LSB/g: 1g tương ứng 4096 đơn vị raw */
#define MPU6050_GYRO_SCALE  16.4f     /* LSB/deg/s: 1 deg/s tương ứng 16.4 đơn vị raw */

/**
 * @brief Gia tốc trọng trường Trái Đất (m/s^2)
 *
 * Giá trị tiêu chuẩn: 9.81 m/s^2 (tại vĩ độ 45° trên mực nước biển).
 * Được dùng để:
 *   - Chuyển đổi:  raw_value / ACC_SCALE * GRAVITY = m/s^2
 *   - Kiểm tra:    tổng vector gia tốc khi đứng yên ≈ 9.81 m/s^2 (≈ 1g)
 *   - Phát hiện té ngã: nếu tổng vector << 9.81 hoặc >> 9.81, có dấu hiệu té
 */
#define MPU6050_GRAVITY     9.81f     /* Gia tốc trọng trường (m/s^2) */

/**
 * @brief Tần số lấy mẫu (Sample Rate) - đơn vị: Hz
 *
 * MPU6050 được cấu hình lấy mẫu ở 100Hz, tức 100 mẫu mỗi giây.
 * Mỗi mẫu cách nhau 10ms:
 *   - dt = 1 / 100 = 0.01s = 10ms
 *
 * TẠI SAO CHỌN 100Hz?
 *   - Chuyển động của con người (đi bộ, té ngã) có tần số < 50Hz
 *   - Lấy mẫu 100Hz thỏa mãn định lý Nyquist (cần ≥ 2x tần số tín hiệu)
 *   - Đủ nhanh để phát hiện té ngã (thường diễn ra trong 300-500ms)
 *   - Không quá nhanh để tránh quá tải CPU và tiêu tốn năng lượng
 *
 * Giá trị này được dùng làm dt mặc định trong roll_pitch_update().
 */
#define MPU6050_SAMPLE_RATE 100.0f    /* 100Hz → mỗi giây 100 mẫu */

/**
 * @brief Khoảng thời gian giữa hai mẫu liên tiếp (đơn vị: giây)
 *
 * dt = 1 / 100 = 0.01s = 10ms
 *
 * dt được dùng trong:
 *   - roll_pitch_update(): tích phân tốc độ góc (gyro * dt) để tính góc
 *   - Tích phân gyro:  góc_mới = góc_cũ + tốc_độ_góc * dt
 *       Ví dụ: roll = roll + gyro_x * dt
 *       Nếu gyro_x = 50 deg/s, dt = 0.01s → roll tăng thêm 0.5 độ
 *
 * Khi dt quá lớn hoặc quá nhỏ, sai số tích phân sẽ tăng lên đáng kể.
 */
#define MPU6050_DT          (1.0f / MPU6050_SAMPLE_RATE)  /* 0.01 giây = 10ms */

/**
 * @brief Hệ số alpha cho Complementary Filter (bộ lọc bù)
 *
 * == GIẢI THÍCH CHI TIẾT ==
 *
 * Complementary Filter kết hợp ƯU ĐIỂM của hai nguồn dữ liệu để cho ra
 * góc ước lượng tốt nhất:
 *
 *   góc_ra = α × (góc_trước + gyro × dt) + (1 - α) × góc_accel
 *
 * Trong đó:
 *   - α (alpha): trọng số dành cho gyroscope (0 < α < 1)
 *   - góc_trước + gyro × dt: tích phân góc từ gyroscope
 *       * Ưu điểm: mượt, ổn định, đáp ứng nhanh với chuyển động
 *       * Nhược điểm: trôi dần theo thời gian (drift) do sai số tích lũy
 *   - góc_accel: góc tính từ gia tốc kế (atan2)
 *       * Ưu điểm: giá trị tuyệt đối, không bị drift
 *       * Nhược điểm: nhiễu khi có rung động, không đáp ứng tốt khi chuyển động nhanh
 *
 * α = 0.80 được chọn cho project này (cân bằng giữa responsiveness và độ ổn định):
 *
 *   80% gyro (nhanh, mượt) + 20% accel (sửa drift)
 *
 * SO SÁNH CÁC GIÁ TRỊ α:
 *   α = 0.98:  ưu tiên gyro tối đa → góc rất mượt, nhưng drift chậm (phù hợp máy bay)
 *   α = 0.80:  cân bằng → phản ứng nhanh, drift thấp (phù hợp phát hiện té ngã)
 *   α = 0.50:  accel và gyro ngang hàng → góc dễ bị nhiễu rung
 *   α = 0.20:  ưu tiên accel → góc tuyệt đối nhưng nhiễu, giật cục
 */
#define ROLL_PITCH_ALPHA    0.80f     /* Cân bằng: 80% gyro + 20% accel */

#endif /* MPU6050_CONSTANTS_H */
