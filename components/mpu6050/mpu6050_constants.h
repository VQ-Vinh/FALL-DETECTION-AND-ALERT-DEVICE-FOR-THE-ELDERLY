/*
 * Hằng số dùng chung cho module MPU6050.
 * File này là nguồn chân lý duy nhất cho mọi thông số cảm biến -
 * thay đổi tại đây, các module #include sẽ tự đồng bộ.
 */

#ifndef MPU6050_CONSTANTS_H
#define MPU6050_CONSTANTS_H

/*
 * Hệ số chuyển raw → đơn vị vật lý, khớp với dải đo trong mpu6050_init().
 * Nếu đổi dải đo, phải cập nhật cả scale factor ở đây và giá trị thanh ghi trong .c.
 */
#define MPU6050_ACC_SCALE   4096.0f   /* ±8g → 4096 LSB/g */
#define MPU6050_GYRO_SCALE  16.4f     /* ±2000 deg/s → 16.4 LSB/deg/s */

/*
 * Gia tốc trọng trường (m/s²). Dùng để chuyển g → m/s² và kiểm tra trạng thái:
 * khi đứng yên tổng vector ≈ 9.81; té ngã: << 9.81 (rơi) hoặc >> 9.81 (va chạm).
 */
#define MPU6050_GRAVITY     9.81f

/*
 * Tần số lấy mẫu 100Hz (~10ms/lần). Đủ nhanh cho phát hiện té ngã (300-500ms),
 * thỏa Nyquist với chuyển động người (< 50Hz), không quá tải CPU.
 */
#define MPU6050_SAMPLE_RATE 100.0f

/*
 * Khoảng thời gian giữa hai mẫu (giây). Dùng cho tích phân gyro: góc += tốc_độ × dt.
 */
#define MPU6050_DT          (1.0f / MPU6050_SAMPLE_RATE)  /* 0.01s */

/*
 * Hệ số α cho Complementary Filter: 80% gyro + 20% accel.
 * Gyro đáp ứng nhanh, mượt; accel kéo góc về giá trị thực chống drift.
 * 0.80 là cân bằng phù hợp cho phát hiện té ngã.
 */
#define ROLL_PITCH_ALPHA    0.80f

#endif /* MPU6050_CONSTANTS_H */
