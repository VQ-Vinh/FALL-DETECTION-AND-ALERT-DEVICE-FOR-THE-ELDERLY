/**
 * @file mpu6050_constants.h
 * @brief Shared constants cho MPU6050 sensor configuration
 *
 * Cac gia tri nay phai dong nhat trong toan bo project:
 * - mpu6050.c: doc du lieu raw, convert sang don vi vat ly
 * - roll_pitch.c: tinh goc tu du lieu accel/gyro
 */

#ifndef MPU6050_CONSTANTS_H
#define MPU6050_CONSTANTS_H

/**
 * @brief MPU6050 scale factors - phai khop voi cau hinh phan cung
 *
 * ACC_SCALE = 4096 LSB/g voi day ±8g
 *   -> raw = 4096 khi gia_toc = 1g (9.81 m/s²)
 *
 * GYRO_SCALE = 16.4 LSB/deg/s voi day ±2000 deg/s
 *   -> raw = 16.4 khi toc_do = 1 deg/s
 */
#define MPU6050_ACC_SCALE   4096.0f
#define MPU6050_GYRO_SCALE  16.4f

/**
 * @brief Gia toc tro luc
 */
#define MPU6050_GRAVITY     9.81f

/**
 * @brief Tan so lay mau (Hz)
 */
#define MPU6050_SAMPLE_RATE 100.0f   // 100Hz = 10ms/sample

/**
 * @brief Thoi gian giua cac mau (giay)
 */
#define MPU6050_DT          (1.0f / MPU6050_SAMPLE_RATE)  // 0.01s = 10ms

/**
 * @brief Complementary filter alpha
 *   α = 0.80: 80% gyro, 20% accel (phan ung nhanh hon)
 *   α = 0.98: 98% gyro, 2% accel (on dinh hon, chong drift)
 */
#define ROLL_PITCH_ALPHA    0.80f

#endif // MPU6050_CONSTANTS_H
