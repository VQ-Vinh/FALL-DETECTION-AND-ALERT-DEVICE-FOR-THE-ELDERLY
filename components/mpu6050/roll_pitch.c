/**
 * @file roll_pitch.c
 * @brief Triển khai tính toán góc Roll/Pitch bằng Complementary Filter
 *
 * Module này chịu trách nhiệm ước lượng góc nghiêng của thiết bị theo
 * hai trục:
 *   - Roll:  góc nghiêng trái/phải (quanh trục X)
 *   - Pitch: góc nghiêng tiến/lùi (quanh trục Y)
 *
 * == VẤN ĐỀ CẦN GIẢI QUYẾT ==
 * Cả accelerometer và gyroscope đều có điểm yếu riêng:
 *   - Accelerometer: cho góc tuyệt đối (không drift) nhưng NHIỄU khi
 *     có rung động (do phản ứng với gia tốc tức thời, không chỉ trọng lực)
 *   - Gyroscope: cho giá trị MƯỢT, ỔN ĐỊNH nhưng bị DRIFT (sai số tích lũy
 *     qua tích phân, mỗi giây sai thêm vài độ)
 *
 * == GIẢI PHÁP: COMPLEMENTARY FILTER (LỌC BÙ) ==
 * Kết hợp ưu điểm của cả hai: gyro cho đáp ứng nhanh, mượt; accel
 * "kéo" góc về giá trị thực để chống drift.
 *
 * Công thức:
 *   góc = α × (góc_cũ + gyro × dt) + (1-α) × góc_accel
 *
 * Với α = 0.80 (xem mpu6050_constants.h):
 *   - 80% tin vào gyroscope (tích phân, mượt)
 *   - 20% tin vào accelerometer (chống drift)
 */

/* ======================================================================== *
 *                            THƯ VIỆN                                     *
 * ======================================================================== */

#include "roll_pitch.h"            /* API roll_pitch */
#include "mpu6050_constants.h"     /* MPU6050_DT, ROLL_PITCH_ALPHA */
#include <math.h>                  /* atan2f, sqrtf, M_PI */
#include "esp_timer.h"             /* esp_timer_get_time() cho timestamp */

/* ======================================================================== *
 *                      BIẾN STATIC (NỘI BỘ MODULE)                        *
 * ======================================================================== */

/**
 * @brief Góc Roll và Pitch hiện tại (đơn vị: độ)
 *
 * Đây là kết quả đầu ra của complementary filter, được cập nhật mỗi lần
 * gọi roll_pitch_update(). Các giá trị này được duy trì liên tục qua các
 * lần cập nhật (không reset về 0 giữa các lần gọi).
 *
 * Công thức cập nhật cuối cùng:
 *   roll_new  = α × (roll_old  + gyro_x × dt) + (1-α) × accel_roll
 *
 * Trong đó:
 *   - roll_old: giá trị roll từ lần cập nhật trước
 *   - gyro_x × dt: tích phân tốc độ góc (thay đổi góc trong dt)
 *   - accel_roll: góc tuyệt đối từ accelerometer (atan2)
 *
 * Lưu ý: Giá trị khởi tạo = 0. Nếu thiết bị được đặt nghiêng khi bật,
 * cần vài lần cập nhật để filter "hội tụ" về giá trị đúng (do accel
 * kéo dần về giá trị thực qua hệ số 1-α).
 */
static float roll = 0.0f;   /* Góc Roll: nghiêng trái/phải (quanh trục X), đơn vị độ */
static float pitch = 0.0f;  /* Góc Pitch: nghiêng tiến/lùi (quanh trục Y), đơn vị độ */

/**
 * @brief Timestamp của lần cập nhật trước (microsecond)
 *
 * Dùng để tính dt thực tế giữa hai lần gọi roll_pitch_update().
 * Việc tính dt chính xác rất quan trọng cho tích phân gyroscope:
 *   Sai số 1ms trong dt = sai số 0.1 độ với gyro 100 deg/s
 *
 * Sử dụng esp_timer_get_time() (độ phân giải microsecond) thay vì
 * giả định dt = 10ms, vì trong thực tế có thể có:
 *   - Jitter (sai lệch thời gian do hệ điều hành)
 *   - Task bị preempt bởi tác vụ khác
 *   - Delay không đồng đều trong vòng lặp
 */
static int64_t prev_timestamp_us = 0;  /* Timestamp lần trước (µs) */

/* ======================================================================== *
 *                      KHỞI TẠO                                           *
 * ======================================================================== */

/**
 * @brief Khởi tạo module roll_pitch
 *
 * Đặt góc roll và pitch về 0, reset timestamp.
 * Nên gọi hàm này một lần khi khởi động hệ thống, trước vòng lặp chính.
 *
 * Lưu ý: Nếu thiết bị được đặt ở tư thế không nằm ngang, góc sẽ hội tụ
 * dần về giá trị đúng trong 3-5 lần cập nhật đầu tiên (do α=0.80,
 * mỗi lần chỉ thay đổi 20% theo hướng của accel).
 */
void roll_pitch_init(void) {
    roll = 0.0f;               /* Reset góc Roll về 0 */
    pitch = 0.0f;              /* Reset góc Pitch về 0 */
    prev_timestamp_us = 0;     /* Reset timestamp (báo hiệu lần gọi đầu) */
}

/* ======================================================================== *
 *                      CẬP NHẬT GÓC (CORE ALGORITHM)                      *
 * ======================================================================== */

/**
 * @brief Cập nhật góc Roll và Pitch với dữ liệu cảm biến mới
 *
 * Đây là hàm quan trọng nhất của module, thực hiện 3 bước:
 *
 *   Bước 1 - TÍNH DT: Tính khoảng thời gian từ lần gọi trước đến lần
 *   gọi hiện tại. Quan trọng cho tích phân gyroscope chính xác.
 *
 *   Bước 2 - GÓC TỪ ACCEL: Tính góc tuyệt đối từ vector trọng lực
 *   đo bởi accelerometer. Đây là giá trị "tham chiếu" chống drift.
 *
 *   Bước 3 - COMPLEMENTARY FILTER: Kết hợp tích phân gyro (mượt)
 *   và góc accel (tuyệt đối) để có góc ước lượng tối ưu.
 *
 * @param accel_x  Gia tốc trục X (m/s^2) - từ mpu6050_convert_accel
 * @param accel_y  Gia tốc trục Y (m/s^2)
 * @param accel_z  Gia tốc trục Z (m/s^2)
 * @param gyro_x   Tốc độ góc trục X (deg/s) - từ mpu6050_convert_gyro
 * @param gyro_y   Tốc độ góc trục Y (deg/s)
 * @param gyro_z   Tốc độ góc trục Z (deg/s) - không dùng trực tiếp
 */
void roll_pitch_update(float accel_x, float accel_y, float accel_z,
                       float gyro_x, float gyro_y, float gyro_z) {

    /* ================================================================== *
     *  BƯỚC 1: TÍNH DT (DELTA TIME) THỰC TẾ
     *
     *  dt = thời gian từ lần cập nhật trước đến lần này (giây)
     *
     *  Lần đầu gọi (prev_timestamp_us == 0): dùng dt mặc định 10ms.
     *  Các lần sau: tính từ timestamp thực tế.
     *
     *  TẠI SAO CẦN GIỚI HẠN DT?
     *    - dt < 0.001s (1ms): Không thể xảy ra trong thực tế (vòng lặp
     *      100Hz = 10ms), có thể do lỗi timer. Dùng dt tối thiểu để
     *      tránh chia cho 0 hoặc kết quả vô lý.
     *    - dt > 0.050s (50ms): Có thể xảy ra nếu task bị treo hoặc bị
     *      preempt lâu. Giới hạn dt để tránh tích phân quá mức (jump)
     *      khi quay lại.
     * ================================================================== */
    int64_t current_timestamp_us = esp_timer_get_time();  /* Lấy thời gian hiện tại (µs) */
    float dt;                                              /* Delta time (giây) */

    if (prev_timestamp_us == 0) {
        /* Lần gọi đầu tiên: chưa có timestamp trước, dùng giá trị mặc định */
        dt = MPU6050_DT;  /* 10ms (từ hằng số, xem mpu6050_constants.h) */
    } else {
        /* Tính dt từ timestamp thực tế */
        dt = (current_timestamp_us - prev_timestamp_us) / 1000000.0f;  /* µs → s */

        /* Giới hạn dt để tránh các giá trị bất thường */
        if (dt < 0.001f) dt = 0.001f;   /* Tối thiểu 1ms */
        if (dt > 0.050f) dt = 0.050f;   /* Tối đa 50ms */
    }
    prev_timestamp_us = current_timestamp_us;  /* Lưu cho lần gọi sau */

    /* ================================================================== *
     *  BƯỚC 2: TÍNH GÓC TỪ ACCELEROMETER
     *
     *  Khi thiết bị đứng yên, vector gia tốc đo được CHỈ là trọng lực
     *  hướng xuống dưới. Từ hướng của vector này so với các trục cảm
     *  biến, ta tính được góc nghiêng.
     *
     *  CÔNG THỨC (chuyển đổi từ toán học sang thực tế):
     *
     *  ROLL = atan2(-Ay, Az) × 180/π - 90°
     *
     *  Giải thích:
     *    - Công thức chuẩn: roll_accel = atan2(Ay, Az)
     *    - Nhưng MPU6050 có trục Y dương ở mặt bên trái, nên cần đảo
     *      dấu: atan2(-Ay, Az)
     *    - Trừ 90° vì khi nằm ngang, Az = g (dương), Ay = 0:
     *        atan2(0, g) = 0°, nhưng thực tế roll = 0°
     *        Muốn được vậy: atan2(-0, g) - 90° = 0 - 90° = -90° → cần
     *        điều chỉnh công thức cho phù hợp.
     *      Thực tế: Công thức (1) cho kết quả đúng trong dải [-180, 180]
     *
     *  PITCH = atan2(-Ax, √(Ay² + Az²)) × 180/π
     *
     *  Giải thích:
     *    - Công thức chuẩn: pitch = atan2(-Ax, sqrt(Ay² + Az²))
     *    - sqrt(Ay² + Az²) là hình chiếu của trọng lực lên mặt phẳng YZ
     *    - Dùng atan2 thay vì atan để có kết quả trong [-180, 180]
     *    - Dấu âm ở Ax: vì MPU6050 trục X dương hướng ra trước,
     *      khi cúi xuống, Ax dương → pitch dương (cúi = dương)
     *
     *  GIỚI HẠN CỦA GÓC ACCEL:
     *    - Chỉ chính xác khi thiết bị KHÔNG có gia tốc (đứng yên
     *      hoặc chuyển động đều, v = const)
     *    - Khi có gia tốc ngoài (rung, lắc, té...), kết quả bị nhiễu
     *    - Đây chính là lý do cần complementary filter
     * ================================================================== */

    /* Roll từ accel (nghiêng trái/phải, quanh trục X) */
    float accel_roll = atan2f(-accel_y, accel_z) * 180.0f / M_PI - 90.0f;

    /* Pitch từ accel (nghiêng tiến/lùi, quanh trục Y) */
    float accel_pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;

    /* ================================================================== *
     *  BƯỚC 3: TÍCH PHÂN GYROSCOPE
     *
     *  Gyroscope đo TỐC ĐỘ GÓC (deg/s), không phải góc.
     *  Để có góc, ta phải TÍCH PHÂN: góc = ∫(tốc_độ) dt
     *
     *  Trong thực tế (xử lý rời rạc, discrete-time):
     *    góc_mới = góc_cũ + tốc_độ × dt
     *
     *  Ví dụ: gyro_x = 50 deg/s, dt = 0.01s (10ms)
     *    → roll tăng thêm 50 × 0.01 = 0.5 độ
     *
     *  TẠI SAO CHỈ DÙNG GYRO_X CHO ROLL VÀ GYRO_Y CHO PITCH?
     *    - Roll là góc quanh trục X → dùng gyro_x
     *    - Pitch là góc quanh trục Y → dùng gyro_y
     *    - gyro_z (xoay quanh trục Z = Yaw) không dùng trong project này
     *
     *  VẤN ĐỀ DRIFT:
     *    Mỗi lần tích phân, sai số nhỏ trong gyro (sau khi trừ bias)
     *    được cộng dồn. Ví dụ bias còn 0.5 deg/s:
     *      Sau 1 giây: sai 0.5°
     *      Sau 60 giây: sai 30°
     *      Sau 10 phút: sai 300° (hoàn toàn vô dụng!)
     *    → Bước 4 (complementary filter) giải quyết drift này.
     * ================================================================== */

    /* Tích phân gyro: cộng dồn tốc độ góc * dt vào góc hiện tại */
    roll  += gyro_x * dt;    /* Roll: tích phân tốc độ quay quanh trục X */
    pitch += gyro_y * dt;    /* Pitch: tích phân tốc độ quay quanh trục Y */

    /* ================================================================== *
     *  BƯỚC 4: COMPLEMENTARY FILTER (LỌC BÙ)
     *
     *  Công thức:
     *    góc_final = α × góc_gyro + (1-α) × góc_accel
     *
     *  Trong đó:
     *    - góc_gyro  = roll_old + gyro_x × dt  (từ bước 3, đã gán vào roll)
     *    - góc_accel = accel_roll              (từ bước 2)
     *    - α = 0.80 (xem mpu6050_constants.h)
     *
     *  == GIẢI THÍCH TRỰC GIÁC ==
     *  Hãy tưởng tượng bạn đang đi trên một con đường:
     *    - Gyroscope: đôi mắt nhắm nghiền, bước đều nhưng lệch dần (drift)
     *    - Accelerometer: GPS, luôn biết vị trí chính xác nhưng bị nhiễu
     *    - Complementary filter: mở mắt 20% thời gian để nhìn GPS và
     *      điều chỉnh hướng đi, 80% còn lại tin vào bước đi của mình
     *
     *  == PHÂN TÍCH TẦN SỐ ==
     *  Complementary filter hoạt động như một bộ lọc thông thấp (low-pass)
     *  cho accel (chặn nhiễu tần số cao) và thông cao (high-pass) cho gyro
     *  (chặn drift tần số thấp):
     *    - Accel: tín hiệu nhiễu tần số cao (rung động) bị chặn bởi (1-α)
     *    - Gyro: drift tần số thấp (trôi chậm) bị chặn bởi α (chỉ lấy
     *      thay đổi ngắn hạn)
     *
     *  == TẦN SỐ CẮT ==
     *  fc = (1-α) / (2π × dt) = 0.20 / (2π × 0.01) ≈ 3.18 Hz
     *  → Tín hiệu accel > 3.18Hz bị suy giảm (nhiễu rung)
     *  → Tín hiệu gyro < 3.18Hz bị suy giảm (drift chậm)
     *  → Đây là tần số cắt hợp lý cho chuyển động của con người
     * ================================================================== */

    /* Complementary filter: kết hợp gyro (từ bước 3, đã lưu trong roll/pitch)
     * và accel (từ bước 2) theo tỷ lệ α:(1-α) */
    roll  = ROLL_PITCH_ALPHA * roll  + (1.0f - ROLL_PITCH_ALPHA) * accel_roll;
    pitch = ROLL_PITCH_ALPHA * pitch + (1.0f - ROLL_PITCH_ALPHA) * accel_pitch;

    /* Kết thúc một lần cập nhật. roll và pitch đã có giá trị mới,
     * sẵn sàng cho lần gọi get_roll()/get_pitch() tiếp theo. */
}

/* ======================================================================== *
 *                      LẤY GIÁ TRỊ GÓC                                    *
 * ======================================================================== */

/**
 * @brief Lấy góc Roll hiện tại (độ)
 *
 * Góc Roll là góc nghiêng của thiết bị quanh trục X (trục dọc).
 * Dương khi nghiêng sang phải, âm khi nghiêng sang trái.
 *
 * Giá trị này được cập nhật mỗi khi gọi roll_pitch_update().
 * Nên gọi sau mỗi lần cập nhật để lấy kết quả mới nhất.
 *
 * @return float Góc Roll hiện tại (độ), trong khoảng [-180, 180]
 */
float get_roll(void) {
    return roll;  /* Trả về góc Roll (static) */
}

/**
 * @brief Lấy góc Pitch hiện tại (độ)
 *
 * Góc Pitch là góc nghiêng của thiết bị quanh trục Y (trục ngang).
 * Dương khi cúi xuống (mặt trước hướng xuống), âm khi ngửa lên.
 *
 * @return float Góc Pitch hiện tại (độ), trong khoảng [-180, 180]
 */
float get_pitch(void) {
    return pitch;  /* Trả về góc Pitch (static) */
}
