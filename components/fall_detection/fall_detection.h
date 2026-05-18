/**
 * @file fall_detection.h
 * @brief Header file cho module phát hiện ngã (Fall Detection)
 *
 * ===================== TỔNG QUAN THUẬT TOÁN =====================
 * Module này hiện thực state machine phát hiện ngã dành cho thiết bị đeo
 * trên người cao tuổi. Dữ liệu đầu vào là gia tốc (accel), vận tốc góc (gyro),
 * góc pitch và roll từ cảm biến MPU6050 (lấy qua bộ lọc bổ sung).
 *
 * Ý tưởng vật lý: Khi một người ngã, cơ thể trải qua 3 giai đoạn điển hình:
 *   1. RƠI TỰ DO (FREEFALL)  – Mất trọng lượng, accel ≈ 0g (rơi tự do)
 *   2. VA CHẠM (IMPACT)      – Đập xuống sàn, accel tăng đột biến ≥ 1.5-2.0g
 *   3. NẰM YÊN (LIE DOWN)    – Cơ thể nằm ngửa/sấp, góc nghiêng ≥ 70°, gyro thấp
 *
 * State machine 5 trạng thái:
 *   IDLE → FREEFALL → IMPACT → WAIT_LIE_DOWN → SOS_TRIGGERED
 *
 * Chu trình phát hiện đầy đủ:
 *   IDLE ——(accel < 0.5g)——→ FREEFALL ——(accel ≥ 2.0g)——→ IMPACT
 *     ↑                                                       |
 *     |←—(timeout 1s, góc < 70°)———|                          |
 *     |                                                     ↓
 *     |                                              WAIT_LIE_DOWN
 *     |←——(góc < 70° hoặc di chuyển)————|                |
 *     |                                                   ↓
 *     |←——(nút Reset)—— SOS_TRIGGERED ←—(3.5s, gyro thấp, accel ~1g)
 *
 * ===================== TẠI SAO LẠI CẦN NHIỀU TRẠNG THÁI? =====================
 * - Chỉ dùng ngưỡng accel đơn thuần sẽ gây dương tính giả (vỗ tay, nhảy, xuống bậc thang)
 * - Chỉ dùng góc nghiêng sẽ không phân biệt được nằm ngủ hay ngã
 * - Kết hợp: rơi → va chạm → nằm yên là signature đặc trưng của cú ngã thật
 */

#ifndef FALL_DETECTION_H
#define FALL_DETECTION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * ===================== fall_state_t =====================
 * @brief Các trạng thái trong state machine phát hiện ngã
 *
 * Giải thích chi tiết từng trạng thái:
 * ----------------------------------------------------------------
 * STATE_IDLE (0)
 *   - Trạng thái bình thường, thiết bị đang theo dõi liên tục.
 *   - Ở trạng thái này, accel trung bình ~1g (trọng lực), góc pitch/roll nhỏ (người đứng).
 *   - Nếu accel < ngưỡng freefall → chuyển sang FREEFALL.
 *
 * STATE_FREEFALL (1)
 *   - Phát hiện hiện tượng "mất trọng lượng": accel giảm đột ngột dưới 0.5g.
 *   - Khi người rơi tự do, cảm biến gần như không chịu gia tốc nào ngoài lực cản không khí.
 *   - Một timer 150ms được khởi động: nếu hết timer mà không có impact → reset về IDLE
 *     (đây là cơ chế chống nhiễu: rơi điện thoại, cúi nhanh cũng gây freefall ngắn).
 *
 * STATE_IMPACT (2)
 *   - Phát hiện va chạm: accel ≥ 2.0g (hoặc ngưỡng cấu hình).
 *   - Va chạm tạo ra xung lực lớn trong thời gian rất ngắn (10-50ms).
 *   - Timer 1s được khởi động: sau 1s kiểm tra góc nghiêng.
 *   - Nếu góc nghiêng ≥ 70° → người đang nằm → chuyển WAIT_LIE_DOWN.
 *   - Nếu góc < 70° → người đã đứng dậy → reset về IDLE (dương tính giả kiểu "ngã xong đứng dậy").
 *
 * STATE_WAIT_LIE_DOWN (3)
 *   - Giai đoạn xác nhận: đợi 3.5s để chắc chắn người đó nằm yên, không cựa quậy.
 *   - Kiểm tra đồng thời: (1) góc nghiêng ≥ 70°, (2) gyro < 20°/s (không chuyển động),
 *     (3) filtered_accel ≈ 1g (cơ thể ổn định trên sàn).
 *   - Nếu có chuyển động (gyro ≥ 20°/s) → reset bộ đếm, chờ lại.
 *   - Nếu góc < 70° (đứng dậy) → reset về IDLE ngay lập tức.
 *   - Hết 3.5s ổn định → chuyển SOS_TRIGGERED và gọi callback báo động.
 *
 * STATE_SOS_TRIGGERED (4)
 *   - Đã xác nhận ngã, báo động đang kích hoạt.
 *   - Chỉ thoát khi người dùng hoặc ứng dụng gọi fall_detection_reset().
 *   - Ở trạng thái này, module tiếp tục cập nhật filtered_accel/pitch/roll
 *     để ứng dụng có thể hiển thị trạng thái hiện tại.
 */
typedef enum {
    STATE_IDLE = 0,              /**< Bình thường, đang theo dõi, không phát hiện bất thường */
    STATE_FREEFALL = 1,          /**< Phát hiện mất trọng lượng (rơi tự do), accel < ngưỡng freefall */
    STATE_IMPACT = 2,            /**< Phát hiện va chạm, accel ≥ ngưỡng impact */
    STATE_WAIT_LIE_DOWN = 3,     /**< Đợi 3.5s xác nhận nạn nhân nằm yên, không cử động */
    STATE_SOS_TRIGGERED = 4      /**< Đã xác nhận ngã, kích hoạt SOS, chờ reset */
} fall_state_t;

/**
 * ===================== fall_detection_config_t =====================
 * @brief Cấu hình các ngưỡng phát hiện ngã và tham số thời gian
 *
 * Tất cả các tham số đều có thể tùy chỉnh để phù hợp với từng đối tượng người dùng.
 * Người cao tuổi thường ngã chậm hơn nên các timeout có thể cần tăng lên.
 *
 * Các thông số vật lý quan trọng:
 *   - filter_alpha: quyết định tốc độ đáp ứng của bộ lọc thông thấp.
 *     α = 0.5 → cân bằng giữa độ mượt và độ trễ (~20ms ở 100Hz).
 *     α = 0.8 → rất mượt nhưng trễ ~80ms, có thể bỏ sót impact.
 *   - accel_freefall_abs: ngưỡng phát hiện rơi tự do.
 *     1g = 9.8 m/s² là gia tốc trọng trường. Khi rơi tự do, accel ≈ 0g.
 *     Ngưỡng 0.5g là dung sai cho nhiễm cảm biến và chuyển động không lý tưởng.
 *   - accel_impact_abs: ngưỡng phát hiện va chạm.
 *     Ngã từ độ cao ~1m tạo xung ~2-3g. Cúi người nhanh có thể tạo ~1.5g.
 *   - lying_angle_threshold: góc nghiêng xác định "nằm".
 *     Người đứng: pitch, roll ≈ 0°. Người nằm: pitch hoặc roll ≥ 70° (gần ngang).
 */
typedef struct {
    float filter_alpha;                /**< Hệ số bộ lọc thông thấp (0.0-1.0). Giá trị càng lớn, tín hiệu càng mượt nhưng phản ứng càng chậm. Công thức: y(n) = α·y(n-1) + (1-α)·x(n) */
    float accel_freefall_abs;          /**< Ngưỡng gia tốc phát hiện rơi tự do (đơn vị: g). Giá trị tuyệt đối, thường ≤ 0.5g. Nếu accel < ngưỡng này → FREEFALL. */
    float accel_impact_abs;            /**< Ngưỡng gia tốc phát hiện va chạm (đơn vị: g). Giá trị tuyệt đối, thường ≥ 1.5-2.0g. Nếu accel ≥ ngưỡng này → IMPACT. */
    float lying_angle_threshold;       /**< Ngưỡng góc xác định trạng thái nằm (đơn vị: độ). Là góc lớn nhất giữa |pitch| và |roll|. Thường ≥ 70° (gần song song với mặt đất). */
    uint32_t timeout_freefall;         /**< Thời gian tối đa ở trạng thái FREEFALL (ms). Nếu quá thời gian này mà không có impact → reset. Tránh nhiễu rung lắc kéo dài. */
    uint32_t timeout_impact_check;     /**< Thời gian chờ sau impact (ms) trước khi kiểm tra góc. Cho cảm biến ổn định sau chấn động, thường 1000ms. */
    uint32_t wait_lie_down_time;       /**< Thời gian xác nhận nằm yên (ms). Nạn nhân phải nằm yên (gyro thấp, góc lớn) trong khoảng thời gian này mới xác nhận ngã. Thường 3500ms (3.5 giây). */
} fall_detection_config_t;

/**
 * ===================== fall_detection_result_t =====================
 * @brief Cấu trúc kết quả phát hiện ngã, được fall_detection_get_result() trả về
 *
 * Chứa đồng thời:
 *   - Kết luận (fall_detected): boolean đã ngã hay chưa
 *   - Trạng thái hiện tại (current_state): để UI hiển thị tiến trình
 *   - Giá trị cảm biến đã lọc (filtered): dữ liệu mượt, phù hợp hiển thị
 */
typedef struct {
    bool fall_detected;                 /**< Cờ báo đã phát hiện ngã (true nếu state = STATE_SOS_TRIGGERED) */
    fall_state_t current_state;         /**< Trạng thái hiện tại của state machine, dùng cho màn hình hoặc web dashboard */
    float current_accel_g;              /**< Gia tốc đã lọc (filtered) hiện tại, đơn vị g (1g ≈ 9.8m/s²). Ổn định hơn raw, phù hợp hiển thị. */
    float current_gyro_dps;             /**< Vận tốc góc đã lọc (filtered) hiện tại, đơn vị độ/giây. Dùng để kiểm tra chuyển động. */
    float current_pitch;                /**< Góc pitch đã lọc (filtered) hiện tại, đơn vị độ. Pitch = góc nghiêng trước-sau. */
    float current_roll;                 /**< Góc roll đã lọc (filtered) hiện tại, đơn vị độ. Roll = góc nghiêng trái-phải. */
} fall_detection_result_t;

/**
 * ===================== HÀM API =====================
 * Các hàm public của module fall_detection.
 * Quy trình sử dụng chuẩn:
 *   1. fall_detection_init() — khởi tạo với config mặc định, hoặc
 *      fall_detection_init_config(&my_config) — khởi tạo với config tùy chỉnh
 *   2. fall_detection_set_callback(my_callback) — đăng ký hàm callback khi ngã
 *   3. fall_detection_update(accel, gyro, pitch, roll) — gọi mỗi 10ms (100Hz)
 *   4. Khi có ngã → callback được gọi tự động
 *   5. fall_detection_reset() — reset sau khi xử lý ngã
 */

/**
 * @brief Khởi tạo module phát hiện ngã với cấu hình mặc định
 *
 * Gọi fall_detection_init_config(&default_config) với cấu hình đã được tối ưu
 * cho người cao tuổi. Nếu cần thay đổi ngưỡng, dùng fall_detection_init_config()
 * với cấu hình tự định nghĩa.
 */
void fall_detection_init(void);

/**
 * @brief Khởi tạo module phát hiện ngã với cấu hình tùy chỉnh
 *
 * @param config Con trỏ tới cấu hình fall_detection_config_t mong muốn.
 *               Hàm copy toàn bộ cấu hình vào biến static nội bộ.
 *
 * Quá trình khởi tạo:
 *   1. Copy config vào biến tĩnh (config = *cfg)
 *   2. Tạo mutex (xSemaphoreCreateMutex) để bảo vệ state giữa task chính và timer callback
 *   3. Reset tất cả biến state về giá trị ban đầu: current_state = IDLE,
 *      filtered_accel = 1.0g, filtered_pitch/roll = 0°
 *   4. Log thông số cấu hình để debug
 */
void fall_detection_init_config(const fall_detection_config_t *config);

/**
 * @brief Cập nhật dữ liệu cảm biến và chạy state machine 1 bước
 *
 * @param accel_g   Gia tốc tổng (magnitude) từ cảm biến, đơn vị g
 * @param gyro_dps  Vận tốc góc tổng (magnitude), đơn vị độ/giây
 * @param pitch     Góc pitch (nghiêng trước-sau), đơn vị độ
 * @param roll      Góc roll (nghiêng trái-phải), đơn vị độ
 *
 * Hàm này được thiết kế để gọi từ mpu6050_task ở tần số 100Hz (mỗi 10ms).
 *
 * Quy trình xử lý trong 1 lần gọi:
 *   1. LOW-PASS FILTER: Lọc tất cả 4 giá trị đầu vào để loại bỏ nhiễu cảm biến
 *   2. TÍNH max_tilt: max(|pitch|, |roll|) — góc nghiêng lớn nhất
 *   3. STATE MACHINE: Dùng raw_accel cho freefall/impact (phản ứng nhanh)
 *                     Dùng filtered cho lying detection (ổn định)
 *   4. GIẢI PHÓNG MUTEX
 *
 * @note Raw accel được dùng cho freefall/impact vì đây là các sự kiện nhanh (ms),
 *       cần phát hiện tức thời. Filtered accel sẽ làm trễ sự kiện ~20-50ms,
 *       có thể bỏ lỡ xung impact ngắn.
 */
void fall_detection_update(float accel_g, float gyro_dps, float pitch, float roll);

/**
 * @brief Lấy toàn bộ kết quả phát hiện ngã hiện tại
 *
 * @return fall_detection_result_t Cấu trúc chứa trạng thái, các giá trị cảm biến đã lọc,
 *                                  và cờ fall_detected.
 *
 * Hàm này đọc mutex-safe: lấy mutex → copy dữ liệu → trả mutex.
 */
fall_detection_result_t fall_detection_get_result(void);

/**
 * @brief Kiểm tra nhanh xem đã kích hoạt báo động chưa
 *
 * @return true  Nếu state == STATE_SOS_TRIGGERED và alert_triggered == true
 * @return false Nếu chưa có báo động hoặc đã reset
 *
 * Đây là hàm lightweight, mutex-safe, dùng cho các task kiểm tra định kỳ.
 */
bool fall_detection_is_alert_triggered(void);

/**
 * @brief Reset module về trạng thái IDLE, dừng tất cả timer
 *
 * Hàm này được gọi khi:
 *   - Người dùng nhấn nút Reset trên thiết bị
 *   - Ứng dụng muốn tắt báo động sau khi đã xử lý (ví dụ: đã gửi tin nhắn Telegram)
 *   - Phát hiện dương tính giả, cần khởi động lại quá trình theo dõi
 *
 * Tác dụng:
 *   1. Set current_state = STATE_IDLE, alert_triggered = false
 *   2. Reset filtered_accel = 1.0g (trọng lực tiêu chuẩn), filtered_pitch/roll = 0°
 *   3. Dừng và hủy timer freefall và impact nếu đang chạy
 */
void fall_detection_reset(void);

/**
 * @brief Định nghĩa kiểu callback được gọi khi phát hiện ngã
 *
 * Callback này chạy trong context của task gọi fall_detection_update()
 * (thường là mpu6050_task). Trong callback KHÔNG được gọi các hàm blocking
 * hoặc delay dài.
 *
 * Ứng dụng thường dùng callback để:
 *   - Bật còi báo động (buzzer)
 *   - Gửi tin nhắn Telegram/SMS
 *   - Gửi tín hiệu đến server
 *
 * @note Callback được gọi trong khi mutex đã được release để tránh deadlock
 *       (xem chi tiết trong fall_detection.c STATE_SOS_TRIGGERED).
 */
typedef void (*fall_alert_callback_t)(void);

/**
 * @brief Đăng ký hàm callback sẽ được gọi khi phát hiện ngã
 *
 * @param callback Con trỏ hàm kiểu fall_alert_callback_t.
 *                 Truyền NULL để hủy đăng ký.
 *
 * @note Callback chỉ nên set 1 lần trong quá trình init.
 *       Không có cơ chế bảo vệ mutex cho việc set callback (chỉ đơn giản gán).
 */
void fall_detection_set_callback(fall_alert_callback_t callback);

/**
 * @brief Lấy trạng thái hiện tại của state machine (kiểu fall_state_t)
 *
 * @return fall_state_t Giá trị enum từ STATE_IDLE đến STATE_SOS_TRIGGERED
 *
 * Wrapper của fall_detection_get_state_internal() với kiểu trả về fall_state_t.
 */
fall_state_t fall_detection_get_state(void);

/**
 * ===================== INTERNAL GETTERS (cho Webserver) =====================
 * Các hàm này được thiết kế riêng cho webserver dashboard.
 * Webserver cần đọc state mà không phụ thuộc vào kiểu dữ liệu enum.
 * Các hàm này vẫn dùng mutex để đảm bảo an toàn.
 */

/**
 * @brief Lấy trạng thái dưới dạng uint8_t (cho webserver JSON)
 *
 * @return uint8_t 0=IDLE, 1=FREEFALL, 2=IMPACT, 3=WAIT_LIE_DOWN, 4=SOS_TRIGGERED
 */
uint8_t fall_detection_get_state_internal(void);

/**
 * @brief Lấy góc nghiêng lớn nhất (max(|pitch|, |roll|)) không qua mutex
 *
 * @return float Góc nghiêng lớn nhất ở thời điểm hiện tại, đơn vị độ
 *
 * @note Hàm này chỉ đọc biến float (atomic trên hầu hết vi điều khiển 32-bit),
 *       nên không cần mutex. Dùng trong webserver để tránh blocking.
 */
float fall_detection_get_max_tilt_internal(void);

/**
 * @brief Lấy giá trị gia tốc đã lọc (không qua mutex)
 *
 * @return float Giá trị filtered_accel hiện tại, đơn vị g
 *
 * @note Tương tự fall_detection_get_max_tilt_internal(), đọc atomic,
 *       không cần mutex.
 */
float fall_detection_get_filtered_accel_internal(void);

#ifdef __cplusplus
}
#endif

#endif // FALL_DETECTION_H
