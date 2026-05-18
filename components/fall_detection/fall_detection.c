/**
 * @file fall_detection.c
 * @brief Hiện thực thuật toán phát hiện ngã (Fall Detection) dùng state machine + FreeRTOS timers
 *
 * ===================== GIỚI THIỆU =====================
 * Module này phát hiện ngã ở người cao tuổi thông qua cảm biến MPU6050.
 * Thuật toán dựa trên 3 đặc điểm vật lý không thể thiếu của một cú ngã thật:
 *   (1) Rơi tự do trong thời gian ngắn (accel ~ 0g)
 *   (2) Va chạm mạnh (accel ≥ 1.5-2.0g)
 *   (3) Nằm yên sau va chạm (góc nghiêng ≥ 70°, không di chuyển)
 *
 * Nếu thiếu bất kỳ 1 trong 3 giai đoạn trên, sự kiện bị coi là dương tính giả và bỏ qua.
 *
 * ===================== STATE MACHINE =====================
 *                ┌──────────────────────────────────────────────────┐
 *                │                    IDLE                         │
 *                │  (accel ~1g, góc nhỏ, đang theo dõi)           │
 *                └─────────┬─────────┬─────────────────────────────┘
 *                           │         ▲
 *              accel < 0.5g │         │ reset (nút hoặc ứng dụng)
 *                           │         │
 *                ┌─────────▼─────────┴─────────────────────────────┐
 *                │                 FREEFALL                        │
 *                │  (mất trọng lượng, đang rơi)                    │
 *                │  Timer 150ms đếm ngược                          │
 *                └─────────┬─────────┬─────────────────────────────┘
 *                           │         ▲
 *               accel ≥ 2g │         │ timeout 150ms (không có impact)
 *                           │         │ → coi là nhiễu, reset
 *                ┌─────────▼─────────┴─────────────────────────────┐
 *                │                 IMPACT                          │
 *                │  (va chạm mạnh, chấn động)                      │
 *                │  Timer 1000ms đếm ngược                         │
 *                └─────────┬─────────┬─────────────────────────────┘
 *                           │         │
 *           timer 1s fire,  │         │ timer 1s fire,
 *           góc ≥ 70°      │         │ góc < 70°
 *                           │         │ (đã đứng dậy)
 *                ┌─────────▼─────────┴─────────────────────────────┐
 *                │              WAIT_LIE_DOWN                      │
 *                │  (xác nhận nằm yên 3.5s)                        │
 *                │  Kiểm tra: góc ≥ 70° + gyro < 20°/s + accel~1g │
 *                └──────┬──────────────────┬───────────────────────┘
 *                        │                  │
 *              3.5s ổn định               │ góc < 70° hoặc
 *              + gyro thấp                │ có chuyển động
 *              + accel ~1g                │
 *                        │                  │
 *                ┌───────▼──────────────────▼───────────────────────┐
 *                │           SOS_TRIGGERED              → IDLE     │
 *                │  (báo động đã kích hoạt)   (reset từ ngoài)     │
 *                └─────────────────────────────────────────────────┘
 *
 * ===================== GIẢI THÍCH VẬT LÝ =====================
 * - Khi người đứng yên, cảm biến đo được 1g theo phương thẳng đứng (trọng lực).
 * - Khi rơi tự do, cảm biến ở trạng thái không trọng lượng (microgravity) ≈ 0g.
 * - Khi va chạm, mặt đất tạo phản lực lên cơ thể, accel tăng đột biến (2-3g).
 * - Sau va chạm, cơ thể nằm ngang (pitch hoặc roll ≥ 70°) và ổn định (gyro thấp).
 */

#include "fall_detection.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include <math.h>

/**
 * @brief Tag cho hệ thống log ESP (dùng ESP_LOGW, ESP_LOGI, ESP_LOGD)
 *
 * Tag này xuất hiện trong mọi dòng log để xác định nguồn là module fall_detection.
 * Ví dụ: "FALL_DETECT: -> STATE_FREEFALL (accel: 0.32g)"
 */
static const char *TAG = "FALL_DETECT";

/**
 * ===================== MUTEX BẢO VỆ STATE =====================
 *
 * TẠI SAO CẦN MUTEX?
 * Biến current_state được truy cập từ 2 nguồn khác nhau:
 *   1. mpu6050_task (chính) — gọi fall_detection_update() mỗi 10ms @ 100Hz
 *   2. Timer callback — chạy trong FreeRTOS daemon task (timersTask)
 *
 * Nếu không có mutex, tình huống race condition xảy ra:
 *   - Task chính đang ở STATE_FREEFALL, chuẩn bị chuyển sang STATE_IMPACT
 *   - Timer freefall timeout fire, muốn set về STATE_IDLE
 *   → Kết quả không xác định, state machine hỏng
 *
 * Mutex đảm bảo: chỉ 1 thread (task/timer) được đọc/ghi state tại 1 thời điểm.
 * Thời gian giữ mutex rất ngắn (< 5μs) để không ảnh hưởng đến real-time.
 */
static SemaphoreHandle_t s_state_mutex = NULL;

/**
 * ===================== CẤU HÌNH MẶC ĐỊNH =====================
 *
 * Đây là bộ tham số được tối ưu cho người cao tuổi dựa trên nghiên cứu:
 * - Người cao tuổi thường ngã chậm hơn (thời gian rơi dài hơn)
 * - Va chạm yếu hơn do cơ thể nhẹ hơn / loãng xương
 * - Thời gian nằm sau ngã dài hơn (khó đứng dậy)
 *
 * Giá trị các ngưỡng:
 *   filter_alpha       = 0.5     → Cân bằng giữa lọc nhiễu và phản ứng nhanh
 *                                    50% giá trị cũ + 50% giá trị mới
 *                                    Thời gian lên đến 63% giá trị ổn định: τ ≈ 20ms (với 100Hz)
 *   accel_freefall_abs = 0.5g    → Gia tốc dưới 0.5g được coi là rơi tự do
 *                                    Ngưỡng này dung sai cho nhiễm 0.1-0.2g từ cảm biến
 *   accel_impact_abs   = 2.0g    → Va chạm thật tạo xung 2-3g, va chạm nhẹ ~1.2-1.5g
 *   lying_angle_threshold = 70°  → Góc nằm: người đứng pitch=0°, người ngã pitch≈90°
 *   timeout_freefall   = 150ms   → Rơi từ 1m mất ~450ms. 150ms đủ để xác nhận rơi,
 *                                   không quá dài để tránh nhiễu (lắc tay mạnh)
 *   timeout_impact_check = 1000ms→ Chờ 1s để cảm biến ổn định sau chấn động,
 *                                   tránh đọc góc sai lệch do rung lắc tạm thời
 *   wait_lie_down_time = 3500ms  → 3.5s xác nhận nạn nhân nằm yên (không di chuyển)
 *                                   Đủ dài để tránh dương tính giả (với người chỉ ngồi xuống)
 *                                   Đủ ngắn để gửi cảnh báo kịp thời
 */
static fall_detection_config_t default_config = {
    .filter_alpha = 0.5f,
    .accel_freefall_abs = 0.5f,
    .accel_impact_abs = 2.0f,
    .lying_angle_threshold = 70.0f,
    .timeout_freefall = 150,
    .timeout_impact_check = 1000,
    .wait_lie_down_time = 3500,
};

/**
 * ===================== BIẾN STATE TOÀN CỤC =====================
 *
 * Các biến static (private) chỉ được sử dụng trong file này.
 * Mọi truy cập đến current_state, alert_triggered đều phải qua mutex.
 *
 * Ngoại lệ: filtered_accel, filtered_pitch, filtered_roll, filtered_gyro
 * là các biến float (32-bit). Trên ESP32, đọc/ghi float 32-bit là 1 lệnh
 * load/store đơn (atomic) nên không cần mutex.
 */

/** @brief Trạng thái hiện tại của state machine. Bảo vệ bởi s_state_mutex. */
static fall_state_t current_state = STATE_IDLE;

/** @brief Bản sao cấu hình đang dùng (copy từ fall_detection_init_config). */
static fall_detection_config_t config;

/**
 * @brief Con trỏ hàm callback được gọi khi phát hiện ngã.
 *
 * Callback này không được bảo vệ bởi mutex vì chỉ gán 1 lần trong init.
 * Nội dung callback chạy trong context của mpu6050_task.
 */
static fall_alert_callback_t alert_callback = NULL;

/** @brief Cờ báo trạng thái SOS đã kích hoạt. Đọc qua fall_detection_is_alert_triggered(). */
static bool alert_triggered = false;

/**
 * ===================== GIÁ TRỊ ĐÃ LỌC (FILTERED) =====================
 *
 * Bộ lọc thông thấp (low-pass) được áp dụng lên tất cả 4 kênh:
 *   - filtered_accel:  Gia tốc đã lọc, đơn vị g
 *   - filtered_pitch:  Góc pitch đã lọc, đơn vị độ
 *   - filtered_roll:   Góc roll đã lọc, đơn vị độ
 *   - filtered_gyro:   Vận tốc góc đã lọc, đơn vị độ/giây
 *
 * Mục đích: loại bỏ nhiễu tần số cao từ cảm biến (rung động cơ học, nhiễu điện)
 * Chỉ dùng filtered_accel khi kiểm tra điều kiện phụ (accel ≈ 1g khi nằm yên).
 * Với freefall/impact, dùng raw_accel (giá trị chưa lọc) để phản ứng nhanh.
 */
static float filtered_accel = 1.0f;     /**< Khởi tạo = 1.0g (trọng lực tiêu chuẩn, người đang đứng yên) */
static float filtered_pitch = 0.0f;     /**< Khởi tạo = 0° (không nghiêng về trước/sau) */
static float filtered_roll = 0.0f;      /**< Khởi tạo = 0° (không nghiêng về trái/phải) */
static float filtered_gyro = 0.0f;      /**< Khởi tạo = 0°/s (không chuyển động) */

/**
 * @brief Bộ đếm thời gian ổn định ở trạng thái WAIT_LIE_DOWN
 *
 * Khi vào WAIT_LIE_DOWN (người nằm, góc ≥ 70°), bắt đầu đếm từ 0.
 * Mỗi lần phát hiện chuyển động (gyro ≥ 20°/s), stable_start bị reset = 0.
 * Khi (now - stable_start) * portTICK_PERIOD_MS ≥ wait_lie_down_time → xác nhận ngã.
 *
 * Giá trị là tick count của FreeRTOS (1 tick = portTICK_PERIOD_MS ms).
 * Trên ESP32 với configTICK_RATE_HZ = 1000, 1 tick = 1ms.
 */
static uint32_t stable_start = 0;

/**
 * ===================== FreeRTOS Software Timers =====================
 *
 * Timer trong FreeRTOS chạy trong daemon task (timersTask) riêng biệt,
 * không phải trong mpu6050_task. Do đó, timer callback truy cập current_state
 * phải thông qua mutex s_state_mutex.
 *
 * s_freefall_timer: Timer đếm 150ms ở trạng thái FREEFALL.
 *   - Khởi động khi vào FREEFALL
 *   - Nếu timer fire mà vẫn ở FREEFALL → reset về IDLE (vì không có impact)
 *   - Nếu có impact → dừng timer và chuyển IMPACT
 *
 * s_impact_timer: Timer đếm 1000ms ở trạng thái IMPACT.
 *   - Khởi động khi vào IMPACT
 *   - Khi fire, kiểm tra max_tilt:
 *     * ≥ 70° → chuyển WAIT_LIE_DOWN (nạn nhân nằm)
 *     * < 70° → reset về IDLE (nạn nhân đã đứng dậy)
 */
static TimerHandle_t s_freefall_timer = NULL;
static TimerHandle_t s_impact_timer = NULL;

/**
 * ===================== BỘ LỌC THÔNG THẤP (LOW-PASS FILTER) =====================
 *
 * CÔNG THỨC:
 *   y(n) = α * y(n-1) + (1 - α) * x(n)
 *
 * Trong đó:
 *   y(n)      = giá trị đầu ra tại thời điểm n (đã lọc)
 *   y(n-1)    = giá trị đầu ra tại thời điểm n-1 (giá trị lọc trước đó)
 *   x(n)      = giá trị đầu vào mới (raw từ cảm biến)
 *   α (alpha) = hệ số lọc (0.0 ≤ α ≤ 1.0)
 *
 * GIẢI THÍCH TOÁN HỌC:
 * Đây là bộ lọc IIR (Infinite Impulse Response) bậc 1, tương đương mạch RC.
 * Đáp ứng tần số: cutoff frequency f_c = α / (2π * Δt) với Δt = chu kỳ lấy mẫu.
 * Với α = 0.5 và Δt = 0.01s (100Hz): f_c ≈ 7.96 Hz
 *   → Tín hiệu trên 8Hz bị suy giảm (nhiễu cảm biến, rung động cơ học)
 *   → Tín hiệu dưới 8Hz được giữ nguyên (chuyển động của cơ thể người)
 *
 * TẠI SAO α = 0.5?
 * - α quá lớn (0.9): Rất mượt, nhưng chậm, bỏ sót impact ngắn (~20ms)
 * - α quá nhỏ (0.1): Phản ứng nhanh, nhưng nhiễu nhiều, dương tính giả
 * - α = 0.5: Cân bằng, đáp ứng ~2-3 mẫu để đạt 90% giá trị thật
 *
 * Đặc tính động học:
 *   Sau n bước: y(n) = α^n * y(0) + (1-α) * Σ(α^(n-k) * x(k)) với k=0..n-1
 *   = Tích chập giữa tín hiệu vào và hàm suy giảm exponential
 *
 * @param new_value          Giá trị mới từ cảm biến (raw)
 * @param previous_filtered  Giá trị đã lọc ở bước trước (y(n-1))
 * @param alpha              Hệ số lọc (thường 0.0-1.0)
 * @return float             Giá trị đã lọc ở bước hiện tại (y(n))
 */
static float low_pass_filter(float new_value, float previous_filtered, float alpha)
{
    /**
     * Công thức lọc: y(n) = α * y(n-1) + (1-α) * x(n)
     *
     * Giải thích trực quan:
     * - Nếu α = 0: y(n) = x(n) → không lọc, tín hiệu raw
     * - Nếu α = 1: y(n) = y(n-1) → giữ nguyên, bỏ qua tín hiệu mới
     * - Nếu α = 0.5: y(n) = 0.5*y(n-1) + 0.5*x(n) → trung bình 50-50
     *
     * Hệ quả: α càng gần 1, bộ lọc càng nhớ lâu, càng mượt nhưng phản ứng chậm.
     */
    return alpha * previous_filtered + (1.0f - alpha) * new_value;
}

/**
 * ===================== TIMER CALLBACKS =====================
 *
 * CÁC HÀM NÀY CHẠY TRONG FreeRTOS DAEMON TASK (timersTask)
 *
 * Đặc điểm của daemon task:
 *   - Stack rất nhỏ (~256 words) → KHÔNG được dùng nhiều stack
 *   - Không được gọi hàm blocking (vTaskDelay, xSemaphoreTake lâu, ...)
 *   - Nên giữ mutex trong thời gian ngắn nhất có thể (< 10ms)
 *   - Không được gọi ESP_LOG với chuỗi quá dài
 *
 * Đây là lý do các timer callback chỉ set state và log ngắn gọn.
 */

/**
 * @brief Callback timer freefall timeout
 *
 * Được gọi sau config.timeout_freefall (150ms) kể từ khi vào FREEFALL.
 *
 * Kịch bản:
 *   (A) Người thật sự rơi: trong 150ms sẽ có impact → timer bị stop ở STATE_FREEFALL
 *       → callback không chạy (vì timer đã bị stop trước khi fire)
 *   (B) Nhiễu (lắc tay, rung): accel giảm dưới 0.5g nhưng kéo dài > 150ms
 *       → timer fire, callback kiểm tra "current_state == FREEFALL?" → đúng
 *       → set về IDLE (bỏ qua sự kiện này)
 *
 * Logic: Nếu còn ở FREEFALL nghĩa là chưa có impact → reset.
 */
static void freefall_timeout_callback(TimerHandle_t xTimer)
{
    /* Đánh dấu tham số không dùng để tránh warning của compiler */
    (void)xTimer;

    /**
     * Lấy mutex với timeout 50ms.
     * Nếu không lấy được → timer task sẽ bỏ qua (không nên block).
     * Điều này hiếm xảy ra vì mpu6050_task giữ mutex rất ngắn (~5μs).
     */
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        /**
         * Kiểm tra kép: current_state vẫn là FREEFALL?
         * Nếu đã chuyển sang IMPACT (có impact trước khi timer fire) → không reset.
         * Đây là cơ chế bảo vệ: tránh reset nhầm state đúng.
         */
        if (current_state == STATE_FREEFALL) {
            current_state = STATE_IDLE;
            ESP_LOGD(TAG, " -> STATE_IDLE (freefall timeout - no impact detected)");
        }
        xSemaphoreGive(s_state_mutex);
    }
}

/**
 * @brief Callback timer impact timeout
 *
 * Được gọi sau config.timeout_impact_check (1000ms) kể từ khi vào IMPACT.
 *
 * Sau va chạm, đợi 1s để:
 *   - Cảm biến ổn định (hết rung lắc)
 *   - Bộ lọc low-pass hội tụ về giá trị thực
 *   - Người có thời gian nằm yên (nếu thật sự ngã)
 *
 * Sau đó kiểm tra góc nghiêng (max_tilt) để quyết định:
 *   - max_tilt ≥ 70°: Người nằm → sang WAIT_LIE_DOWN
 *   - max_tilt < 70°: Người đã đứng dậy / không phải ngã → reset IDLE
 *
 * Tại sao 70°?
 * Người đứng thẳng: pitch ≈ 0°, roll ≈ 0° (góc 0° so với phương ngang)
 * Người nằm ngửa: pitch ≈ 90° (đầu hướng lên) hoặc roll ≈ 90° (nằm nghiêng)
 * Người ngồi: pitch ≈ 45-60° (nửa nằm nửa ngồi)
 * Ngưỡng 70° là mức phân tách rõ ràng giữa "ngồi" và "nằm".
 */
static void impact_timeout_callback(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        /**
         * Kiểm tra: vẫn còn ở IMPACT?
         * Nếu đã chuyển trạng thái khác (do fall_detection_reset từ bên ngoài),
         * thì không làm gì cả.
         */
        if (current_state == STATE_IMPACT) {
            /* max_tilt = góc nghiêng lớn nhất, dùng filtered (giá trị ổn định) */
            float max_tilt = fmaxf(fabsf(filtered_pitch), fabsf(filtered_roll));
            if (max_tilt >= config.lying_angle_threshold) {
                /**
                 * Góc ≥ 70°: Nạn nhân đang nằm.
                 * Reset stable_start = 0 để bắt đầu đếm thời gian nằm yên.
                 * Chuyển sang WAIT_LIE_DOWN để xác nhận thêm 3.5s.
                 */
                stable_start = 0;
                current_state = STATE_WAIT_LIE_DOWN;
                ESP_LOGD(TAG, " -> STATE_WAIT_LIE_DOWN (max_tilt: %.1f°)", max_tilt);
            } else {
                /**
                 * Góc < 70°: Nạn nhân đã đứng dậy hoặc không phải ngã.
                 * Reset trực tiếp về IDLE, bỏ qua sự kiện này.
                 */
                current_state = STATE_IDLE;
                ESP_LOGD(TAG, " -> STATE_IDLE (impact timeout, max_tilt: %.1f° < 70°)", max_tilt);
            }
        }
        xSemaphoreGive(s_state_mutex);
    }
}

/**
 * @brief Helper: khởi động FreeRTOS software timer
 *
 * Hàm này xử lý 2 trường hợp:
 *   - Timer chưa tồn tại (*timer == NULL): tạo timer mới bằng xTimerCreate
 *   - Timer đã tồn tại: thay đổi chu kỳ (xTimerChangePeriod) và khởi động (xTimerStart)
 *
 * Cấu hình timer:
 *   - period_ms: chu kỳ timeout (ms)
 *   - pdFALSE: timer one-shot (chỉ fire 1 lần, không tự động restart)
 *   - NULL: không dùng tham số cho callback
 *   - callback: hàm callback khi timer fire
 *
 * @param timer       Con trỏ tới TimerHandle_t (con trỏ cấp 2)
 * @param callback    Hàm callback khi timer fire
 * @param period_ms   Chu kỳ timer (ms)
 */
static void start_timer(TimerHandle_t *timer, TimerCallbackFunction_t callback, uint32_t period_ms)
{
    if (*timer == NULL) {
        /**
         * Tạo timer mới với:
         *   - Tên: "fall_timer" (dùng để debug)
         *   - Chu kỳ: period_ms (tính bằng ticks FreeRTOS)
         *   - pdFALSE: one-shot (tự động dừng sau 1 lần fire)
         *   - NULL: ID của timer (không dùng)
         *   - callback: hàm sẽ được gọi khi timer fire
         */
        *timer = xTimerCreate("fall_timer",
                              pdMS_TO_TICKS(period_ms),
                              pdFALSE, NULL, callback);
    }
    if (*timer != NULL) {
        /**
         * xTimerChangePeriod thay đổi chu kỳ nếu timer đã tồn tại.
         * xTimerStart khởi động timer.
         * Tham số 0 = không block (gọi từ task context, không phải ISR).
         */
        xTimerChangePeriod(*timer, pdMS_TO_TICKS(period_ms), 0);
        xTimerStart(*timer, 0);
    }
}

/**
 * @brief Helper: dừng FreeRTOS software timer
 *
 * Chỉ dừng timer nếu timer đã được tạo (khác NULL).
 * Không hủy timer (vì timer có thể dùng lại sau).
 *
 * @param timer Con trỏ tới TimerHandle_t
 */
static void stop_timer(TimerHandle_t *timer)
{
    if (*timer != NULL) {
        /**
         * xTimerStop dừng timer.
         * Timer vẫn tồn tại trong bộ nhớ, có thể khởi động lại sau.
         * Tham số 0: không block.
         */
        xTimerStop(*timer, 0);
    }
}

/**
 * ===================== KHỞI TẠO (INITIALIZATION) =====================
 */

/**
 * @brief Khởi tạo module với cấu hình mặc định
 *
 * Gọi fall_detection_init_config(&default_config).
 * Sử dụng bộ tham số mặc định (tối ưu cho người cao tuổi).
 */
void fall_detection_init(void)
{
    fall_detection_init_config(&default_config);
}

/**
 * @brief Khởi tạo module với cấu hình tùy chỉnh
 *
 * @param cfg Con trỏ tới cấu hình fall_detection_config_t
 *
 * Các bước:
 *   1. Copy cấu hình vào biến static config (để dùng trong suốt vòng đời)
 *   2. Tạo mutex s_state_mutex nếu chưa tồn tại
 *   3. Reset tất cả biến về trạng thái ban đầu
 *   4. Log thông số cấu hình
 */
void fall_detection_init_config(const fall_detection_config_t *cfg)
{
    /* Bước 1: Copy toàn bộ cấu hình (struct copy) */
    config = *cfg;

    /* Bước 2: Tạo mutex binary để bảo vệ state machine */
    if (s_state_mutex == NULL) {
        /**
         * xSemaphoreCreateMutex tạo mutex có priority inheritance.
         * Nếu task ưu tiên thấp giữ mutex và task ưu tiên cao chờ,
         * task thấp tạm thời được nâng ưu tiên để giải phóng mutex nhanh hơn.
         * Điều này tránh "priority inversion" trong hệ thống real-time.
         */
        s_state_mutex = xSemaphoreCreateMutex();
        if (s_state_mutex == NULL) {
            /**
             * Nếu không tạo được mutex → lỗi nghiêm trọng.
             * Nguyên nhân: heap hết bộ nhớ.
             * Module sẽ không hoạt động, mọi fall_detection_update đều return ngay.
             */
            ESP_LOGE(TAG, "Failed to create state mutex");
            return;
        }
    }

    /* Bước 3: Reset state với mutex protection */
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_state = STATE_IDLE;       /**< Bắt đầu ở trạng thái theo dõi */
        alert_triggered = false;          /**< Chưa có báo động */
        filtered_accel = 1.0f;            /**< Giả sử đang đứng yên, accel = 1g */
        filtered_pitch = 0.0f;            /**< Reset góc pitch */
        filtered_roll = 0.0f;             /**< Reset góc roll */
        filtered_gyro = 0.0f;             /**< Reset vận tốc góc */
        xSemaphoreGive(s_state_mutex);
    }

    /* Bước 4: Log thông số cấu hình */
    ESP_LOGI(TAG, "Fall detection initialized");
    ESP_LOGI(TAG, "  Freefall: %.2fg | Impact: %.2fg | Lying: %.1f°",
             config.accel_freefall_abs, config.accel_impact_abs, config.lying_angle_threshold);
    ESP_LOGI(TAG, "  Timeouts: FF=%ums, Impact=%ums, LieConfirm=%ums",
             config.timeout_freefall, config.timeout_impact_check, config.wait_lie_down_time);
}

/**
 * ===================== STATE MACHINE CHÍNH (CORE UPDATE) =====================
 *
 * @brief Cập nhật dữ liệu cảm biến và thực thi 1 bước state machine
 *
 * @param accel_g   Gia tốc tổng (magnitude), đơn vị g
 * @param gyro_dps  Vận tốc góc tổng (magnitude), đơn vị độ/giây
 * @param pitch     Góc pitch (nghiêng trục X), đơn vị độ
 * @param roll      Góc roll (nghiêng trục Y), đơn vị độ
 *
 * Hàm này được gọi từ mpu6050_task với tần số 100Hz (mỗi 10ms 1 lần).
 *
 * ===== GIẢI THÍCH RAW vs FILTERED =====
 *
 * Trong module này, có 2 loại dữ liệu được sử dụng:
 *
 * 1. raw_accel (giá trị gốc từ cảm biến):
 *    - Dùng để phát hiện freefall và impact.
 *    - Lý do: freefall và impact là sự kiện rất nhanh (10-50ms).
 *      Nếu dùng filtered_accel, bộ lọc low-pass sẽ làm trễ tín hiệu,
 *      có thể bỏ lỡ hoàn toàn xung impact.
 *    - Ví dụ: Xung impact chỉ kéo dài 20ms. Với α=0.5,
 *      filtered_accel sau 20ms (2 mẫu) chỉ đạt 75% giá trị thật.
 *      Có thể không vượt ngưỡng → bỏ lỡ ngã.
 *
 * 2. filtered_accel, filtered_pitch, filtered_roll, filtered_gyro
 *    (giá trị đã qua low-pass filter):
 *    - Dùng để kiểm tra điều kiện "nằm yên" (lying detection).
 *    - Ở giai đoạn này, trạng thái đã ổn định (kéo dài hàng giây),
 *      việc làm trễ 20-50ms là không đáng kể.
 *    - Filter giúp loại bỏ nhiễu, tránh dao động ngưỡng.
 *
 * ===== QUY TRÌNH XỬ LÝ MỖI BƯỚC =====
 *   1. Lọc low-pass cả 4 kênh
 *   2. Tính max_tilt = max(|pitch_filtered|, |roll_filtered|)
 *   3. Lưu raw_accel trước khi filter
 *   4. Lấy mutex và chạy state machine switch-case
 */
void fall_detection_update(float accel_g, float gyro_dps, float pitch, float roll)
{
    /**
     * Kiểm tra an toàn: nếu mutex chưa được tạo (init chưa hoàn tất),
     * bỏ qua lần cập nhật này. Điều này xảy ra khi fall_detection_init()
     * chưa được gọi hoặc thất bại.
     */
    if (s_state_mutex == NULL) {
        return;  /* Chưa khởi tạo: bỏ qua */
    }

    /**
     * Bước 1: Áp dụng bộ lọc low-pass cho cả 4 kênh.
     *
     * Các phép toán float 32-bit trên ESP32 là atomic (1 lệnh load/store),
     * nên không cần mutex cho việc đọc/ghi filtered_*.
     * Tuy nhiên, cần đảm bảo rằng filtered_* được cập nhật trước khi
     * timer callback đọc chúng → không có vấn đề vì timer callback
     * chạy sau đó vài trăm ms.
     */
    filtered_accel = low_pass_filter(accel_g, filtered_accel, config.filter_alpha);
    filtered_pitch = low_pass_filter(pitch, filtered_pitch, config.filter_alpha);
    filtered_roll = low_pass_filter(roll, filtered_roll, config.filter_alpha);
    filtered_gyro = low_pass_filter(gyro_dps, filtered_gyro, config.filter_alpha);

    /**
     * Bước 2: Tính góc nghiêng lớn nhất (max_tilt).
     *
     * max_tilt = max(|pitch|, |roll|)
     *
     * Giải thích:
     * - Nếu người nằm ngửa: pitch gần 90° (đầu hướng lên), roll ≈ 0°
     *   → max_tilt ≈ 90°
     * - Nếu người nằm nghiêng: roll gần 90°, pitch nhỏ
     *   → max_tilt ≈ 90°
     * - Nếu người nằm sấp: pitch gần -90° (đầu hướng xuống), |pitch| ≈ 90°
     *   → max_tilt ≈ 90°
     * - Nếu người đứng: pitch ≈ 0°, roll ≈ 0°
     *   → max_tilt ≈ 0°
     *
     * Dùng max(|pitch|, |roll|) thay vì từng góc riêng lẻ vì:
     * ta chỉ cần biết "có đang nằm hay không" mà không cần biết tư thế cụ thể.
     * Nếu |pitch| đã ≥ 70° (nằm ngửa/sấp) thì |roll| không cần xét.
     * Nếu |roll| ≥ 70° (nằm nghiêng) thì |pitch| không cần xét.
     *
     * fabsf = float absolute value (trị tuyệt đối)
     * fmaxf = float maximum (lấy giá trị lớn hơn)
     */
    float max_tilt = fmaxf(fabsf(filtered_pitch), fabsf(filtered_roll));

    /**
     * Bước 3: Lưu raw_accel (giá trị gốc) trước khi mutex.
     *
     * raw_accel được dùng thay vì filtered_accel trong việc phát hiện
     * freefall và impact. Lý do: freefall và impact là sự kiện ngắn (ms),
     * filtered làm trễ có thể bỏ sót.
     *
     * Ví dụ: Impact tạo xung 2.5g kéo dài 15ms.
     *   - raw_accel = 2.5g (phát hiện ngay lập tức)
     *   - filtered_accel (α=0.5): sau 10ms = 0.5*1.0 + 0.5*2.5 = 1.75g
     *     → có thể vẫn chưa vượt ngưỡng 2.0g
     *   - filtered_accel sau 20ms: 0.5*1.75 + 0.5*2.5 = 2.125g (đã muộn)
     */
    float raw_accel = accel_g;

    /* Bước 4: Lấy mutex để truy cập state machine an toàn */
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        /**
         * local_state là bản sao của current_state tại thời điểm lấy mutex.
         * Dùng local_state trong switch-case thay vì đọc current_state trực tiếp
         * để tránh thay đổi giá trị giữa chừng (mặc dù đã có mutex).
         */
        fall_state_t local_state = current_state;

        /**
         * Debug counter: in log mỗi 100 lần gọi (tức mỗi 1 giây ở 100Hz).
         * Dùng static để giữ giá trị giữa các lần gọi.
         * Cung cấp thông tin real-time về trạng thái hiện tại.
         */
        static uint32_t debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            ESP_LOGI(TAG, "Accel: %.2fg | MaxTilt: %.1f° | State: %d",
                     filtered_accel, max_tilt, local_state);
        }

        /**
         * ===================== STATE MACHINE SWITCH =====================
         *
         * Đây là trái tim của thuật toán phát hiện ngã.
         * 5 trạng thái (IDLE, FREEFALL, IMPACT, WAIT_LIE_DOWN, SOS_TRIGGERED)
         * chuyển đổi dựa trên dữ liệu cảm biến và timeout.
         */
        switch (local_state) {

            /* ========== STATE_IDLE ==========
             * Trạng thái nghỉ (bình thường).
             * Chờ accel giảm dưới ngưỡng freefall.
             * Đây là trạng thái chiếm > 99.9% thời gian hoạt động. */
            case STATE_IDLE:
                /**
                 * Phát hiện rơi tự do: dùng raw_accel.
                 * raw_accel < config.accel_freefall_abs (thường 0.5g)
                 *
                 * Tại sao điều kiện này đúng khi rơi?
                 * - Người đứng yên: accel = 1g (trọng lực hướng xuống)
                 * - Khi rơi tự do: cảm biến và người rơi cùng gia tốc
                 *   → accel tương đối ≈ 0g (trạng thái không trọng lượng)
                 *
                 * Giá trị 0.5g là dung sai:
                 * - Cảm biến có nhiễu ±0.1g
                 * - Rơi không hoàn toàn tự do (có lực cản không khí)
                 * - Người ngã từ ghế/tư thế đứng không phải rơi hoàn toàn tự do
                 */
                if (raw_accel < config.accel_freefall_abs) {
                    current_state = STATE_FREEFALL;
                    start_timer(&s_freefall_timer, freefall_timeout_callback, config.timeout_freefall);
                    ESP_LOGW(TAG, " -> STATE_FREEFALL (accel: %.2fg)", raw_accel);
                }
                break;

            /* ========== STATE_FREEFALL ==========
             * Phát hiện rơi tự do. Chờ impact (va chạm).
             * Timer 150ms: nếu quá thời gian mà không có impact → reset.
             *
             * Tại sao cần timer?
             * - Rơi thật: impact xảy ra trong 100-400ms (tùy độ cao)
             * - Nhiễu (lắc tay): accel giảm < 0.5g trong thời gian ngắn
             *   → nếu kéo dài > 150ms mà vẫn freefall → bỏ qua
             */
            case STATE_FREEFALL:
                /**
                 * Phát hiện va chạm: dùng raw_accel.
                 * raw_accel >= config.accel_impact_abs (thường 2.0g)
                 *
                 * Tại sao va chạm tạo ra accel lớn hơn 1g?
                 * - Định luật 2 Newton: F = ma
                 * - Khi va chạm, vận tốc giảm từ v → 0 trong Δt rất ngắn
                 * - Gia tốc = Δv/Δt, Δt càng nhỏ, gia tốc càng lớn
                 * - Ví dụ: rơi 1m, v = 4.43 m/s, va chạm trong 20ms
                 *   → a = 4.43/0.02 = 221 m/s² ≈ 22.6g
                 * - Thực tế, cơ thể hấp thụ lực, accel đo được ~2-3g
                 */
                if (raw_accel >= config.accel_impact_abs) {
                    stop_timer(&s_freefall_timer);  /* Dừng timer freefall (không cần nữa) */
                    current_state = STATE_IMPACT;
                    start_timer(&s_impact_timer, impact_timeout_callback, config.timeout_impact_check);
                    ESP_LOGW(TAG, " -> STATE_IMPACT (accel: %.2fg)", raw_accel);
                }
                break;

            /* ========== STATE_IMPACT ==========
             * Đã phát hiện va chạm. Chờ timer 1s fire.
             * Trong thời gian này, raw/filtered accel vẫn được cập nhật
             * (phần code trước switch), nhưng không có hành động nào khác.
             *
             * Không kiểm tra góc ngay lập tức vì:
             * - Sau va chạm, cảm biến rung lắc (accel dao động mạnh)
             * - Góc pitch/roll có thể chưa ổn định
             * - Cần đợi ~1s để cảm biến trở về trạng thái tĩnh
             *
             * Timer impact_timeout_callback sẽ xử lý:
             *   - Lấy filtered_pitch, filtered_roll (đã ổn định sau 100 mẫu)
             *   - So sánh max_tilt với lying_angle_threshold
             */
            case STATE_IMPACT:
                /* Không làm gì, chờ timer impact fire */
                break;

            /* ========== STATE_WAIT_LIE_DOWN ==========
             * Đã có impact và góc nằm ≥ 70°. Cần xác nhận 3 điều kiện
             * trong 3.5s để chắc chắn là ngã thật:
             *
             * 1. max_tilt ≥ 70° (góc nằm)
             * 2. gyro_dps < 20°/s (không cử động)
             * 3. filtered_accel ≈ 1g (cơ thể ổn định trên mặt đất)
             *
             * Nếu bất kỳ điều kiện nào sai:
             *   - gyro ≥ 20°/s → reset stable_start (nạn nhân cựa quậy, đang cố đứng dậy)
             *   - max_tilt < 70° → reset về IDLE (nạn nhân đã đứng dậy được)
             */
            case STATE_WAIT_LIE_DOWN: {
                /**
                 * Kiểm tra 1: Góc nghiêng ≥ ngưỡng nằm?
                 * Dùng max_tilt (từ filtered) thay vì raw angle.
                 * Lý do: filtered ổn định, không bị nhiễu rung tức thời.
                 */
                if (max_tilt >= config.lying_angle_threshold) {
                    /**
                     * Kiểm tra 2: Không có chuyển động?
                     * gyro_dps < 20.0f: vận tốc góc rất thấp.
                     *
                     * Giải thích: Khi nằm yên, gyro ≈ 0-5°/s (nhiễu cảm biến).
                     * Khi cựa quậy, gyro có thể đạt 50-200°/s.
                     * Ngưỡng 20°/s phân biệt "yên tĩnh" và "có chuyển động".
                     *
                     * Tại sao không dùng filtered_gyro?
                     * filtered_gyro có α=0.5, mất ~30ms để phản hồi.
                     * Nếu nạn nhân đang nằm nhưng cựa quậy nhẹ, raw gyro
                     * sẽ tăng ngay, nhưng filtered gyro có thể vẫn dưới 20°/s.
                     * Dùng raw gyro để phát hiện chuyển động tức thời.
                     */
                    if (gyro_dps < 20.0f) {
                        /**
                         * Kiểm tra 3: Đủ thời gian nằm yên?
                         * stable_start lưu tick khi bắt đầu thấy "nằm yên".
                         * Nếu lần đầu phát hiện: gán stable_start = now.
                         * Nếu đã có stable_start: kiểm tra elapsed ≥ wait_lie_down_time.
                         *
                         * Công thức thời gian:
                         *   elapsed_ms = (now_tick - stable_start) * portTICK_PERIOD_MS
                         *   Với FreeRTOS tick rate 1000Hz: portTICK_PERIOD_MS = 1ms
                         *   → elapsed_ms = now_tick - stable_start (ms)
                         */
                        if (stable_start == 0) {
                            /**
                             * Lần đầu tiên phát hiện nằm yên.
                             * Ghi lại thời điểm bắt đầu.
                             * Nếu sau 3.5s vẫn yên → xác nhận ngã.
                             * Nếu có chuyển động → stable_start bị reset (else ở dưới).
                             */
                            stable_start = xTaskGetTickCount();
                        } else if ((xTaskGetTickCount() - stable_start) * portTICK_PERIOD_MS >= config.wait_lie_down_time) {
                            /**
                             * Đã nằm yên đủ 3.5s.
                             * Kiểm tra cuối: filtered_accel ≈ 1g?
                             *
                             * Tại sao accel ≈ 1g khi nằm yên?
                             * Khi cơ thể nằm trên mặt đất (hoặc giường), cảm biến
                             * vẫn chịu tác dụng của trọng lực 1g, nhưng lúc này
                             * trọng lực chiếu lên trục khác (trục nằm ngang so với
                             * thân người). Giá trị magnitude vẫn là 1g.
                             *
                             * Nếu filtered_accel khác 1g (sai khác > 0.3g):
                             *   - Đang rơi (accel < 0.7g) → chưa ổn định
                             *   - Đang bị nâng lên/xoay (accel > 1.3g) → chưa ổn định
                             *   → không xác nhận ngã
                             *
                             * Ngưỡng 0.3g dung sai cho:
                             *   - Nhiễu cảm biến ±0.1g
                             *   - Cơ thể không nằm hoàn toàn phẳng
                             */
                            if (fabsf(filtered_accel - 1.0f) < 0.3f) {
                                /**
                                 * === XÁC NHẬN NGÃ THẬT ===
                                 * Tất cả 3 điều kiện đã thỏa mãn:
                                 *   1. Đã có freefall (bước trước)
                                 *   2. Đã có impact (bước trước)
                                 *   3. Đang nằm yên, góc ≥ 70°, gyro thấp, accel ổn định
                                 *
                                 * Chuyển STATE_SOS_TRIGGERED và gọi callback.
                                 */
                                current_state = STATE_SOS_TRIGGERED;
                                alert_triggered = true;
                                stable_start = 0;
                                ESP_LOGW(TAG, " *** FALL DETECTED ***");

                                /**
                                 * Gọi callback báo động.
                                 *
                                 * QUAN TRỌNG: Release mutex trước khi gọi callback!
                                 *
                                 * Tại sao?
                                 * Callback có thể thực hiện:
                                 *   - Gửi tin nhắn Telegram (HTTP request, chậm)
                                 *   - Bật buzzer (PWM, không blocking)
                                 *   - fall_detection_reset() (nếu muốn tự động reset)
                                 *
                                 * Nếu giữ mutex trong khi gọi callback:
                                 *   - Callback gọi fall_detection_reset() → lấy mutex → DEADLOCK!
                                 *   - FreeRTOS mutex không cho phép lấy lại từ cùng 1 task
                                 *     nếu không có cơ chế recursive mutex.
                                 *
                                 * Giải pháp: nhả mutex, gọi callback, lấy lại mutex sau.
                                 * Rủi ro: state có thể thay đổi trong khi mutex được nhả.
                                 * Nhưng lúc này state đã là SOS_TRIGGERED rồi, callback
                                 * không thay đổi state, chỉ thực hiện hành động bên ngoài.
                                 */
                                if (alert_callback != NULL) {
                                    xSemaphoreGive(s_state_mutex);  /* Nhả mutex trước khi gọi callback */
                                    alert_callback();                /* Gọi callback báo động */

                                    /**
                                     * Sau khi callback kết thúc, lấy lại mutex.
                                     * Kiểm tra: callback có thể đã gọi fall_detection_reset()
                                     * và giải phóng (xóa) mutex. Nếu s_state_mutex == NULL,
                                     * không thể lấy mutex → return.
                                     */
                                    if (s_state_mutex == NULL) return;

                                    /**
                                     * Lấy lại mutex với timeout 100ms.
                                     * Nếu không lấy được (do task khác đang giữ),
                                     * log warning và return (không thể cập nhật state).
                                     */
                                    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                                        ESP_LOGW(TAG, "Failed to re-take mutex after callback");
                                        return;
                                    }
                                }
                            }
                        }
                    } else {
                        /**
                         * Có chuyển động (gyro ≥ 20°/s):
                         * Reset bộ đếm thời gian. Nạn nhân đang cựa quậy,
                         * có thể đang cố đứng dậy.
                         *
                         * stable_start = 0 nghĩa là "bắt đầu lại từ đầu".
                         * Lần sau khi gyro < 20°/s, stable_start sẽ được
                         * gán lại = now.
                         *
                         * Điều này tránh trường hợp: nạn nhân ngã, nằm yên 2s,
                         * cựa quậy 1s, nằm yên 2s → không đủ 3.5s liên tục
                         * → không báo ngã giả.
                         */
                        stable_start = 0;
                    }
                } else {
                    /**
                     * Góc < 70°: Nạn nhân đã đứng dậy.
                     *
                     * Kịch bản điển hình:
                     *   - Người ngã xong tự đứng dậy
                     *   - Người ngã nhưng thực chất chỉ ngồi xuống (góc 45°)
                     *   - Đọc sai góc cảm biến (nhiễu)
                     *
                     * Reset trực tiếp về IDLE, không báo động.
                     * stable_start = 0 cho lần sau.
                     */
                    ESP_LOGD(TAG, " -> STATE_IDLE (tilt: %.1f° < 70°)", max_tilt);
                    stable_start = 0;
                    current_state = STATE_IDLE;
                }
                break;
            }

            /* ========== STATE_SOS_TRIGGERED ==========
             * Ngã đã được xác nhận. Báo động đã kích hoạt.
             *
             * Ở trạng thái này, module vẫn cập nhật filtered_* (phần code
             * trước switch), nhưng không thay đổi state.
             *
             * Chỉ có thể thoát STATE_SOS_TRIGGERED bằng:
             *   1. fall_detection_reset() từ bên ngoài (nút bấm, ứng dụng)
             *   2. fall_detection_init lại module
             *
             * Thiết kế này đảm bảo báo động không bị tắt ngẫu nhiên.
             * Người dùng PHẢI chủ động reset để xác nhận "tôi vẫn ổn". */
            case STATE_SOS_TRIGGERED:
                /* Chờ reset từ bên ngoài */
                break;

            /**
             * default: Phòng trường hợp state bị lỗi (giá trị rác).
             * Reset về IDLE để khôi phục hoạt động.
             * Đây là cơ chế an toàn (fail-safe) cho state machine.
             */
            default:
                current_state = STATE_IDLE;
                break;
        }

        /* Giải phóng mutex */
        xSemaphoreGive(s_state_mutex);
    }
}

/**
 * ===================== GETTERS (LẤY DỮ LIỆU) =====================
 */

/**
 * @brief Lấy kết quả phát hiện ngã hiện tại
 *
 * @return fall_detection_result_t Cấu trúc chứa:
 *   - current_accel_g: Gia tốc đã lọc (g)
 *   - current_gyro_dps: Vận tốc góc đã lọc (°/s)
 *   - current_pitch: Góc pitch đã lọc (°)
 *   - current_roll: Góc roll đã lọc (°)
 *   - fall_detected: true nếu state == STATE_SOS_TRIGGERED
 *   - current_state: trạng thái hiện tại
 *
 * 4 giá trị filtered được copy trước mutex (atomic, không cần mutex).
 * fall_detected và current_state được lấy trong mutex.
 */
fall_detection_result_t fall_detection_get_result(void)
{
    /**
     * Copy filtered values trước (atomic reads, không cần mutex).
     * Các giá trị float này được cập nhật mỗi 10ms, việc đọc tại bất kỳ
     * thời điểm nào cũng cho giá trị hợp lệ (có thể trễ 1-2 mẫu, không vấn đề).
     */
    fall_detection_result_t result = {
        .current_accel_g = filtered_accel,
        .current_gyro_dps = filtered_gyro,
        .current_pitch = filtered_pitch,
        .current_roll = filtered_roll,
    };

    /**
     * Lấy mutex để đọc current_state và tính fall_detected.
     * fall_detected = (current_state == STATE_SOS_TRIGGERED)
     */
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        result.fall_detected = (current_state == STATE_SOS_TRIGGERED);
        result.current_state = current_state;
        xSemaphoreGive(s_state_mutex);
    }

    return result;
}

/**
 * @brief Kiểm tra nhanh trạng thái báo động (lightweight)
 *
 * @return true  Nếu đã trigger SOS
 * @return false Nếu chưa trigger
 *
 * Dùng mutex với timeout 10ms. Nếu không lấy được mutex (task khác
 * đang giữ), trả về false (mặc định an toàn: "chưa có báo động").
 */
bool fall_detection_is_alert_triggered(void)
{
    bool triggered = false;
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        triggered = alert_triggered;
        xSemaphoreGive(s_state_mutex);
    }
    return triggered;
}

/**
 * @brief Lấy trạng thái state machine (kiểu fall_state_t)
 *
 * Wrapper gọi fall_detection_get_state_internal() và ép kiểu.
 */
fall_state_t fall_detection_get_state(void)
{
    return (fall_state_t)fall_detection_get_state_internal();
}

/**
 * ===================== RESET =====================
 *
 * @brief Reset toàn bộ module về trạng thái IDLE
 *
 * Hàm này có thể được gọi từ:
 *   - Callback báo động (để tự động reset sau khi gửi tin nhắn)
 *   - Nút nhấn Reset trên thiết bị
 *   - Webserver API
 *   - Bất kỳ task nào
 *
 * Tác dụng:
 *   1. Mutex lock: reset current_state, alert_triggered, filtered_*, stable_start
 *   2. mutex unlock
 *   3. Dừng tất cả timer (freefall và impact) nếu đang chạy
 *
 * @note filtered_* được reset để tránh "bộ nhớ" của bộ lọc low-pass
 *       giữ lại dữ liệu cũ khi reset.
 */
void fall_detection_reset(void)
{
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_state = STATE_IDLE;    /**< Về trạng thái theo dõi */
        alert_triggered = false;       /**< Tắt báo động */
        filtered_accel = 1.0f;         /**< Giả sử đang đứng yên */
        filtered_pitch = 0.0f;         /**< Reset góc pitch */
        filtered_roll = 0.0f;          /**< Reset góc roll */
        stable_start = 0;              /**< Reset bộ đếm */
        xSemaphoreGive(s_state_mutex);
    }

    /**
     * Dừng các FreeRTOS timer.
     * Các timer này KHÔNG được xóa (xTimerDelete) vì có thể dùng lại sau.
     * stop_timer chỉ gọi xTimerStop, timer vẫn tồn tại trong bộ nhớ.
     */
    stop_timer(&s_freefall_timer);  /* Dừng timer freefall nếu đang chạy */
    stop_timer(&s_impact_timer);    /* Dừng timer impact nếu đang chạy */

    ESP_LOGI(TAG, "Reset to IDLE");
}

/**
 * ===================== CALLBACK (ĐĂNG KÝ) =====================
 */

/**
 * @brief Đăng ký hàm callback báo động
 *
 * @param callback Con trỏ hàm void(*)(void). NULL để hủy.
 *
 * Hàm này được thiết kế để gọi 1 lần trong init.
 * Không có mutex bảo vệ vì:
 *   - Callback được gán trước khi module hoạt động
 *   - Không thay đổi callback trong quá trình chạy
 *   - Nếu cần thay đổi, phải đảm bảo không có ngã đang xảy ra
 *
 * Callback chạy trong context của mpu6050_task (người gọi fall_detection_update).
 * Xem STATE_SOS_TRIGGERED để biết cách callback được gọi.
 */
void fall_detection_set_callback(fall_alert_callback_t callback)
{
    alert_callback = callback;
}

/**
 * ===================== INTERNAL GETTERS (cho Webserver) =====================
 *
 * Các hàm này được thiết kế riêng cho webserver dashboard (HTTP handler).
 * Webserver cần đọc state dưới dạng số nguyên (uint8_t) để trả về JSON.
 *
 * Lưu ý: filtered_accel và max_tilt KHÔNG dùng mutex vì:
 *   - float 32-bit read là atomic trên ESP32
 *   - Đọc giá trị cũ hơn 10-20ms là chấp nhận được (webserver không cần real-time
 *     tuyệt đối)
 *   - Tránh blocking webserver task khi mpu6050_task đang giữ mutex
 *
 * Ngược lại, fall_detection_get_state_internal() vẫn dùng mutex vì:
 *   - current_state là enum (int), đọc/write không atomic trên mọi kiến trúc
 *   - State machine cần dữ liệu chính xác tại thời điểm đọc
 */

/**
 * @brief Lấy trạng thái hiện tại dưới dạng uint8_t (cho webserver JSON)
 *
 * @return uint8_t 0=IDLE, 1=FREEFALL, 2=IMPACT, 3=WAIT_LIE_DOWN, 4=SOS_TRIGGERED
 */
uint8_t fall_detection_get_state_internal(void)
{
    uint8_t state = STATE_IDLE;  /* Giá trị mặc định nếu không lấy được mutex */
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state = (uint8_t)current_state;  /* Ép kiểu enum → uint8_t */
        xSemaphoreGive(s_state_mutex);
    }
    return state;
}

/**
 * @brief Lấy góc nghiêng lớn nhất max(|pitch|, |roll|) không qua mutex
 *
 * @return float Góc nghiêng lớn nhất (độ), dùng cho webserver
 *
 * Atomic read: float 32-bit chỉ là 1 lệnh load trên ESP32.
 * Giá trị có thể cũ 1-10ms, nhưng chấp nhận được cho dashboard.
 */
float fall_detection_get_max_tilt_internal(void)
{
    return fmaxf(fabsf(filtered_pitch), fabsf(filtered_roll));
}

/**
 * @brief Lấy giá trị gia tốc đã lọc không qua mutex
 *
 * @return float Gia tốc (g), dùng cho webserver
 *
 * Atomic read, tương tự fall_detection_get_max_tilt_internal().
 */
float fall_detection_get_filtered_accel_internal(void)
{
    return filtered_accel;
}
