/*
 * Fall Detection — State Machine phát hiện ngã cho MPU6050 trên ESP32
 *
 * Thuật toán dựa trên 3 pha vật lý của một cú ngã:
 * rơi tự do → va chạm → nằm yên. Thiếu bất kỳ pha nào → bỏ qua (dương tính giả).
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
 */

#include "fall_detection.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include <math.h>

// Tag cho tất cả log output của module này
static const char *TAG = "FALL_DETECT";

// Mutex bảo vệ current_state khỏi race condition giữa mpu6050_task và timer callbacks
static SemaphoreHandle_t s_state_mutex = NULL;

// Cấu hình mặc định — tối ưu cho người cao tuổi, thiết bị đeo thắt lưng
//   filter_alpha = 0.5       → cân bằng giữa lọc nhiễu và phản ứng nhanh
//   freefall    = 0.5g       → ngưỡng phát hiện rơi tự do (≤ 0.5g là rơi)
//   impact      = 2.0g       → ngưỡng phát hiện va chạm (≥ 2.0g là đập mạnh)
//   lying_angle = 70°        → phân tách ngồi (~45-60°) và nằm (~90°)
//   timeout_ff  = 150ms      → đủ xác nhận rơi, không quá dài (tránh nhiễu lắc)
//   timeout_imp = 1000ms     → chờ cảm biến ổn định sau chấn động
//   wait_lie    = 5000ms     → xác nhận nằm yên 5s, lọc nhiễu cựa quậy ngắn
//   gyro_lie    = 50°/s      → nếu gyro ≥ 50° → có cựa quậy, reset bộ đếm
static fall_detection_config_t default_config = {
    .filter_alpha = 0.5f,
    .accel_freefall_abs = 0.5f,
    .accel_impact_abs = 2.0f,
    .lying_angle_threshold = 70.0f,
    .timeout_freefall = 150,
    .timeout_impact_check = 1000,
    .wait_lie_down_time = 5000,
};

// State machine — các biến static, chỉ truy cập trong file này
// current_state và alert_triggered phải qua mutex; filtered_* là atomic (float 32-bit)
static fall_state_t current_state = STATE_IDLE;
static fall_detection_config_t config;
static fall_alert_callback_t alert_callback = NULL;  // Gán 1 lần trong init, không cần mutex
static bool alert_triggered = false;

// Giá trị đã qua low-pass filter — dùng cho kiểm tra "nằm yên", không dùng cho freefall/impact
static float filtered_accel = 1.0f;
static float filtered_pitch = 0.0f;
static float filtered_roll = 0.0f;
static float filtered_gyro = 0.0f;

// Tick FreeRTOS khi bắt đầu thấy "nằm yên" — reset = 0 nếu có chuyển động
static uint32_t stable_start = 0;

// FreeRTOS software timers — chạy trong daemon task riêng, phải dùng mutex
// s_freefall_timer: 150ms; nếu fire mà còn ở FREEFALL → không có impact → reset
// s_impact_timer: 1000ms; lúc fire, kiểm tra max_tilt để quyết định WAIT_LIE_DOWN hay IDLE
static TimerHandle_t s_freefall_timer = NULL;
static TimerHandle_t s_impact_timer = NULL;

// Low-pass filter IIR bậc 1: y(n) = α * y(n-1) + (1-α) * x(n)
// α = 0.5: cân bằng — đủ nhanh cho phát hiện impact, đủ mượt cho góc nghiêng
static float low_pass_filter(float new_value, float previous_filtered, float alpha)
{
    return alpha * previous_filtered + (1.0f - alpha) * new_value;
}

// Timer callbacks — chạy trong FreeRTOS daemon task (stack nhỏ, không blocking)

// Nếu fire khi còn ở FREEFALL → không có impact → reset về IDLE
static void freefall_timeout_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (current_state == STATE_FREEFALL) {
            current_state = STATE_IDLE;
            ESP_LOGD(TAG, " -> STATE_IDLE (freefall timeout - no impact detected)");
        }
        xSemaphoreGive(s_state_mutex);
    }
}

// 1s sau impact: kiểm tra góc nghiêng. ≥ 70° → WAIT_LIE_DOWN, < 70° → reset (người đã đứng dậy)
static void impact_timeout_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (current_state == STATE_IMPACT) {
            float max_tilt = fmaxf(fabsf(filtered_pitch), fabsf(filtered_roll));
            if (max_tilt >= config.lying_angle_threshold) {
                stable_start = 0;
                current_state = STATE_WAIT_LIE_DOWN;
                ESP_LOGD(TAG, " -> STATE_WAIT_LIE_DOWN (max_tilt: %.1f°)", max_tilt);
            } else {
                current_state = STATE_IDLE;
                ESP_LOGD(TAG, " -> STATE_IDLE (impact timeout, max_tilt: %.1f° < 70°)", max_tilt);
            }
        }
        xSemaphoreGive(s_state_mutex);
    }
}

// Tạo hoặc khởi động lại one-shot timer — nếu NULL thì tạo mới, nếu có rồi thì change period + start
static void start_timer(TimerHandle_t *timer, TimerCallbackFunction_t callback, uint32_t period_ms)
{
    if (*timer == NULL) {
        *timer = xTimerCreate("fall_timer",
                              pdMS_TO_TICKS(period_ms),
                              pdFALSE, NULL, callback);
    }
    if (*timer != NULL) {
        xTimerChangePeriod(*timer, pdMS_TO_TICKS(period_ms), 0);
        xTimerStart(*timer, 0);
    }
}

// Dừng timer (không hủy — có thể khởi động lại sau)
static void stop_timer(TimerHandle_t *timer)
{
    if (*timer != NULL) {
        xTimerStop(*timer, 0);
    }
}

// === KHỞI TẠO ===

void fall_detection_init(void)
{
    fall_detection_init_config(&default_config);
}

void fall_detection_init_config(const fall_detection_config_t *cfg)
{
    config = *cfg;

    if (s_state_mutex == NULL) {
        s_state_mutex = xSemaphoreCreateMutex();
        if (s_state_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create state mutex");
            return;
        }
    }

    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_state = STATE_IDLE;
        alert_triggered = false;
        filtered_accel = 1.0f;
        filtered_pitch = 0.0f;
        filtered_roll = 0.0f;
        filtered_gyro = 0.0f;
        xSemaphoreGive(s_state_mutex);
    }

    ESP_LOGI(TAG, "Fall detection initialized");
    ESP_LOGI(TAG, "  Freefall: %.2fg | Impact: %.2fg | Lying: %.1f°",
             config.accel_freefall_abs, config.accel_impact_abs, config.lying_angle_threshold);
    ESP_LOGI(TAG, "  Timeouts: FF=%ums, Impact=%ums, LieConfirm=%ums",
             config.timeout_freefall, config.timeout_impact_check, config.wait_lie_down_time);
}

// === STATE MACHINE CHÍNH — gọi từ mpu6050_task ở 100Hz, mỗi lần 1 bước ===
// Dùng raw_accel cho freefall/impact (sự kiện nhanh, filter làm trễ hỏng chuyện)
// Dùng filtered_* cho lying detection (trạng thái ổn định, cần sạch nhiễu)
void fall_detection_update(float accel_g, float gyro_dps, float pitch, float roll)
{
    // Chưa init → bỏ qua
    if (s_state_mutex == NULL) {
        return;
    }

    // Low-pass filter cả 4 kênh (atomic float, không cần mutex)
    filtered_accel = low_pass_filter(accel_g, filtered_accel, config.filter_alpha);
    filtered_pitch = low_pass_filter(pitch, filtered_pitch, config.filter_alpha);
    filtered_roll = low_pass_filter(roll, filtered_roll, config.filter_alpha);
    filtered_gyro = low_pass_filter(gyro_dps, filtered_gyro, config.filter_alpha);

    // Góc nghiêng lớn nhất — không cần biết tư thế cụ thể, chỉ cần "có nằm không?"
    float max_tilt = fmaxf(fabsf(filtered_pitch), fabsf(filtered_roll));

    // Raw accel cho freefall/impact — filter làm trễ có thể bỏ sót xung ngắn
    float raw_accel = accel_g;

    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        fall_state_t local_state = current_state;

        // Log định kỳ 1 lần/giây cho debug
        static uint32_t debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            ESP_LOGI(TAG, "Accel: %.2fg | MaxTilt: %.1f° | State: %d",
                     filtered_accel, max_tilt, local_state);
        }

        switch (local_state) {

            // IDLE: chờ phát hiện rơi tự do (raw_accel < 0.5g)
            case STATE_IDLE:
                if (raw_accel < config.accel_freefall_abs) {
                    current_state = STATE_FREEFALL;
                    start_timer(&s_freefall_timer, freefall_timeout_callback, config.timeout_freefall);
                    ESP_LOGW(TAG, " -> STATE_FREEFALL (accel: %.2fg)", raw_accel);
                }
                break;

            // FREEFALL: chờ impact (raw_accel ≥ 2.0g) hoặc timeout 150ms → reset
            case STATE_FREEFALL:
                if (raw_accel >= config.accel_impact_abs) {
                    stop_timer(&s_freefall_timer);
                    current_state = STATE_IMPACT;
                    start_timer(&s_impact_timer, impact_timeout_callback, config.timeout_impact_check);
                    ESP_LOGW(TAG, " -> STATE_IMPACT (accel: %.2fg)", raw_accel);
                }
                break;

            // IMPACT: chờ timer 1s để cảm biến ổn định rồi kiểm tra góc
            case STATE_IMPACT:
                break;

            // WAIT_LIE_DOWN: xác nhận nằm yên 5s, 3 điều kiện
            case STATE_WAIT_LIE_DOWN: {
                if (max_tilt >= config.lying_angle_threshold) {
                    // Gyro < 50°/s → nạn nhân yên tĩnh (tăng từ 20 lên 50 sau test thực tế)
                    if (gyro_dps < 50.0f) {
                        if (stable_start == 0) {
                            stable_start = xTaskGetTickCount();
                        } else if ((xTaskGetTickCount() - stable_start) * portTICK_PERIOD_MS >= config.wait_lie_down_time) {
                            // Đủ 5s nằm yên + accel ~1g → xác nhận ngã thật
                            if (fabsf(filtered_accel - 1.0f) < 0.3f) {
                                current_state = STATE_SOS_TRIGGERED;
                                alert_triggered = true;
                                stable_start = 0;
                                ESP_LOGW(TAG, " *** FALL DETECTED ***");

                                // Nhả mutex trước gọi callback để tránh deadlock
                                // (callback có thể gọi fall_detection_reset() lấy mutex)
                                if (alert_callback != NULL) {
                                    xSemaphoreGive(s_state_mutex);
                                    alert_callback();
                                    if (s_state_mutex == NULL) return;
                                    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                                        ESP_LOGW(TAG, "Failed to re-take mutex after callback");
                                        return;
                                    }
                                }
                            }
                        }
                    } else {
                        // Gyro ≥ 50°/s → có chuyển động → reset bộ đếm, yêu cầu 5s liên tục không ngắt quãng
                        stable_start = 0;
                    }
                } else {
                    // Góc < 70° → người đã đứng dậy hoặc không phải ngã
                    ESP_LOGD(TAG, " -> STATE_IDLE (tilt: %.1f° < 70°)", max_tilt);
                    stable_start = 0;
                    current_state = STATE_IDLE;
                }
                break;
            }

            // SOS_TRIGGERED: báo động đã kích hoạt, chỉ thoát bằng reset từ ngoài
            case STATE_SOS_TRIGGERED:
                break;

            default:
                current_state = STATE_IDLE;
                break;
        }

        xSemaphoreGive(s_state_mutex);
    }
}

// === GETTERS ===

// filtered_* là atomic (float 32-bit), đọc trước mutex
// current_state và fall_detected phải qua mutex
fall_detection_result_t fall_detection_get_result(void)
{
    fall_detection_result_t result = {
        .current_accel_g = filtered_accel,
        .current_gyro_dps = filtered_gyro,
        .current_pitch = filtered_pitch,
        .current_roll = filtered_roll,
    };

    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        result.fall_detected = (current_state == STATE_SOS_TRIGGERED);
        result.current_state = current_state;
        xSemaphoreGive(s_state_mutex);
    }

    return result;
}

// Lightweight check — nếu không lấy được mutex thì trả về false (an toàn)
bool fall_detection_is_alert_triggered(void)
{
    bool triggered = false;
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        triggered = alert_triggered;
        xSemaphoreGive(s_state_mutex);
    }
    return triggered;
}

fall_state_t fall_detection_get_state(void)
{
    return (fall_state_t)fall_detection_get_state_internal();
}

// === RESET ===

// Có thể gọi từ callback, nút nhấn, webserver API
// Dừng timer, reset state, filtered_* về giá trị ban đầu
void fall_detection_reset(void)
{
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_state = STATE_IDLE;
        alert_triggered = false;
        filtered_accel = 1.0f;
        filtered_pitch = 0.0f;
        filtered_roll = 0.0f;
        stable_start = 0;
        xSemaphoreGive(s_state_mutex);
    }

    stop_timer(&s_freefall_timer);
    stop_timer(&s_impact_timer);

    ESP_LOGI(TAG, "Reset to IDLE");
}

// === CALLBACK ===

void fall_detection_set_callback(fall_alert_callback_t callback)
{
    alert_callback = callback;
}

// === INTERNAL GETTERS (cho webserver, không cần real-time tuyệt đối) ===
// filtered_* là atomic, current_state phải qua mutex

uint8_t fall_detection_get_state_internal(void)
{
    uint8_t state = STATE_IDLE;
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state = (uint8_t)current_state;
        xSemaphoreGive(s_state_mutex);
    }
    return state;
}

float fall_detection_get_max_tilt_internal(void)
{
    return fmaxf(fabsf(filtered_pitch), fabsf(filtered_roll));
}

float fall_detection_get_filtered_accel_internal(void)
{
    return filtered_accel;
}
