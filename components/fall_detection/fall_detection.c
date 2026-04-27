/**
 * @file fall_detection.c
 * @brief Thuật toán phát hiện ngã - State machine với FreeRTOS timers
 *
 * LUỒNG PHÁT HIỆN NGÃ:
 * IDLE → FREEFALL (accel < 0.5g) → IMPACT (accel ≥ 1.5g) → WAIT_LIE_DOWN (đợi 3.5s) → SOS
 *
 * Ngưỡng được tối ưu cho người cao tuổi:
 * - Freefall: < 0.5g (trọng lực giảm khi rơi)
 * - Impact: ≥ 1.5g (va chạm sau khi ngã)
 * - Lying: max(|pitch|, |roll|) ≥ 70° (nằm ngửa)
 */

#include "fall_detection.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "FALL_DETECT";

// ========== MUTEX BẢO VỆ STATE ==========
// State machine chạy trong mpu6050_task (100Hz)
// Timer callbacks cũng truy cập state → cần mutex để tránh race condition
static SemaphoreHandle_t s_state_mutex = NULL;

// ========== CẤU HÌNH MẶC ĐỊNH ==========
static fall_detection_config_t default_config = {
    .filter_alpha = 0.5f,           // 50% cũ + 50% mới (phản ứng nhanh)
    .accel_freefall_abs = 0.5f,     // ≤0.5g = rơi tự do
    .accel_impact_abs = 1.5f,       // ≥1.5g = va chạm
    .lying_angle_threshold = 70.0f, // ≥70° = nằm
    .timeout_freefall = 150,        // 150ms: rơi tự do kéo dài tối đa
    .timeout_impact_check = 1000,   // 1s sau va chạm mới kiểm tra góc
    .wait_lie_down_time = 3500,     // 3.5s xác nhận nằm yên
};

// ========== BIẾN STATE ==========
static fall_state_t current_state = STATE_IDLE;
static orientation_t current_orientation = ORIENTATION_UNKNOWN;
static orientation_t prev_orientation = ORIENTATION_UNKNOWN;
static fall_detection_config_t config;
static fall_alert_callback_t alert_callback = NULL;
static bool alert_triggered = false;

// Giá trị đã lọc (low-pass filter)
static float filtered_accel = 1.0f;
static float filtered_pitch = 0.0f;
static float filtered_roll = 0.0f;
static float filtered_gyro = 0.0f;

// Timer đếm thời gian ở trạng thái ổn định
static uint32_t stable_start = 0;

// FreeRTOS Software Timers
static TimerHandle_t s_freefall_timer = NULL;
static TimerHandle_t s_impact_timer = NULL;

// ========== LOW-PASS FILTER ==========
// Lọc nhiễu cảm biến: y(n) = α * y(n-1) + (1-α) * x(n)
// α càng lớn → càng mượt nhưng phản ứng chậm
static float low_pass_filter(float new_value, float previous_filtered, float alpha)
{
    return alpha * previous_filtered + (1.0f - alpha) * new_value;
}

// ========== TIMER CALLBACKS ==========
// Timer chạy trong FreeRTOS daemon task, stack rất nhỏ
// KHÔNG gọi các hàm blocking ở đây

// Hết thời gian freefall (150ms) mà không có impact → reset
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

// Hết thời gian chờ impact (1s) → kiểm tra góc để quyết định
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

// Helper: khởi động timer
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

// Helper: dừng timer
static void stop_timer(TimerHandle_t *timer)
{
    if (*timer != NULL) {
        xTimerStop(*timer, 0);
    }
}

// ========== KHỞI TẠO ==========
void fall_detection_init(void)
{
    fall_detection_init_config(&default_config);
}

void fall_detection_init_config(const fall_detection_config_t *cfg)
{
    config = *cfg;

    // Tạo mutex để bảo vệ state variables
    if (s_state_mutex == NULL) {
        s_state_mutex = xSemaphoreCreateMutex();
        if (s_state_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create state mutex");
            return;
        }
    }

    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_state = STATE_IDLE;
        current_orientation = ORIENTATION_UNKNOWN;
        prev_orientation = ORIENTATION_UNKNOWN;
        alert_triggered = false;
        filtered_accel = 1.0f;
        filtered_pitch = 0.0f;
        filtered_roll = 0.0f;
        filtered_gyro = 0.0f;
        xSemaphoreGive(s_state_mutex);
    }

    // Tạo timers (one-shot, tự động stop sau khi fire)
    start_timer(&s_freefall_timer, freefall_timeout_callback, config.timeout_freefall);
    start_timer(&s_impact_timer, impact_timeout_callback, config.timeout_impact_check);

    ESP_LOGI(TAG, "Fall detection initialized");
    ESP_LOGI(TAG, "  Freefall: %.2fg | Impact: %.2fg | Lying: %.1f°",
             config.accel_freefall_abs, config.accel_impact_abs, config.lying_angle_threshold);
    ESP_LOGI(TAG, "  Timeouts: FF=%ums, Impact=%ums, LieConfirm=%ums",
             config.timeout_freefall, config.timeout_impact_check, config.wait_lie_down_time);
}

// ========== STATE MACHINE ==========
// Được gọi từ mpu6050_task mỗi 10ms (100Hz)
void fall_detection_update(float accel_g, float gyro_dps, float pitch, float roll)
{
    if (s_state_mutex == NULL) {
        return;  // Chưa khởi tạo
    }

    // Áp dụng low-pass filter (float operations atomic, không cần mutex)
    filtered_accel = low_pass_filter(accel_g, filtered_accel, config.filter_alpha);
    filtered_pitch = low_pass_filter(pitch, filtered_pitch, config.filter_alpha);
    filtered_roll = low_pass_filter(roll, filtered_roll, config.filter_alpha);
    filtered_gyro = low_pass_filter(gyro_dps, filtered_gyro, config.filter_alpha);

    // max_tilt = góc nghiêng lớn nhất (không phải khoảng cách euclidean)
    float max_tilt = fmaxf(fabsf(filtered_pitch), fabsf(filtered_roll));

    // Baseline = running average khi IDLE (để phát hiện thay đổi)
    static float baseline = 1.0f;
    static uint32_t baseline_samples = 0;

    // Bảo vệ state machine bằng mutex
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        fall_state_t local_state = current_state;

        // Cập nhật baseline khi đang đứng yên
        if (local_state == STATE_IDLE && baseline_samples < 500) {
            baseline = (baseline * 0.95f) + (filtered_accel * 0.05f);
            baseline_samples++;
        }

        // Debug log mỗi 100 cycles (~1 giây)
        static uint32_t debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            ESP_LOGI(TAG, "Accel: %.2fg | Baseline: %.2fg | MaxTilt: %.1f° | State: %d",
                     filtered_accel, baseline, max_tilt, local_state);
        }

        // ========== STATE MACHINE ==========
        switch (local_state) {
            case STATE_IDLE:
                // Theo dõi rơi tự do: accel < 0.5g
                if (filtered_accel < config.accel_freefall_abs) {
                    current_state = STATE_FREEFALL;
                    start_timer(&s_freefall_timer, freefall_timeout_callback, config.timeout_freefall);
                    ESP_LOGW(TAG, " -> STATE_FREEFALL (accel: %.2fg)", filtered_accel);
                }
                break;

            case STATE_FREEFALL:
                // Chờ va chạm: accel >= 1.5g
                if (filtered_accel >= config.accel_impact_abs) {
                    stop_timer(&s_freefall_timer);
                    current_state = STATE_IMPACT;
                    start_timer(&s_impact_timer, impact_timeout_callback, config.timeout_impact_check);
                    ESP_LOGW(TAG, " -> STATE_IMPACT (accel: %.2fg)", filtered_accel);
                }
                break;

            case STATE_IMPACT:
                // Chờ 1s, timer sẽ fire và kiểm tra góc
                break;

            case STATE_WAIT_LIE_DOWN: {
                // Kiểm tra: góc ≥ 70° + không chuyển động (gyro < 20 deg/s) + đợi 3.5s
                if (max_tilt >= config.lying_angle_threshold) {
                    if (gyro_dps < 20.0f) {
                        if (stable_start == 0) {
                            stable_start = xTaskGetTickCount();
                        } else if ((xTaskGetTickCount() - stable_start) * portTICK_PERIOD_MS >= config.wait_lie_down_time) {
                            // Xác nhận ngã: góc ≥ 70° + accel ≈ 1g (ổn định)
                            if (max_tilt >= config.lying_angle_threshold &&
                                fabsf(filtered_accel - 1.0f) < 0.3f) {
                                current_state = STATE_SOS_TRIGGERED;
                                alert_triggered = true;
                                stable_start = 0;
                                ESP_LOGW(TAG, " *** FALL DETECTED ***");

                                // Gọi callback (báo động + Telegram)
                                if (alert_callback != NULL) {
                                    xSemaphoreGive(s_state_mutex);
                                    alert_callback();
                                    // Callback có thể gọi reset → kiểm tra mutex
                                    if (s_state_mutex == NULL) return;
                                    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                                        ESP_LOGW(TAG, "Failed to re-take mutex after callback");
                                        return;
                                    }
                                }
                            }
                        }
                    } else {
                        // Vẫn còn chuyển động, reset timer
                        stable_start = 0;
                    }
                } else {
                    // Góc < 70° → không phải ngã (đứng dậy)
                    ESP_LOGD(TAG, " -> STATE_IDLE (tilt: %.1f° < 70°)", max_tilt);
                    stable_start = 0;
                    current_state = STATE_IDLE;
                }
                break;
            }

            case STATE_SOS_TRIGGERED:
                // Chờ nút bấm reset từ bên ngoài
                break;

            default:
                current_state = STATE_IDLE;
                break;
        }

        xSemaphoreGive(s_state_mutex);
    }
}

// ========== GETTERS ==========
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
        result.current_orientation = current_orientation;
        xSemaphoreGive(s_state_mutex);
    }

    return result;
}

bool fall_detection_is_alert_triggered(void)
{
    bool triggered = false;
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        triggered = alert_triggered;
        xSemaphoreGive(s_state_mutex);
    }
    return triggered;
}

// ========== RESET ==========
void fall_detection_reset(void)
{
    if (s_state_mutex != NULL && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_state = STATE_IDLE;
        alert_triggered = false;
        filtered_accel = 1.0f;
        filtered_pitch = 0.0f;
        filtered_roll = 0.0f;
        stable_start = 0;
        current_orientation = ORIENTATION_UNKNOWN;
        prev_orientation = ORIENTATION_UNKNOWN;
        xSemaphoreGive(s_state_mutex);
    }

    stop_timer(&s_freefall_timer);
    stop_timer(&s_impact_timer);

    ESP_LOGI(TAG, "Reset to IDLE");
}

// ========== CALLBACK ==========
void fall_detection_set_callback(fall_alert_callback_t callback)
{
    // Callback chỉ được set 1 lần lúc init
    // Callback chạy trong mpu6050_task context
    alert_callback = callback;
}

// ========== INTERNAL GETTERS (cho webserver) ==========
// Webserver đọc state mà không cần mutex vì chỉ đọc

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
