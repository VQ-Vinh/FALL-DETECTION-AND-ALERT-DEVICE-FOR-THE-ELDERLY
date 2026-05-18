/**
 * @file fall_detection.h
 * @brief Header file cho module phát hiện ngã
 *
 * Thuật toán phát hiện ngã dựa trên state machine:
 * 1. FREEFALL: Phát hiện gia tốc < 0.5g (không trọng lượng)
 * 2. IMPACT: Phát hiện gia tốc ≥ 1.5g (va chạm)
 * 3. WAIT_LIE_DOWN: Đợi 3.5s xác nhận nằm yên
 * 4. SOS_TRIGGERED: Kích hoạt báo động
 */

#ifndef FALL_DETECTION_H
#define FALL_DETECTION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ========== TRẠNG THÁI PHÁT HIỆN NGÃ ==========
typedef enum {
    STATE_IDLE = 0,           // Bình thường, theo dõi
    STATE_FREEFALL = 1,        // Phát hiện rơi tự do
    STATE_IMPACT = 2,          // Phát hiện va chạm
    STATE_WAIT_LIE_DOWN = 3,    // Đợi xác nhận nằm
    STATE_SOS_TRIGGERED = 4    // Kích hoạt SOS
} fall_state_t;

// ========== CẤU HÌNH NGƯỠNG ==========
typedef struct {
    float filter_alpha;            // Hệ số lọc (0.0-1.0)
    float accel_freefall_abs;     // Ngưỡng freefall (≤0.5g)
    float accel_impact_abs;      // Ngưỡng impact (≥1.5g)
    float lying_angle_threshold;  // Ngưỡng góc nằm (≥70°)
    uint32_t timeout_freefall;   // Timeout freefall (150ms)
    uint32_t timeout_impact_check; // Delay kiểm tra impact (1000ms)
    uint32_t wait_lie_down_time; // Thời gian xác nhận nằm (3500ms)
} fall_detection_config_t;

// ========== KẾT QUẢ ==========
typedef struct {
    bool fall_detected;
    fall_state_t current_state;
    float current_accel_g;
    float current_gyro_dps;
    float current_pitch;
    float current_roll;
} fall_detection_result_t;

// ========== HÀM API ==========
void fall_detection_init(void);
void fall_detection_init_config(const fall_detection_config_t *config);

// Cập nhật với dữ liệu cảm biến mới (gọi 100Hz)
void fall_detection_update(float accel_g, float gyro_dps, float pitch, float roll);

// Lấy kết quả hiện tại
fall_detection_result_t fall_detection_get_result(void);
bool fall_detection_is_alert_triggered(void);

// Reset về trạng thái IDLE
void fall_detection_reset(void);

// Callback khi phát hiện ngã
typedef void (*fall_alert_callback_t)(void);
void fall_detection_set_callback(fall_alert_callback_t callback);

// Lấy trạng thái hiện tại
fall_state_t fall_detection_get_state(void);

// Internal getters (cho webserver)
uint8_t fall_detection_get_state_internal(void);
float fall_detection_get_max_tilt_internal(void);
float fall_detection_get_filtered_accel_internal(void);

#ifdef __cplusplus
}
#endif

#endif // FALL_DETECTION_H
