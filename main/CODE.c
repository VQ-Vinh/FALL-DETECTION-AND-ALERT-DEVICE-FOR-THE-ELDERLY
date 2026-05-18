/**
 * @file CODE.c
 * @brief Chương trình chính - Fall Detection Device cho người cao tuổi.
 *
 * =========================== GIỚI THIỆU HỆ THỐNG ===========================
 * Thiết bị phát hiện té ngã dành cho người già, sử dụng cảm biến MPU6050 (gia
 * tốc kế + con quay hồi chuyển) để phân tích chuyển động và phát hiện té ngã.
 * Khi phát hiện té ngã, thiết bị sẽ:
 *   - Bật còi báo (buzzer) + LED nhấp nháy
 *   - Gửi thông báo qua Telegram (bot API)
 *   - Cung cấp dashboard web để theo dõi
 * Nút CANCEL cho phép hủy báo động giả, nút SOS kích hoạt khẩn cấp thủ công.
 *
 * =========================== TỔ CHỨC CODE =================================
 * 1. GPIO + I2C initialization
 * 2. MPU6050 task (100Hz): đọc cảm biến, phát hiện té ngã
 * 3. Button task: xử lý nút bấm CANCEL/SOS
 * 4. Webserver: dashboard + API
 * 5. Telegram: gửi thông báo qua Telegram Bot API
 *
 * =========================== LUỒNG DỮ LIỆU ================================
 * MPU6050 (cảm biến)
 *   → mpu6050_task (đọc raw data ở 100Hz)
 *     → fall_detection_update() (thuật toán phát hiện té ngã)
 *       → [FALL DETECTED] → alert_callback()
 *         ├──→ telegram_send_fall_alert() (gửi tin nhắn Telegram)
 *         └──→ start_fall_alert() (bật buzzer + LED nhấp nháy)
 *
 * Nút bấm vật lý:
 *   ├── BTN_SOS (GPIO 6): giữ 3 giây → SOS Telegram + báo động
 *   └── BTN_CANCEL (GPIO 5): giữ 3 giây khi đang báo → hủy báo + Telegram
 *
 * =========================== HỆ ĐIỀU HÀNH =================================
 * Chạy trên FreeRTOS (ESP-IDF framework) với các task:
 *   - mpu6050_task:  priority 5, stack 4096, vòng lặp 100Hz
 *   - btn_task:      priority 3, stack 2048, chờ GPIO event từ queue
 *   - main loop:     vòng lặp chính delay 1 giây, kiểm tra WiFi & flag
 */

// =========================== THƯ VIỆN =====================================
// Các thư viện ESP-IDF hệ thống:
#include "driver/gpio.h"      // Điều khiển GPIO: cấu hình chân, đọc/ghi mức logic
#include "driver/i2c.h"       // Giao tiếp I2C với MPU6050 (cảm biến)
#include "esp_log.h"          // Log hệ thống (ESP_LOGI, ESP_LOGW, ESP_LOGE)
#include "esp_err.h"          // Kiểu esp_err_t và hàm esp_err_to_name()
#include "nvs_flash.h"        // Non-Volatile Storage: lưu cấu hình WiFi

// Thư viện FreeRTOS (hệ điều hành thời gian thực):
#include "freertos/FreeRTOS.h" // FreeRTOS base: TickType_t, portTICK_PERIOD_MS, pdMS_TO_TICKS
#include "freertos/task.h"     // Quản lý task: xTaskCreate, vTaskDelay, vTaskDelete
#include "freertos/queue.h"    // Hàng đợi (queue) cho truyền thông liên task
#include "freertos/timers.h"   // Software timer: xTimerCreate, xTimerStart, xTimerStop

// Thư viện dự án (thư mục components/ hoặc main/):
#include "mpu6050.h"           // Driver cảm biến MPU6050: đọc raw data, hiệu chuẩn
#include "roll_pitch.h"        // Tính góc Roll và Pitch từ dữ liệu MPU6050
#include "wifi.h"              // Kết nối WiFi station mode
#include "webserver.h"         // Web server: dashboard HTML + REST API
#include "fall_detection.h"    // Thuật toán phát hiện té ngã (state machine)
#include "telegram.h"          // Gửi thông báo qua Telegram Bot API

// =========================== CẤU HÌNH I2C =================================
// I2C là chuẩn giao tiếp nối tiếp 2 dây (SCL + SDA) dùng để đọc dữ liệu từ
// cảm biến MPU6050. Địa chỉ I2C mặc định của MPU6050 là 0x68 (AD0 nối GND).
// ============================================================================
#define I2C_MASTER_SCL_IO       9   // Chân GPIO9: Clock line (SCL)
#define I2C_MASTER_SDA_IO       8   // Chân GPIO8: Data line (SDA)
#define I2C_MASTER_NUM          I2C_NUM_0  // Bộ điều khiển I2C số 0 (ESP32 có 2 bộ)
#define I2C_MASTER_FREQ_HZ      400000 // Tần số I2C: 400kHz (Fast Mode, tối đa cho MPU6050)

// =========================== CẤU HÌNH GPIO =================================
// Định nghĩa các chân GPIO kết nối với thiết bị ngoại vi.
// GPIO: General-Purpose Input/Output - chân vào/ra đa năng.
// ============================================================================
#define BUZ_PIN         0       // GPIO0: Buzzer (output) - còi báo động, mức 1 = ON
#define LED_PIN         1       // GPIO1: LED (output) - đèn báo trạng thái
#define BTN_CANCEL_PIN  5       // GPIO5: Nút hủy báo động giả (input, pull-up nội)
#define BTN_SOS_PIN     6       // GPIO6: Nút SOS khẩn cấp (input, pull-up nội)

// =========================== CẤU HÌNH THỜI GIAN ============================
// Các hằng số thời gian (tính bằng mili-giây) dùng cho báo động, debounce, v.v.
// ============================================================================
#define ALERT_DURATION_MS  30000   // Thời gian báo động tối đa: 30 giây, sau đó tự tắt
#define LED_BLINK_PERIOD    1000    // Chu kỳ LED nhấp nháy khi báo động: 1 giây (500ms on/500ms off)
#define LED_ERROR_BLINK_PERIOD 200  // Chu kỳ LED nhấp nháy khi lỗi: 200ms (nhanh hơn)
#define SOS_HOLD_TIME_MS    3000    // Thời gian giữ nút SOS để kích hoạt: 3 giây
#define CANCEL_HOLD_TIME_MS 3000    // Thời gian giữ nút CANCEL để hủy báo: 3 giây
#define DEBOUNCE_TIME_MS    50      // Thời gian chống dội (debounce) cho nút bấm: 50ms
                                     // (ngăn nhiễu cơ học khi nhấn nút)

// =========================== HẰNG SỐ HỆ THỐNG =============================
// Các hằng số cấu hình cho task FreeRTOS và các chức năng hệ thống.
// ============================================================================
#define MPU6050_TASK_STACK      4096    // Kích thước stack (bytes) cho task MPU6050
#define MPU6050_TASK_PRIORITY  5        // Độ ưu tiên (0=thấp nhất, 24=cao nhất trong FreeRTOS)
#define MPU6050_SAMPLE_RATE_MS  10      // Chu kỳ lấy mẫu: 10ms = 100Hz (100 mẫu/giây)
#define BTN_TASK_STACK          2048    // Kích thước stack (bytes) cho task nút bấm
#define BTN_TASK_PRIORITY      3        // Độ ưu tiên cho task nút bấm
#define GPIO_QUEUE_SIZE         20      // Số lượng phần tử tối đa trong hàng đợi GPIO event
#define WIFI_CONNECT_TIMEOUT_S  30      // Thời gian chờ kết nối WiFi tối đa: 30 giây
#define CALIBRATION_SAMPLES     500     // Số mẫu lấy trong quá trình hiệu chuẩn (500 × 10ms = 5 giây)
#define CALIB_BUZZER_DURATION_MS 1000   // Thời gian bíp còi sau khi hiệu chuẩn xong: 1 giây
#define MAIN_LOOP_DELAY_MS      1000    // Chu kỳ vòng lặp chính: 1 giây

// =========================== CẤU HÌNH TELEGRAM =============================
// Token Bot Telegram và Chat ID dùng để gửi thông báo.
// WARNING: Nhúng token vào mã nguồn là mối nguy bảo mật. Trong phiên bản sản
// xuất, nên chuyển các giá trị này vào sdkconfig hoặc NVS (Non-Volatile Storage)
// để tránh lộ thông tin khi mã nguồn bị chia sẻ.
// ============================================================================
#define TELEGRAM_BOT_TOKEN  "8659816659:AAEFwAc-LdtDNuVEGbUHt_cpOwP_ilWfSjA"
#define TELEGRAM_CHAT_ID    "-5239342658"

// =========================== BIẾN TOÀN CỤC =================================
// Biến toàn cục (static trong file) dùng cho log hệ thống và hàng đợi GPIO.
// ============================================================================
/// @brief   Thẻ (tag) dùng cho ESP_LOG*, giúp nhận diện log thuộc module nào.
///          "MAIN" là tag cho module chính này.
static const char *TAG = "MAIN";

/// @brief Hàng đợi (queue) nhận sự kiện GPIO từ ISR (ngắt).
///        ISR (Interrupt Service Routine) gửi số hiệu GPIO vào queue,
///        task btn_task() nhận và xử lý. Cơ chế này giúp xử lý nút bấm
///        không bị block trong ISR (ISR phải ngắn).
static QueueHandle_t gpio_queue = NULL;

// =========================== TRẠNG THÁI BÁO ĐỘNG ===========================
// Máy trạng thái (state machine) quản lý trạng thái của hệ thống báo động.
// Máy trạng thái giúp kiểm soát luồng hoạt động rõ ràng, dễ bảo trì.
// ============================================================================
typedef enum {
    ALERT_STATE_IDLE,           // Trạng thái rỗi: không báo động, hệ thống chờ. Ðây là trạng thái
                                // bình thường khi không có sự kiện ngã hay lỗi.
    ALERT_STATE_FALLING,        // Trạng thái đang báo động (phát hiện ngã hoặc SOS):
                                // LED nhấp nháy, còi kêu, timer tự tắt đang đếm ngược.
    ALERT_STATE_ERROR           // Trạng thái lỗi: mất I2C (cảm biến) hoặc mất WiFi.
                                // LED nhấp nháy NHANH (200ms) để cảnh báo trực quan.
} alert_state_t;

/// @brief   Biến trạng thái báo động hiện tại. Khởi tạo là IDLE.
static alert_state_t s_alert_state = ALERT_STATE_IDLE;

/// @brief   Timer tự động tắt báo động sau ALERT_DURATION_MS (30 giây).
///          Timer dùng một lần (one-shot), không lặp lại (pdFALSE).
static TimerHandle_t s_alert_timer = NULL;

/// @brief   Timer tạo hiệu ứng nhấp nháy LED trong khi báo động.
///          Timer này lặp lại (auto-reload = pdTRUE) với chu kỳ LED_BLINK_PERIOD.
///          Mỗi lần timer cháy, callback alert_led_toggle() được gọi.
static TimerHandle_t s_led_timer = NULL;

/// @brief   Timer đếm thời gian giữ nút SOS (3 giây). Khi timer cháy,
///          nếu nút vẫn đang được giữ → kích hoạt SOS.
static TimerHandle_t s_sos_timer = NULL;

/// @brief   Timer đếm thời gian giữ nút CANCEL (3 giây). Khi timer cháy,
///          nếu nút vẫn đang được giữ → hủy báo động giả.
static TimerHandle_t s_cancel_timer = NULL;

/// @brief   Trạng thái hiện tại của LED (bật/tắt). Dùng trong callback
///          alert_led_toggle() để đảo trạng thái.
static bool s_led_state = false;

/// @brief   Cờ báo động đang hoạt động. true = đang báo (còi + LED).
///          Dùng để ngăn chặn báo động chồng lấn (start_fall_alert kiểm tra cờ này).
static bool s_alert_active = false;

/// @brief   Cờ trạng thái lỗi (I2C/WiFi). true = đang có lỗi.
static bool s_error_state = false;

/// @brief   Cờ đánh dấu nút SOS đang được giữ. Được đặt = true khi nhấn,
///          reset = false khi nhả nút hoặc timer SOS cháy.
static bool s_sos_button_held = false;

/// @brief   Cờ đánh dấu nút CANCEL đang được giữ. Tương tự s_sos_button_held.
static bool s_cancel_button_held = false;

/// @brief   Cờ báo hiệu cần gửi Telegram SOS. Được đặt trong timer callback
///          sos_timer_callback(), xử lý trong MAIN LOOP.
///          WHY: Không gọi hàm HTTP (telegram_send_*) trong timer callback vì
///          timer callback chạy trong context của software timer task, không
///          nên thực hiện các tác vụ chặn (blocking) như HTTP request.
static bool s_sos_telegram_pending = false;

/// @brief   Cờ báo hiệu cần gửi Telegram hủy báo động giả.
///          Tương tự s_sos_telegram_pending, dùng main loop để xử lý.
static bool s_cancel_telegram_pending = false;

/// @brief   Lưu thời điểm (tick count) xảy ra ngắt cuối cùng từ nút CANCEL.
///          Dùng cho debounce: nếu ngắt mới đến trong vòng 50ms, bỏ qua.
///          Tránh nhiễu do rung cơ học (chattering) của nút bấm.
static uint32_t s_last_cancel_isr = 0;

/// @brief   Lưu thời điểm (tick count) xảy ra ngắt cuối cùng từ nút SOS.
///          Dùng cho debounce, tương tự s_last_cancel_isr.
static uint32_t s_last_sos_isr = 0;

// =========================== DOUBLE-BUFFER CẢM BIẾN ========================
// Kỹ thuật double-buffer (ping-pong) cho phép MPU6050 task (writer) ghi dữ liệu
// vào một buffer, trong khi web server (reader) đọc buffer còn lại mà không
// cần khóa (lock). Điều này loại bỏ xung đột dữ liệu (race condition) khi hai
// task truy cập cùng lúc. Buffer 0 và Buffer 1 luân phiên nhau.
//
// Cơ chế:
//   - s_writer_index = 0 hoặc 1, cho biết buffer nào đang được ghi
//   - Reader tính reader_index = 1 - s_writer_index (buffer còn lại)
//   - Sau mỗi lần ghi, writer đảo s_writer_index
// ============================================================================

/// @brief   Mảng 2 buffer chứa dữ liệu cảm biến đã xử lý.
///          Mỗi phần tử là struct sensor_data_t định nghĩa trong mpu6050.h.
///          Khởi tạo với giá trị zero và data_ready = false.
sensor_data_t g_sensor_buffers[2] = {
    {.total_accel_g = 0, .total_gyro = 0, .roll = 0, .pitch = 0, .data_ready = false},
    {.total_accel_g = 0, .total_gyro = 0, .roll = 0, .pitch = 0, .data_ready = false}
};

/// @brief   Buffer index dùng cho writer (task MPU6050). Giá trị 0 hoặc 1.
///          Sau mỗi lần ghi, đảo giá trị: s_writer_index = 1 - s_writer_index.
static uint8_t s_writer_index = 0;

/// @brief   Hàm lấy writer index hiện tại, dùng cho web server (reader) để
///          xác định buffer cần đọc (reader_index = 1 - g_get_writer_index()).
/// @return  uint8_t: giá trị hiện tại của s_writer_index (0 hoặc 1).
uint8_t g_get_writer_index(void) {
    return s_writer_index;
}

// =========================== HÀM BÁO ĐỘNG ==================================
// Các hàm quản lý báo động: bật/tắt LED, bắt đầu/dừng báo động ngã và lỗi.
// Các hàm này được gọi từ mpu6050_task (khi phát hiện ngã) hoặc từ timer
// callback (SOS) và main loop.
// ============================================================================

/**
 * @brief Tắt LED báo động.
 *        Ghi mức 0 (LOW) vào chân LED_PIN và cập nhật trạng thái LED.
 */
static void alert_led_off(void)
{
    gpio_set_level(LED_PIN, 0);   // Đặt chân LED về mức 0 (tắt)
    s_led_state = false;           // Ghi nhận trạng thái LED đã tắt
}

/**
 * @brief Đảo trạng thái LED (toggle).
 *        Được gọi bởi LED timer callback để tạo hiệu ứng nhấp nháy.
 *        Mỗi lần timer cháy, hàm này sẽ bật/tắt LED luân phiên.
 */
static void alert_led_toggle(void)
{
    s_led_state = !s_led_state;                           // Đảo trạng thái: true→false, false→true
    gpio_set_level(LED_PIN, s_led_state ? 1 : 0);        // Ghi mới: 1 nếu bật, 0 nếu tắt
}

/// @brief   Khai báo trước (forward declaration) hàm stop_fall_alert để
///          alert_timer_callback có thể gọi nó. Hàm stop_fall_alert được
///          định nghĩa bên dưới.
static void stop_fall_alert(void);

/**
 * @brief Callback timer báo động tự động tắt.
 *        Được gọi khi timer s_alert_timer cháy (sau 30 giây).
 * @param xTimer Handle của timer (không dùng, chỉ để tương thích API).
 *
 * WHY: Đảm bảo báo động không kéo dài vô hạn khi người dùng không can thiệp.
 *      Ví dụ người già ngã và bất tỉnh → báo động tự tắt sau 30s để tiết kiệm pin.
 */
static void alert_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;                // Đánh dấu tham số không dùng để tránh cảnh báo compiler
    ESP_LOGI(TAG, "Alert duration ended - auto stop");
    stop_fall_alert();           // Gọi hàm dừng báo động
}

/**
 * @brief Bắt đầu báo động té ngã.
 *        Bật còi (buzzer), LED nhấp nháy, và khởi động timer tự tắt.
 *
 * Thuật toán:
 *   - Kiểm tra nếu đã báo động thì thoát (tránh chồng lấn)
 *   - Bật BUZZER (buzzer kêu liên tục)
 *   - Tạo hoặc khởi động LED timer (nhấp nháy 1 giây)
 *   - Tạo hoặc khởi động auto-stop timer (30 giây)
 */
static void start_fall_alert(void)
{
    if (s_alert_active) return;  // Nếu báo động đang hoạt động, không làm gì
                                  // WHY: tránh báo động bị gọi nhiều lần từ nhiều nguồn
                                  // (phát hiện ngã + SOS đồng thời)

    s_alert_active = true;        // Đánh dấu báo động đang hoạt động
    s_alert_state = ALERT_STATE_FALLING;  // Chuyển trạng thái máy trạng thái

    gpio_set_level(BUZ_PIN, 1);  // Buzzer on: còi kêu liên tục. GPIO mức 1 = HIGH.

    // LED blink timer (chu kỳ 1 giây, lặp lại)
    if (s_led_timer == NULL) {
        // Nếu timer chưa được tạo, tạo mới:
        //   "led_blink" = tên timer (debug)
        //   pdMS_TO_TICKS(LED_BLINK_PERIOD) = chu kỳ tính bằng ticks FreeRTOS
        //   pdTRUE = auto-reload (lặp lại)
        //   NULL = không dùng tham số
        //   alert_led_toggle = callback function
        s_led_timer = xTimerCreate("led_blink", pdMS_TO_TICKS(LED_BLINK_PERIOD),
                                    pdTRUE, NULL, (TimerCallbackFunction_t)alert_led_toggle);
    }
    if (s_led_timer) {
        xTimerStart(s_led_timer, 0);  // Khởi động timer LED (bắt đầu nhấp nháy)
    }

    // Auto-stop timer (30 giây, chạy 1 lần)
    if (s_alert_timer == NULL) {
        // Tạo timer một lần (one-shot, pdFALSE = không lặp)
        s_alert_timer = xTimerCreate("alert_timeout", pdMS_TO_TICKS(ALERT_DURATION_MS),
                                      pdFALSE, NULL, alert_timer_callback);
    }
    if (s_alert_timer) {
        xTimerStart(s_alert_timer, 0);  // Bắt đầu đếm ngược 30 giây
    }

    ESP_LOGW(TAG, ">>> FALL ALERT STARTED <<<");  // Log cảnh báo (mức WARN)
}

/**
 * @brief Dừng báo động té ngã.
 *        Tắt còi, dừng LED nhấp nháy, reset máy trạng thái fall detection.
 *
 * Hành động:
 *   - Đặt alert_active = false, alert_state = IDLE
 *   - Tắt buzzer (mức LOW)
 *   - Dừng tất cả timer (cả LED và alert timeout)
 *   - Tắt LED
 *   - Reset fall_detection state machine để sẵn sàng phát hiện lần ngã mới
 */
static void stop_fall_alert(void)
{
    s_alert_active = false;       // Hủy cờ báo động
    s_alert_state = ALERT_STATE_IDLE;  // Quay về trạng thái rỗi

    gpio_set_level(BUZ_PIN, 0);  // Tắt còi: GPIO mức 0 = LOW

    // Dừng các timer (nếu tồn tại)
    if (s_led_timer) xTimerStop(s_led_timer, 0);    // Ngừng LED nhấp nháy
    if (s_alert_timer) xTimerStop(s_alert_timer, 0); // Hủy timer tự động tắt (nếu còn)

    alert_led_off();              // Tắt LED và cập nhật trạng thái

    // Reset state machine của fall detection để sẵn sàng phát hiện lần ngã tiếp theo.
    // fall_detection_reset() đưa máy trạng thái về trạng thái ban đầu (thường là STATE_IDLE).
    // WHY: Nếu không reset, state machine có thể vẫn mắc kẹt ở trạng thái "đã ngã"
    //      và sẽ bỏ qua lần ngã thứ hai.
    fall_detection_reset();

    ESP_LOGI(TAG, "Fall alert stopped - system reset to IDLE");
}

/**
 * @brief Bắt đầu báo động lỗi (I2C/WiFi).
 *        LED nhấp nháy NHANH (chu kỳ 200ms) để phân biệt với báo động ngã.
 *        Không bật còi vì lỗi không phải tình huống khẩn cấp.
 *
 * WHY tách biệt với start_fall_alert():
 *   - Báo động lỗi không nên kích hoạt Telegram (tránh spam thông báo)
 *   - LED nhấp nháy nhanh hơn giúp phân biệt bằng mắt thường
 *   - Không có auto-stop timer (lỗi kéo dài đến khi được khắc phục)
 */
static void start_error_alert(void)
{
    if (s_alert_active) return;   // Nếu đang báo động ngã, không ghi đè
                                  // WHY: ưu tiên báo động ngã cao hơn báo động lỗi

    s_error_state = true;          // Đánh dấu trạng thái lỗi
    s_alert_state = ALERT_STATE_ERROR;  // Chuyển sang trạng thái lỗi

    if (s_led_timer) {
        // Thay đổi chu kỳ LED timer từ 1000ms (báo thường) xuống 200ms (báo lỗi)
        xTimerChangePeriod(s_led_timer, pdMS_TO_TICKS(LED_ERROR_BLINK_PERIOD), 0);
        xTimerStart(s_led_timer, 0);   // Bắt đầu nhấp nháy
    }

    ESP_LOGE(TAG, ">>> ERROR STATE <<<");  // Log lỗi (mức ERROR)
}

/**
 * @brief Dừng báo động lỗi (khi I2C/WiFi đã phục hồi).
 *        Dừng LED nhấp nháy, đưa về trạng thái IDLE.
 */
static void stop_error_alert(void)
{
    s_error_state = false;                 // Xóa cờ lỗi
    s_alert_state = ALERT_STATE_IDLE;      // Quay về trạng thái rỗi

    if (s_led_timer) xTimerStop(s_led_timer, 0);  // Dừng LED nhấp nháy
    alert_led_off();                       // Tắt LED hoàn toàn

    ESP_LOGI(TAG, "Error state cleared");
}

// =========================== GPIO & ISR =====================================
// Xử lý ngắt GPIO cho nút bấm.
//
// ISR (Interrupt Service Routine):
//   - Hàm được gọi ngay khi có thay đổi mức điện áp trên chân GPIO
//   - THỜI GIAN CHẠY CỰC KỲ NGẮN: không được dùng ESP_LOG*, vTaskDelay, malloc...
//   - ISR chỉ GHI VÀO QUEUE (hàng đợi) và trả về
//   - Btn_task (chạy ở priority 3) đọc queue và xử lý logic nút bấm
//
// Cơ chế:
//   GPIO intr (ngắt cạnh lên/xuống)
//     → ISR push GPIO number vào queue
//       → btn_task pop từ queue
//         → xác định GPIO nào, press hay release
//           → khởi động/dừng timer
//             → timer callback (sau 3s) thực thi hành động
// ============================================================================

/**
 * @brief   Interrupt Service Routine (ISR) cho GPIO.
 *          Được gọi ở chế độ IRAM_ATTR (nằm trong RAM để đảm bảo tốc độ).
 * @param   arg   Con trỏ đến số hiệu GPIO (BTN_CANCEL_PIN hoặc BTN_SOS_PIN).
 *
 * Thuật toán debounce:
 *   1. Lấy thời gian hiện tại (tick count từ FreeRTOS trong ISR)
 *   2. So sánh với thời gian lần ngắt trước
 *   3. Nếu chênh lệch < DEBOUNCE_TIME_MS (50ms) → bỏ qua (nhiễu rung)
 *   4. Ngược lại → cập nhật thời gian và gửi GPIO number vào queue
 */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;                   // Ép kiểu con trỏ → uint32_t (số GPIO)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;       // Cờ báo cần context switch ngay sau ISR

    // Debounce: bỏ qua nếu gọi trong vòng 50ms
    uint32_t now = xTaskGetTickCountFromISR();           // Lấy tick count hiện tại (an toàn trong ISR)
    // Chọn con trỏ thời điểm lần ngắt cuối theo GPIO
    uint32_t *last_time = (gpio_num == BTN_CANCEL_PIN) ? &s_last_cancel_isr : &s_last_sos_isr;

    // Nếu khoảng thời gian từ lần ngắt trước < DEBOUNCE_TIME_MS (50ms) → bỏ qua
    // WHY: Khi nhấn nút cơ học, tiếp điểm rung lên xuống trong ~20-50ms,
    //      tạo ra nhiều ngắt giả. Debounce lọc bỏ các ngắt này.
    if ((now - *last_time) * portTICK_PERIOD_MS < DEBOUNCE_TIME_MS) {
        return;                                          // Bỏ qua ngắt do nhiễu
    }
    *last_time = now;                                    // Cập nhật thời điểm ngắt cuối

    // Gửi số GPIO vào queue để btn_task xử lý (từ context của ISR)
    // xHigherPriorityTaskWoken được đặt = pdTRUE nếu việc gửi queue đánh thức
    // một task có độ ưu tiên cao hơn task hiện tại. Yêu cầu context switch.
    xQueueSendFromISR(gpio_queue, &gpio_num, &xHigherPriorityTaskWoken);
}

/**
 * @brief Callback timer cho nút SOS.
 *        Được gọi sau 3 giây giữ nút (nếu nút vẫn đang được giữ).
 *
 * @param xTimer Handle timer (không dùng).
 *
 * Logic:
 *   - Kiểm tra s_sos_button_held (nút đang được giữ?)
 *   - Kiểm tra !s_alert_active (không trùng với báo động đang chạy?)
 *   - Nếu thỏa mãn: đặt cờ sos_telegram_pending, gọi start_fall_alert()
 *   - Reset cờ held để tránh kích hoạt lại
 */
static void sos_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;                                          // Tham số không dùng
    if (s_sos_button_held && !s_alert_active) {
        // Nút vẫn đang giữ và không có báo động nào khác
        s_sos_telegram_pending = true;   // Flag để main loop gọi Telegram
                                          // WHY: timer callback chạy trong context timer task,
                                          //      Không gọi HTTP (blocking) ở đây.
        start_fall_alert();               // Bật báo động (buzzer + LED)
        ESP_LOGW(TAG, ">>> SOS ALERT TRIGGERED (held 3s) <<<");
    }
    s_sos_button_held = false;           // Reset cờ giữ nút (dù có trigger hay không)
}

/**
 * @brief Callback timer cho nút CANCEL.
 *        Được gọi sau 3 giây giữ nút (nếu nút vẫn đang được giữ).
 *
 * @param xTimer Handle timer (không dùng).
 *
 * Logic:
 *   - Kiểm tra s_cancel_button_held (nút đang được giữ?)
 *   - Kiểm tra s_alert_active (có báo động để hủy không?)
 *   - Nếu thỏa mãn: dừng báo động, đặt cờ cancel_telegram_pending
 */
static void cancel_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (s_cancel_button_held && s_alert_active) {
        // Nút vẫn đang giữ và có báo động đang hoạt động
        s_cancel_telegram_pending = true; // Flag để main loop gọi Telegram
        stop_fall_alert();                 // Dừng báo động, fall_detection_reset() được gọi bên trong
        ESP_LOGW(TAG, ">>> FALSE ALARM CANCELLED (held 3s) <<<");
    }
    s_cancel_button_held = false;          // Reset cờ
}

/**
 * @brief Task xử lý nút bấm (CANCEL và SOS).
 * @param param Tham số task (không dùng).
 *
 * Luồng xử lý:
 *   1. Chờ nhận GPIO number từ queue (chờ vô hạn - portMAX_DELAY)
 *   2. Xác định GPIO nào (dùng if/else)
 *   3. Đọc mức GPIO để biết press (0) hay release (1) — pull-up nên press = 0
 *   4. Press:  đặt cờ held, reset timer đếm 3s
 *      Release: xóa cờ held, dừng timer
 *
 * WHY dùng queue thay vì xử lý trực tiếp trong ISR?
 *   - ISR phải ngắn, không được chặn (block)
 *   - Task có thể dùng các API FreeRTOS (xTimerReset, ESP_LOGI...) mà ISR không được dùng
 */
static void btn_task(void *param)
{
    (void)param;
    uint32_t gpio_num;                     // Biến nhận số GPIO từ queue

    while (1) {                            // Vòng lặp vô tận (FreeRTOS task pattern)
        // Chờ nhận dữ liệu từ gpio_queue. portMAX_DELAY = chờ vô hạn đến khi có dữ liệu.
        // xQueueReceive sẽ tiết kiệm CPU (task chuyển sang blocked state).
        if (xQueueReceive(gpio_queue, &gpio_num, portMAX_DELAY)) {
            if (gpio_num == BTN_CANCEL_PIN) {
                // === XỬ LÝ NÚT CANCEL ===
                // gpio_get_level: đọc mức logic hiện tại trên chân GPIO
                // Nút bấm có pull-up nội, nên:
                //   0 (LOW)  = nhấn (press)  - nối mass
                //   1 (HIGH) = không nhấn (release)
                if (gpio_get_level(BTN_CANCEL_PIN) == 0) {
                    // Press: bắt đầu đếm 3 giây
                    // Ý nghĩa: người dùng vừa nhấn nút CANCEL
                    ESP_LOGI(TAG, "Button CANCEL pressed (GPIO %lu) - hold 3s to cancel", gpio_num);
                    s_cancel_button_held = true;   // Đánh dấu nút đang được giữ

                    if (s_cancel_timer == NULL) {
                        // Nếu timer chưa tạo → tạo mới (one-shot, 3000ms)
                        s_cancel_timer = xTimerCreate("cancel_timer",
                                                       pdMS_TO_TICKS(CANCEL_HOLD_TIME_MS),
                                                       pdFALSE, NULL, cancel_timer_callback);
                    }
                    if (s_cancel_timer != NULL) {
                        // Reset timer: đặt lại bộ đếm về 0 và bắt đầu đếm lại
                        // WHY: nếu người dùng nhấn, nhả, nhấn lại, timer sẽ bắt đầu lại từ đầu
                        xTimerReset(s_cancel_timer, 0);
                    }
                } else {
                    // Release: hủy timer (người dùng nhả nút trước 3 giây)
                    ESP_LOGI(TAG, "Button CANCEL released (GPIO %lu)", gpio_num);
                    s_cancel_button_held = false;        // Xóa cờ giữ nút
                    if (s_cancel_timer != NULL) {
                        xTimerStop(s_cancel_timer, 0);   // Dừng đếm ngược
                    }
                }
            } else if (gpio_num == BTN_SOS_PIN) {
                // === XỬ LÝ NÚT SOS ===
                // Tương tự nút CANCEL nhưng kích hoạt báo động khẩn cấp
                if (gpio_get_level(BTN_SOS_PIN) == 0) {
                    // Press: bắt đầu đếm 3 giây
                    ESP_LOGI(TAG, "Button SOS pressed (GPIO %lu) - hold 3s to trigger", gpio_num);
                    s_sos_button_held = true;            // Đánh dấu nút đang được giữ

                    if (s_sos_timer == NULL) {
                        // Tạo timer mới (one-shot, 3000ms)
                        s_sos_timer = xTimerCreate("sos_timer",
                                                   pdMS_TO_TICKS(SOS_HOLD_TIME_MS),
                                                   pdFALSE, NULL, sos_timer_callback);
                    }
                    if (s_sos_timer != NULL) {
                        xTimerReset(s_sos_timer, 0);     // Reset và bắt đầu đếm 3s
                    }
                } else {
                    // Release: hủy timer
                    ESP_LOGI(TAG, "Button SOS released (GPIO %lu)", gpio_num);
                    s_sos_button_held = false;
                    if (s_sos_timer != NULL) {
                        xTimerStop(s_sos_timer, 0);      // Dừng đếm ngược
                    }
                }
            }
        }
    }
}

/**
 * @brief Cấu hình tất cả GPIO, I2C và tạo button handler task.
 *
 * Cấu hình bao gồm:
 *   - LED_PIN và BUZ_PIN → OUTPUT (điều khiển LED và còi)
 *   - BTN_CANCEL_PIN và BTN_SOS_PIN → INPUT (có pull-up nội, ngắt cạnh bất kỳ)
 *   - Khởi tạo hàng đợi GPIO event
 *   - Cài đặt ISR service và đăng ký ISR handler cho 2 nút
 *   - Tạo FreeRTOS task cho button handler
 *   - Cấu hình I2C master (SCL GPIO9, SDA GPIO8, 400kHz)
 *
 * WHY gộp cấu hình GPIO và I2C trong một hàm?
 *   - Cả hai đều là cấu hình phần cứng ở đầu chương trình
 *   - Giúp gọn main() và dễ quản lý thứ tự khởi tạo
 */
void gpio_conf(void) {
    // LED và Buzzer là OUTPUT (ESP32 điều khiển mức điện áp ra)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,                   // Không cần ngắt cho output
        .mode = GPIO_MODE_OUTPUT,                          // Chế độ xuất (output)
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << BUZ_PIN),  // Chọn chân: bitmask 2 chân
        .pull_down_en = GPIO_PULLDOWN_DISABLE,             // Không dùng pull-down
        .pull_up_en = GPIO_PULLUP_DISABLE                  // Không dùng pull-up (output không cần)
    };
    gpio_config(&io_conf);                                 // Áp dụng cấu hình

    // Nút CANCEL: INPUT với pull-up, ngắt ở cả cạnh lên và xuống
    // WHY any edge: cần phát hiện cả press (cạnh xuống) và release (cạnh lên)
    // để quản lý trạng thái "đang giữ nút" và timer đếm 3 giây.
    io_conf.mode = GPIO_MODE_INPUT;                        // Chế độ nhập (input)
    io_conf.intr_type = GPIO_INTR_ANYEDGE;                 // Ngắt cạnh bất kỳ (lên & xuống)
    io_conf.pin_bit_mask = (1ULL << BTN_CANCEL_PIN);       // Chọn chân CANCEL
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;               // Bật pull-up nội (50kOhm)
    gpio_config(&io_conf);                                 // Áp dụng

    // Nút SOS: cấu hình tương tự nút CANCEL
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << BTN_SOS_PIN);          // Chọn chân SOS
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Tạo queue cho GPIO events
    // WHY: queue là cơ chế giao tiếp an toàn giữa ISR (ngắt) và task (FreeRTOS).
    //      ISR gửi (send) dữ liệu, task nhận (receive) và xử lý.
    //      Kích thước: GPIO_QUEUE_SIZE = 20 phần tử, mỗi phần tử = uint32_t (4 bytes).
    gpio_queue = xQueueCreate(GPIO_QUEUE_SIZE, sizeof(uint32_t));

    // Cài đặt ISR service toàn cục cho GPIO
    // Tham số 0 = sử dụng cấu hình mặc định (ESP_INTR_FLAG_DEFAULT)
    gpio_install_isr_service(0);

    // Đăng ký ISR handler cho từng chân GPIO
    // Khi có ngắt trên chân BTN_CANCEL_PIN → gọi gpio_isr_handler với tham số là số chân
    gpio_isr_handler_add(BTN_CANCEL_PIN, gpio_isr_handler, (void *)BTN_CANCEL_PIN);
    gpio_isr_handler_add(BTN_SOS_PIN, gpio_isr_handler, (void *)BTN_SOS_PIN);

    // Tạo button handler task chạy trên core hiện tại
    // xTaskCreate(function, name, stack_size, params, priority, handle)
    xTaskCreate(btn_task, "btn_task", BTN_TASK_STACK, NULL, BTN_TASK_PRIORITY, NULL);

    // ========== CẤU HÌNH I2C ==========
    // Cấu hình giao tiếp I2C với MPU6050
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,          // ESP32 là master (điều khiển bus)
        .sda_io_num = I2C_MASTER_SDA_IO,  // Chân SDA = GPIO8
        .scl_io_num = I2C_MASTER_SCL_IO,  // Chân SCL = GPIO9
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Bật pull-up cho SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Bật pull-up cho SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ,  // Tốc độ = 400kHz (Fast Mode)
    };

    // Áp dụng cấu hình I2C thông qua driver
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to configure I2C: %s", esp_err_to_name(ret));
        start_error_alert();    // Báo lỗi LED nếu I2C không cấu hình được
        return;                 // Dừng khởi tạo, không thể giao tiếp cảm biến
    }

    // Cài đặt driver I2C (cấp phát tài nguyên hệ thống)
    // Tham số: port, mode, RX buffer = 0, TX buffer = 0, không dùng DMA
    ret = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to install I2C driver: %s", esp_err_to_name(ret));
        start_error_alert();    // Báo lỗi nếu không cài được driver
        return;
    }
    ESP_LOGI("I2C", "I2C initialized");
}

// =========================== TASK MPU6050 ===================================
// Task chuyên dụng đọc dữ liệu từ cảm biến MPU6050 và cập nhật thuật toán phát
// hiện té ngã. Tốc độ lấy mẫu: 100Hz (mỗi 10ms).
//
// Task này thực hiện:
//   1. Khởi tạo MPU6050 (cấu hình thanh ghi, kiểm tra kết nối)
//   2. Hiệu chuẩn (calibration) gyroscope: 500 mẫu × 10ms = 5 giây
//   3. Vòng lặp 100Hz:
//      a. Đọc raw data (accel + gyro)
//      b. Chuyển đổi sang đơn vị vật lý (g, deg/s)
//      c. Tính tổng gia tốc và tổng vận tốc góc
//      d. Tính góc Roll và Pitch
//      e. Ghi vào double-buffer (cho web server)
//      f. Cập nhật fall detection
//      g. LED indicator cho trạng thái WAIT_LIE_DOWN
// ============================================================================

/**
 * @brief   Task chính đọc và xử lý dữ liệu từ MPU6050.
 * @param   param Tham số truyền khi tạo task (không dùng).
 *
 * Chi tiết xử lý:
 *   - Khởi tạo cảm biến: mpu6050_init() gửi lệnh I2C cấu hình MPU6050
 *   - HIỆU CHUẨN 5 GIÂY (500 mẫu): thuật toán ước lượng bias (độ lệch) của
 *     gyroscope khi đứng yên, giúp loại bỏ drift (trôi) trong tính toán góc.
 *   - Vòng lặp: mỗi 10ms thực hiện:
 *       1. mpu6050_read_raw_data(): đọc 6 giá trị raw (3 accel + 3 gyro)
 *       2. mpu6050_convert_*(): chuyển raw → giá trị vật lý
 *       3. mpu6050_get_total_*(): tính tổng vector (sqrt(x²+y²+z²))
 *       4. roll_pitch_update() + get_roll/pitch(): góc định hướng
 *       5. Ghi vào double-buffer (luân phiên buffer 0/1)
 *       6. fall_detection_update(): cập nhật máy trạng thái phát hiện ngã
 *
 *   FALL DETECTION CALLBACK:
 *     Khi phát hiện ngã, fall_detection_update() gọi fall_alert_callback(),
 *     hàm này gửi Telegram + bật báo động.
 */
void mpu6050_task(void *param) {
    (void)param;
    esp_err_t ret;                     // Mã lỗi ESP, kiểm tra sau mỗi lần gọi API
    int16_t raw_ax, raw_ay, raw_az;    // Gia tốc raw (ADC counts) - trục X, Y, Z
    int16_t raw_gx, raw_gy, raw_gz;    // Vận tốc góc raw (ADC counts) - trục X, Y, Z
    float accel_x, accel_y, accel_z;   // Gia tốc sau chuyển đổi (đơn vị: g ≈ 9.8 m/s²)
    float gyro_x, gyro_y, gyro_z;      // Vận tốc góc sau chuyển đổi (đơn vị: deg/s)
    float gyro_bias[3] = {0};          // Mảng bias của gyroscope [0]=X, [1]=Y, [2]=Z
                                         // Bias = giá trị offset khi cảm biến đứng yên
                                         // (do chế tạo, mỗi cảm biến có bias khác nhau)

    // === Khởi tạo MPU6050 ===
    // Gửi lệnh I2C để:
    //   - Wake up MPU6050 (mặc định sleep mode sau reset)
    //   - Cấu hình full scale: ±2g (accel), ±250°/s (gyro)
    //   - Cấu hình digital low-pass filter (giảm nhiễu)
    ret = mpu6050_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to initialize: %s", esp_err_to_name(ret));
        start_error_alert();           // Báo lỗi (LED nhấp nháy nhanh)
        vTaskDelete(NULL);             // Tự xóa task này (không thể chạy nếu không có cảm biến)
        return;                        // Không bao giờ đến đây (vTaskDelete đã xóa task)
    }

    // ========== HIỆU CHUẨN 5 GIÂY ==========
    // Mục đích: đo giá trị bias của gyroscope để loại trừ khỏi dữ liệu thực tế.
    // Phương pháp: giữ thiết bị ĐỨNG YÊN, lấy 500 mẫu, tính trung bình.
    // WHY cần hiệu chuẩn:
    //   - Gyroscope lý tưởng: đứng yên → output = 0 deg/s
    //   - Thực tế: linh kiện bán dẫn có drift theo nhiệt độ, sai số chế tạo
    //   - Bias càng nhỏ → tính toán góc Roll/Pitch càng chính xác
    ESP_LOGI("CALIB", "Calibrating... Keep device steady!");
    ESP_LOGI("CALIB", "LED will blink during 5s calibration...");

    // LED blink trong khi calibration: 500 mẫu × 10ms = 5 giây
    // Mỗi 100 mẫu là một chu kỳ blink (50 on + 50 off = 1 giây)
    // Hiệu ứng: LED nhấp nháy chậm báo hiệu đang hiệu chuẩn
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        gpio_set_level(LED_PIN, (i % 100 < 50) ? 1 : 0);  // LED blink: mỗi 100 mẫu
        mpu6050_calibrate_sample(I2C_MASTER_NUM);          // Thu thập 1 mẫu calibration
        vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_RATE_MS)); // Đợi 10ms (đúng tần số 100Hz)
    }
    // Kết thúc calibration: tính bias từ các mẫu đã thu thập
    // gyro_bias[3] nhận kết quả (đơn vị: deg/s)
    mpu6050_calibrate_finish(NULL, gyro_bias);

    // In bias ra console để debug
    ESP_LOGI("CALIB", "Calibration done: gyro bias [%.2f, %.2f, %.2f] deg/s",
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    // ========== BUZZER XÁC NHẬN ==========
    // Bíp còi 1 giây báo hiệu hoàn tất hiệu chuẩn
    ESP_LOGI("CALIB", "Calibration complete!");
    gpio_set_level(BUZ_PIN, 1);        // Buzzer ON
    vTaskDelay(pdMS_TO_TICKS(CALIB_BUZZER_DURATION_MS));  // Giữ 1 giây
    gpio_set_level(BUZ_PIN, 0);        // Buzzer OFF
    gpio_set_level(LED_PIN, 0);       // Tắt LED sau khi hiệu chuẩn

    // Khởi tạo bộ tính roll/pitch (bộ lọc bổ sung - complementary filter)
    // WHY complementary filter:
    //   - Accelerometer: chính xác tĩnh, nhiễu khi động
    //   - Gyroscope: chính xác động, drift khi tĩnh
    //   - Kết hợp: dùng trọng số (thường 0.98 gyro + 0.02 accel) để ước lượng
    //     góc tốt hơn từng loại riêng lẻ.
    roll_pitch_init();
    ESP_LOGI("RollPitch", "Initialized");

    // ========== VÒNG LẶP CHÍNH 100Hz ==========
    while (1) {
        // === Bước 1: Đọc dữ liệu thô (raw data) từ MPU6050 ===
        // MPU6050 có 6 thanh ghi dữ liệu 16-bit (2 bytes mỗi trục):
        //   ACCEL_XOUT_H/L, ACCEL_YOUT_H/L, ACCEL_ZOUT_H/L
        //   GYRO_XOUT_H/L, GYRO_YOUT_H/L, GYRO_ZOUT_H/L
        // Hàm đọc tất cả 12 bytes (6 × 16-bit) qua I2C trong 1 lần
        ret = mpu6050_read_raw_data(I2C_MASTER_NUM, &raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        if (ret != ESP_OK) {
            // Lỗi I2C: thường do mất kết nối cảm biến (cáp đứt, chân tiếp xúc kém)
            ESP_LOGE("MPU6050", "Read failed!");
            start_error_alert();              // Báo lỗi LED
            vTaskDelay(pdMS_TO_TICKS(1000));  // Đợi 1 giây thử lại
            continue;                          // Bỏ qua vòng lặp hiện tại
        }

        // === Bước 2: Chuyển đổi raw → đơn vị vật lý ===
        // Công thức: value_phys = raw / sensitivity
        //   Accel: sensitivity = 16384 counts/g (ở ±2g) → accel_g = raw / 16384
        //   Gyro:  sensitivity = 131 counts/°/s (ở ±250°/s) → gyro_dps = raw / 131
        mpu6050_convert_accel(raw_ax, raw_ay, raw_az, &accel_x, &accel_y, &accel_z);
        mpu6050_convert_gyro(raw_gx, raw_gy, raw_gz, &gyro_x, &gyro_y, &gyro_z);

        // === Bước 3: Tính tổng vector ===
        float total_accel_g;                   // |a| = sqrt(ax² + ay² + az²) [g]
        mpu6050_get_total_accel(accel_x, accel_y, accel_z, &total_accel_g);
        float total_gyro = mpu6050_get_total_gyro(gyro_x, gyro_y, gyro_z);  // |g| = sqrt(gx² + gy² + gz²) [deg/s]

        // === Bước 4: Tính góc Roll và Pitch ===
        // Roll (φ): góc xoay quanh trục X (nghiêng trái-phải)
        // Pitch (θ): góc xoay quanh trục Y (ngẩng-cúi)
        // Công thức (từ accelerometer):
        //   roll  = atan2(ay, az)
        //   pitch = atan2(-ax, sqrt(ay² + az²))
        // Sau đó kết hợp với gyroscope qua complementary filter
        roll_pitch_update(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        float roll = get_roll();     // Lấy góc Roll hiện tại (đơn vị: độ)
        float pitch = get_pitch();   // Lấy góc Pitch hiện tại (đơn vị: độ)

        // === Bước 5: Ghi vào double-buffer ===
        // Ghi tất cả dữ liệu đã xử lý vào buffer hiện tại của writer
        g_sensor_buffers[s_writer_index].total_accel_g = total_accel_g;
        g_sensor_buffers[s_writer_index].total_gyro = total_gyro;
        g_sensor_buffers[s_writer_index].roll = roll;
        g_sensor_buffers[s_writer_index].pitch = pitch;
        g_sensor_buffers[s_writer_index].data_ready = true;  // Đánh dấu dữ liệu đã sẵn sàng

        // === Bước 6: Swap buffer (ping-pong) ===
        // Writer chuyển sang buffer còn lại để ghi lần tiếp theo
        // Reader (webserver) sẽ đọc buffer còn lại: 1 - s_writer_index
        s_writer_index = 1 - s_writer_index;  // 0→1 hoặc 1→0

        // === Bước 7: Xóa error state nếu I2C đã hoạt động trở lại ===
        // Nếu trước đó có lỗi I2C và bây giờ đọc thành công → tắt báo lỗi
        if (s_error_state) {
            stop_error_alert();     // LED về bình thường
        }

        // === Bước 8: Cập nhật Fall Detection ===
        // Truyền dữ liệu cho thuật toán phát hiện ngã
        // fall_detection_update() kiểm tra các ngưỡng và máy trạng thái
        // Nếu phát hiện ngã → gọi callback (fall_alert_callback)
        fall_detection_update(total_accel_g, total_gyro, pitch, roll);

        // === Bước 9: LED indicator cho trạng thái WAIT_LIE_DOWN ===
        // Trạng thái 3 (WAIT_LIE_DOWN) = đã phát hiện impact, chờ xác nhận nằm
        // LED nhấp nháy 500ms để báo hiệu cần xác nhận (tránh báo động giả)
        static bool was_wait_lie = false;           // Flag nhớ trạng thái trước đó
        bool is_wait_lie = (fall_detection_get_state_internal() == 3);  // Kiểm tra state hiện tại
        if (is_wait_lie) {
            static uint32_t blk = 0;               // Bộ đếm nhấp nháy (local static)
            blk = (blk + 1) % 10;                  // Mod 10: chu kỳ 10 lần = 10×10ms = 100ms
            gpio_set_level(LED_PIN, blk < 5);      // 5 on / 5 off → 50% duty cycle
        } else if (was_wait_lie) {
            gpio_set_level(LED_PIN, 0);            // Thoát trạng thái → tắt LED
        }
        was_wait_lie = is_wait_lie;                 // Cập nhật biến lịch sử

        vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_RATE_MS));  // Chờ 10ms cho đúng 100Hz
    }
}

// =========================== CALLBACK PHÁT HIỆN NGÃ =========================
// Hàm này được fall_detection module gọi khi phát hiện té ngã.
// Được đăng ký với fall_detection_set_callback() trong app_main().
// ============================================================================

/**
 * @brief   Callback khi phát hiện té ngã (fall detected).
 *          Gửi thông báo Telegram và bật báo động (còi + LED).
 *
 * Được gọi từ fall_detection_update() (trong context của mpu6050_task)
 * khi máy trạng thái fall detection chuyển sang trạng thái FALL_CONFIRMED.
 */
static void fall_alert_callback(void)
{
    // Gửi tin nhắn Telegram: "Phát hiện té ngã!" + thời gian
    telegram_send_fall_alert();

    // Bật báo động: còi kêu, LED nhấp nháy, timer tự tắt 30 giây
    start_fall_alert();
}

// =========================== MAIN ===========================================
// Hàm app_main() là điểm vào của chương trình ESP-IDF.
// Được gọi sau khi FreeRTOS kernel khởi động và scheduler bắt đầu chạy.
//
// Thứ tự khởi tạo:
//   1. NVS flash: lưu trữ thông tin WiFi (SSID, password) không bay hơi
//   2. GPIO + I2C: cấu hình chân, driver I2C, tạo button task
//   3. Telegram: cấu hình bot token và chat ID
//   4. Fall detection: khởi tạo máy trạng thái, đăng ký callback
//   5. WiFi station mode: kết nối đến router
//   6. Tạo MPU6050 task: bắt đầu đọc cảm biến
//   7. Chờ WiFi: tối đa 30 giây, nếu thành công → start webserver
//   8. Main loop: kiểm tra flag Telegram, trạng thái WiFi
// ============================================================================
void app_main(void) {
    // ------------- Bước 1: Khởi tạo NVS -------------
    // NVS (Non-Volatile Storage) dùng để lưu cấu hình WiFi (SSID, password)
    // mà không mất khi mất nguồn. Cần khởi tạo trước khi gọi wifi_init_sta().
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Lỗi hết trang nhớ hoặc phiên bản NVS cũ → xóa và khởi tạo lại
        ESP_ERROR_CHECK(nvs_flash_erase());   // Xóa toàn bộ NVS partition
        ret = nvs_flash_init();                // Khởi tạo lại
    }
    ESP_ERROR_CHECK(ret);                      // Nếu vẫn lỗi → dừng chương trình

    // ------------- Bước 2: Cấu hình GPIO + I2C -------------
    ESP_LOGI(TAG, "======== SYSTEM START ========");
    gpio_conf();                               // Khởi tạo GPIO, I2C, button task

    // ------------- Bước 3: Khởi tạo Telegram -------------
    // Cấu hình bot Telegram: gán token và chat ID
    // Các HTTP request sẽ được gửi từ các hàm telegram_send_*
    telegram_init(TELEGRAM_BOT_TOKEN, TELEGRAM_CHAT_ID);

    // ------------- Bước 4: Khởi tạo Fall Detection -------------
    // Khởi tạo máy trạng thái (state machine) phát hiện té ngã
    // Thiết lập các ngưỡng: gia tốc, vận tốc góc, góc nghiêng
    fall_detection_init();

    // Đăng ký callback: khi phát hiện ngã → gọi fall_alert_callback
    fall_detection_set_callback(fall_alert_callback);

    // ------------- Bước 5: Khởi tạo WiFi -------------
    // Bắt đầu kết nối WiFi ở chế độ Station (client)
    // Thông tin WiFi được đọc từ NVS (đã được cấu hình trước đó)
    wifi_init_sta();

    // ------------- Bước 6: Tạo MPU6050 task -------------
    // Task này chạy độc lập với main loop, xử lý cảm biến ở 100Hz
    // xTaskCreate(func, name, stack, param, priority, handle)
    xTaskCreate(mpu6050_task, "mpu6050_task", MPU6050_TASK_STACK, NULL, MPU6050_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "App initialization complete - waiting for WiFi...");

    // ------------- Bước 7: Chờ kết nối WiFi -------------
    // Chờ tối đa WIFI_CONNECT_TIMEOUT_S (30) giây
    // Mỗi giây kiểm tra wifi_is_connected()
    bool wifi_connected = false;
    for (int i = 0; i < WIFI_CONNECT_TIMEOUT_S; i++) {
        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));  // Đợi 1 giây
        if (wifi_is_connected()) {
            ESP_LOGI(TAG, "WiFi connected, starting webserver...");
            // Khởi động webserver (dashboard HTML + REST API)
            // Cần WiFi để server có địa chỉ IP
            webserver_start();

            // Gửi tin nhắn Telegram thông báo hệ thống khởi động
            telegram_send_startup();

            wifi_connected = true;   // Đánh dấu kết nối thành công
            break;                    // Thoát vòng lặp chờ
        }
    }

    // Sau 30 giây mà không kết nối được WiFi
    if (!wifi_connected) {
        ESP_LOGE(TAG, "WiFi connection failed!");
        start_error_alert();         // Báo lỗi: LED nhấp nháy nhanh
                                      // Hệ thống vẫn chạy (cảm biến, nút bấm) nhưng
                                      // không có Telegram và web dashboard
    }

    // ========== MAIN LOOP ==========
    // Vòng lặp chính: chạy với chu kỳ 1 giây
    // Chức năng:
    //   1. Xử lý flag Telegram SOS/CANCEL (gửi HTTP request)
    //   2. Kiểm tra trạng thái WiFi
    //   3. Log trạng thái hệ thống
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));  // Đợi 1 giây

        // === Xử lý Telegram pending flags ===
        // Các flag này được đặt trong timer callback (SOS/CANCEL timer)
        // WHY xử lý ở đây:
        //   - Timer callback chạy trong context của FreeRTOS timer task
        //   - Các hàm HTTP (telegram_send_*) có thể block (chờ TCP/TLS)
        //   - Block trong timer callback là không được phép
        //   - Giải pháp: set flag, main loop kiểm tra và gọi hàm
        if (s_sos_telegram_pending) {
            s_sos_telegram_pending = false;               // Xóa flag
            telegram_send_sos_alert();                     // Gửi tin nhắn SOS
        }
        if (s_cancel_telegram_pending) {
            s_cancel_telegram_pending = false;             // Xóa flag
            telegram_send_cancel_alert();                  // Gửi tin nhắn hủy báo
        }

        // === Kiểm tra WiFi ===
        // Nếu mất WiFi và không đang báo động → báo lỗi
        // Nếu WiFi phục hồi và đang ở trạng thái lỗi → xóa lỗi
        if (!wifi_is_connected() && !s_alert_active) {
            ESP_LOGE(TAG, "WiFi disconnected!");
            start_error_alert();          // Báo lỗi WiFi mất kết nối
        } else if (wifi_is_connected() && s_error_state) {
            stop_error_alert();           // WiFi phục hồi → tắt báo lỗi
        }

        // Log trạng thái tổng thể mỗi giây
        ESP_LOGI(TAG, "System running - WiFi: %s | Alert: %s | Error: %s",
                 wifi_is_connected() ? "OK" : "NOT CONNECTED",
                 s_alert_active ? "ACTIVE" : "IDLE",
                 s_error_state ? "YES" : "NO");
    }
}
