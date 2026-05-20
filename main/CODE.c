// =========================== GIỚI THIỆU HỆ THỐNG ===========================
// CODE.c — Chương trình chính. Thiết bị phát hiện té ngã cho người già, dùng
// cảm biến MPU6050 (gia tốc kế + con quay hồi chuyển). Khi phát hiện ngã:
//   - Bật còi + LED nhấp nháy
//   - Gửi thông báo Telegram
//   - Dashboard web theo dõi trực tuyến
// Nút CANCEL hủy báo động giả; nút SOS kích hoạt khẩn cấp thủ công.
//
// =========================== TỔ CHỨC CODE =================================
// 1. GPIO + I2C initialization
// 2. MPU6050 task (100Hz): đọc cảm biến, phát hiện té ngã
// 3. Button task: xử lý nút bấm CANCEL/SOS
// 4. Webserver: dashboard + API
// 5. Telegram: gửi thông báo qua Telegram Bot API
//
// =========================== LUỒNG DỮ LIỆU ================================
// MPU6050 (cảm biến)
//   → mpu6050_task (đọc raw data ở 100Hz)
//     → fall_detection_update() (thuật toán phát hiện té ngã)
//       → [FALL DETECTED] → alert_callback()
//         ├──→ telegram_send_fall_alert() (gửi tin nhắn Telegram)
//         └──→ start_fall_alert() (bật buzzer + LED nhấp nháy)
//
// Nút bấm vật lý:
//   ├── BTN_SOS (GPIO 6): giữ 3 giây → SOS Telegram + báo động
//   └── BTN_CANCEL (GPIO 5): giữ 3 giây khi đang báo → hủy báo + Telegram
//
// =========================== HỆ ĐIỀU HÀNH =================================
// Chạy trên FreeRTOS (ESP-IDF framework) với các task:
//   - mpu6050_task:  priority 5, stack 4096, vòng lặp 100Hz
//   - btn_task:      priority 3, stack 2048, chờ GPIO event từ queue
//   - main loop:     vòng lặp chính delay 1 giây, kiểm tra WiFi & flag

// =========================== THƯ VIỆN =====================================
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "mpu6050.h"
#include "roll_pitch.h"
#include "wifi.h"
#include "webserver.h"
#include "fall_detection.h"
#include "telegram.h"

// =========================== CẤU HÌNH I2C =================================
#define I2C_MASTER_SCL_IO       9   // GPIO9: SCL
#define I2C_MASTER_SDA_IO       8   // GPIO8: SDA
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000

// =========================== CẤU HÌNH GPIO =================================
#define BUZ_PIN         0
#define LED_PIN         1
#define BTN_CANCEL_PIN  5
#define BTN_SOS_PIN     6

// =========================== CẤU HÌNH THỜI GIAN ============================
#define ALERT_DURATION_MS  30000   // Báo động tự tắt sau 30 giây
#define LED_BLINK_PERIOD    1000
#define LED_ERROR_BLINK_PERIOD 200
#define SOS_HOLD_TIME_MS    3000
#define CANCEL_HOLD_TIME_MS 3000
#define DEBOUNCE_TIME_MS    50

// =========================== HẰNG SỐ HỆ THỐNG =============================
#define MPU6050_TASK_STACK      4096
#define MPU6050_TASK_PRIORITY  5
#define MPU6050_SAMPLE_RATE_MS  10      // 100Hz
#define BTN_TASK_STACK          2048
#define BTN_TASK_PRIORITY      3
#define GPIO_QUEUE_SIZE         20
#define WIFI_CONNECT_TIMEOUT_S  30
#define CALIBRATION_SAMPLES     500     // 500 x 10ms = 5 giây
#define CALIB_BUZZER_DURATION_MS 1000
#define MAIN_LOOP_DELAY_MS      1000

// Token và Chat ID Telegram — cấu hình qua idf.py menuconfig
// (Kconfig.projbuild → CONFIG_TELEGRAM_BOT_TOKEN, CONFIG_TELEGRAM_CHAT_ID)

// =========================== BIẾN TOÀN CỤC =================================
static const char *TAG = "MAIN";

// Hàng đợi nhận sự kiện GPIO từ ISR. ISR gửi số GPIO vào đây,
// btn_task đọc ra để xử lý — tránh làm nặng ISR.
static QueueHandle_t gpio_queue = NULL;

// =========================== TRẠNG THÁI BÁO ĐỘNG ===========================
typedef enum {
    ALERT_STATE_IDLE,
    ALERT_STATE_FALLING,        // Đang báo: còi + LED + timer tự tắt
    ALERT_STATE_ERROR           // Mất I2C hoặc WiFi: LED nhấp nháy nhanh
} alert_state_t;

static alert_state_t s_alert_state = ALERT_STATE_IDLE;

static TimerHandle_t s_alert_timer = NULL;    // Tự tắt sau 30 giây (one-shot)
static TimerHandle_t s_led_timer = NULL;      // Nhấp nháy LED khi báo động
static TimerHandle_t s_sos_timer = NULL;      // Đếm 3s giữ nút SOS
static TimerHandle_t s_cancel_timer = NULL;   // Đếm 3s giữ nút CANCEL

static bool s_led_state = false;
static bool s_alert_active = false;           // Ngăn báo động chồng lấn
static bool s_error_state = false;

static bool s_sos_button_held = false;
static bool s_cancel_button_held = false;

// Cờ báo hiệu gửi Telegram — timer callback chỉ đặt flag, main loop xử lý HTTP
// vì timer callback không được blocking (HTTP request chặn).
static bool s_sos_telegram_pending = false;
static bool s_cancel_telegram_pending = false;

// Thời điểm ngắt cuối — dùng cho debounce chống rung cơ học
static uint32_t s_last_cancel_isr = 0;
static uint32_t s_last_sos_isr = 0;

// =========================== DOUBLE-BUFFER CẢM BIẾN ========================
// Double-buffer (ping-pong) cho phép MPU6050 task (writer) ghi dữ liệu vào
// một buffer trong khi web server (reader) đọc buffer còn lại, loại bỏ race
// condition mà không cần khóa (lock). Writer ghi vào buffer s_writer_index,
// reader tính reader_index = 1 - s_writer_index.
// ============================================================================

sensor_data_t g_sensor_buffers[2] = {
    {.total_accel_g = 0, .total_gyro = 0, .roll = 0, .pitch = 0, .data_ready = false},
    {.total_accel_g = 0, .total_gyro = 0, .roll = 0, .pitch = 0, .data_ready = false}
};

static uint8_t s_writer_index = 0;

uint8_t g_get_writer_index(void) {
    return s_writer_index;
}

// =========================== CÁC HÀM BÁO ĐỘNG ==============================
static void alert_led_off(void)
{
    gpio_set_level(LED_PIN, 0);
    s_led_state = false;
}

static void alert_led_toggle(void)
{
    s_led_state = !s_led_state;
    gpio_set_level(LED_PIN, s_led_state ? 1 : 0);
}

static void stop_fall_alert(void);

// Tự động tắt báo sau 30 giây — phòng trường hợp người dùng bất tỉnh không
// can thiệp được. Cũng giúp tiết kiệm pin.
static void alert_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    ESP_LOGI(TAG, "Alert duration ended - auto stop");
    stop_fall_alert();
}

static void start_fall_alert(void)
{
    if (s_alert_active) return;     // Tránh kích hoạt chồng lấn (ngã + SOS cùng lúc)

    s_alert_active = true;
    s_alert_state = ALERT_STATE_FALLING;

    gpio_set_level(BUZ_PIN, 1);

    if (s_led_timer == NULL) {
        s_led_timer = xTimerCreate("led_blink", pdMS_TO_TICKS(LED_BLINK_PERIOD),
                                    pdTRUE, NULL, (TimerCallbackFunction_t)alert_led_toggle);
    }
    if (s_led_timer) {
        xTimerStart(s_led_timer, 0);
    }

    if (s_alert_timer == NULL) {
        s_alert_timer = xTimerCreate("alert_timeout", pdMS_TO_TICKS(ALERT_DURATION_MS),
                                      pdFALSE, NULL, alert_timer_callback);
    }
    if (s_alert_timer) {
        xTimerStart(s_alert_timer, 0);
    }

    ESP_LOGW(TAG, ">>> FALL ALERT STARTED <<<");
}

static void stop_fall_alert(void)
{
    s_alert_active = false;
    s_alert_state = ALERT_STATE_IDLE;

    gpio_set_level(BUZ_PIN, 0);

    if (s_led_timer) xTimerStop(s_led_timer, 0);
    if (s_alert_timer) xTimerStop(s_alert_timer, 0);

    alert_led_off();

    // Reset state machine của fall detection, nếu không nó sẽ mắc kẹt ở trạng
    // thái "đã ngã" và bỏ qua lần té tiếp theo.
    fall_detection_reset();

    ESP_LOGI(TAG, "Fall alert stopped - system reset to IDLE");
}

// Báo lỗi riêng biệt: LED nhấp nháy nhanh (200ms), không bật còi, không gửi
// Telegram để tránh spam. Không có auto-stop vì lỗi kéo dài đến khi phục hồi.
static void start_error_alert(void)
{
    if (s_alert_active) return;     // Ưu tiên báo ngã cao hơn báo lỗi

    s_error_state = true;
    s_alert_state = ALERT_STATE_ERROR;

    if (s_led_timer) {
        xTimerChangePeriod(s_led_timer, pdMS_TO_TICKS(LED_ERROR_BLINK_PERIOD), 0);
        xTimerStart(s_led_timer, 0);
    }

    ESP_LOGE(TAG, ">>> ERROR STATE <<<");
}

static void stop_error_alert(void)
{
    s_error_state = false;
    s_alert_state = ALERT_STATE_IDLE;

    if (s_led_timer) xTimerStop(s_led_timer, 0);
    alert_led_off();

    ESP_LOGI(TAG, "Error state cleared");
}

// =========================== GPIO & ISR =====================================
// Cơ chế xử lý nút bấm:
//   GPIO ngắt cạnh lên/xuống → ISR push số GPIO vào queue → btn_task (FreeRTOS)
//   nhận từ queue, xác định press/release, khởi động/dừng timer 3s → timer
//   callback thực thi hành động.
//
// ISR phải thật ngắn: không ESP_LOG*, vTaskDelay, malloc — chỉ ghi queue rồi ra.
// ============================================================================

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Debounce: bỏ qua nếu hai ngắt cách nhau dưới 50ms
    uint32_t now = xTaskGetTickCountFromISR();
    uint32_t *last_time = (gpio_num == BTN_CANCEL_PIN) ? &s_last_cancel_isr : &s_last_sos_isr;

    // Nút cơ học rung ~20-50ms khi nhấn, tạo nhiều ngắt giả
    if ((now - *last_time) * portTICK_PERIOD_MS < DEBOUNCE_TIME_MS) {
        return;
    }
    *last_time = now;

    xQueueSendFromISR(gpio_queue, &gpio_num, &xHigherPriorityTaskWoken);
}

static void sos_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (s_sos_button_held && !s_alert_active) {
        s_sos_telegram_pending = true;   // Main loop sẽ gửi Telegram — timer
        start_fall_alert();               // callback không được blocking
        ESP_LOGW(TAG, ">>> SOS ALERT TRIGGERED (held 3s) <<<");
    }
    s_sos_button_held = false;
}

static void cancel_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (s_cancel_button_held && s_alert_active) {
        s_cancel_telegram_pending = true;
        stop_fall_alert();
        ESP_LOGW(TAG, ">>> FALSE ALARM CANCELLED (held 3s) <<<");
    }
    s_cancel_button_held = false;
}

static void btn_task(void *param)
{
    (void)param;
    uint32_t gpio_num;

    while (1) {
        // Chờ queue — task tự động blocked, không tốn CPU
        if (xQueueReceive(gpio_queue, &gpio_num, portMAX_DELAY)) {
            if (gpio_num == BTN_CANCEL_PIN) {
                if (gpio_get_level(BTN_CANCEL_PIN) == 0) {
                    ESP_LOGI(TAG, "Button CANCEL pressed (GPIO %lu) - hold 3s to cancel", gpio_num);
                    s_cancel_button_held = true;
                    if (s_cancel_timer == NULL) {
                        s_cancel_timer = xTimerCreate("cancel_timer",
                                                       pdMS_TO_TICKS(CANCEL_HOLD_TIME_MS),
                                                       pdFALSE, NULL, cancel_timer_callback);
                    }
                    if (s_cancel_timer != NULL) {
                        // Reset timer — nếu nhấn-nhả-nhấn lại, đếm lại từ đầu
                        xTimerReset(s_cancel_timer, 0);
                    }
                } else {
                    ESP_LOGI(TAG, "Button CANCEL released (GPIO %lu)", gpio_num);
                    s_cancel_button_held = false;
                    if (s_cancel_timer != NULL) {
                        xTimerStop(s_cancel_timer, 0);
                    }
                }
            } else if (gpio_num == BTN_SOS_PIN) {
                if (gpio_get_level(BTN_SOS_PIN) == 0) {
                    ESP_LOGI(TAG, "Button SOS pressed (GPIO %lu) - hold 3s to trigger", gpio_num);
                    s_sos_button_held = true;
                    if (s_sos_timer == NULL) {
                        s_sos_timer = xTimerCreate("sos_timer",
                                                   pdMS_TO_TICKS(SOS_HOLD_TIME_MS),
                                                   pdFALSE, NULL, sos_timer_callback);
                    }
                    if (s_sos_timer != NULL) {
                        xTimerReset(s_sos_timer, 0);
                    }
                } else {
                    ESP_LOGI(TAG, "Button SOS released (GPIO %lu)", gpio_num);
                    s_sos_button_held = false;
                    if (s_sos_timer != NULL) {
                        xTimerStop(s_sos_timer, 0);
                    }
                }
            }
        }
    }
}

// Gộp cấu hình GPIO + I2C vào một hàm cho gọn main() và dễ quản lý thứ tự init.
void gpio_conf(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << BUZ_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);

    // Nút CANCEL: ngắt cả 2 cạnh để phát hiện press (xuống) lẫn release (lên)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << BTN_CANCEL_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << BTN_SOS_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Queue giao tiếp an toàn giữa ISR và task FreeRTOS
    gpio_queue = xQueueCreate(GPIO_QUEUE_SIZE, sizeof(uint32_t));

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_CANCEL_PIN, gpio_isr_handler, (void *)BTN_CANCEL_PIN);
    gpio_isr_handler_add(BTN_SOS_PIN, gpio_isr_handler, (void *)BTN_SOS_PIN);

    xTaskCreate(btn_task, "btn_task", BTN_TASK_STACK, NULL, BTN_TASK_PRIORITY, NULL);

    // ========== CẤU HÌNH I2C ==========
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to configure I2C: %s", esp_err_to_name(ret));
        start_error_alert();
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to install I2C driver: %s", esp_err_to_name(ret));
        start_error_alert();
        return;
    }
    ESP_LOGI("I2C", "I2C initialized");
}

// =========================== TASK MPU6050 ===================================
// Task chuyên dụng: đọc MPU6050 ở 100Hz, xử lý dữ liệu, cập nhật fall detection.
// Các bước trong vòng lặp: raw → convert → total vector → roll/pitch →
// double-buffer → fall detection → LED indicator.
// ============================================================================

void mpu6050_task(void *param) {
    (void)param;
    esp_err_t ret;
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float gyro_bias[3] = {0};

    ret = mpu6050_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to initialize: %s", esp_err_to_name(ret));
        start_error_alert();
        vTaskDelete(NULL);
        return;
    }

    // ========== HIỆU CHUẨN 5 GIÂY ==========
    // Gyroscope lý tưởng: đứng yên → 0 deg/s. Thực tế có sai số chế tạo và
    // drift nhiệt → cần đo bias để loại trừ. Giữ thiết bị yên, lấy 500 mẫu.
    ESP_LOGI("CALIB", "Calibrating... Keep device steady!");
    ESP_LOGI("CALIB", "LED will blink during 5s calibration...");

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        gpio_set_level(LED_PIN, (i % 100 < 50) ? 1 : 0);
        mpu6050_calibrate_sample(I2C_MASTER_NUM);
        vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_RATE_MS));
    }
    mpu6050_calibrate_finish(NULL, gyro_bias);

    ESP_LOGI("CALIB", "Calibration done: gyro bias [%.2f, %.2f, %.2f] deg/s",
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    ESP_LOGI("CALIB", "Calibration complete!");
    gpio_set_level(BUZ_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(CALIB_BUZZER_DURATION_MS));
    gpio_set_level(BUZ_PIN, 0);
    gpio_set_level(LED_PIN, 0);

    // Complementary filter: kết hợp accelerometer (chính xác tĩnh) và gyroscope
    // (chính xác khi động) với trọng số ~0.98 gyro + 0.02 accel.
    roll_pitch_init();
    ESP_LOGI("RollPitch", "Initialized");

    // ========== VÒNG LẶP CHÍNH 100Hz ==========
    while (1) {
        ret = mpu6050_read_raw_data(I2C_MASTER_NUM, &raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        if (ret != ESP_OK) {
            ESP_LOGE("MPU6050", "Read failed!");
            start_error_alert();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        mpu6050_convert_accel(raw_ax, raw_ay, raw_az, &accel_x, &accel_y, &accel_z);
        mpu6050_convert_gyro(raw_gx, raw_gy, raw_gz, &gyro_x, &gyro_y, &gyro_z);

        float total_accel_g;
        mpu6050_get_total_accel(accel_x, accel_y, accel_z, &total_accel_g);
        float total_gyro = mpu6050_get_total_gyro(gyro_x, gyro_y, gyro_z);

        roll_pitch_update(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        float roll = get_roll();
        float pitch = get_pitch();

        // Double-buffer: ghi vào buffer writer, đảo index
        g_sensor_buffers[s_writer_index].total_accel_g = total_accel_g;
        g_sensor_buffers[s_writer_index].total_gyro = total_gyro;
        g_sensor_buffers[s_writer_index].roll = roll;
        g_sensor_buffers[s_writer_index].pitch = pitch;
        g_sensor_buffers[s_writer_index].data_ready = true;

        s_writer_index = 1 - s_writer_index;

        // I2C phục hồi → tắt báo lỗi
        if (s_error_state) {
            stop_error_alert();
        }

        fall_detection_update(total_accel_g, total_gyro, pitch, roll);

        // LED indicator cho trạng thái WAIT_LIE_DOWN (state 3): đã impact,
        // chờ xác nhận nằm → LED nhấp nháy 50% duty cycle 100ms
        static bool was_wait_lie = false;
        bool is_wait_lie = (fall_detection_get_state_internal() == 3);
        if (is_wait_lie) {
            static uint32_t blk = 0;
            blk = (blk + 1) % 10;
            gpio_set_level(LED_PIN, blk < 5);
        } else if (was_wait_lie) {
            gpio_set_level(LED_PIN, 0);
        }
        was_wait_lie = is_wait_lie;

        vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_RATE_MS));
    }
}

// =========================== CALLBACK PHÁT HIỆN NGÃ =========================
// Được fall_detection gọi (từ mpu6050_task) khi phát hiện té ngã.
// Đã đăng ký qua fall_detection_set_callback() trong app_main().
// ============================================================================

static void fall_alert_callback(void)
{
    telegram_send_fall_alert();
    start_fall_alert();
}

// =========================== MAIN ===========================================
// app_main() — điểm vào ESP-IDF, chạy sau khi FreeRTOS khởi động.
// Thứ tự init: NVS → GPIO+I2C → Telegram → FallDetection → WiFi → MPU6050
// task → chờ WiFi → main loop.
// ============================================================================
void app_main(void) {
    // NVS lưu cấu hình WiFi không bay hơi, cần init trước wifi_init_sta()
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "======== SYSTEM START ========");
    gpio_conf();

    telegram_init(CONFIG_TELEGRAM_BOT_TOKEN, CONFIG_TELEGRAM_CHAT_ID);

    fall_detection_init();
    fall_detection_set_callback(fall_alert_callback);

    wifi_init_sta();

    xTaskCreate(mpu6050_task, "mpu6050_task", MPU6050_TASK_STACK, NULL, MPU6050_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "App initialization complete - waiting for WiFi...");

    // Chờ WiFi tối đa 30 giây, mỗi giây kiểm tra một lần
    bool wifi_connected = false;
    for (int i = 0; i < WIFI_CONNECT_TIMEOUT_S; i++) {
        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));
        if (wifi_is_connected()) {
            ESP_LOGI(TAG, "WiFi connected, starting webserver...");
            webserver_start();
            telegram_send_startup();
            wifi_connected = true;
            break;
        }
    }

    if (!wifi_connected) {
        ESP_LOGE(TAG, "WiFi connection failed!");
        start_error_alert();
        // Hệ thống vẫn chạy (cảm biến, nút bấm) — chỉ mất Telegram + web
    }

    // ========== MAIN LOOP ==========
    // 1) Gửi Telegram pending (SOS / hủy báo) — không gọi HTTP từ timer callback
    // 2) Kiểm tra WiFi
    // 3) Log trạng thái
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));

        if (s_sos_telegram_pending) {
            s_sos_telegram_pending = false;
            telegram_send_sos_alert();
        }
        if (s_cancel_telegram_pending) {
            s_cancel_telegram_pending = false;
            telegram_send_cancel_alert();
        }

        if (!wifi_is_connected() && !s_alert_active) {
            ESP_LOGE(TAG, "WiFi disconnected!");
            start_error_alert();
        } else if (wifi_is_connected() && s_error_state) {
            stop_error_alert();
        }

        ESP_LOGI(TAG, "System running - WiFi: %s | Alert: %s | Error: %s",
                 wifi_is_connected() ? "OK" : "NOT CONNECTED",
                 s_alert_active ? "ACTIVE" : "IDLE",
                 s_error_state ? "YES" : "NO");
    }
}
