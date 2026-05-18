/**
 * @file CODE.c
 * @brief Chương trình chính - Fall Detection Device
 *
 * Tổ chức:
 * 1. GPIO + I2C initialization
 * 2. MPU6050 task (100Hz): đọc cảm biến, phát hiện ngã
 * 3. Button task: xử lý nút bấm CANCEL/SOS
 * 4. Webserver: dashboard + API
 * 5. Telegram: gửi thông báo
 *
 * Luồng dữ liệu:
 * MPU6050 → mpu6050_task → fall_detection_update() → [FALL] → alert_callback()
 *                                                              ├──→ telegram_send_fall_alert()
 *                                                              └──→ start_fall_alert() → buzzer + LED
 */

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
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

// ========== CẤU HÌNH I2C ==========
#define I2C_MASTER_SCL_IO       9
#define I2C_MASTER_SDA_IO       8
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000

// ========== CẤU HÌNH GPIO ==========
#define BUZ_PIN         0       // Buzzer (output)
#define LED_PIN         1       // LED (output)
#define BTN_CANCEL_PIN  5       // Nút hủy báo động (input, pull-up)
#define BTN_SOS_PIN     6       // Nút SOS giữ 3s (input, pull-up)

// ========== CẤU HÌNH THỜI GIAN ==========
#define ALERT_DURATION_MS  20000   // Báo động tự dừng sau 20s
#define LED_BLINK_PERIOD    1000    // LED blink 1s
#define LED_ERROR_BLINK_PERIOD 200   // LED blink nhanh khi lỗi
#define SOS_HOLD_TIME_MS    3000    // Giữ nút 3s để trigger SOS
#define DEBOUNCE_TIME_MS    50      // Debounce 50ms

// ========== SYSTEM CONSTANTS ==========
#define MPU6050_TASK_STACK      4096    // Stack size bytes
#define MPU6050_TASK_PRIORITY  5       // Priority
#define MPU6050_SAMPLE_RATE_MS  10      // 100Hz = 10ms
#define BTN_TASK_STACK          2048    // Stack size bytes
#define BTN_TASK_PRIORITY      3       // Priority
#define GPIO_QUEUE_SIZE         20      // GPIO event queue
#define WIFI_CONNECT_TIMEOUT_S  30      // WiFi timeout seconds
#define CALIBRATION_SAMPLES     500     // Calibration samples
#define CALIB_BUZZER_DURATION_MS 1000  // Buzzer beep duration
#define MAIN_LOOP_DELAY_MS      1000    // Main loop delay

// ========== TELEGRAM (SECURITY: nên chuyển sang sdkconfig) ==========
#define TELEGRAM_BOT_TOKEN  "8659816659:AAEFwAc-LdtDNuVEGbUHt_cpOwP_ilWfSjA"
#define TELEGRAM_CHAT_ID    "-5239342658"

static const char *TAG = "MAIN";

// GPIO event queue
static QueueHandle_t gpio_queue = NULL;

// ========== TRẠNG THÁI BÁO ĐỘNG ==========
typedef enum {
    ALERT_STATE_IDLE,
    ALERT_STATE_FALLING,     // Đang báo động: LED blink, buzzer on
    ALERT_STATE_ERROR        // Lỗi: LED blink nhanh (I2C/WiFi)
} alert_state_t;

static alert_state_t s_alert_state = ALERT_STATE_IDLE;
static TimerHandle_t s_alert_timer = NULL;   // Timer tắt tự động
static TimerHandle_t s_led_timer = NULL;      // Timer LED blink
static TimerHandle_t s_sos_timer = NULL;      // Timer giữ nút SOS
static bool s_led_state = false;
static bool s_alert_active = false;
static bool s_error_state = false;
static bool s_sos_button_held = false;        // SOS button đang được giữ
static bool s_sos_telegram_pending = false;   // Flag gửi Telegram từ main loop
static uint32_t s_last_cancel_isr = 0;        // Debounce cho CANCEL button
static uint32_t s_last_sos_isr = 0;          // Debounce cho SOS button

// ========== DOUBLE-BUFFER SENSOR DATA ==========
// Buffer 0 và Buffer 1: writer交替写入, reader đọc buffer còn lại
sensor_data_t g_sensor_buffers[2] = {
    {.total_accel = 0, .total_accel_g = 0, .total_gyro = 0, .roll = 0, .pitch = 0, .data_ready = false},
    {.total_accel = 0, .total_accel_g = 0, .total_gyro = 0, .roll = 0, .pitch = 0, .data_ready = false}
};
static uint8_t s_writer_index = 0;

// Lấy writer index để webserver tính reader index
uint8_t g_get_writer_index(void) {
    return s_writer_index;
}

// ========== ALERT FUNCTIONS ==========
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

// Timer callback: hết thời gian báo động → tắt
static void alert_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    ESP_LOGI(TAG, "Alert duration ended - auto stop");
    stop_fall_alert();
}

// Bắt đầu báo động (buzzer + LED blink)
static void start_fall_alert(void)
{
    if (s_alert_active) return;

    s_alert_active = true;
    s_alert_state = ALERT_STATE_FALLING;

    gpio_set_level(BUZ_PIN, 1);  // Buzzer on

    // LED blink timer (1s period)
    if (s_led_timer == NULL) {
        s_led_timer = xTimerCreate("led_blink", pdMS_TO_TICKS(LED_BLINK_PERIOD),
                                    pdTRUE, NULL, (TimerCallbackFunction_t)alert_led_toggle);
    }
    if (s_led_timer) {
        xTimerStart(s_led_timer, 0);
    }

    // Auto-stop timer (20s)
    if (s_alert_timer == NULL) {
        s_alert_timer = xTimerCreate("alert_timeout", pdMS_TO_TICKS(ALERT_DURATION_MS),
                                      pdFALSE, NULL, alert_timer_callback);
    }
    if (s_alert_timer) {
        xTimerStart(s_alert_timer, 0);
    }

    ESP_LOGW(TAG, ">>> FALL ALERT STARTED <<<");
}

// Dừng báo động
static void stop_fall_alert(void)
{
    s_alert_active = false;
    s_alert_state = ALERT_STATE_IDLE;

    gpio_set_level(BUZ_PIN, 0);

    if (s_led_timer) xTimerStop(s_led_timer, 0);
    if (s_alert_timer) xTimerStop(s_alert_timer, 0);

    alert_led_off();

    // Reset state machine để sẵn sàng phát hiện lần ngã tiếp theo
    fall_detection_reset();

    ESP_LOGI(TAG, "Fall alert stopped - system reset to IDLE");
}

// Báo động lỗi (LED blink nhanh)
static void start_error_alert(void)
{
    if (s_alert_active) return;

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

// ========== GPIO & ISR ==========

// ISR với debounce
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Debounce: bỏ qua nếu gọi trong 50ms
    uint32_t now = xTaskGetTickCountFromISR();
    uint32_t *last_time = (gpio_num == BTN_CANCEL_PIN) ? &s_last_cancel_isr : &s_last_sos_isr;

    if ((now - *last_time) * portTICK_PERIOD_MS < DEBOUNCE_TIME_MS) {
        return;
    }
    *last_time = now;

    xQueueSendFromISR(gpio_queue, &gpio_num, &xHigherPriorityTaskWoken);
}

// Timer callback cho SOS: firesau 3s giữ nút
static void sos_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (s_sos_button_held && !s_alert_active) {
        s_sos_telegram_pending = true;  // Flag để main loop gọi Telegram
        start_fall_alert();
        ESP_LOGW(TAG, ">>> SOS ALERT TRIGGERED (held 3s) <<<");
    }
    s_sos_button_held = false;
}

// Button handler task
static void btn_task(void *param)
{
    (void)param;
    uint32_t gpio_num;

    while (1) {
        if (xQueueReceive(gpio_queue, &gpio_num, portMAX_DELAY)) {
            if (gpio_num == BTN_CANCEL_PIN) {
                // Nút CANCEL: hủy báo động
                ESP_LOGI(TAG, "Button CANCEL pressed (GPIO %lu)", gpio_num);
                if (s_alert_active) {
                    telegram_send_cancel_alert();
                    stop_fall_alert();
                    fall_detection_reset();
                    ESP_LOGI(TAG, "False alarm cancelled");
                }
            } else if (gpio_num == BTN_SOS_PIN) {
                // Nút SOS: kiểm tra press/release
                if (gpio_get_level(BTN_SOS_PIN) == 0) {
                    // Press: bắt đầu đếm 3s
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
                    // Release: hủy timer
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

// Cấu hình GPIO
void gpio_conf(void) {
    // LED và Buzzer là output
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << BUZ_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);

    // Nút CANCEL: input với pull-up,触发 ở falling edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << BTN_CANCEL_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Nút SOS: input với pull-up, trigger ở any edge (phát hiện press/release)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << BTN_SOS_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Tạo queue cho GPIO events
    gpio_queue = xQueueCreate(GPIO_QUEUE_SIZE, sizeof(uint32_t));

    // Cài đặt ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_CANCEL_PIN, gpio_isr_handler, (void *)BTN_CANCEL_PIN);
    gpio_isr_handler_add(BTN_SOS_PIN, gpio_isr_handler, (void *)BTN_SOS_PIN);

    // Tạo button handler task
    xTaskCreate(btn_task, "btn_task", BTN_TASK_STACK, NULL, BTN_TASK_PRIORITY, NULL);

    // Cấu hình I2C
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

// ========== MPU6050 TASK ==========
// Task chạy ở 100Hz (mỗi 10ms)
void mpu6050_task(void *param) {
    (void)param;
    esp_err_t ret;
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float acc_bias[3] = {0}, gyro_bias[3] = {0};

    ret = mpu6050_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to initialize: %s", esp_err_to_name(ret));
        start_error_alert();
        vTaskDelete(NULL);
        return;
    }

    // ========== HIỆU CHUẨN 5 GIÂY ==========
    ESP_LOGI("CALIB", "Calibrating... Keep device steady!");
    ESP_LOGI("CALIB", "LED will blink during 5s calibration...");

    // LED blink trong khi calibration: 500 samples × 10ms = 5 giây
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        gpio_set_level(LED_PIN, (i % 100 < 50) ? 1 : 0);
        mpu6050_calibrate_sample(I2C_MASTER_NUM);
        vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_RATE_MS));
    }
    mpu6050_calibrate_finish(acc_bias, gyro_bias);

    ESP_LOGI("CALIB", "Calibration done: accel bias [%.2f, %.2f, %.2f], gyro bias [%.2f, %.2f, %.2f]",
             acc_bias[0], acc_bias[1], acc_bias[2],
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    // ========== BUZZER XÁC NHẬN ==========
    ESP_LOGI("CALIB", "Calibration complete!");
    gpio_set_level(BUZ_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(CALIB_BUZZER_DURATION_MS));
    gpio_set_level(BUZ_PIN, 0);
    gpio_set_level(LED_PIN, 0);

    roll_pitch_init();
    ESP_LOGI("RollPitch", "Initialized");

    // ========== VÒNG LẶP CHÍNH ==========
    while (1) {
        // Đọc dữ liệu từ MPU6050
        ret = mpu6050_read_raw_data(I2C_MASTER_NUM, &raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        if (ret != ESP_OK) {
            ESP_LOGE("MPU6050", "Read failed!");
            start_error_alert();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Chuyển đổi sang đơn vị vật lý
        mpu6050_convert_accel(raw_ax, raw_ay, raw_az, &accel_x, &accel_y, &accel_z);
        mpu6050_convert_gyro(raw_gx, raw_gy, raw_gz, &gyro_x, &gyro_y, &gyro_z);

        float total_accel_ms2, total_accel_g;
        total_accel_ms2 = mpu6050_get_total_accel(accel_x, accel_y, accel_z, &total_accel_g);
        float total_gyro = mpu6050_get_total_gyro(gyro_x, gyro_y, gyro_z);

        // Tính góc roll/pitch
        roll_pitch_update(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        float roll = get_roll();
        float pitch = get_pitch();

        // Ghi vào double-buffer
        g_sensor_buffers[s_writer_index].total_accel = total_accel_ms2;
        g_sensor_buffers[s_writer_index].total_accel_g = total_accel_g;
        g_sensor_buffers[s_writer_index].total_gyro = total_gyro;
        g_sensor_buffers[s_writer_index].roll = roll;
        g_sensor_buffers[s_writer_index].pitch = pitch;
        g_sensor_buffers[s_writer_index].data_ready = true;

        // Swap buffer (reader sẽ đọc buffer còn lại)
        s_writer_index = 1 - s_writer_index;

        // Xóa error state nếu I2C hoạt động
        if (s_error_state) {
            stop_error_alert();
        }

        // Cập nhật fall detection
        fall_detection_update(total_accel_g, total_gyro, pitch, roll);

        // LED blink 100ms ở trạng thái WAIT_LIE_DOWN
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

        vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_RATE_MS));  // 100Hz
    }
}

// ========== FALL ALERT CALLBACK ==========
// Được gọi khi fall detection phát hiện ngã
static void fall_alert_callback(void)
{
    telegram_send_fall_alert();
    start_fall_alert();
}

// ========== MAIN ==========
void app_main(void) {
    // Khởi tạo NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "======== SYSTEM START ========");
    gpio_conf();

    // Khởi tạo Telegram
    telegram_init(TELEGRAM_BOT_TOKEN, TELEGRAM_CHAT_ID);

    // Khởi tạo fall detection
    fall_detection_init();
    fall_detection_set_callback(fall_alert_callback);

    // Khởi tạo WiFi
    wifi_init_sta();

    // Tạo MPU6050 task
    xTaskCreate(mpu6050_task, "mpu6050_task", MPU6050_TASK_STACK, NULL, MPU6050_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "App initialization complete - waiting for WiFi...");

    // Đợi WiFi kết nối
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

    // Không kết nối được WiFi sau 30s
    if (!wifi_connected) {
        ESP_LOGE(TAG, "WiFi connection failed!");
        start_error_alert();
    }

    // ========== MAIN LOOP ==========
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));

        // Xử lý SOS Telegram từ main loop (không gọi trong timer callback)
        if (s_sos_telegram_pending) {
            s_sos_telegram_pending = false;
            telegram_send_sos_alert();
        }

        // Kiểm tra WiFi
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
