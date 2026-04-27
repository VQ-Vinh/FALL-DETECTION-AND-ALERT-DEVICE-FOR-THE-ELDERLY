/**
 * @file wifi.c
 * @brief Kết nối WiFi Station mode + NTP time sync
 *
 * Chức năng:
 * - Kết nối WiFi ở chế độ Station (client)
 * - Tự động reconnect mỗi 10s nếu mất kết nối
 * - Đồng bộ thời gian qua NTP (pool.ntp.org)
 * - Múi giờ: ICT (UTC+7, Việt Nam)
 */

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "wifi.h"
#include "freertos/timers.h"
#include "sys/time.h"
#include "lwip/apps/sntp.h"

static const char *TAG = "WIFI";
static int s_retry_num = 0;
static bool s_wifi_connected = false;

// Timer retry: khi mất kết nối, thử lại mỗi 10s (vô hạn)
static TimerHandle_t s_retry_timer = NULL;

// NTP Server
static const char *NTP_SERVER = "pool.ntp.org";
static const char *TZ_STRING = "ICT-7";  // Indochina Time (UTC+7)

// Thiết lập múi giờ
static void set_timezone(void)
{
    setenv("TZ", TZ_STRING, 1);
    tzset();
    ESP_LOGI(TAG, "Timezone: %s", TZ_STRING);
}

// Kiểm tra trạng thái WiFi
bool wifi_is_connected(void) {
    return s_wifi_connected;
}

// ========== NTP TIME SYNC ==========
static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP...");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char *)NTP_SERVER);
    sntp_init();
}

static void sync_time(void)
{
    ESP_LOGI(TAG, "Syncing time from NTP...");
    time_t now = 0;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        time(&now);
        if (now > 1609451200) break;  // > Jan 1, 2021
    }
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    ESP_LOGI(TAG, "Current time: %s", strftime_buf);
}

// ========== TIMER CALLBACK ==========
// Khi mất kết nối, timer fire → thử kết nối lại
static void wifi_retry_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (!s_wifi_connected) {
        if (s_retry_num >= WIFI_MAX_RETRY) {
            s_retry_num = 0;  // Reset để retry vô hạn
        }
        ESP_LOGI(TAG, "WiFi retry, attempting reconnect...");
        esp_wifi_connect();
        s_retry_num++;
    }
}

// ========== EVENT HANDLER ==========
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    // STA started → bắt đầu kết nối
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA started");
        esp_wifi_connect();

    // Mất kết nối → đánh dấu + start timer retry
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc_event = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi disconnected: reason=%d", disc_event->reason);
        s_wifi_connected = false;

        if (s_retry_timer != NULL) {
            xTimerStop(s_retry_timer, 0);
        }

        // Tạo timer retry 10s (one-shot)
        if (s_retry_timer == NULL) {
            s_retry_timer = xTimerCreate("wifi_retry",
                                         pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS),
                                         pdFALSE,  // One-shot
                                         NULL,
                                         wifi_retry_timer_callback);
        }
        if (s_retry_timer != NULL) {
            xTimerStart(s_retry_timer, 0);
        }

    // Kết nối thành công → dừng timer retry
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected to AP");
        s_wifi_connected = true;
        s_retry_num = 0;
        if (s_retry_timer != NULL) {
            xTimerStop(s_retry_timer, 0);
        }

    // Nhận IP → khởi tạo SNTP + sync time
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi CONNECTED - IP: " IPSTR, IP2STR(&ip_event->ip_info.ip));
        s_retry_num = 0;
        initialize_sntp();
        vTaskDelay(pdMS_TO_TICKS(2000));
        sync_time();
    }
}

// ========== KHỞI TẠO WIFI ==========
void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    set_timezone();  // Đặt múi giờ trước

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Đăng ký event handlers
    esp_event_handler_instance_t handler_instance;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         &wifi_event_handler, NULL, &handler_instance);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                         &wifi_event_handler, NULL, &handler_instance);

    // Cấu hình WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    memset(wifi_config.sta.ssid, 0, sizeof(wifi_config.sta.ssid));
    memset(wifi_config.sta.password, 0, sizeof(wifi_config.sta.password));

    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);

    ESP_LOGI(TAG, "WiFi SSID: [%s]", (char*)wifi_config.sta.ssid);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_set_max_tx_power(MAX_TX_POWER);

    ESP_LOGI(TAG, "WiFi initialized");
}
