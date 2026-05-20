/*
 * wifi.c - Kết nối WiFi (Station) và đồng bộ NTP
 *
 * Module này cho ESP32 đóng vai trò WiFi client (Station) — kết nối vào
 * router như điện thoại/laptop. Sau khi có IP, tự động đồng bộ thời gian
 * qua NTP để các module khác (cảm biến, Telegram) có timestamp giờ Việt Nam.
 *
 * ===================== KIẾN TRÚC EVENT-DRIVEN =====================
 * ESP-IDF quản lý WiFi qua event loop. Khi stack có chuyển biến (khởi động
 * xong, kết nối/mất kết nối, có IP,...), nó phát sự kiện. Một handler duy
 * nhất (wifi_event_handler) phản ứng với từng loại.
 *
 * Chọn event-driven thay vì polling vì:
 *   - CPU chỉ hoạt động khi có sự kiện, không tốn vòng lặp vô ích
 *   - Phản ứng tức thời, không cần delay/poll định kỳ
 *   - Event loop chạy task riêng, không ảnh hưởng task khác
 *
 * Chuỗi sự kiện boot điển hình:
 *   wifi_init_sta() → esp_wifi_start()
 *     → WIFI_EVENT_STA_START      → esp_wifi_connect()
 *     → WIFI_EVENT_STA_CONNECTED  → set connected flag, stop retry timer
 *     → IP_EVENT_STA_GOT_IP       → start SNTP, sync time
 *
 * ===================== CƠ CHẾ RETRY =====================
 * IoT device KHÔNG ĐƯỢC PHÉP BỎ CUỘC. Nếu mất kết nối, module retry
 * vô hạn định, mỗi lần cách nhau 10 giây (WIFI_RETRY_INTERVAL_MS).
 *
 * Thay vì gọi esp_wifi_connect() ngay trong event handler (có thể spam),
 * dùng FreeRTOS timer one-shot: mất kết nối → start timer 10s → timer fire
 * → gọi connect. Timer là one-shot nên chỉ fire 1 lần, nhưng event
 * DISCONNECTED sẽ được gọi lại và tạo timer mới nếu vẫn lỗi.
 *
 * Tại sao timer?
 *   - Event handler phải trả về nhanh, không gọi connect trực tiếp
 *   - Giãn cách retry, không spam AP khi sóng yếu
 *   - Dễ stop/hủy khi kết nối thành công
 *
 * ===================== NTP TIME SYNC =====================
 * Khi có IP, khởi tạo SNTP client polling pool.ntp.org.
 * Chờ tối đa 5 giây (10 lần × 500ms) để timestamp > 01/01/2021
 * (ngưỡng này phân biệt RTC đã sync với giá trị mặc định từ boot).
 * Nếu quá 5 giây chưa sync được, bỏ qua — thiết bị vẫn chạy, chỉ thiếu
 * timestamp chính xác. localtime() trả về giờ ICT nhờ set_timezone().
 */

#include <string.h>
#include "esp_wifi.h"         /* WiFi API: init, connect, disconnect, config */
#include "esp_event.h"        /* Event loop: đăng ký và xử lý sự kiện */
#include "esp_log.h"          /* Log system */
#include "esp_netif.h"        /* Network interface: tạo và quản lý netif */
#include "wifi.h"             /* Header của module (chứa cấu hình WiFi) */
#include "freertos/timers.h"  /* FreeRTOS timer cho retry */
#include "sys/time.h"         /* time(), struct timeval, timezone */
#include "lwip/apps/sntp.h"   /* SNTP (Simple NTP) client */

static const char *TAG = "WIFI";

/* Đếm retry: dùng để log. Reset mỗi khi connected. Khi >= MAX_RETRY thì reset về 0 (vòng lặp mới) */
static int s_retry_num = 0;

/* true khi có link-layer với AP. Chưa chắc có IP (DHCP chưa xong) */
static bool s_wifi_connected = false;

/*
 * Timer one-shot (pdFALSE) cho retry.
 * Không gọi esp_wifi_connect() trực tiếp trong event handler vì:
 *   - Event handler phải trả về nhanh
 *   - Cần giãn cách retry tránh spam AP
 *   - Dễ dừng khi connected
 */
static TimerHandle_t s_retry_timer = NULL;

/* NTP_SERVER: pool.ntp.org (cân tải toàn cầu). Có thể đổi thành vn.pool.ntp.org */
static const char *NTP_SERVER = "pool.ntp.org";
/* Múi giờ POSIX: "ICT-7" là UTC+7 (POSIX ngược dấu). Cho localtime() trả về giờ VN */
static const char *TZ_STRING = "ICT-7";

/*
 * Đặt biến môi trường TZ để localtime() trả về giờ Việt Nam (UTC+7).
 * POSIX TZ string ngược dấu: "ICT-7" nghĩa là UTC+7.
 * Gọi TRƯỚC sync NTP.
 */
static void set_timezone(void)
{
    setenv("TZ", TZ_STRING, 1);
    tzset();
    ESP_LOGI(TAG, "Timezone: %s", TZ_STRING);
}

/*
 * Các module khác (telegram, webserver) gọi hàm này để kiểm tra
 * có nên gửi request ra mạng không.
 */
bool wifi_is_connected(void) {
    return s_wifi_connected;
}

/* ===================== NTP ===================== */

/* Khởi tạo SNTP client ở chế độ poll, dùng server pool.ntp.org.
 * Gọi sau GOT_IP — SNTP chạy ngầm, sync_time() đợi kết quả. */
static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP...");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char *)NTP_SERVER);
    sntp_init();
}

/*
 * Chờ SNTP sync: poll tối đa 5 giây (10×500ms) cho đến khi
 * time() > 01/01/2021 (ngưỡng phân biệt RTC đã sync với giá trị mặc định
 * từ boot — ESP32 thường boot với năm 1970 hoặc 2016).
 * Nếu quá 5 giây chưa sync được thì bỏ qua, không block nữa.
 */
static void sync_time(void)
{
    ESP_LOGI(TAG, "Syncing time from NTP...");
    time_t now = 0;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        time(&now);
        if (now > 1609451200) break;
    }
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    ESP_LOGI(TAG, "Current time: %s", strftime_buf);
}

/* ===================== TIMER RETRY CALLBACK ===================== */

/*
 * Timer one-shot fire → thử kết nối lại.
 * Retry vô hạn: IoT device không được phép bỏ cuộc — reset biến đếm
 * mỗi khi chạm WIFI_MAX_RETRY để tránh overflow.
 */
static void wifi_retry_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (!s_wifi_connected) {
        if (s_retry_num >= WIFI_MAX_RETRY) {
            s_retry_num = 0;
        }
        ESP_LOGI(TAG, "WiFi retry, attempting reconnect...");
        esp_wifi_connect();
        s_retry_num++;
    }
}

/* ===================== TRÌNH XỬ LÝ SỰ KIỆN WIFI ===================== */

/*
 * wifi_event_handler — Xử lý 4 sự kiện WiFi+IP theo mô hình event-driven.
 *
 * event_base phân biệt WIFI_EVENT vs IP_EVENT.
 * event_id cho biết sự kiện cụ thể.
 *
 * WIFI_EVENT_STA_START:
 *   WiFi driver đã start. Chỉ gọi 1 lần. esp_wifi_connect() kick off kết nối đầu.
 *
 * WIFI_EVENT_STA_DISCONNECTED:
 *   Mất kết nối (mất sóng, AP tắt, timeout...). reason code trong event_data.
 *   Set connected=false → dừng timer cũ → tạo timer one-shot 10s → retry sau.
 *
 * WIFI_EVENT_STA_CONNECTED:
 *   Link-layer với AP thành công. Chưa có IP (DHCP chưa xong).
 *   Set connected=true → stop retry timer → chờ GOT_IP.
 *
 * IP_EVENT_STA_GOT_IP:
 *   DHCP thành công, đã có IP. start SNTP + delay 2s + sync time.
 *   Từ đây có thể truy cập Internet.
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA started");
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc_event = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi disconnected: reason=%d", disc_event->reason);
        s_wifi_connected = false;

        if (s_retry_timer != NULL) {
            xTimerStop(s_retry_timer, 0);
        }

        if (s_retry_timer == NULL) {
            s_retry_timer = xTimerCreate("wifi_retry",
                                         pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS),
                                         pdFALSE,
                                         NULL,
                                         wifi_retry_timer_callback);
        }
        if (s_retry_timer != NULL) {
            xTimerStart(s_retry_timer, 0);
        }

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected to AP");
        s_wifi_connected = true;
        s_retry_num = 0;
        if (s_retry_timer != NULL) {
            xTimerStop(s_retry_timer, 0);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi CONNECTED - IP: " IPSTR, IP2STR(&ip_event->ip_info.ip));
        s_retry_num = 0;
        initialize_sntp();
        vTaskDelay(pdMS_TO_TICKS(2000));
        sync_time();
    }
}

/* ===================== KHỞI TẠO WIFI ===================== */

/*
 * wifi_init_sta — Khởi tạo WiFi Station.
 *
 * Trình tự bắt buộc của ESP-IDF:
 *   1-3: netif + event loop + WiFi netif
 *   4:   set timezone (trước NTP)
 *   5:   WiFi driver init
 *   6:   register event handlers (WIFI_EVENT + IP_EVENT)
 *   7:   config SSID/PASS
 *   8-11: set mode, set config, start, set TX power
 *
 * Sau esp_wifi_start(), event WIFI_EVENT_STA_START → esp_wifi_connect()
 * được gọi tự động trong event handler.
 */
void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    set_timezone();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t handler_instance;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         &wifi_event_handler, NULL, &handler_instance);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                         &wifi_event_handler, NULL, &handler_instance);

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
