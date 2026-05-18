/**
 * @file wifi.c
 * @brief Module quản lý kết nối WiFi (Station mode) và đồng bộ thời gian NTP
 *
 * ===================== CHỨC NĂNG CHÍNH =====================
 * 1. Kết nối WiFi ở chế độ Station (client) - ESP32 kết nối vào router
 * 2. Tự động reconnect khi mất kết nối (timer retry 10s, vô hạn)
 * 3. Đồng bộ thời gian qua NTP (Network Time Protocol)
 * 4. Cài đặt múi giờ ICT (Indochina Time, UTC+7 - Việt Nam)
 *
 * ===================== KIẾN TRÚC SỰ KIỆN (EVENT-DRIVEN) =====================
 * Hệ thống dùng event loop của ESP-IDF. Các sự kiện WiFi được đăng ký
 * trong wifi_init_sta() và xử lý trong wifi_event_handler():
 *
 *   WIFI_EVENT_STA_START       -> Gọi esp_wifi_connect() lần đầu
 *   WIFI_EVENT_STA_CONNECTED    -> Đánh dấu đã kết nối, dừng retry timer
 *   WIFI_EVENT_STA_DISCONNECTED -> Đánh dấu mất kết nối, start retry timer
 *   IP_EVENT_STA_GOT_IP        -> Có IP -> khởi tạo SNTP + đồng bộ thời gian
 *
 * ===================== CƠ CHẾ RETRY =====================
 * - Khi mất kết nối: timer one-shot 10s được tạo
 * - Timer fire: gọi esp_wifi_connect() thử lại
 * - Nếu thành công: timer bị hủy (xTimerStop)
 * - Nếu vẫn lỗi: timer fire lại (one-shot nên chỉ chạy 1 lần, nhưng
 *   event DISCONNECTED lại được gọi và tạo timer mới)
 * - Retry vô hạn (không giới hạn số lần thử)
 *
 * ===================== NTP TIME SYNC =====================
 * - Khi có IP (GOT_IP), khởi tạo SNTP client
 * - SNTP poll server pool.ntp.org để lấy thời gian
 * - Đồng bộ trong vòng 5 giây (10 lần thử, mỗi lần 500ms)
 * - Ngưỡng: thời gian > 01/01/2021 (1609451200) là hợp lệ
 * - Sau sync: in thời gian hiện tại (theo múi giờ ICT)
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

/* Tag cho ESP_LOG (hiển thị tiền tố [WIFI]) */
static const char *TAG = "WIFI";

/*
 * Biến đếm số lần retry hiện tại.
 * Reset về 0 khi kết nối thành công.
 * Dùng để log số lần thử, không giới hạn retry (reset khi đạt MAX_RETRY).
 */
static int s_retry_num = 0;

/* Cờ trạng thái kết nối WiFi. True = đã kết nối, False = mất kết nối. */
static bool s_wifi_connected = false;

/*
 * ===================== TIMER RETRY =====================
 *
 * Timer one-shot (pdFALSE): chỉ fire một lần sau mỗi lần start.
 * Khi mất kết nối (DISCONNECTED event), timer được start với thời gian
 * WIFI_RETRY_INTERVAL_MS (10 giây). Khi timer fire, gọi esp_wifi_connect().
 *
 * Tại sao dùng timer thay vì loop?
 *   - Tiết kiệm CPU: timer chỉ chạy khi cần retry
 *   - Không block event handler: chỉ start timer, không gọi connect trực tiếp
 *   - Dễ quản lý: stop khi kết nối thành công
 */
static TimerHandle_t s_retry_timer = NULL;

/*
 * ===================== CẤU HÌNH NTP =====================
 *
 * NTP_SERVER: Server NTP mặc định (pool.ntp.org - bộ cân tải NTP toàn cầu).
 * Có thể thay bằng server địa phương như "vn.pool.ntp.org" để nhanh hơn.
 *
 * TZ_STRING: Chuỗi định nghĩa múi giờ theo POSIX.
 *   "ICT-7": Indochina Time, UTC-7 (lưu ý: POSIX ngược dấu với UTC offset).
 *   Thực tế UTC+7 = ICT (Việt Nam, Thái Lan, Campuchia, Lào).
 *   Có thể thay bằng "VNT-7" nhưng ICT là chuẩn hơn.
 */
static const char *NTP_SERVER = "pool.ntp.org";
static const char *TZ_STRING = "ICT-7";  /* Múi giờ Đông Dương (UTC+7) */

/*
 * set_timezone - Thiết lập múi giờ cho hệ thống.
 *
 * Sử dụng biến môi trường TZ (POSIX standard):
 *   setenv("TZ", "ICT-7", 1): đặt múi giờ
 *   tzset(): áp dụng thay đổi
 *
 * Lưu ý: POSIX TZ string dùng dấu ngược (UTC-7 = UTC+7 thực tế).
 * Gọi hàm này TRƯỚC khi sync NTP để localtime() trả về đúng giờ Việt Nam.
 */
static void set_timezone(void)
{
    setenv("TZ", TZ_STRING, 1);
    tzset();
    ESP_LOGI(TAG, "Timezone: %s", TZ_STRING);
}

/*
 * wifi_is_connected - Kiểm tra trạng thái kết nối WiFi.
 *
 * @return: true nếu đã kết nối đến AP, false nếu chưa hoặc mất kết nối.
 *
 * Được các module khác (MPU6050, Telegram) gọi để kiểm tra trước khi
 * thực hiện các tác vụ cần mạng.
 */
bool wifi_is_connected(void) {
    return s_wifi_connected;
}

/* ===================== ĐỒNG BỘ THỜI GIAN NTP ===================== */

/*
 * initialize_sntp - Khởi tạo SNTP client.
 *
 * Cấu hình:
 *   - SNTP_OPMODE_POLL: chế độ polling (hỏi server định kỳ)
 *   - Server: pool.ntp.org (có thể thay đổi biến NTP_SERVER)
 *   - sntp_init(): bắt quá trình đồng bộ (chạy ngầm)
 *
 * Gọi sau khi có IP (IP_EVENT_STA_GOT_IP).
 */
static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP...");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char *)NTP_SERVER);
    sntp_init();
}

/*
 * sync_time - Đợi đồng bộ thời gian từ NTP server.
 *
 * Quy trình:
 *   1. Chờ tối đa 5 giây (10 lần * 500ms)
 *   2. Mỗi lần: gọi time(&now) kiểm tra thời gian hiện tại
 *   3. Nếu now > 1609451200 (01/01/2021) -> coi như đã sync thành công
 *   4. In thời gian hiện tại theo định dạng YYYY-MM-DD HH:MM:SS
 *
 * Ngưỡng 1609451200: timestamp ngày 01/01/2021 00:00:00 UTC.
 * Nếu now > ngưỡng này -> RTC đã được cập nhật. Nếu chưa -> RTC vẫn là
 * thời gian mặc định của ESP32 (thường là 1970-01-01 hoặc 2016-01-01).
 */
static void sync_time(void)
{
    ESP_LOGI(TAG, "Syncing time from NTP...");
    time_t now = 0;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        time(&now);
        if (now > 1609451200) break;  /* Kiểm tra thời gian > 01/01/2021 */
    }
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);     /* Chuyển đổi sang giờ địa phương (ICT) */
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    ESP_LOGI(TAG, "Current time: %s", strftime_buf);
}

/* ===================== TIMER RETRY CALLBACK ===================== */

/*
 * wifi_retry_timer_callback - Callback khi timer retry fire.
 *
 * Được gọi từ FreeRTOS timer (trong context của timer daemon task).
 *
 * Hành vi:
 *   1. Kiểm tra nếu vẫn chưa kết nối (s_wifi_connected == false)
 *   2. Nếu s_retry_num >= WIFI_MAX_RETRY: reset biến đếm (để retry vô hạn)
 *   3. Gọi esp_wifi_connect() để thử kết nối lại
 *   4. Tăng biến đếm s_retry_num
 *
 * Timer là one-shot, nên sau khi fire, nếu mất kết nối lần nữa,
 * DISCONNECTED event sẽ được gọi và tạo timer mới.
 *
 * Retry vô hạn: vì thiết bị IoT cần luôn cố gắng kết nối lại,
 * không bỏ cuộc sau N lần thử.
 */
static void wifi_retry_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (!s_wifi_connected) {
        if (s_retry_num >= WIFI_MAX_RETRY) {
            s_retry_num = 0;  /* Reset để retry vô hạn (vòng lặp mới) */
        }
        ESP_LOGI(TAG, "WiFi retry, attempting reconnect...");
        esp_wifi_connect();
        s_retry_num++;
    }
}

/* ===================== TRÌNH XỬ LÝ SỰ KIỆN WIFI ===================== */

/*
 * wifi_event_handler - Xử lý tất cả sự kiện WiFi và IP.
 *
 * Hệ thống sự kiện của ESP-IDF: event_base phân loại nhóm sự kiện
 * (WIFI_EVENT, IP_EVENT), event_id là sự kiện cụ thể.
 *
 * Các sự kiện được xử lý:
 * ======================== NHÓM WIFI_EVENT ========================
 *
 * WIFI_EVENT_STA_START:
 *   - WiFi Station đã khởi động (sau esp_wifi_start())
 *   - Hành vi: gọi esp_wifi_connect() để bắt đầu kết nối đến AP
 *   - Lưu ý: event này chỉ gọi MỘT LẦN duy nhất sau wifi_init_sta()
 *
 * WIFI_EVENT_STA_CONNECTED:
 *   - Kết nối vật lý đến AP thành công (đã có link layer)
 *   - Hành vi: đánh dấu s_wifi_connected = true, dừng retry timer
 *   - Lưu ý: chưa có IP (DHCP đang xử lý), chưa thể truy cập mạng
 *   - GOT_IP event sẽ đến sau khi DHCP hoàn tất
 *
 * WIFI_EVENT_STA_DISCONNECTED:
 *   - Mất kết nối với AP (do nhiều nguyên nhân: mất sóng, AP tắt, ...)
 *   - event_data chứa reason code (lý do mất kết nối)
 *   - Hành vi:
 *     a. Đánh dấu s_wifi_connected = false
 *     b. Dừng timer cũ (nếu có)
 *     c. Tạo timer mới (one-shot, 10s) để thử kết nối lại
 *     d. Khi timer fire -> wifi_retry_timer_callback -> esp_wifi_connect()
 *
 * ======================== NHÓM IP_EVENT ========================
 *
 * IP_EVENT_STA_GOT_IP:
 *   - ESP32 đã nhận được địa chỉ IP từ DHCP server
 *   - Đây là thời điểm có thể truy cập mạng (Internet)
 *   - Hành vi:
 *     a. In địa chỉ IP ra log
 *     b. Reset biến đếm retry
 *     c. Khởi tạo SNTP (initialize_sntp)
 *     d. Đợi 2 giây cho SNTP poll lần đầu
 *     e. Đồng bộ thời gian (sync_time)
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {

    /* ----- WIFI_EVENT_STA_START: WiFi Station đã khởi động ----- */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA started");
        esp_wifi_connect();  /* Bắt đầu quá trình kết nối */

    /* ----- WIFI_EVENT_STA_DISCONNECTED: Mất kết nối ----- */
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc_event = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi disconnected: reason=%d", disc_event->reason);
        s_wifi_connected = false;

        /* Dừng timer cũ nếu đang chạy */
        if (s_retry_timer != NULL) {
            xTimerStop(s_retry_timer, 0);
        }

        /*
         * Tạo timer retry one-shot (pdFALSE) với thời gian WIFI_RETRY_INTERVAL_MS.
         * Timer này sẽ gọi wifi_retry_timer_callback sau 10 giây.
         * Nếu timer đã tồn tại (từ lần mất kết nối trước), không tạo lại.
         */
        if (s_retry_timer == NULL) {
            s_retry_timer = xTimerCreate("wifi_retry",
                                         pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS),
                                         pdFALSE,  /* One-shot: chỉ fire 1 lần */
                                         NULL,
                                         wifi_retry_timer_callback);
        }
        if (s_retry_timer != NULL) {
            xTimerStart(s_retry_timer, 0);  /* Bắt đầu đếm thời gian */
        }

    /* ----- WIFI_EVENT_STA_CONNECTED: Kết nối vật lý thành công ----- */
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected to AP");
        s_wifi_connected = true;
        s_retry_num = 0;
        /* Hủy timer retry (không cần retry nữa) */
        if (s_retry_timer != NULL) {
            xTimerStop(s_retry_timer, 0);
        }

    /* ----- IP_EVENT_STA_GOT_IP: Đã có địa chỉ IP ----- */
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi CONNECTED - IP: " IPSTR, IP2STR(&ip_event->ip_info.ip));
        s_retry_num = 0;
        /* Khởi tạo SNTP để đồng bộ thời gian */
        initialize_sntp();
        /*
         * Đợi 2 giây cho SNTP có thời gian poll server lần đầu.
         * sync_time() sẽ kiểm tra và đợi thêm nếu cần.
         */
        vTaskDelay(pdMS_TO_TICKS(2000));
        sync_time();
    }
}

/* ===================== KHỞI TẠO WIFI ===================== */

/*
 * wifi_init_sta - Khởi tạo WiFi ở chế độ Station.
 *
 * Quy trình khởi tạo (theo thứ tự bắt buộc của ESP-IDF WiFi stack):
 *
 *   1. esp_netif_init()              - Khởi tạo TCP/IP network interface
 *   2. esp_event_loop_create_default() - Tạo event loop mặc định
 *   3. esp_netif_create_default_wifi_sta() - Tạo netif cho WiFi Station
 *   4. set_timezone()                - Đặt múi giờ ICT (trước NTP)
 *   5. esp_wifi_init(&cfg)           - Khởi tạo WiFi driver với cấu hình mặc định
 *   6. Đăng ký event handlers        - WIFI_EVENT và IP_EVENT
 *   7. Cấu hình WiFi (SSID, PASS, auth mode)
 *   8. esp_wifi_set_mode(WIFI_MODE_STA) - Đặt chế độ Station
 *   9. esp_wifi_set_config()         - Áp dụng cấu hình SSID/PASS
 *  10. esp_wifi_start()              - Khởi động WiFi
 *  11. esp_wifi_set_max_tx_power()   - Đặt công suất phát tối đa
 *
 * Sau khi esp_wifi_start(), event WIFI_EVENT_STA_START sẽ được gọi,
 * và trong event handler đó, esp_wifi_connect() được gọi để kết nối.
 *
 * Lưu ý:
 *   - SSID và PASS được lấy từ #define trong wifi.h
 *   - Nên chuyển SSID/PASS vào sdkconfig hoặc NVS sau này
 *   - Công suất phát MAX_TX_POWER = 40 dBm (tối đa ESP32 là 20 dBm,
 *     40 dBm sẽ được ESP32 tự động giới hạn về mức tối đa cho phép)
 */
void wifi_init_sta(void) {
    /* Bước 1-3: Khởi tạo network interface và event loop */
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    /* Bước 4: Đặt múi giờ Việt Nam (ICT) trước khi có mạng */
    set_timezone();

    /* Bước 5: Khởi tạo WiFi driver với cấu hình mặc định */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    /* Bước 6: Đăng ký event handlers */
    esp_event_handler_instance_t handler_instance;
    /*
     * Đăng ký toàn bộ sự kiện WiFi (từ STA_START đến STA_DISCONNECTED)
     * dùng ESP_EVENT_ANY_ID để bắt tất cả event_id của WIFI_EVENT.
     */
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         &wifi_event_handler, NULL, &handler_instance);
    /*
     * Đăng ký riêng sự kiện GOT_IP từ IP_EVENT.
     * Lưu ý: dùng cùng handler nhưng handler_instance sẽ bị ghi đè.
     * Trong thực tế, nếu cần phân biệt, nên dùng 2 instance khác nhau.
     */
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                         &wifi_event_handler, NULL, &handler_instance);

    /* Bước 7: Cấu hình WiFi */
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,  /* Chỉ kết nối AP có WPA2 */
        },
    };

    /* Xóa bộ đệm SSID và Password trước khi copy (an toàn hơn) */
    memset(wifi_config.sta.ssid, 0, sizeof(wifi_config.sta.ssid));
    memset(wifi_config.sta.password, 0, sizeof(wifi_config.sta.password));

    /* Copy SSID và Password từ #define */
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);

    ESP_LOGI(TAG, "WiFi SSID: [%s]", (char*)wifi_config.sta.ssid);

    /* Bước 8-11: Áp dụng cấu hình và khởi động WiFi */
    esp_wifi_set_mode(WIFI_MODE_STA);       /* Chế độ Station (client) */
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); /* Áp dụng SSID/PASS */
    esp_wifi_start();                       /* Khởi động WiFi */
    esp_wifi_set_max_tx_power(MAX_TX_POWER); /* Đặt công suất phát */

    ESP_LOGI(TAG, "WiFi initialized");
}
