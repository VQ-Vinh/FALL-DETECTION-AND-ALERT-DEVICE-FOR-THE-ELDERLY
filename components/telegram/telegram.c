/*
 * telegram.c - Gửi tin nhắn Telegram async qua FreeRTOS queue
 *
 * ===================== TẠI SAO CẦN ASYNC QUEUE? =====================
 *
 * Gửi tin nhắn Telegram là một tác vụ chậm: esp_http_client_perform()
 * có thể block vài giây chờ DNS, TCP handshake, SSL, HTTP response.
 * Nếu gọi trực tiếp từ mpu6050_task (100Hz) hay timer callback,
 * toàn bộ hệ thống sẽ bị trễ hoặc crash vì stack timer quá nhỏ.
 *
 * Giải pháp: queue-based async.
 *
 *   Caller (mpu6050_task / button ISR / ...)
 *     ↓ xQueueSend() — non-blocking, 0 timeout
 *   ┌───────────────────┐
 *   │   FreeRTOS Queue  │  (4 phần tử, mỗi phần tử là kiểu message)
 *   └───────────────────┘
 *     ↓ xQueueReceive() — blocking, chờ đến khi có message
 *   telegram_task (stack 4KB, priority 4)
 *     ↓ send_telegram_message() — HTTP request thực sự
 *   Telegram API
 *
 * Caller không bao giờ bị block — chỉ mất vài microsecond để push queue.
 * telegram_task chạy nền, thức dậy khi có message, gửi xong lại ngủ.
 *
 * Queue sâu 4 phần tử: vì trong thực tế khó có hơn 4 message liên tiếp
 * (startup + fall + sos + cancel là kịch bản max).
 */

#include <string.h>
#include <stdio.h>
#include "esp_http_client.h"    /* HTTP client: cấu hình, thực thi, cleanup */
#include "esp_log.h"            /* Log system của ESP-IDF */
#include "telegram.h"           /* Public API của module */
#include "telegram_messages.h"  /* Nội dung tin nhắn (người dùng có thể sửa) */
#include "esp_crt_bundle.h"     /* Bundle chứng chỉ SSL (cho HTTPS) */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"     /* Queue FreeRTOS cho message async */
#include "freertos/task.h"      /* Task FreeRTOS cho background worker */

static const char *TAG = "TELEGRAM";

/* Mảng nội dung tin nhắn — index khớp với enum telegram_msg_type_t bên dưới */
static const char* TELEGRAM_MESSAGES[] = {
    MSG_STARTUP,
    MSG_FALL_ALERT,
    MSG_SOS_ALERT,
    MSG_CANCEL_ALERT
};

/* Bot token và chat ID — copy từ telegram_init(), dùng khi build URL gọi Telegram API */
static char s_bot_token[128] = {0};
static char s_chat_id[32] = {0};

static bool s_initialized = false;  /* true sau khi telegram_init() thành công */

static char s_response_buffer[512]; /* Đọc response Telegram API để log debug */

static QueueHandle_t s_telegram_queue = NULL;      /* Queue FreeRTOS: tối đa 4 message chờ */
static TaskHandle_t s_telegram_task_handle = NULL;  /* Background task handle */

/* Kiểu dữ liệu cho queue — index khớp với mảng TELEGRAM_MESSAGES[] */
typedef enum {
    TELEGRAM_MSG_STARTUP,
    TELEGRAM_MSG_FALL_ALERT,
    TELEGRAM_MSG_SOS,
    TELEGRAM_MSG_CANCEL
} telegram_msg_type_t;

/* ===================== TRÌNH XỬ LÝ SỰ KIỆN HTTP ===================== */

/*
 * http_event_handler — copy response từ Telegram API vào buffer để log.
 * Chỉ quan tâm HTTP_EVENT_ON_DATA (data chunk đến) và HTTP_EVENT_ERROR.
 */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA: {
        int len = evt->data_len;
        if (len > (int)sizeof(s_response_buffer) - 1) {
            len = sizeof(s_response_buffer) - 1;
        }
        memcpy(s_response_buffer, evt->data, len);
        s_response_buffer[len] = '\0';
        break;
    }
    case HTTP_EVENT_ERROR:
        ESP_LOGE(TAG, "HTTP error");
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* ===================== MÃ HÓA URL (PERCENT-ENCODING) ===================== */

/*
 * url_encode — Percent-encode cho Telegram API query string.
 *
 * Telegram API nhận text trong URL (?text=...), nên ký tự đặc biệt
 * (space, &, =, %, +, \n,...) phải được mã hóa thành %XX.
 * Giữ nguyên chữ, số, -, _, ., ~ (RFC 3986 unreserved).
 */
static void url_encode(const char *src, char *dst, int dst_size)
{
    const char *hex = "0123456789ABCDEF";
    int j = 0;

    for (int i = 0; src[i] && j < dst_size - 3; i++) {
        unsigned char c = src[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
            c == '-' || c == '_' || c == '.' || c == '~') {
            dst[j++] = c;
        } else if (c == ' ') {
            dst[j++] = '%'; dst[j++] = '2'; dst[j++] = '0';
        } else if (c == '\n') {
            dst[j++] = '%'; dst[j++] = '0'; dst[j++] = 'A';
        } else {
            dst[j++] = '%';
            dst[j++] = hex[c >> 4];
            dst[j++] = hex[c & 0x0F];
        }
    }
    dst[j] = '\0';
}

/* ===================== GỬI TIN NHẮN QUA TELEGRAM API ===================== */

/*
 * send_telegram_message — Gửi HTTP GET đến Telegram Bot API.
 *
 * URL mẫu: https://api.telegram.org/bot<TOKEN>/sendMessage?chat_id=<ID>&text=<MSG>
 * Hàm này blocking (chờ HTTP response) nên chỉ gọi từ background task.
 */
static esp_err_t send_telegram_message(const char *message)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "Telegram not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    char encoded_msg[256];
    url_encode(message, encoded_msg, sizeof(encoded_msg));

    char url[512];
    int len = snprintf(url, sizeof(url),
             "https://api.telegram.org/bot%s/sendMessage?chat_id=%s&text=%s",
             s_bot_token, s_chat_id, encoded_msg);
    if (len >= sizeof(url)) {
        ESP_LOGE(TAG, "URL too long!");
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Sending Telegram message...");

    memset(s_response_buffer, 0, sizeof(s_response_buffer));

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .event_handler = http_event_handler,
        .timeout_ms = 15000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        int content_len = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "HTTP status=%d, content_len=%d", status, content_len);

        if (content_len > 0 && content_len < sizeof(s_response_buffer)) {
            int read_len = esp_http_client_read(client, s_response_buffer, content_len);
            if (read_len > 0) {
                s_response_buffer[read_len] = '\0';
                ESP_LOGI(TAG, "Response: %s", s_response_buffer);
            }
        }

        if (status == 200) {
            ESP_LOGI(TAG, "Telegram: Message sent OK!");
        } else {
            ESP_LOGW(TAG, "Telegram: status=%d", status);
        }
    } else {
        ESP_LOGE(TAG, "HTTP failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

/* ===================== BACKGROUND TASK (XỬ LÝ HÀNG ĐỢI) ===================== */

/*
 * telegram_task — Vòng lặp background: chờ queue → gửi HTTP.
 * Stack 4KB (đủ cho SSL handshake). Priority 4 (cao) để ưu tiên gửi cảnh báo.
 */
static void telegram_task(void *param)
{
    (void)param;
    telegram_msg_type_t msg_type;

    ESP_LOGI(TAG, "Telegram task started");

    while (true) {
        /* Block vô hạn — task chỉ chạy khi có message, tiết kiệm CPU */
        if (xQueueReceive(s_telegram_queue, &msg_type, portMAX_DELAY) == pdTRUE) {
            if (msg_type <= TELEGRAM_MSG_CANCEL) {
                ESP_LOGI(TAG, "Processing message type %d", msg_type);
                send_telegram_message(TELEGRAM_MESSAGES[msg_type]);
            }
        }
    }
}

/* ===================== PUBLIC API (GỌI TỪ MAIN CODE) ===================== */

/*
 * telegram_init — Lưu token, tạo queue (4 phần tử), spawn background task.
 * Gọi DUY NHẤT một lần. Queue sâu 4 là đủ (startup + fall + sos + cancel).
 */
void telegram_init(const char *bot_token, const char *chat_id)
{
    if (bot_token == NULL || chat_id == NULL) {
        ESP_LOGE(TAG, "Invalid token or chat_id");
        return;
    }

    strncpy(s_bot_token, bot_token, sizeof(s_bot_token) - 1);
    strncpy(s_chat_id, chat_id, sizeof(s_chat_id) - 1);
    s_initialized = true;

    if (s_telegram_queue == NULL) {
        s_telegram_queue = xQueueCreate(4, sizeof(telegram_msg_type_t));
        if (s_telegram_queue != NULL) {
            xTaskCreate(telegram_task, "telegram_task", 4096, NULL, 4, &s_telegram_task_handle);
        }
    }

    ESP_LOGI(TAG, "Telegram init: chat_id=%s", s_chat_id);
}

/* Các hàm dưới đây đều non-blocking: chỉ push message vào queue, không đợi gửi */

void telegram_send_startup(void)
{
    if (s_telegram_queue != NULL) {
        telegram_msg_type_t msg = TELEGRAM_MSG_STARTUP;
        xQueueSend(s_telegram_queue, &msg, 0);
        ESP_LOGI(TAG, "Startup message queued");
    }
}

void telegram_send_fall_alert(void)
{
    if (s_telegram_queue != NULL) {
        telegram_msg_type_t msg = TELEGRAM_MSG_FALL_ALERT;
        xQueueSend(s_telegram_queue, &msg, 0);
        ESP_LOGW(TAG, "Fall alert queued");
    }
}

void telegram_send_sos_alert(void)
{
    if (s_telegram_queue != NULL) {
        telegram_msg_type_t msg = TELEGRAM_MSG_SOS;
        xQueueSend(s_telegram_queue, &msg, 0);
        ESP_LOGW(TAG, "SOS alert queued");
    }
}

void telegram_send_cancel_alert(void)
{
    if (s_telegram_queue != NULL) {
        telegram_msg_type_t msg = TELEGRAM_MSG_CANCEL;
        xQueueSend(s_telegram_queue, &msg, 0);
        ESP_LOGI(TAG, "Cancel alert queued");
    }
}

bool telegram_is_initialized(void)
{
    return s_initialized;
}
