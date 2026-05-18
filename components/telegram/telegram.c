/**
 * @file telegram.c
 * @brief Telegram Bot async message sender
 *
 * Kiến trúc:
 * - Main code gọi telegram_send_*() → đẩy message vào queue
 * - telegram_task chạy background, đọc queue và gọi API
 *
 * Tại sao async?
 * - esp_http_client_perform() có thể block vài giây
 * - Không block được mpu6050_task (100Hz)
 * - Timer callbacks có stack rất nhỏ, không thể gọi HTTP ở đó
 */

#include <string.h>
#include <stdio.h>
#include "esp_http_client.h"
#include "esp_log.h"
#include "telegram.h"
#include "telegram_messages.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "TELEGRAM";

// Danh sách tin nhắn
static const char* TELEGRAM_MESSAGES[] = {
    MSG_STARTUP,
    MSG_FALL_ALERT,
    MSG_SOS_ALERT,
    MSG_CANCEL_ALERT
};

// Bot configuration
static char s_bot_token[128] = {0};
static char s_chat_id[32] = {0};
static bool s_initialized = false;

// Response buffer
static char s_response_buffer[512];

// Async queue
static QueueHandle_t s_telegram_queue = NULL;
static TaskHandle_t s_telegram_task_handle = NULL;

// Message types
typedef enum {
    TELEGRAM_MSG_STARTUP,
    TELEGRAM_MSG_FALL_ALERT,
    TELEGRAM_MSG_SOS,
    TELEGRAM_MSG_CANCEL
} telegram_msg_type_t;

// ========== HTTP EVENT HANDLER ==========
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

// ========== URL ENCODE ==========
// Mã hóa URL (chuyển ký tự đặc biệt thành %XX)
static void url_encode(const char *src, char *dst, int dst_size)
{
    const char *hex = "0123456789ABCDEF";
    int j = 0;

    for (int i = 0; src[i] && j < dst_size - 3; i++) {
        unsigned char c = src[i];
        // Các ký tự an toàn
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
            c == '-' || c == '_' || c == '.' || c == '~') {
            dst[j++] = c;
        } else if (c == ' ') {
            dst[j++] = '%';
            dst[j++] = '2';
            dst[j++] = '0';
        } else if (c == '\n') {
            dst[j++] = '%';
            dst[j++] = '0';
            dst[j++] = 'A';
        } else {
            dst[j++] = '%';
            dst[j++] = hex[c >> 4];
            dst[j++] = hex[c & 0x0F];
        }
    }
    dst[j] = '\0';
}

// ========== SEND MESSAGE ==========
static esp_err_t send_telegram_message(const char *message)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "Telegram not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    // URL encode message
    char encoded_msg[256];
    url_encode(message, encoded_msg, sizeof(encoded_msg));

    // Build URL
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

    // HTTP request
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

// ========== TELEGRAM TASK (BACKGROUND) ==========
static void telegram_task(void *param)
{
    (void)param;
    telegram_msg_type_t msg_type;

    ESP_LOGI(TAG, "Telegram task started");

    while (true) {
        // Block đợi message trong queue
        if (xQueueReceive(s_telegram_queue, &msg_type, portMAX_DELAY) == pdTRUE) {
            if (msg_type <= TELEGRAM_MSG_CANCEL) {
                ESP_LOGI(TAG, "Processing message type %d", msg_type);
                send_telegram_message(TELEGRAM_MESSAGES[msg_type]);
            }
        }
    }
}

// ========== PUBLIC API ==========
void telegram_init(const char *bot_token, const char *chat_id)
{
    if (bot_token == NULL || chat_id == NULL) {
        ESP_LOGE(TAG, "Invalid token or chat_id");
        return;
    }

    strncpy(s_bot_token, bot_token, sizeof(s_bot_token) - 1);
    strncpy(s_chat_id, chat_id, sizeof(s_chat_id) - 1);
    s_initialized = true;

    // Tạo queue và task
    if (s_telegram_queue == NULL) {
        s_telegram_queue = xQueueCreate(4, sizeof(telegram_msg_type_t));
        if (s_telegram_queue != NULL) {
            xTaskCreate(telegram_task, "telegram_task", 4096, NULL, 4, &s_telegram_task_handle);
        }
    }

    ESP_LOGI(TAG, "Telegram init: chat_id=%s", s_chat_id);
}

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
