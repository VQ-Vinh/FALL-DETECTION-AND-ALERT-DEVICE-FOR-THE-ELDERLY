/**
 * @file telegram.c
 * @brief Module gửi tin nhắn Telegram bất đồng bộ (async) qua hàng đợi FreeRTOS
 *
 * ===================== KIẾN TRÚC ASYNC (HÀNG ĐỢI) =====================
 * Lý do sử dụng queue-based async:
 *   1. Main code (VD: mpu6050_task) chạy ở 100Hz, không thể bị block
 *   2. esp_http_client_perform() có thể block vài giây do network latency
 *   3. Timer callbacks (FreeRTOS) có stack rất nhỏ, không đủ để gọi HTTP
 *   4. Giải pháp: các hàm telegram_send_*() chỉ đẩy message vào queue
 *      -> telegram_task (task riêng, stack 4KB) đọc queue và gọi API
 *
 * ===================== LUỒNG XỬ LÝ =====================
 *   telegram_init()
 *     -> Tạo queue (4 phần tử)
 *     -> Tạo telegram_task (background)
 *   telegram_send_*()
 *     -> xQueueSend() -> đẩy message vào queue (non-blocking)
 *   telegram_task()
 *     -> xQueueReceive() -> chờ message (blocking với portMAX_DELAY)
 *     -> send_telegram_message() -> URL encode -> HTTP GET -> Telegram API
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

/* Tag dùng cho ESP_LOG (hiển thị tiền tố [TELEGRAM]) */
static const char *TAG = "TELEGRAM";

/*
 * Danh sách các tin nhắm Telegram (mảng string).
 * THỨ TỰ PHẢI KHỚP với enum telegram_msg_type_t bên dưới:
 *   0 = MSG_STARTUP    (Khởi động)
 *   1 = MSG_FALL_ALERT (Phát hiện ngã)
 *   2 = MSG_SOS_ALERT  (Nút khẩn cấp)
 *   3 = MSG_CANCEL_ALERT (Hủy báo động)
 */
static const char* TELEGRAM_MESSAGES[] = {
    MSG_STARTUP,
    MSG_FALL_ALERT,
    MSG_SOS_ALERT,
    MSG_CANCEL_ALERT
};

/* ===================== CẤU HÌNH BOT ===================== */

/* Bot token: mã xác thực BotFather, dạng "123456:ABC-DEF1234ghIkl-zyx57W2v1u123ew11" */
static char s_bot_token[128] = {0};

/* Chat ID: ID của cuộc hội thoại Telegram (người nhận), dạng số nguyên */
static char s_chat_id[32] = {0};

/* Cờ trạng thái: true nếu telegram_init() đã được gọi thành công */
static bool s_initialized = false;

/* ===================== BỘ ĐỆM PHẢN HỒI ===================== */

/* Bộ đệm lưu response từ Telegram API (dùng trong http_event_handler) */
static char s_response_buffer[512];

/* ===================== HÀNG ĐỢI ASYNC (FREERTOS QUEUE) ===================== */

/* Queue chứa các message type cần gửi. Kích thước: 4 phần tử. */
static QueueHandle_t s_telegram_queue = NULL;

/* Handle của background task (telegram_task) - dùng để quản lý task */
static TaskHandle_t s_telegram_task_handle = NULL;

/*
 * Kiểu dữ liệu cho message trong queue.
 * THỨ TỰ PHẢI KHỚP với mảng TELEGRAM_MESSAGES[] bên trên.
 */
typedef enum {
    TELEGRAM_MSG_STARTUP,     /* 0: Tin nhắn khởi động */
    TELEGRAM_MSG_FALL_ALERT,  /* 1: Cảnh báo ngã */
    TELEGRAM_MSG_SOS,         /* 2: Cảnh báo SOS */
    TELEGRAM_MSG_CANCEL       /* 3: Hủy báo động */
} telegram_msg_type_t;

/* ===================== TRÌNH XỬ LÝ SỰ KIỆN HTTP ===================== */

/*
 * http_event_handler - Xử lý các sự kiện trong quá trình HTTP request.
 *
 * Các sự kiện:
 *   HTTP_EVENT_ON_DATA: Dữ liệu response đến từng chunk -> copy vào bộ đệm
 *   HTTP_EVENT_ERROR:   Lỗi HTTP (timeout, DNS, SSL,...) -> log lỗi
 */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA: {
        /* Copy dữ liệu response vào bộ đệm, giới hạn kích thước */
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
 * url_encode - Chuyển chuỗi ký tự thành định dạng an toàn cho URL.
 *
 * Tại sao cần: Telegram API nhận text trong query string (?text=...).
 * Các ký tự đặc biệt (space, \n, &, =, %, +, ...) phải được mã hóa
 * thành %XX nếu không sẽ phá vỡ cấu trúc URL.
 *
 * Bảng mã hóa:
 *   - Chữ cái (a-z, A-Z) và số (0-9): giữ nguyên (an toàn)
 *   - Dấu gạch ngang (-), gạch dưới (_), chấm (.), ngã (~): giữ nguyên
 *   - Khoảng trắng (0x20): -> %20
 *   - Xuống dòng (\n, 0x0A): -> %0A
 *   - Ký tự khác: -> %XX (XX = mã hex của ký tự)
 */
static void url_encode(const char *src, char *dst, int dst_size)
{
    /* Bảng hex để chuyển đổi nibble -> ký tự */
    const char *hex = "0123456789ABCDEF";
    int j = 0;

    /* Duyệt từng ký tự nguồn, dừng nếu hết bộ đệm đích */
    for (int i = 0; src[i] && j < dst_size - 3; i++) {
        unsigned char c = src[i];
        /* Ký tự an toàn (không cần mã hóa) - RFC 3986 unreserved characters */
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
            c == '-' || c == '_' || c == '.' || c == '~') {
            dst[j++] = c;
        } else if (c == ' ') {
            /* Khoảng trắng -> %20 */
            dst[j++] = '%';
            dst[j++] = '2';
            dst[j++] = '0';
        } else if (c == '\n') {
            /* Xuống dòng -> %0A */
            dst[j++] = '%';
            dst[j++] = '0';
            dst[j++] = 'A';
        } else {
            /* Ký tự đặc biệt khác -> %XX */
            dst[j++] = '%';
            dst[j++] = hex[c >> 4];      /* nibble cao */
            dst[j++] = hex[c & 0x0F];    /* nibble thấp */
        }
    }
    /* Kết thúc chuỗi đích */
    dst[j] = '\0';
}

/* ===================== GỬI TIN NHẮN QUA TELEGRAM API ===================== */

/*
 * send_telegram_message - Hàm thực hiện HTTP GET đến Telegram Bot API.
 *
 * URL mẫu: https://api.telegram.org/bot<TOKEN>/sendMessage?chat_id=<ID>&text=<MSG>
 *
 * Quy trình:
 *   1. Kiểm tra đã khởi tạo chưa (s_initialized)
 *   2. URL encode nội dung tin nhắn (message)
 *   3. Build URL hoàn chỉnh với token, chat_id, text
 *   4. Khởi tạo HTTP client với cấu hình (SSL bundle, timeout 15s)
 *   5. Thực thi HTTP request (esp_http_client_perform)
 *   6. Kiểm tra status code (200 = OK)
 *   7. Đọc response body (nếu có)
 *   8. Cleanup HTTP client
 */
static esp_err_t send_telegram_message(const char *message)
{
    /* Kiểm tra trạng thái khởi tạo */
    if (!s_initialized) {
        ESP_LOGW(TAG, "Telegram not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    /* Bước 1: Mã hóa URL nội dung tin nhắn */
    char encoded_msg[256];
    url_encode(message, encoded_msg, sizeof(encoded_msg));

    /* Bước 2: Xây dựng URL đầy đủ */
    char url[512];
    int len = snprintf(url, sizeof(url),
             "https://api.telegram.org/bot%s/sendMessage?chat_id=%s&text=%s",
             s_bot_token, s_chat_id, encoded_msg);
    if (len >= sizeof(url)) {
        ESP_LOGE(TAG, "URL too long!");
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Sending Telegram message...");

    /* Xóa bộ đệm response cũ */
    memset(s_response_buffer, 0, sizeof(s_response_buffer));

    /* Bước 3: Cấu hình HTTP client */
    esp_http_client_config_t config = {
        .url = url,                          /* URL đầy đủ */
        .method = HTTP_METHOD_GET,           /* Phương thức GET (Telegram API hỗ trợ GET) */
        .event_handler = http_event_handler, /* Callback xử lý sự kiện HTTP */
        .timeout_ms = 15000,                 /* Timeout 15 giây */
        .crt_bundle_attach = esp_crt_bundle_attach, /* Bundle SSL cho HTTPS (xác thực chứng chỉ) */
    };

    /* Bước 4: Khởi tạo HTTP client handle */
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return ESP_FAIL;
    }

    /* Bước 5: Thực thi HTTP request (blocking - chạy trong background task) */
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        /* Lấy status code và content length từ response */
        int status = esp_http_client_get_status_code(client);
        int content_len = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "HTTP status=%d, content_len=%d", status, content_len);

        /* Đọc response body nếu có */
        if (content_len > 0 && content_len < sizeof(s_response_buffer)) {
            int read_len = esp_http_client_read(client, s_response_buffer, content_len);
            if (read_len > 0) {
                s_response_buffer[read_len] = '\0';
                ESP_LOGI(TAG, "Response: %s", s_response_buffer);
            }
        }

        /* Kiểm tra status code 200 = thành công */
        if (status == 200) {
            ESP_LOGI(TAG, "Telegram: Message sent OK!");
        } else {
            ESP_LOGW(TAG, "Telegram: status=%d", status);
        }
    } else {
        ESP_LOGE(TAG, "HTTP failed: %s", esp_err_to_name(err));
    }

    /* Bước 6: Dọn dẹp HTTP client */
    esp_http_client_cleanup(client);
    return err;
}

/* ===================== BACKGROUND TASK (XỬ LÝ HÀNG ĐỢI) ===================== */

/*
 * telegram_task - FreeRTOS task chạy nền, xử lý queue message.
 *
 * Luồng hoạt động:
 *   while(1):
 *     - xQueueReceive() BLOCK chờ message từ queue
 *     - Khi có message: gọi send_telegram_message() với nội dung tương ứng
 *
 * Stack size: 4096 bytes (đủ cho HTTP client với SSL)
 * Priority: 4 (cao hơn task thường để đảm bảo gửi kịp cảnh báo)
 */
static void telegram_task(void *param)
{
    (void)param;
    telegram_msg_type_t msg_type;

    ESP_LOGI(TAG, "Telegram task started");

    while (true) {
        /*
         * Block vô hạn (portMAX_DELAY) chờ message từ queue.
         * Tiết kiệm CPU: task chỉ chạy khi có message.
         */
        if (xQueueReceive(s_telegram_queue, &msg_type, portMAX_DELAY) == pdTRUE) {
            /* Kiểm tra msg_type hợp lệ (tránh out-of-bound) */
            if (msg_type <= TELEGRAM_MSG_CANCEL) {
                ESP_LOGI(TAG, "Processing message type %d", msg_type);
                /* Gửi tin nhắn tương ứng từ mảng TELEGRAM_MESSAGES[] */
                send_telegram_message(TELEGRAM_MESSAGES[msg_type]);
            }
        }
    }
}

/* ===================== PUBLIC API (GỌI TỪ MAIN CODE) ===================== */

/*
 * telegram_init - Khởi tạo module Telegram.
 *
 * @param bot_token: Token từ BotFather (dạng "123456:ABC-DEF1234ghIkl-zyx57W2v1u123ew11")
 * @param chat_id:   ID chat Telegram người nhận (dạng số hoặc @username)
 *
 * Công việc:
 *   1. Lưu bot_token và chat_id vào biến static
 *   2. Đặt cờ s_initialized = true
 *   3. Tạo queue FreeRTOS (4 phần tử, mỗi phần tử là telegram_msg_type_t ~ 4 byte)
 *   4. Tạo background task (telegram_task) nếu chưa có
 *
 * Lưu ý: Queue dùng 4 phần tử vì trong thời gian ngắn có thể có tối đa
 *         startup + fall + sos + cancel (4 message liên tiếp).
 */
void telegram_init(const char *bot_token, const char *chat_id)
{
    /* Kiểm tra tham số đầu vào */
    if (bot_token == NULL || chat_id == NULL) {
        ESP_LOGE(TAG, "Invalid token or chat_id");
        return;
    }

    /* Copy token và chat_id vào bộ nhớ static (giới hạn kích thước) */
    strncpy(s_bot_token, bot_token, sizeof(s_bot_token) - 1);
    strncpy(s_chat_id, chat_id, sizeof(s_chat_id) - 1);
    s_initialized = true;

    /* Tạo queue và task (chỉ một lần) */
    if (s_telegram_queue == NULL) {
        /*
         * xQueueCreate(4, sizeof(telegram_msg_type_t)):
         *   - 4: queue depth (tối đa 4 message chờ)
         *   - sizeof(telegram_msg_type_t): kích thước mỗi phần tử (~4 byte)
         */
        s_telegram_queue = xQueueCreate(4, sizeof(telegram_msg_type_t));
        if (s_telegram_queue != NULL) {
            /*
             * xTaskCreate(task_func, name, stack_size, param, priority, handle):
             *   - stack_size: 4096 bytes
             *   - priority: 4 (cao)
             */
            xTaskCreate(telegram_task, "telegram_task", 4096, NULL, 4, &s_telegram_task_handle);
        }
    }

    ESP_LOGI(TAG, "Telegram init: chat_id=%s", s_chat_id);
}

/*
 * telegram_send_startup - Đẩy message khởi động vào queue.
 * Không blocking: chỉ gọi xQueueSend() với timeout = 0.
 */
void telegram_send_startup(void)
{
    if (s_telegram_queue != NULL) {
        telegram_msg_type_t msg = TELEGRAM_MSG_STARTUP;
        xQueueSend(s_telegram_queue, &msg, 0);
        ESP_LOGI(TAG, "Startup message queued");
    }
}

/*
 * telegram_send_fall_alert - Đẩy message cảnh báo ngã vào queue.
 * Gọi từ mpu6050_task khi phát hiện ngã.
 */
void telegram_send_fall_alert(void)
{
    if (s_telegram_queue != NULL) {
        telegram_msg_type_t msg = TELEGRAM_MSG_FALL_ALERT;
        xQueueSend(s_telegram_queue, &msg, 0);
        ESP_LOGW(TAG, "Fall alert queued");
    }
}

/*
 * telegram_send_sos_alert - Đẩy message SOS (nút khẩn cấp) vào queue.
 * Gọi từ button_task hoặc GPIO ISR.
 */
void telegram_send_sos_alert(void)
{
    if (s_telegram_queue != NULL) {
        telegram_msg_type_t msg = TELEGRAM_MSG_SOS;
        xQueueSend(s_telegram_queue, &msg, 0);
        ESP_LOGW(TAG, "SOS alert queued");
    }
}

/*
 * telegram_send_cancel_alert - Đẩy message hủy báo động vào queue.
 * Gọi khi người dùng nhấn nút xác nhận an toàn.
 */
void telegram_send_cancel_alert(void)
{
    if (s_telegram_queue != NULL) {
        telegram_msg_type_t msg = TELEGRAM_MSG_CANCEL;
        xQueueSend(s_telegram_queue, &msg, 0);
        ESP_LOGI(TAG, "Cancel alert queued");
    }
}

/*
 * telegram_is_initialized - Kiểm tra module Telegram đã khởi tạo chưa.
 *
 * @return: true nếu telegram_init() đã được gọi thành công, false nếu chưa.
 */
bool telegram_is_initialized(void)
{
    return s_initialized;
}
