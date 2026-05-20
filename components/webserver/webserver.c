/*
 * webserver.c - HTTP server: dashboard HTML + REST API JSON
 *
 * Server nhúng trên ESP32 dùng esp_http_server, phục vụ:
 *   GET /              → Dashboard HTML  (từ embedded file index.html)
 *   GET /style.css     → Stylesheet      (từ embedded file style.css)
 *   GET /app.js        → JavaScript      (từ embedded file app.js)
 *   GET /api/data      → JSON sensor data
 *   GET /api/status    → JSON device status
 *
 * ===================== DOUBLE-BUFFER =====================
 * Dữ liệu cảm biến được chia sẻ giữa MPU6050 task (writer) và HTTP handler
 * (reader) qua hai buffer luân phiên (g_sensor_buffers[2]):
 *
 *   MPU6050 ghi vào buffer[writer_index], rồi swap writer_index.
 *   HTTP handler đọc từ buffer[1 - writer_index] (buffer cũ, ổn định).
 *
 * Reader và writer KHÔNG BAO GIỜ chạm cùng một buffer ⇒ không cần mutex,
 * tránh deadlock, giảm latency. Dữ liệu reader thấy luôn nhất quán.
 *
 * ===================== FILE UI TĨNH =====================
 * Dashboard HTML/CSS/JS được tách riêng thành file trong components/webui/,
 * nhúng vào firmware bằng cơ chế EMBED_FILES của ESP-IDF.
 * Không cần SPIFFS, vẫn single-binary, không lo file hỏng/thất lạc.
 * Muốn sửa dashboard chỉ cần sửa file .html/.css/.js rồi rebuild.
 */

#include <string.h>
#include <stdio.h>
#include <sys/param.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "webserver.h"
#include "webui.h"

static const char *TAG = "WEB";

static httpd_handle_t server = NULL;  /* Handle HTTP server — dùng để stop & register handlers */

/*
 * volatile vì uptime có thể được ghi từ ISR hoặc task khác — compiler
 * không cache trong thanh ghi mà đọc/ghi RAM mỗi lần.
 */
static volatile uint32_t system_uptime_sec = 0;

/* Prototypes */
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t style_get_handler(httpd_req_t *req);
static esp_err_t script_get_handler(httpd_req_t *req);
static esp_err_t data_get_handler(httpd_req_t *req);
static esp_err_t status_get_handler(httpd_req_t *req);
static esp_err_t favicon_get_handler(httpd_req_t *req);

/*
 * File UI tĩnh được nhúng qua component webui (EMBED_FILES).
 * Mỗi file khai báo extern symbol _binary_<tên>_start / _binary_<tên>_end.
 * Xem webui.h để biết danh sách symbol.
 */

// ========== HÀM TRỢ GIÚP (HELPERS) ==========

/*
 * CORS headers: cho phép browser từ domain khác gọi API thiết bị.
 * Cần thiết khi dashboard truy cập từ IP của ESP32 (CORS policy mặc định chặn).
 * Allow-Origin: * (mọi origin), Methods: GET/POST/OPTIONS.
 */
static void set_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

/* Helper: gửi JSON 200 + CORS headers */
static esp_err_t send_json_response(httpd_req_t *req, const char *json, size_t len)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, "200 OK");
    set_cors_headers(req);
    return httpd_resp_send(req, json, len);
}



// ========== BỘ XỬ LÝ URI (URI HANDLERS) ==========

/* Helper: gửi file nhúng (HTML/CSS/JS) với content type tương ứng */
static esp_err_t serve_embedded(httpd_req_t *req, const char *ct,
                                const uint8_t *start, const uint8_t *end)
{
    httpd_resp_set_type(req, ct);
    httpd_resp_set_status(req, "200 OK");
    set_cors_headers(req);
    return httpd_resp_send(req, (const char *)start, end - start);
}

/* GET / — dashboard HTML */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /");
    return serve_embedded(req, "text/html; charset=utf-8",
                          index_html_start, index_html_end);
}

/* GET /style.css — dashboard stylesheet */
static esp_err_t style_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /style.css");
    return serve_embedded(req, "text/css; charset=utf-8",
                          style_css_start, style_css_end);
}

/* GET /app.js — dashboard JavaScript */
static esp_err_t script_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /app.js");
    return serve_embedded(req, "application/javascript; charset=utf-8",
                          app_js_start, app_js_end);
}

/*
 * GET /api/data — endpoint chính, JS dashboard gọi mỗi 100ms.
 *
 * Cơ chế double-buffer trong hàm này:
 *   1. extern g_get_writer_index() → lấy index buffer writer đang ghi (0 hoặc 1)
 *   2. reader_index = 1 - writer_index → buffer an toàn để đọc
 *   3. memcpy toàn bộ struct vào local → atomic read (writer không chạm buffer này)
 *
 * JSON trả về gồm: accel_g, gyro, roll, pitch, ready, wifi_connected,
 * alert_active, uptime, fall_state, max_tilt, filt_accel.
 */
static esp_err_t data_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /api/data");

    char buf[256];
    uint8_t fall_state = 0;
    float fall_max_tilt = 0.0f;
    float fall_filtered_accel = 0.0f;

    /* Đọc sensor data từ double-buffer (an toàn, không mutex) */
    extern uint8_t g_get_writer_index(void);
    sensor_data_t local_data;
    uint8_t reader_index = 1 - g_get_writer_index();
    memcpy(&local_data, &g_sensor_buffers[reader_index], sizeof(sensor_data_t));

    /* Lấy thông số fall detection từ module fall_detection */
    extern uint8_t fall_detection_get_state_internal(void);
    extern float fall_detection_get_max_tilt_internal(void);
    extern float fall_detection_get_filtered_accel_internal(void);
    fall_state = fall_detection_get_state_internal();
    fall_max_tilt = fall_detection_get_max_tilt_internal();
    fall_filtered_accel = fall_detection_get_filtered_accel_internal();

    /* Kiểm tra WiFi */
    extern bool wifi_is_connected(void);
    bool wifi_ok = wifi_is_connected();

    /* Build JSON: accel_g (2 decimals), gyro (0), roll/pitch (1), bool→int, uptime %lu */
    snprintf(buf, sizeof(buf),
        "{\"accel_g\":%.2f,\"gyro\":%.0f,\"roll\":%.1f,\"pitch\":%.1f,"
        "\"ready\":%d,\"wifi_connected\":%d,\"alert_active\":%d,\"uptime\":%lu,"
        "\"fall_state\":%d,\"max_tilt\":%.1f,\"filt_accel\":%.2f}",
        local_data.total_accel_g, local_data.total_gyro,
        local_data.roll, local_data.pitch, local_data.data_ready ? 1 : 0,
        wifi_ok ? 1 : 0, (fall_state == 4) ? 1 : 0, system_uptime_sec,
        fall_state, fall_max_tilt, fall_filtered_accel);

    return send_json_response(req, buf, strlen(buf));
}

/* GET /api/status — endpoint nhẹ, trả về 4 trường: wifi, alert, error, uptime */
static esp_err_t status_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /api/status");

    extern bool wifi_is_connected(void);
    bool wifi_ok = wifi_is_connected();

    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"wifi_connected\":%d,\"alert_active\":%d,\"error_state\":0,\"uptime\":%lu}",
        wifi_ok ? 1 : 0, 0, system_uptime_sec);

    return send_json_response(req, buf, strlen(buf));
}

/*
 * OPTIONS /api/ * — CORS preflight.
 * Browser gửi request này trước cross-origin request để hỏi server
 * có cho phép không. Trả về 204 + CORS headers.
 */
static esp_err_t options_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "OPTIONS %s", req->uri);
    httpd_resp_set_status(req, "204 No Content");
    set_cors_headers(req);
    return httpd_resp_send(req, NULL, 0);
}

/* GET /favicon.ico — không có favicon, trả 204 để tránh 404 trên console browser */
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /favicon.ico");
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

// ========== URI HANDLER DEFINITIONS ==========

/* Định nghĩa các URI handler — httpd_register_uri_handler() map URI+method → handler */
static const httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};

static const httpd_uri_t uri_style = {
    .uri = "/style.css",
    .method = HTTP_GET,
    .handler = style_get_handler
};

static const httpd_uri_t uri_script = {
    .uri = "/app.js",
    .method = HTTP_GET,
    .handler = script_get_handler
};

static const httpd_uri_t uri_api_data = {
    .uri = "/api/data",
    .method = HTTP_GET,
    .handler = data_get_handler
};

static const httpd_uri_t uri_api_status = {
    .uri = "/api/status",
    .method = HTTP_GET,
    .handler = status_get_handler
};

static const httpd_uri_t uri_favicon = {
    .uri = "/favicon.ico",
    .method = HTTP_GET,
    .handler = favicon_get_handler
};

static const httpd_uri_t uri_options_data = {
    .uri = "/api/data",
    .method = HTTP_OPTIONS,
    .handler = options_handler
};

static const httpd_uri_t uri_options_all = {
    .uri = "/api/*",
    .method = HTTP_OPTIONS,
    .handler = options_handler
};

// ========== ĐIỀU KHIỂN SERVER (SERVER CONTROL) ==========

/*
 * webserver_start_with_callbacks — start HTTP server (port 80, 8 URI handlers max).
 *
 * cbs bị bỏ qua (read-only mode — không nhận lệnh điều khiển từ client).
 * Nếu httpd_start OK, đăng ký 6 handlers:
 *   GET /, GET /api/data, GET /api/status, GET /favicon.ico,
 *   OPTIONS /api/data, OPTIONS /api/ *
 */
void webserver_start_with_callbacks(const void *cbs)
{
    (void)cbs;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 10;
    config.stack_size = 4096;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_style);
        httpd_register_uri_handler(server, &uri_script);
        httpd_register_uri_handler(server, &uri_api_data);
        httpd_register_uri_handler(server, &uri_api_status);
        httpd_register_uri_handler(server, &uri_favicon);
        httpd_register_uri_handler(server, &uri_options_data);
        httpd_register_uri_handler(server, &uri_options_all);

        ESP_LOGI(TAG, "HTTP server started (read-only)");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        server = NULL;
    }
}

/* Wrapper gọi webserver_start_with_callbacks(NULL) — dùng khi không cần callback */
void webserver_start(void)
{
    webserver_start_with_callbacks(NULL);
}

/* Dừng server. Kiểm tra handle tránh double-stop. */
void webserver_stop(void)
{
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}
