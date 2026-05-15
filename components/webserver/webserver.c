/**
 * @file webserver.c
 * @brief HTTP server cung cấp dashboard và API
 *
 * Sử dụng double-buffer để đọc sensor data:
 * - Writer (mpu6050_task): ghi vào buffer 0 hoặc 1
 * - Reader (HTTP handler): đọc từ buffer còn lại
 *
 * Cách hoạt động:
 * 1. mpu6050_task ghi dữ liệu vào buffer[s_writer_index]
 * 2. mpu6050_task swap s_writer_index = 1 - s_writer_index
 * 3. HTTP handler đọc buffer[1 - s_writer_index]
 * → Reader và Writer không bao giờ truy cập cùng 1 buffer
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

static const char *TAG = "WEB";

// HTTP server handle
static httpd_handle_t server = NULL;

// System uptime (giây)
static volatile uint32_t system_uptime_sec = 0;

// ========== PROTOTYPES ==========
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t data_get_handler(httpd_req_t *req);
static esp_err_t status_get_handler(httpd_req_t *req);
static esp_err_t favicon_get_handler(httpd_req_t *req);

// ========== HTML DASHBOARD ==========
// Dashboard hiển thị sensor data và điều khiển
static const char HTML_PAGE[] =
    "<!DOCTYPE html>"
    "<html lang='en'>"
    "<head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>Fall Detection Monitor</title>"
    "<style>"
    "body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; "
    "background: #1a1a2e; color: #eee; margin: 0; padding: 20px; }"
    ".container { max-width: 600px; margin: 0 auto; }"
    "h1 { text-align: center; color: #e94560; margin-bottom: 20px; }"
    ".status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 20px; }"
    ".status-card { background: #16213e; padding: 15px; border-radius: 8px; text-align: center; }"
    ".status-card label { display: block; color: #888; font-size: 12px; margin-bottom: 5px; }"
    ".status-card .value { font-size: 24px; font-weight: bold; }"
    ".sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 20px; }"
    ".sensor-card { background: #16213e; padding: 15px; border-radius: 8px; }"
    ".sensor-card label { display: block; color: #888; font-size: 12px; }"
    ".sensor-card .value { font-size: 20px; font-weight: bold; color: #0f3460; }"
    ".btn-group { display: flex; gap: 10px; justify-content: center; }"
    "button { padding: 12px 24px; border: none; border-radius: 6px; cursor: pointer; font-size: 14px; }"
    "#stopBtn { background: #e94560; color: white; }"
    "#resetBtn { background: #0f3460; color: white; }"
    "button:hover { opacity: 0.8; }"
    ".alert-active { background: #e94560; }"
    ".normal { color: #4ecca3; }"
    ".warning { color: #ffc107; }"
    ".error { color: #e94560; }"
    "</style>"
    "</head>"
    "<body>"
    "<div class='container'>"
    "<h1>Fall Detection Device</h1>"
    "<div class='status-grid'>"
    "<div class='status-card'>"
    "<label>WiFi Status</label>"
    "<div class='value' id='wifi'>Checking...</div>"
    "</div>"
    "<div class='status-card'>"
    "<label>Alert Status</label>"
    "<div class='value' id='alert'>IDLE</div>"
    "</div>"
    "</div>"
    "<div class='sensor-grid'>"
    "<div class='sensor-card'>"
    "<label>Acceleration (m/s²)</label>"
    "<div class='value' id='accel'>0.00</div>"
    "</div>"
    "<div class='sensor-card'>"
    "<label>Acceleration (g)</label>"
    "<div class='value' id='accel_g'>0.00</div>"
    "</div>"
    "<div class='sensor-card'>"
    "<label>Gyroscope (deg/s)</label>"
    "<div class='value' id='gyro'>0.00</div>"
    "</div>"
    "<div class='sensor-card'>"
    "<label>Roll (°)</label>"
    "<div class='value' id='roll'>0.0</div>"
    "</div>"
    "<div class='sensor-card'>"
    "<label>Pitch (°)</label>"
    "<div class='value' id='pitch'>0.0</div>"
    "</div>"
    "<div class='sensor-card'>"
    "<label>Uptime (s)</label>"
    "<div class='value' id='uptime'>0</div>"
    "</div>"
    "</div>"
    "</div>"
    "<script>"
    "function updateData() {"
    "fetch('/api/data').then(r=>r.json()).then(d=>{"
    "document.getElementById('accel').textContent = d.accel.toFixed(2);"
    "document.getElementById('accel_g').textContent = d.accel_g.toFixed(2);"
    "document.getElementById('gyro').textContent = d.gyro.toFixed(2);"
    "document.getElementById('roll').textContent = d.roll.toFixed(1);"
    "document.getElementById('pitch').textContent = d.pitch.toFixed(1);"
    "document.getElementById('uptime').textContent = d.uptime;"
    "document.getElementById('wifi').textContent = d.wifi_connected ? 'Connected' : 'Disconnected';"
    "document.getElementById('wifi').className = 'value ' + (d.wifi_connected ? 'normal' : 'error');"
    "var alertEl = document.getElementById('alert');"
    "if(d.alert_active) { alertEl.textContent = 'ACTIVE'; alertEl.className = 'value warning'; }"
    "else { alertEl.textContent = 'IDLE'; alertEl.className = 'value normal'; }"
    "});"
    "}"
    "setInterval(updateData, 100);"
    "</script>"
    "</body>"
    "</html>";

// ========== HELPERS ==========
static void set_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static esp_err_t send_json_response(httpd_req_t *req, const char *json, size_t len)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, "200 OK");
    set_cors_headers(req);
    return httpd_resp_send(req, json, len);
}

static esp_err_t send_html_response(httpd_req_t *req, const char *html, size_t len)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_status(req, "200 OK");
    set_cors_headers(req);
    return httpd_resp_send(req, html, len);
}

static esp_err_t send_error_response(httpd_req_t *req, int status_code, const char *message)
{
    char status_str[32];
    snprintf(status_str, sizeof(status_str), "%d %s", status_code,
             status_code == 404 ? "Not Found" :
             status_code == 500 ? "Internal Server Error" : "Error");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, status_str);
    set_cors_headers(req);
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"error\":\"%s\"}", message);
    return httpd_resp_send(req, buf, strlen(buf));
}

// ========== URI HANDLERS ==========

// GET / - Dashboard HTML
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /");
    return send_html_response(req, HTML_PAGE, strlen(HTML_PAGE));
}

// GET /api/data - Sensor data JSON
static esp_err_t data_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /api/data");

    char buf[256];
    uint8_t fall_state = 0;
    float fall_max_tilt = 0.0f;
    float fall_filtered_accel = 0.0f;

    // Đọc sensor data từ double-buffer
    extern uint8_t g_get_writer_index(void);
    sensor_data_t local_data;
    uint8_t reader_index = 1 - g_get_writer_index();
    memcpy(&local_data, &g_sensor_buffers[reader_index], sizeof(sensor_data_t));

    // Lấy fall detection state
    extern uint8_t fall_detection_get_state_internal(void);
    extern float fall_detection_get_max_tilt_internal(void);
    extern float fall_detection_get_filtered_accel_internal(void);
    fall_state = fall_detection_get_state_internal();
    fall_max_tilt = fall_detection_get_max_tilt_internal();
    fall_filtered_accel = fall_detection_get_filtered_accel_internal();

    // Lấy WiFi status (直接调用wifi模块)
    extern bool wifi_is_connected(void);
    bool wifi_ok = wifi_is_connected();

    // Build JSON response
    snprintf(buf, sizeof(buf),
        "{\"accel\":%.2f,\"accel_g\":%.2f,\"gyro\":%.2f,\"roll\":%.2f,\"pitch\":%.2f,"
        "\"ready\":%d,\"wifi_connected\":%d,\"alert_active\":%d,\"error_state\":0,\"uptime\":%lu,"
        "\"fall_state\":%d,\"max_tilt\":%.1f,\"filt_accel\":%.2f}",
        local_data.total_accel, local_data.total_accel_g, local_data.total_gyro,
        local_data.roll, local_data.pitch, local_data.data_ready ? 1 : 0,
        wifi_ok ? 1 : 0, 0, system_uptime_sec,
        fall_state, fall_max_tilt, fall_filtered_accel);

    return send_json_response(req, buf, strlen(buf));
}

// GET /api/status - Device status
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

// OPTIONS /api/* - CORS preflight
static esp_err_t options_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "OPTIONS %s", req->uri);
    httpd_resp_set_status(req, "204 No Content");
    set_cors_headers(req);
    return httpd_resp_send(req, NULL, 0);
}

// GET /favicon.ico - Return 204
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /favicon.ico");
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

// ========== URI HANDLER DEFINITIONS ==========
static const httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
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

// ========== SERVER CONTROL ==========
void webserver_start_with_callbacks(const void *cbs)
{
    (void)cbs;  // Not used - read-only webserver

    // HTTP server config
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.stack_size = 4096;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
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

void webserver_start(void)
{
    webserver_start_with_callbacks(NULL);
}

void webserver_stop(void)
{
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}
