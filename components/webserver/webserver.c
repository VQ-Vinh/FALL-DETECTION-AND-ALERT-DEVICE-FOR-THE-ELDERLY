/*
 * webserver.c - HTTP server: dashboard HTML + REST API JSON
 *
 * Server nhúng trên ESP32 dùng esp_http_server, phục vụ:
 *   GET /              → Dashboard HTML (nhúng trong C-string)
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
 * ===================== HTML NHÚNG TRONG C-STRING =====================
 * Dashboard HTML được khai báo static const char HTML_PAGE[]. Lý do:
 *   - Không dùng SPIFFS (project không mount file system)
 *   - Single-binary: một firmware duy nhất, không lo file hỏng/thất lạc
 *   - Dashboard ~2KB, nhẹ, dữ liệu động lấy qua JS fetch() API
 *
 * Nhược điểm: mỗi lần sửa dashboard phải rebuild firmware.
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

static httpd_handle_t server = NULL;  /* Handle HTTP server — dùng để stop & register handlers */

/*
 * volatile vì uptime có thể được ghi từ ISR hoặc task khác — compiler
 * không cache trong thanh ghi mà đọc/ghi RAM mỗi lần.
 */
static volatile uint32_t system_uptime_sec = 0;

/* Prototypes */
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t data_get_handler(httpd_req_t *req);
static esp_err_t status_get_handler(httpd_req_t *req);
static esp_err_t favicon_get_handler(httpd_req_t *req);

/* ===================== HTML DASHBOARD ===================== */
/*
 * Dashboard dark theme với 2 thẻ Status (WiFi + Fall State) và 4 thẻ Sensor
 * (Accel, Gyro, Roll, Pitch). JS cuối trang poll /api/data mỗi 100ms.
 *
 * Giải thích mảng JavaScript:
 *   SN (State Names) — Tên 5 trạng thái fall detection:
 *      0: IDLE (bình thường), 1: FREEFALL (rơi), 2: IMPACT (va chạm),
 *      3: WAIT_LIE_DOWN (chờ nằm yên), 4: SOS (khẩn cấp)
 *
 *   SC (State Colors) — Màu chữ theo mức nguy hiểm tăng dần:
 *      IDLE xanh (#4ecca3) → FREEFALL vàng (#ffc107) → IMPACT cam (#ff9800)
 *      → WAIT_LIE_DOWN cam (#ff9800) → SOS đỏ (#e94560)
 *
 *   SL (State Backgrounds) — Màu nền tối hơn SC, không lấn át chữ.
 *
 *   updateData() — fetch /api/data → parse JSON → update DOM:
 *     1. Cập nhật accel_g (toFixed 2), gyro (0), roll/pitch (1) decimals
 *     2. WiFi: xanh nếu connected, đỏ nếu disconnected
 *     3. Fall State: tra mảng SN/SC/SL theo giá trị fall_state (0-4),
 *        nếu ngoài range → 'UNKNOWN' với màu mặc định.
 *     4. setInterval(updateData, 100) → dashboard real-time.
 */
static const char HTML_PAGE[] =
    "<!DOCTYPE html>"
    "<html lang='en'>"
    "<head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>Fall Detection Monitor</title>"
    "<style>"
    /* Font hệ thống ưu tiên macOS/Windows/Linux */
    "body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; "
    /* Nền tối, chữ sáng, flex căn giữa */
    "background: #0f0f1a; color: #e0e0e0; margin: 0; padding: 0; min-height: 100vh; display: flex; "
    "align-items: center; justify-content: center; }"
    /* Mobile-first: container tối đa 480px */
    ".container { max-width: 480px; width: 100%; padding: 20px; }"
    "h1 { text-align: center; font-size: 20px; font-weight: 600; color: #ffffff; "
    "margin: 0 0 6px 0; letter-spacing: 0.5px; }"
    ".subtitle { text-align: center; font-size: 12px; color: #666; margin-bottom: 20px; }"
    /* Grid 2 cột cho thẻ WiFi + Fall State */
    ".status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 16px; }"
    /* Gradient nền xanh đậm, bo góc, border mờ */
    ".status-card { background: linear-gradient(135deg, #1a1a2e, #16213e); padding: 16px; "
    "border-radius: 12px; text-align: center; border: 1px solid #2a2a4a; }"
    ".status-card label { display: block; color: #888; font-size: 11px; margin-bottom: 6px; "
    "text-transform: uppercase; letter-spacing: 1px; }"
    /* Giá trị to, đậm, hiệu ứng chuyển màu mượt khi fall_state đổi */
    ".status-card .value { font-size: 22px; font-weight: 700; transition: color 0.3s; }"
    /* Grid 2 cột cho 4 cảm biến */
    ".sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }"
    ".sensor-card { background: #1a1a2e; padding: 14px; border-radius: 10px; "
    "border: 1px solid #2a2a4a; }"
    ".sensor-card label { display: block; color: #666; font-size: 10px; "
    "text-transform: uppercase; letter-spacing: 1px; margin-bottom: 4px; }"
    /* Số liệu cảm biến màu xanh dương sáng */
    ".sensor-card .value { font-size: 22px; font-weight: 700; color: #4fc3f7; }"
    ".sensor-card .unit { font-size: 12px; color: #555; margin-left: 2px; }"
    /* Badge pill — dự phòng, chưa dùng */
    ".badge { display: inline-block; padding: 4px 14px; border-radius: 20px; "
    "font-size: 13px; font-weight: 700; }"
    ".tag { display: inline-block; padding: 3px 10px; border-radius: 4px; "
    "font-size: 10px; font-weight: 600; background: #2a2a4a; color: #888; "
    "vertical-align: middle; margin-left: 6px; }"
    "</style>"
    "</head>"
    "<body>"
    "<div class='container'>"
    "<h1>FALL DETECTION DEVICE</h1>"
    "<div class='subtitle'>Monitoring System</div>"
    /* 2 thẻ trạng thái: WiFi + Fall State. JS cập nhật nội dung + màu */
    "<div class='status-grid'>"
    "<div class='status-card'>"
    "<label>WiFi</label>"
    "<div class='value' id='wifi'>Connecting...</div>"
    "</div>"
    "<div class='status-card'>"
    "<label>Fall State</label>"
    "<div class='value' id='fall_state'>IDLE</div>"
    "</div>"
    "</div>"
    /* 4 thẻ cảm biến: Acceleration, Gyroscope, Roll, Pitch */
    "<div class='sensor-grid'>"
    "<div class='sensor-card'>"
    "<label>Acceleration</label>"
    "<div class='value'><span id='accel_g'>0.00</span><span class='unit'>g</span></div>"
    "</div>"
    "<div class='sensor-card'>"
    "<label>Gyroscope</label>"
    "<div class='value'><span id='gyro'>0.00</span><span class='unit'>deg/s</span></div>"
    "</div>"
    "<div class='sensor-card'>"
    "<label>Roll</label>"
    "<div class='value'><span id='roll'>0.0</span><span class='unit'>deg</span></div>"
    "</div>"
    "<div class='sensor-card'>"
    "<label>Pitch</label>"
    "<div class='value'><span id='pitch'>0.0</span><span class='unit'>deg</span></div>"
    "</div>"
    "</div>"
    "</div>"
    /* JS: poll /api/data mỗi 100ms → cập nhật giá trị + màu sắc */
    "<script>"
    /* SN = State Names: index 0-4 tương ứng 5 trạng thái fall detection */
    "var SN=['IDLE','FREEFALL','IMPACT','WAIT_LIE_DOWN','SOS'];"
    /* SC = State Colors: màu chữ theo mức nguy hiểm tăng dần (xanh→vàng→cam→đỏ) */
    "var SC=['#4ecca3','#ffc107','#ff9800','#ff9800','#e94560'];"
    /* SL = State Backgrounds: màu nền tối hơn SC, không lấn át chữ */
    "var SL=['#1a3a2e','#3a3520','#3a2e20','#3a2e20','#3a1a2e'];"

    "function updateData(){"
    "fetch('/api/data').then(function(r){return r.json()}).then(function(d){"
    "var s=d.fall_state||0;"
    /* Cập nhật 4 giá trị cảm biến với số thập phân phù hợp */
    "document.getElementById('accel_g').textContent=d.accel_g.toFixed(2);"
    "document.getElementById('gyro').textContent=d.gyro.toFixed(0);"
    "document.getElementById('roll').textContent=d.roll.toFixed(1);"
    "document.getElementById('pitch').textContent=d.pitch.toFixed(1);"
    /* WiFi: xanh nếu connected, đỏ nếu không */
    "var w=document.getElementById('wifi');"
    "if(d.wifi_connected){w.textContent='Connected';w.style.color='#4ecca3'}"
    "else{w.textContent='Disconnected';w.style.color='#e94560'}"
    /* Fall State: tra SN (tên), SC (màu chữ), SL (nền) — fallback nếu ngoài range */
    "var e=document.getElementById('fall_state');"
    "e.textContent=SN[s]||'UNKNOWN';e.style.color=SC[s]||'#fff';"
    "e.style.background=SL[s]||'#1a1a2e';"
    "e.style.padding='4px 0';e.style.borderRadius='8px'"
    "})}"
    /* Poll mỗi 100ms — dashboard real-time với độ trễ tối đa 100ms */
    "setInterval(updateData,100);"
    "</script>"
    "</body>"
    "</html>";

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

/* Helper: gửi HTML 200 + CORS headers */
static esp_err_t send_html_response(httpd_req_t *req, const char *html, size_t len)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_status(req, "200 OK");
    set_cors_headers(req);
    return httpd_resp_send(req, html, len);
}

// ========== BỘ XỬ LÝ URI (URI HANDLERS) ==========

/* GET / — trả về dashboard HTML nhúng */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /");
    return send_html_response(req, HTML_PAGE, strlen(HTML_PAGE));
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
