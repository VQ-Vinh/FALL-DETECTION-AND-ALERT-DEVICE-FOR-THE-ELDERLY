/**
 * @file webserver.c
 * @brief HTTP server cung cấp dashboard và API
 *
 * ===================== TỔNG QUAN KIẾN TRÚC =====================
 *
 * File này triển khai một HTTP server nhúng trên ESP32 dùng thư viện
 * esp_http_server. Server phục vụ:
 *   1. Dashboard HTML tại đường dẫn gốc "/"
 *   2. API RESTful JSON tại "/api/data" và "/api/status"
 *
 * ===================== CƠ CHẾ DOUBLE-BUFFER =====================
 *
 * Để đọc dữ liệu cảm biến an toàn giữa các task RTOS, file sử dụng
 * cơ chế double-buffer (hai vùng đệm luân phiên):
 *
 *   - Writer (mpu6050_task): ghi dữ liệu mới từ MPU6050 vào
 *     buffer[s_writer_index], sau đó swap: s_writer_index = 1 - s_writer_index
 *
 *   - Reader (HTTP handler data_get_handler): đọc dữ liệu từ
 *     buffer[1 - g_get_writer_index()] — tức là buffer mà writer
 *     đã không chạm vào trong chu kỳ hiện tại
 *
 *   Ưu điểm: Reader và Writer KHÔNG BAO GIỜ truy cập cùng một buffer
 *   tại cùng một thời điểm ⇒ không cần dùng mutex/semaphore để đồng bộ,
 *   giảm độ trễ và tránh deadlock.
 *
 *   Luồng hoạt động:
 *   1. mpu6050_task ghi dữ liệu cảm biến vào buffer[s_writer_index]
 *   2. mpu6050_task swap s_writer_index (0→1 hoặc 1→0)
 *   3. HTTP handler đọc buffer[1 - s_writer_index] (buffer cũ, ổn định)
 *   4. Lặp lại → reader luôn thấy dữ liệu hoàn chỉnh, nhất quán
 *
 * ===================== VÌ SAO HTML ĐƯỢC NHÚNG DƯỚI DẠNG C-STRING? =====================
 *
 * Toàn bộ trang dashboard được khai báo là một hằng chuỗi C tĩnh
 * (static const char HTML_PAGE[]). Đây là lựa chọn thiết kế có chủ đích:
 *
 *   - Không sử dụng SPIFFS (SPI Flash File System): Dự án không cài đặt
 *     SPIFFS, do đó không thể lưu file HTML riêng trên flash.
 *
 *   - Single-binary deployment: Toàn bộ firmware là một binary duy nhất.
 *     Nhúng HTML vào code giúp triển khai đơn giản, không cần quan tâm
 *     đến partitioning, file system mount, hay checksum file.
 *
 *   - Tin cậy: Không lo file HTML bị hỏng, xóa nhầm, hoặc lỗi đọc flash.
 *
 *   - Dashboard tĩnh, nhẹ: Trang HTML chỉ ~2KB, không đáng kể so với
 *     dung lượng flash. Dữ liệu động được cập nhật qua JavaScript
 *     fetch() tới API, nên HTML hoàn toàn tĩnh là hợp lý.
 *
 * ===================== NHƯỢC ĐIỂM =====================
 *
 *   - Mỗi lần sửa dashboard phải biên dịch lại toàn bộ firmware.
 *   - Không phù hợp nếu dashboard lớn (hàng chục KB trở lên).
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

/**
 * @brief Tag dùng cho log hệ thống (ESP_LOGx)
 *
 * Được ESP-IDF logging framework sử dụng để phân loại message.
 * Ở đây dùng tag "WEB" để dễ lọc log khi debug.
 */
static const char *TAG = "WEB";

/**
 * @brief Handle của HTTP server (toàn cục tĩnh)
 *
 * Lưu handle trả về từ httpd_start() để dùng khi cần:
 *   - Đăng ký URI handler (httpd_register_uri_handler)
 *   - Dừng server (httpd_stop)
 *
 * Khởi tạo NULL, được gán khi server chạy thành công.
 */
static httpd_handle_t server = NULL;

/**
 * @brief Đếm thời gian hoạt động của hệ thống, đơn vị: giây
 *
 * Biến volatile vì có thể được ghi từ ISR (ngắt timer) hoặc từ
 * task khác. volatile báo cho compiler không tối ưu hóa (không
 * cache trong thanh ghi) mà phải đọc/ghi trực tiếp từ RAM mỗi lần.
 *
 * Được gửi qua JSON trong cả /api/data và /api/status.
 */
static volatile uint32_t system_uptime_sec = 0;

// ========== KHAI BÁO TRƯỚC (PROTOTYPES) ==========

static esp_err_t root_get_handler(httpd_req_t *req);      /**< Xử lý GET / */
static esp_err_t data_get_handler(httpd_req_t *req);      /**< Xử lý GET /api/data */
static esp_err_t status_get_handler(httpd_req_t *req);    /**< Xử lý GET /api/status */
static esp_err_t favicon_get_handler(httpd_req_t *req);   /**< Xử lý GET /favicon.ico */

// ========== HTML DASHBOARD (NHÚNG TRONG C-STRING) ==========

/**
 * @brief Trang dashboard HTML nhúng dưới dạng hằng chuỗi C
 *
 * Lý do nhúng: (xem giải thích chi tiết ở đầu file)
 *   - Dự án không dùng SPIFFS (hệ thống file trên flash)
 *   - Single-binary deployment
 *   - Độ tin cậy cao, tránh lỗi đọc file
 *
 * NỘI DUNG DASHBOARD:
 *   - Giao diện tối (dark theme) hiện đại với hai thẻ chính:
 *       + Status Card: WiFi (kết nối/ngắt) và Fall State (trạng thái ngã)
 *       + Sensor Grid: Acceleration (g), Gyroscope (deg/s), Roll (deg), Pitch (deg)
 *   - JavaScript ở cuối trang thực hiện polling API /api/data mỗi 100ms
 *     (có thể điều chỉnh bằng cách sửa số trong setInterval)
 *
 * GIẢI THÍCH MẢNG TRONG JAVASCRIPT:
 *   var SN = ['IDLE','FREEFALL','IMPACT','WAIT_LIE_DOWN','SOS'];
 *     → SN (State Names): tên hiển thị của 5 trạng thái fall detection.
 *       - 0: IDLE - Bình thường, không phát hiện ngã
 *       - 1: FREEFALL - Đang rơi tự do (gia tốc gần 0g)
 *       - 2: IMPACT - Va chạm (gia tốc tăng đột biến)
 *       - 3: WAIT_LIE_DOWN - Chờ nằm yên sau va chạm
 *       - 4: SOS - Khẩn cấp, đã xác nhận ngã, gửi cảnh báo
 *
 *   var SC = ['#4ecca3','#ffc107','#ff9800','#ff9800','#e94560'];
 *     → SC (State Colors): màu chữ tương ứng với mỗi trạng thái.
 *       - IDLE: xanh lá (#4ecca3) - an toàn
 *       - FREEFALL: vàng (#ffc107) - cảnh báo nhẹ
 *       - IMPACT: cam (#ff9800) - cảnh báo trung bình
 *       - WAIT_LIE_DOWN: cam (#ff9800) - đang chờ xác nhận
 *       - SOS: đỏ (#e94560) - nguy hiểm, cần can thiệp
 *
 *   var SL = ['#1a3a2e','#3a3520','#3a2e20','#3a2e20','#3a1a2e'];
 *     → SL (State Backgrounds): màu nền tương ứng mỗi trạng thái,
 *       phối màu tinh tế hơn so với chữ, tạo hiệu ứng đồ họa.
 *
 * HÀM updateData():
 *   - Gửi request GET tới /api/data dùng fetch() API (JavaScript hiện đại)
 *   - Chuyển response JSON thành object JavaScript
 *   - Cập nhật các thẻ <span> trong HTML với dữ liệu cảm biến
 *   - Dùng toFixed() để định dạng số thập phân đẹp hơn
 *   - Xử lý hiển thị WiFi: đổi màu xanh nếu connected, đỏ nếu disconnected
 *   - Xử lý hiển thị Fall State:
 *       + Lấy giá trị số fall_state từ JSON (0-4)
 *       + Tra mảng SN để lấy tên hiển thị
 *       + Tra mảng SC để lấy màu chữ
 *       + Tra mảng SL để lấy màu nền
 *       + Nếu giá trị ngoài khoảng 0-4, hiện 'UNKNOWN'
 *
 *   Gọi setInterval(updateData, 100) để lặp mỗi 100ms ⇒ dashboard
 *   "real-time" với độ trễ tối đa 100ms.
 */
static const char HTML_PAGE[] =
    "<!DOCTYPE html>"
    "<html lang='en'>"
    "<head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>Fall Detection Monitor</title>"
    "<style>"
    /* Reset và font chữ mặc định hệ thống (ưu tiên macOS, Windows, Linux) */
    "body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; "
    /* Nền tối xanh đen, chữ sáng, flexbox căn giữa màn hình */
    "background: #0f0f1a; color: #e0e0e0; margin: 0; padding: 0; min-height: 100vh; display: flex; "
    "align-items: center; justify-content: center; }"
    /* Container chính: rộng tối đa 480px (mobile-first), padding 20px */
    ".container { max-width: 480px; width: 100%; padding: 20px; }"
    /* Tiêu đề chính: chữ trắng đậm, căn giữa, cỡ 20px */
    "h1 { text-align: center; font-size: 20px; font-weight: 600; color: #ffffff; "
    "margin: 0 0 6px 0; letter-spacing: 0.5px; }"
    /* Phụ đề: chữ xám nhỏ 12px */
    ".subtitle { text-align: center; font-size: 12px; color: #666; margin-bottom: 20px; }"
    /* Grid 2 cột cho thẻ trạng thái (WiFi và Fall State) */
    ".status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 16px; }"
    /* Thẻ trạng thái: gradient nền xanh đậm, bo góc, border mờ */
    ".status-card { background: linear-gradient(135deg, #1a1a2e, #16213e); padding: 16px; "
    "border-radius: 12px; text-align: center; border: 1px solid #2a2a4a; }"
    /* Nhãn trong thẻ: chữ xám, in hoa, cách 1 chữ cái */
    ".status-card label { display: block; color: #888; font-size: 11px; margin-bottom: 6px; "
    "text-transform: uppercase; letter-spacing: 1px; }"
    /* Giá trị trong thẻ: chữ to (22px), đậm, hiệu ứng chuyển màu mượt */
    ".status-card .value { font-size: 22px; font-weight: 700; transition: color 0.3s; }"
    /* Grid 2 cột cho cảm biến (Accel, Gyro, Roll, Pitch) */
    ".sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }"
    /* Thẻ cảm biến: nền tối, viền mờ */
    ".sensor-card { background: #1a1a2e; padding: 14px; border-radius: 10px; "
    "border: 1px solid #2a2a4a; }"
    /* Nhãn cảm biến: chữ xám nhỏ hơn, in hoa */
    ".sensor-card label { display: block; color: #666; font-size: 10px; "
    "text-transform: uppercase; letter-spacing: 1px; margin-bottom: 4px; }"
    /* Giá trị cảm biến: chữ xanh dương sáng (#4fc3f7), to, đậm */
    ".sensor-card .value { font-size: 22px; font-weight: 700; color: #4fc3f7; }"
    /* Đơn vị đo: chữ xám nhỏ cạnh giá trị */
    ".sensor-card .unit { font-size: 12px; color: #555; margin-left: 2px; }"
    /* Badge (dạng pill) cho trạng thái, hiện chưa dùng trong HTML */
    ".badge { display: inline-block; padding: 4px 14px; border-radius: 20px; "
    "font-size: 13px; font-weight: 700; }"
    /* Tag nhỏ cho metadata */
    ".tag { display: inline-block; padding: 3px 10px; border-radius: 4px; "
    "font-size: 10px; font-weight: 600; background: #2a2a4a; color: #888; "
    "vertical-align: middle; margin-left: 6px; }"
    "</style>"
    "</head>"
    "<body>"
    "<div class='container'>"
    /* Tiêu đề dashboard */
    "<h1>FALL DETECTION DEVICE</h1>"
    "<div class='subtitle'>Monitoring System</div>"
    /* Grid trạng thái: WiFi + Fall State */
    "<div class='status-grid'>"
    "<div class='status-card'>"
    "<label>WiFi</label>"
    /* Giá trị WiFi được JS cập nhật: "Connected" (xanh) hoặc "Disconnected" (đỏ) */
    "<div class='value' id='wifi'>Connecting...</div>"
    "</div>"
    "<div class='status-card'>"
    "<label>Fall State</label>"
    /* Giá trị Fall State được JS cập nhật: IDLE/FREEFALL/IMPACT/WAIT_LIE_DOWN/SOS */
    "<div class='value' id='fall_state'>IDLE</div>"
    "</div>"
    "</div>"
    /* Grid cảm biến: 4 thẻ hiển thị accel, gyro, roll, pitch */
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
    /* ===== JAVASCRIPT: CẬP NHẬT DỮ LIỆU THEO THỜI GIAN THỰC ===== */
    "<script>"
    /*
     * SN - State Names: Mảng chứa tên hiển thị của 5 trạng thái fall detection.
     * Chỉ số mảng tương ứng với giá trị fall_state từ API (0-4).
     *   0: IDLE          - Bình thường (chữ xanh lá)
     *   1: FREEFALL      - Đang rơi tự do (chữ vàng)
     *   2: IMPACT        - Va chạm (chữ cam)
     *   3: WAIT_LIE_DOWN - Chờ xác nhận nằm yên (chữ cam)
     *   4: SOS           - Khẩn cấp, đã ngã (chữ đỏ)
     */
    "var SN=['IDLE','FREEFALL','IMPACT','WAIT_LIE_DOWN','SOS'];"

    /*
     * SC - State Colors: Mảng màu chữ cho từng trạng thái.
     * Phối màu theo mức độ nguy hiểm tăng dần.
     */
    "var SC=['#4ecca3','#ffc107','#ff9800','#ff9800','#e94560'];"

    /*
     * SL - State Backgrounds: Mảng màu nền cho từng trạng thái.
     * Tông màu tối hơn SC để làm nền, không lấn át chữ.
     */
    "var SL=['#1a3a2e','#3a3520','#3a2e20','#3a2e20','#3a1a2e'];"

    /*
     * Hàm updateData(): Gọi API /api/data, parse JSON, cập nhật DOM.
     * Được gọi bởi setInterval() mỗi 100ms.
     */
    "function updateData(){"
    /* fetch() là Promise-based, gửi GET request tới /api/data */
    "fetch('/api/data').then(function(r){return r.json()}).then(function(d){"
    /* d là object JSON từ server: {accel_g, gyro, roll, pitch, ...} */

    /* Lấy fall_state (mặc định 0 = IDLE nếu không tồn tại) */
    "var s=d.fall_state||0;"

    /* Cập nhật giá trị cảm biến với định dạng số thập phân phù hợp */
    "document.getElementById('accel_g').textContent=d.accel_g.toFixed(2);"
    "document.getElementById('gyro').textContent=d.gyro.toFixed(0);"
    "document.getElementById('roll').textContent=d.roll.toFixed(1);"
    "document.getElementById('pitch').textContent=d.pitch.toFixed(1);"

    /* Cập nhật trạng thái WiFi: đổi màu xanh nếu connected, đỏ nếu không */
    "var w=document.getElementById('wifi');"
    "if(d.wifi_connected){w.textContent='Connected';w.style.color='#4ecca3'}"
    "else{w.textContent='Disconnected';w.style.color='#e94560'}"

    /*
     * Cập nhật trạng thái Fall State:
     *   - textContent = SN[s] (tên trạng thái) hoặc 'UNKNOWN' nếu s ngoài 0-4
     *   - color = SC[s] (màu chữ) hoặc '#fff' (trắng) nếu không xác định
     *   - background = SL[s] (màu nền) hoặc '#1a1a2e' (mặc định) nếu không xác định
     *   - padding/borderRadius để tạo hiệu ứng badge nền bo góc
     */
    "var e=document.getElementById('fall_state');"
    "e.textContent=SN[s]||'UNKNOWN';e.style.color=SC[s]||'#fff';"
    "e.style.background=SL[s]||'#1a1a2e';"
    "e.style.padding='4px 0';e.style.borderRadius='8px'"
    "})}"

    /* Thiết lập polling: cứ 100ms gọi updateData() một lần */
    "setInterval(updateData,100);"
    "</script>"
    "</body>"
    "</html>";

// ========== HÀM TRỢ GIÚP (HELPERS) ==========

/**
 * @brief Thiết lập header CORS cho phản hồi HTTP
 *
 * CORS (Cross-Origin Resource Sharing) cho phép trình duyệt web
 * từ các origin khác nhau gọi API của thiết bị.
 *
 * Điều này cần thiết khi:
 *   - Dashboard được truy cập từ địa chỉ IP khác (CORS policy mặc
 *     định của trình duyệt chặn cross-origin requests)
 *   - Có ứng dụng web bên ngoài muốn đọc dữ liệu từ thiết bị
 *
 * Header được thiết lập:
 *   - Access-Control-Allow-Origin: *    — cho phép tất cả origin
 *   - Access-Control-Allow-Methods: GET, POST, OPTIONS
 *   - Access-Control-Allow-Headers: Content-Type
 *
 * @param req Con trỏ tới cấu trúc HTTP request (dùng để gán header)
 */
static void set_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

/**
 * @brief Gửi phản hồi JSON với CORS headers
 *
 * Hàm tiện ích: thiết lập Content-Type là application/json,
 * status 200 OK, thêm CORS headers, rồi gửi dữ liệu JSON.
 *
 * @param req  Con trỏ tới HTTP request
 * @param json Chuỗi JSON đã được tạo sẵn (cần kết thúc bằng null)
 * @param len  Độ dài chuỗi JSON (dùng strlen nếu không biết trước)
 * @return esp_err_t ESP_OK nếu thành công, mã lỗi nếu thất bại
 */
static esp_err_t send_json_response(httpd_req_t *req, const char *json, size_t len)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, "200 OK");
    set_cors_headers(req);
    return httpd_resp_send(req, json, len);
}

/**
 * @brief Gửi phản hồi HTML với CORS headers
 *
 * Hàm tiện ích: thiết lập Content-Type là text/html (UTF-8),
 * status 200 OK, thêm CORS headers, rồi gửi nội dung HTML.
 *
 * @param req  Con trỏ tới HTTP request
 * @param html Chuỗi HTML (kết thúc bằng null)
 * @param len  Độ dài chuỗi HTML
 * @return esp_err_t ESP_OK nếu thành công, mã lỗi nếu thất bại
 */
static esp_err_t send_html_response(httpd_req_t *req, const char *html, size_t len)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_status(req, "200 OK");
    set_cors_headers(req);
    return httpd_resp_send(req, html, len);
}

// ========== BỘ XỬ LÝ URI (URI HANDLERS) ==========

/**
 * @brief Xử lý GET / — Trả về trang dashboard HTML
 *
 * Gọi send_html_response() với HTML_PAGE (dashboard nhúng).
 * Đây là trang chính của thiết bị, cho phép người dùng
 * giám sát trạng thái và dữ liệu cảm biến qua trình duyệt.
 *
 * @param req Con trỏ tới HTTP request (không dùng tham số)
 * @return esp_err_t Kết quả gửi phản hồi
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /");
    return send_html_response(req, HTML_PAGE, strlen(HTML_PAGE));
}

/**
 * @brief Xử lý GET /api/data — Trả về dữ liệu cảm biến dạng JSON
 *
 * Đây là endpoint quan trọng nhất của webserver. Dashboard
 * JavaScript gọi endpoint này mỗi 100ms để cập nhật giao diện.
 *
 * QUY TRÌNH XỬ LÝ:
 *   1. Đọc dữ liệu cảm biến từ double-buffer (an toàn, không mutex)
 *   2. Lấy fall detection state từ module fall_detection
 *   3. Kiểm tra trạng thái WiFi
 *   4. Build chuỗi JSON với snprintf
 *   5. Gửi phản hồi JSON
 *
 * GIẢI THÍCH DOUBLE-BUFFER TRONG HÀM NÀY:
 *   - extern uint8_t g_get_writer_index(void) lấy chỉ số buffer
 *     mà MPU6050 task đang ghi
 *   - uint8_t reader_index = 1 - g_get_writer_index() → buffer
 *     an toàn để đọc (writer không chạm vào)
 *   - memcpy(&local_data, &g_sensor_buffers[reader_index], ...)
 *     copy toàn bộ struct sensor_data_t vào biến local. Dùng
 *     memcpy thay vì đọc trực tiếp từ buffer để tránh dữ liệu
 *     bị thay đổi giữa chừng (nếu writer swap ngay khi ta đang đọc)
 *
 * CÁC EXTERN FUNCTION:
 *   - fall_detection_get_state_internal(): fall state hiện tại (0-4)
 *   - fall_detection_get_max_tilt_internal(): góc nghiêng tối đa
 *   - fall_detection_get_filtered_accel_internal(): gia tốc đã lọc
 *   - wifi_is_connected(): kiểm tra kết nối WiFi
 *
 * ĐỊNH DẠNG JSON TRẢ VỀ:
 *   {
 *     "accel_g":       <float>   Gia tốc tổng hợp (g)
 *     "gyro":          <float>   Tốc độ góc (deg/s)
 *     "roll":          <float>   Góc Roll (độ)
 *     "pitch":         <float>   Góc Pitch (độ)
 *     "ready":         <int>     1 nếu dữ liệu cảm biến đã sẵn sàng
 *     "wifi_connected":<int>     1 nếu WiFi đã kết nối
 *     "alert_active":  <int>     1 nếu đang trong trạng thái SOS
 *     "uptime":        <long>    Thời gian hoạt động (giây)
 *     "fall_state":    <int>     Trạng thái ngã (0-4)
 *     "max_tilt":      <float>   Góc nghiêng tối đa (độ)
 *     "filt_accel":    <float>   Gia tốc sau bộ lọc (g)
 *   }
 *
 * @param req Con trỏ tới HTTP request
 * @return esp_err_t Kết quả gửi phản hồi JSON
 */
static esp_err_t data_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /api/data");

    /* Buffer cho chuỗi JSON (256 byte đủ cho dữ liệu hiện tại) */
    char buf[256];
    uint8_t fall_state = 0;
    float fall_max_tilt = 0.0f;
    float fall_filtered_accel = 0.0f;

    // ===== ĐỌC SENSOR DATA TỪ DOUBLE-BUFFER =====

    /*
     * extern từ CODE.c (module chính):
     *   g_get_writer_index() — trả về index buffer writer đang ghi (0 hoặc 1)
     */
    extern uint8_t g_get_writer_index(void);

    sensor_data_t local_data;  /* Bản sao local của sensor data */

    /*
     * reader_index = 1 - writer_index
     *   Nếu writer ghi vào buffer[0], reader đọc buffer[1]
     *   Nếu writer ghi vào buffer[1], reader đọc buffer[0]
     *   ⇒ Hai bên không bao giờ đụng nhau ⇒ không cần mutex
     */
    uint8_t reader_index = 1 - g_get_writer_index();

    /*
     * memcpy để copy toàn bộ struct một cách nguyên tử (trên thực tế
     * đây không phải là thao tác nguyên tử với struct lớn, nhưng vì
     * writer không chạm vào reader_index buffer nên dữ liệu an toàn)
     */
    memcpy(&local_data, &g_sensor_buffers[reader_index], sizeof(sensor_data_t));

    // ===== LẤY FALL DETECTION STATE =====

    /*
     * Các hàm extern từ module fall_detection:
     *   fall_detection_get_state_internal()     — trả về state máy trạng thái (0-4)
     *   fall_detection_get_max_tilt_internal()  — trả về góc nghiêng tối đa ghi nhận được
     *   fall_detection_get_filtered_accel_internal() — trả về gia tốc đã qua bộ lọc
     *
     * Dùng "internal" vì các hàm này được định nghĩa trong file fall_detection.c
     * và được export ra để webserver sử dụng.
     */
    extern uint8_t fall_detection_get_state_internal(void);
    extern float fall_detection_get_max_tilt_internal(void);
    extern float fall_detection_get_filtered_accel_internal(void);
    fall_state = fall_detection_get_state_internal();
    fall_max_tilt = fall_detection_get_max_tilt_internal();
    fall_filtered_accel = fall_detection_get_filtered_accel_internal();

    // ===== LẤY WIFI STATUS =====

    /*
     * Gọi trực tiếp hàm từ module WiFi để kiểm tra trạng thái kết nối.
     * wifi_is_connected() được định nghĩa trong component wifi.
     */
    extern bool wifi_is_connected(void);
    bool wifi_ok = wifi_is_connected();

    // ===== BUILD JSON RESPONSE =====

    /*
     * Định dạng JSON:
     *   - accel_g:        gia tốc tổng hợp, 2 số thập phân
     *   - gyro:           tốc độ góc, 0 số thập phân (làm tròn)
     *   - roll, pitch:    góc nghiêng, 1 số thập phân
     *   - ready:          bool -> int (data_ready)
     *   - wifi_connected: bool -> int
     *   - alert_active:   1 nếu fall_state == 4 (SOS)
     *   - uptime:         unsigned long (dùng %lu)
     *   - fall_state:     uint8_t -> int
     *   - max_tilt:       góc nghiêng tối đa, 1 số thập phân
     *   - filt_accel:     gia tốc đã lọc, 2 số thập phân
     */
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

/**
 * @brief Xử lý GET /api/status — Trả về trạng thái thiết bị dạng JSON
 *
 * Endpoint đơn giản cung cấp thông tin tổng quan về thiết bị:
 *   - wifi_connected: trạng thái kết nối WiFi
 *   - alert_active:   trạng thái cảnh báo (hiện luôn false trong code gốc)
 *   - error_state:    cờ lỗi (luôn 0, chưa triển khai)
 *   - uptime:         thời gian hoạt động (giây)
 *
 * So với /api/data, endpoint này nhẹ hơn, chỉ trả về 4 trường.
 *
 * JSON response format:
 *   {
 *     "wifi_connected": <int> 0 hoặc 1,
 *     "alert_active":   <int> 0 hoặc 1,
 *     "error_state":    <int> 0 (luôn 0),
 *     "uptime":         <long> thời gian hoạt động (giây)
 *   }
 *
 * @param req Con trỏ tới HTTP request
 * @return esp_err_t Kết quả gửi phản hồi JSON
 */
static esp_err_t status_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /api/status");

    /* Kiểm tra trạng thái WiFi (extern từ module wifi) */
    extern bool wifi_is_connected(void);
    bool wifi_ok = wifi_is_connected();

    /* Buffer trả về nhỏ gọn: 128 byte đủ */
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"wifi_connected\":%d,\"alert_active\":%d,\"error_state\":0,\"uptime\":%lu}",
        wifi_ok ? 1 : 0, 0, system_uptime_sec);

    return send_json_response(req, buf, strlen(buf));
}

/**
 * @brief Xử lý OPTIONS /api/* — CORS preflight request
 *
 * Trình duyệt gửi request OPTIONS (preflight) trước khi gửi
 * các request cross-origin "không đơn giản" (non-simple).
 *
 * Phản hồi 204 No Content với CORS headers cho trình duyệt biết
 * server cho phép cross-origin requests.
 *
 * Đăng ký cho cả /api/data và /api/* để bảo vệ toàn bộ API.
 *
 * @param req Con trỏ tới HTTP request (chứa URI)
 * @return esp_err_t ESP_OK
 */
static esp_err_t options_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "OPTIONS %s", req->uri);
    httpd_resp_set_status(req, "204 No Content");
    set_cors_headers(req);
    return httpd_resp_send(req, NULL, 0);
}

/**
 * @brief Xử lý GET /favicon.ico — Trả về 204 No Content
 *
 * Trình duyệt tự động gửi request /favicon.ico để lấy biểu tượng
 * tab. Dự án không có favicon, nên trả về 204 (No Content) để
 * tránh lỗi 404 trên console trình duyệt.
 *
 * @param req Con trỏ tới HTTP request
 * @return esp_err_t Kết quả gửi phản hồi 204
 */
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "GET /favicon.ico");
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

// ========== ĐỊNH NGHĨA URI HANDLER (CẤU TRÚC httpd_uri_t) ==========

/**
 * @brief Các cấu trúc httpd_uri_t định nghĩa đường dẫn, phương thức
 *        HTTP, và hàm xử lý tương ứng.
 *
 * Mỗi URI handler được đăng ký với server thông qua
 * httpd_register_uri_handler(). ESP-IDW sẽ so khớp URI và method,
 * sau đó gọi handler tương ứng.
 */

/** GET / → Trang dashboard HTML */
static const httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};

/** GET /api/data → Dữ liệu cảm biến JSON */
static const httpd_uri_t uri_api_data = {
    .uri = "/api/data",
    .method = HTTP_GET,
    .handler = data_get_handler
};

/** GET /api/status → Trạng thái thiết bị JSON */
static const httpd_uri_t uri_api_status = {
    .uri = "/api/status",
    .method = HTTP_GET,
    .handler = status_get_handler
};

/** GET /favicon.ico → 204 No Content */
static const httpd_uri_t uri_favicon = {
    .uri = "/favicon.ico",
    .method = HTTP_GET,
    .handler = favicon_get_handler
};

/** OPTIONS /api/data → CORS preflight */
static const httpd_uri_t uri_options_data = {
    .uri = "/api/data",
    .method = HTTP_OPTIONS,
    .handler = options_handler
};

/** OPTIONS /api/* → CORS preflight (wildcard, bảo vệ mọi API) */
static const httpd_uri_t uri_options_all = {
    .uri = "/api/*",
    .method = HTTP_OPTIONS,
    .handler = options_handler
};

// ========== ĐIỀU KHIỂN SERVER (SERVER CONTROL) ==========

/**
 * @brief Khởi động HTTP server với tham số callback (dành cho tương thích)
 *
 * Hàm này tạo HTTP server với cấu hình:
 *   - Port: 80
 *   - Số URI handler tối đa: 8
 *   - Stack size cho worker task: 4096 bytes
 *   - Timeout nhận/gửi: 10 giây
 *
 * Sau đó đăng ký 6 URI handlers:
 *   1. GET  /            — Dashboard HTML
 *   2. GET  /api/data    — JSON sensor data
 *   3. GET  /api/status  — JSON device status
 *   4. GET  /favicon.ico — 204 No Content
 *   5. OPTIONS /api/data — CORS preflight
 *   6. OPTIONS /api/*    — CORS preflight (wildcard)
 *
 * Tham số cbs bị bỏ qua (void cast) vì webserver ở chế độ read-only:
 * không hỗ trợ callback điều khiển từ client. Điều này giữ cho
 * thiết kế đơn giản và an toàn hơn.
 *
 * @param cbs Con trỏ tới cấu trúc callback (không dùng, để NULL)
 */
void webserver_start_with_callbacks(const void *cbs)
{
    (void)cbs;  /* Đánh dấu biến không dùng để tránh warning compiler */

    /*
     * Cấu hình mặc định cho HTTP server.
     * HTTPD_DEFAULT_CONFIG() là macro của esp_http_server cung cấp
     * các giá trị mặc định hợp lý.
     */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;              /* Cổng HTTP chuẩn */
    config.max_uri_handlers = 8;          /* Đủ cho 6 handler + dư */
    config.stack_size = 4096;             /* Stack cho task xử lý HTTP */
    config.recv_wait_timeout = 10;        /* Timeout nhận (giây) */
    config.send_wait_timeout = 10;        /* Timeout gửi (giây) */

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    /*
     * httpd_start() khởi tạo server và tạo task riêng để xử lý request.
     * Trả về ESP_OK nếu thành công.
     *
     * Nếu thành công, đăng ký tất cả URI handlers.
     * Nếu thất bại, log lỗi và set server = NULL.
     */
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

/**
 * @brief Khởi động HTTP server (phiên bản rút gọn)
 *
 * Wrapper đơn giản gọi webserver_start_with_callbacks(NULL).
 * Được sử dụng khi không cần callback (trường hợp phổ biến).
 */
void webserver_start(void)
{
    webserver_start_with_callbacks(NULL);
}

/**
 * @brief Dừng HTTP server
 *
 * Kiểm tra server != NULL trước khi gọi httpd_stop() để tránh
 * lỗi double-stop. Sau khi dừng, set server = NULL để đánh dấu
 * server đã đóng.
 */
void webserver_stop(void)
{
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}
