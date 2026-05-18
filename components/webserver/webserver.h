/**
 * @file webserver.h
 * @brief Header file cho HTTP webserver (read-only)
 *
 * Định nghĩa cấu trúc dữ liệu sensor, cấu trúc trạng thái thiết bị,
 * và các khai báo extern cho cơ chế double-buffer được dùng chung
 * giữa task MPU6050 (writer) và HTTP handler (reader).
 *
 * Cơ chế double-buffer:
 * - g_sensor_buffers[2]: hai buffer, mỗi buffer chứa một sensor_data_t
 * - g_get_writer_index(): trả về chỉ số (0 hoặc 1) của buffer mà
 *   MPU6050 task đang ghi vào
 * - Bên reader (HTTP handler) luôn đọc buffer[1 - writer_index]
 *   để tránh xung đột dữ liệu giữa reader và writer
 *
 * Cung cấp:
 * - Dashboard HTML tại /
 * - API /api/data (GET) - dữ liệu cảm biến dạng JSON
 * - API /api/status (GET) - trạng thái thiết bị dạng JSON
 */

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_http_server.h"

// ========== CẤU TRÚC DỮ LIỆU ==========

/**
 * @brief Cấu trúc lưu dữ liệu cảm biến MPU6050 đã qua xử lý
 *
 * Đây là kiểu dữ liệu được ghi vào double-buffer. MPU6050 task
 * định kỳ cập nhật các trường này, và HTTP server đọc ra để
 * trả về cho client qua API JSON.
 */
typedef struct {
    float total_accel_g;    /**< Gia tốc tổng hợp (vector magnitude) tính bằng g (9.8 m/s²).
                                  Được tính từ ax, ay, az: sqrt(ax² + ay² + az²) */
    float total_gyro;       /**< Tốc độ góc tổng hợp từ con quay hồi chuyển, đơn vị deg/s.
                                  Thể hiện mức độ xoay/chuyển động đột ngột */
    float roll;             /**< Góc nghiêng Roll (quanh trục X), đơn vị độ.
                                  Dùng để xác định tư thế nằm/ngã của người dùng */
    float pitch;            /**< Góc nghiêng Pitch (quanh trục Y), đơn vị độ.
                                  Kết hợp với roll để phát hiện tư thế bất thường */
    bool data_ready;        /**< Cờ cho biết dữ liệu đã được cập nhật lần đầu tiên.
                                  false = chưa có dữ liệu (đang khởi tạo MPU6050),
                                  true = đã có dữ liệu hợp lệ từ cảm biến */
} sensor_data_t;

/**
 * @brief Cấu trúc trạng thái tổng quát của thiết bị
 *
 * Dùng cho API /api/status. Phản ánh tình trạng hoạt động
 * hiện tại của toàn bộ hệ thống.
 */
typedef struct {
    bool alert_active;      /**< Trạng thái cảnh báo: true nếu đang trong chế độ SOS
                                  (fall_state == FALL_SOS), false nếu bình thường */
    bool error_state;       /**< Cờ lỗi hệ thống: true nếu có lỗi nghiêm trọng
                                  (luôn được set là 0 trong phiên bản hiện tại) */
    bool wifi_connected;    /**< Kết nối WiFi: true nếu đã kết nối thành công,
                                  false nếu đang mất kết nối */
    uint32_t uptime_seconds; /**< Thời gian hoạt động của thiết bị kể từ khi khởi động,
                                  tính bằng giây (được tăng mỗi giây bởi system timer) */
} device_status_t;

// ========== EXTERNALS (từ main/CODE.c) ==========

/**
 * @brief Mảng double-buffer chứa dữ liệu cảm biến
 *
 * g_sensor_buffers[0] và g_sensor_buffers[1] là hai vùng đệm
 * được luân phiên sử dụng:
 * - Writer (MPU6050 task) ghi vào buffer[s_writer_index]
 * - Reader (HTTP handler/data_get_handler) đọc từ buffer[1 - s_writer_index]
 *
 * Việc dùng double-buffer giúp tránh race condition mà không cần
 * khóa mutex, vì reader và writer không bao giờ truy cập cùng
 * một buffer tại cùng một thời điểm.
 */
extern sensor_data_t g_sensor_buffers[2];

/**
 * @brief Lấy chỉ số của buffer mà writer (MPU6050 task) đang ghi vào
 *
 * @return uint8_t Giá trị 0 hoặc 1, cho biết buffer nào đang được
 *                  MPU6050 task cập nhật dữ liệu.
 *
 * Reader (HTTP handler) dùng: reader_index = 1 - g_get_writer_index()
 * để lấy chỉ số buffer an toàn để đọc.
 */
extern uint8_t g_get_writer_index(void);

// ========== HÀM API ==========

/**
 * @brief Khởi động HTTP server với callback (dành cho tương thích)
 *
 * Hiện tại không sử dụng tham số cbs vì webserver chỉ hoạt động
 * ở chế độ read-only (chỉ phục vụ GET, không nhận lệnh điều khiển).
 *
 * @param cbs Con trỏ tới cấu trúc callback (bị bỏ qua, để NULL)
 */
void webserver_start_with_callbacks(const void *cbs);

/**
 * @brief Khởi động HTTP server (phiên bản rút gọn, không callback)
 *
 * Gọi webserver_start_with_callbacks(NULL) để khởi tạo server
 * với cấu hình mặc định (port 80, tối đa 8 URI handlers).
 */
void webserver_start(void);

/**
 * @brief Dừng HTTP server và giải phóng tài nguyên
 *
 * Kiểm tra server handle trước khi dừng, tránh gọi httpd_stop
 * hai lần. Sau khi dừng, set server handle về NULL.
 */
void webserver_stop(void);

#endif
