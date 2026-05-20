/*
 * webserver.h - HTTP server read-only cho dashboard và API JSON
 *
 * ===================== DOUBLE-BUFFER =====================
 * Hai buffer (g_sensor_buffers[2]) được luân phiên:
 *   - Writer (MPU6050 task) ghi vào buffer[writer_index]
 *   - Reader (HTTP handler) đọc từ buffer[1 - writer_index]
 *
 * Reader và writer KHÔNG BAO GIỜ chạm vào cùng một buffer
 * tại cùng một thời điểm → không cần mutex, tránh deadlock.
 *
 * Cung cấp:
 *   GET /          → Dashboard HTML
 *   GET /api/data  → JSON sensor data
 *   GET /api/status → JSON device status
 */

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_http_server.h"

// ========== CẤU TRÚC DỮ LIỆU ==========

/*
 * Dữ liệu cảm biến MPU6050 đã xử lý, dùng chung giữa MPU6050 task và HTTP handler
 * (qua double-buffer). Các trường này được gửi xuống client dưới dạng JSON.
 */
typedef struct {
    float total_accel_g;    /* Gia tốc tổng hợp sqrt(ax²+ay²+az²), đơn vị g. Cốt lõi để phát hiện rơi/va chạm */
    float total_gyro;       /* Tốc độ góc tổng hợp (deg/s). Giúp phân biệt ngã thật với nghiêng từ từ */
    float roll;             /* Góc roll quanh trục X (độ). Xác định tư thế nằm/ngã */
    float pitch;            /* Góc pitch quanh trục Y (độ). Kết hợp với roll để biết chính xác tư thế */
    bool data_ready;        /* false = MPU6050 chưa sẵn sàng (đang init). true = đã có data */
} sensor_data_t;

/* Trạng thái thiết bị — dùng cho API /api/status */
typedef struct {
    bool alert_active;      /* SOS đang kích hoạt? */
    bool error_state;       /* Có lỗi phần cứng? (hiện luôn false) */
    bool wifi_connected;    /* WiFi đã kết nối? */
    uint32_t uptime_seconds; /* Thời gian online (giây), đếm từ boot */
} device_status_t;

// ========== EXTERNALS (từ main/CODE.c) ==========

/* Hai buffer luân phiên — MPU6050 ghi vào buffer[writer], HTTP đọc buffer[1-writer] */
extern sensor_data_t g_sensor_buffers[2];

/* Trả về index (0/1) của buffer MPU6050 task đang ghi. HTTP handler dùng: 1 - index */
extern uint8_t g_get_writer_index(void);

// ========== HÀM API ==========

/* Khởi động server — có callback (hiện bỏ qua, để NULL), port 80, 8 URI handlers */
void webserver_start_with_callbacks(const void *cbs);
void webserver_start(void);  /* Gọi webserver_start_with_callbacks(NULL) */

/* Dừng server, kiểm tra handle tránh double-stop */
void webserver_stop(void);

#endif
