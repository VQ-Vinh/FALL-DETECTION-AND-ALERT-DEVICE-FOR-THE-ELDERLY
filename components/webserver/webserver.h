/**
 * @file webserver.h
 * @brief Header file cho HTTP webserver
 *
 * Cung cấp:
 * - Dashboard HTML tại /
 * - API /api/data (GET) - dữ liệu cảm biến
 * - API /api/status (GET) - trạng thái thiết bị
 * - API /api/alert/stop (POST) - dừng báo động
 * - API /api/reset (POST) - reset hệ thống
 */

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_http_server.h"

// ========== CẤU TRÚC DỮ LIỆU ==========
typedef struct {
    float total_accel;      // m/s²
    float total_accel_g;    // g
    float total_gyro;       // deg/s
    float roll;             // degrees
    float pitch;            // degrees
    bool data_ready;
} sensor_data_t;

typedef struct {
    bool alert_active;
    bool error_state;
    bool wifi_connected;
    uint32_t uptime_seconds;
} device_status_t;

// ========== EXTERNALS (từ main/CODE.c) ==========
extern sensor_data_t g_sensor_buffers[2];
extern uint8_t g_get_writer_index(void);

// ========== CALLBACKS ==========
typedef void (*alert_cancel_cb_t)(void);
typedef void (*system_reset_cb_t)(void);
typedef bool (*wifi_status_cb_t)(void);

typedef struct {
    alert_cancel_cb_t cancel_alert;
    system_reset_cb_t reset_system;
    wifi_status_cb_t get_wifi_status;
} webserver_control_cb_t;

// ========== HÀM API ==========
void webserver_start_with_callbacks(const webserver_control_cb_t *cbs);
void webserver_start(void);
void webserver_stop(void);

#endif
