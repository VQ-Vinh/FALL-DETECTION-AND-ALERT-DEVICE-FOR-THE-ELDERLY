/**
 * @file wifi.h
 * @brief Header file cho module WiFi Station
 */

#ifndef WIFI_H
#define WIFI_H

#include "esp_netif.h"

// Thông số WiFi (hardcoded - nên chuyển sang sdkconfig)
#define WIFI_SSID     "B3 1409"
#define WIFI_PASS     "05122003"
#define WIFI_MAX_RETRY 10        // Số lần retry mỗi batch
#define WIFI_RETRY_INTERVAL_MS 10000  // Retry mỗi 10s
#define MAX_TX_POWER  40         // Công suất phát tối đa (dBm)

// Khởi tạo WiFi Station mode
void wifi_init_sta(void);

// Kiểm tra trạng thái kết nối
bool wifi_is_connected(void);

#endif
