/*
 * wifi.h - Cấu hình và API cho kết nối WiFi (Station mode)
 *
 * Các thông số dưới đây (SSID, PASS, retry, TX power) là hardcode
 * cho tiện — về sau nên chuyển sang menuconfig hoặc NVS.
 */

#ifndef WIFI_H
#define WIFI_H

#include "esp_netif.h"  /* esp_netif_t, IPSTR, IP2STR,... */

/* WiFi 2.4GHz — ESP32 không hỗ trợ 5GHz. SSID tối đa 32 ký tự. */
#define WIFI_SSID     "B3 1409"

/* Mật khẩu WiFi. Tối đa 64 ký tự (WPA2-PSK). */
#define WIFI_PASS     "05122003"

/*
 * Retry vô hạn, nhưng reset biến đếm sau mỗi WIFI_MAX_RETRY lần
 * (để log đỡ tràn và tránh overflow). Ở mức 10 là vừa.
 */
#define WIFI_MAX_RETRY 10

/* Khoảng cách giữa các lần retry. 10 giây — đủ để AP kịp hồi phục */
#define WIFI_RETRY_INTERVAL_MS 10000

/*
 * Công suất phát, đơn vị dBm*4 (theo API ESP-IDF).
 * 40 = 10 dBm — công suất vừa phải, tiết kiệm pin. ESP32 tối đa ~19.5 dBm.
 */
#define MAX_TX_POWER  40

/*
 * Khởi tạo WiFi Station: netif → event loop → WiFi driver → connect.
 * Gọi DUY NHẤT một lần trong app_main() trước các module khác.
 * Tự động retry và đồng bộ NTP sau khi có IP.
 */
void wifi_init_sta(void);

/*
 * Kiểm tra link-layer với AP (đã kết nối vật lý, nhưng chưa chắc có IP).
 * Các module khác gọi hàm này trước khi gửi/truyền dữ liệu qua mạng.
 */
bool wifi_is_connected(void);

#endif /* WIFI_H */
