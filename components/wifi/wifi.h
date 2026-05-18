/**
 * @file wifi.h
 * @brief Header file cho module WiFi Station - cấu hình và khởi tạo kết nối
 *
 * ===================== MỤC ĐÍCH =====================
 * Module này quản lý kết nối WiFi ở chế độ Station (ESP32 là client).
 * Cung cấp API khởi tạo và kiểm tra trạng thái kết nối.
 * Các hằng số cấu hình (SSID, PASS, retry) được định nghĩa tại đây.
 *
 * ===================== CẤU HÌNH =====================
 * File này chứa các thông số cần chỉnh sửa khi triển khai:
 * - WIFI_SSID: Tên WiFi (2.4GHz) cần kết nối
 * - WIFI_PASS: Mật khẩu WiFi
 * - WIFI_MAX_RETRY: Số lần retry trong mỗi chu kỳ
 * - WIFI_RETRY_INTERVAL_MS: Khoảng cách giữa các lần retry
 * - MAX_TX_POWER: Công suất phát WiFi
 *
 * Lưu ý: Các thông số này đang hardcode - nên chuyển sang
 * menuconfig (sdkconfig) hoặc NVS trong tương lai.
 */

#ifndef WIFI_H
#define WIFI_H

#include "esp_netif.h"  /* esp_netif_t, IPSTR, IP2STR,... */

/*
 * WIFI_SSID - Tên của mạng WiFi (Access Point) cần kết nối.
 * Chỉ hỗ trợ băng tần 2.4GHz (ESP32 không hỗ trợ 5GHz).
 * Tối đa 32 ký tự.
 */
#define WIFI_SSID     "B3 1409"

/*
 * WIFI_PASS - Mật khẩu của mạng WiFi.
 * Tối đa 64 ký tự (WPA2-PSK).
 */
#define WIFI_PASS     "05122003"

/*
 * WIFI_MAX_RETRY - Số lần thử kết nối trong mỗi "batch" retry.
 * Khi đạt ngưỡng này, biến đếm reset về 0 và tiếp tục retry (vô hạn).
 * Không phải là giới hạn cứng - thiết bị sẽ retry mãi mãi.
 * Giá trị hiện tại: 10 lần trước khi reset.
 */
#define WIFI_MAX_RETRY 10

/*
 * WIFI_RETRY_INTERVAL_MS - Thời gian chờ giữa các lần retry (milliseconds).
 * Khi mất kết nối, timer one-shot được tạo với thời gian này.
 * Sau mỗi WIFI_RETRY_INTERVAL_MS, một lần thử kết nối mới được thực hiện.
 * Giá trị hiện tại: 10000ms = 10 giây.
 */
#define WIFI_RETRY_INTERVAL_MS 10000

/*
 * MAX_TX_POWER - Công suất phát tối đa (dBm * 4).
 * ESP32 có công suất phát tối đa thực tế khoảng 19.5 dBm.
 * Giá trị 40 (trong API ESP-IDF) tương ứng với 10 dBm (công suất vừa phải).
 * Công suất cao hơn -> phạm vi xa hơn nhưng hao pin hơn.
 */
#define MAX_TX_POWER  40

/*
 * wifi_init_sta - Khởi tạo WiFi ở chế độ Station.
 *
 * Quy trình:
 *   - Khởi tạo netif, event loop, WiFi driver
 *   - Đăng ký event handler (xử lý kết nối, mất kết nối, GOT_IP)
 *   - Cấu hình SSID/PASS từ #define trong file này
 *   - Bắt đầu kết nối (tự động reconnect nếu mất kết nối)
 *   - Đồng bộ thời gian NTP sau khi có IP
 *
 * Gọi hàm này MỘT LẦN duy nhất trong app_main() trước các module khác.
 */
void wifi_init_sta(void);

/*
 * wifi_is_connected - Kiểm tra trạng thái kết nối WiFi.
 *
 * @return: true nếu ESP32 đã kết nối và có link layer với AP,
 *          false nếu chưa kết nối hoặc đã mất kết nối.
 *
 * Lưu ý: Trạng thái "connected" ở đây là link layer (L2),
 * có thể chưa có IP (DHCP). Để kiểm tra có thể truy cập mạng,
 * cần kiểm tra thêm địa chỉ IP (nếu cần).
 */
bool wifi_is_connected(void);

#endif /* WIFI_H */
