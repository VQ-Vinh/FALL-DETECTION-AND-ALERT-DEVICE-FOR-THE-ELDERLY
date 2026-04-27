/**
 * @file telegram.h
 * @brief Header file cho Telegram Bot notifications
 *
 * Gửi thông báo đến Telegram chat:
 * - Startup message khi khởi động
 * - Fall alert khi phát hiện ngã
 * - SOS alert khi nhấn nút khẩn cấp
 * - Cancel alert khi hủy báo động
 *
 * Sử dụng queue-based async: message được queue, telegram_task gửi background
 */

#ifndef TELEGRAM_H
#define TELEGRAM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Khởi tạo với bot token và chat ID
void telegram_init(const char *bot_token, const char *chat_id);

// Gửi tin nhắn startup
void telegram_send_startup(void);

// Gửi cảnh báo ngã
void telegram_send_fall_alert(void);

// Gửi SOS (nút khẩn cấp)
void telegram_send_sos_alert(void);

// Gửi hủy báo động
void telegram_send_cancel_alert(void);

// Kiểm tra đã khởi tạo chưa
bool telegram_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif
