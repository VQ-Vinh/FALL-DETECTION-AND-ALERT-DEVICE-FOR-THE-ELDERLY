/*
 * telegram.h - API gửi thông báo Telegram bất đồng bộ
 *
 * Tất cả hàm telegram_send_*() đều non-blocking: chỉ đẩy message vào
 * FreeRTOS queue, background task riêng sẽ gửi thực sự qua HTTP.
 * Xem telegram.c để hiểu cơ chế queue chi tiết.
 *
 * Luồng dùng:
 *   boot → telegram_init(token, chat_id)
 *   sự kiện → telegram_send_fall_alert() / telegram_send_sos_alert() / ...
 */

#ifndef TELEGRAM_H
#define TELEGRAM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * telegram_init - Khởi tạo module Telegram.
 *
 * Lưu token & chat_id, tạo queue (4 phần tử) và background task.
 * bot_token lấy từ @BotFather trên Telegram.
 * chat_id có thể lấy từ @userinfobot.
 * Gọi DUY NHẤT một lần ở startup.
 */
void telegram_init(const char *bot_token, const char *chat_id);

/* Non-blocking: push message vào queue, gửi thực tế qua background task */
void telegram_send_startup(void);    /* Gửi khi boot xong */
void telegram_send_fall_alert(void); /* Gửi khi phát hiện ngã (từ MPU6050) */
void telegram_send_sos_alert(void);  /* Gửi khi nhấn nút SOS */
void telegram_send_cancel_alert(void); /* Gửi khi hủy báo động */

/* true nếu telegram_init() đã gọi thành công. Kiểm tra trước khi send. */
bool telegram_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEGRAM_H */
