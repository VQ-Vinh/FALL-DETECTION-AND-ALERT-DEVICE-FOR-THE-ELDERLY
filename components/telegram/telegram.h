/**
 * @file telegram.h
 * @brief Header file cho module Telegram Bot - gửi thông báo qua Telegram
 *
 * ===================== MỤC ĐÍCH =====================
 * Module này cung cấp API để gửi thông báo đến một chat Telegram cụ thể
 * thông qua Telegram Bot API (https://core.telegram.org/bots/api).
 * Sử dụng cơ chế queue-based async để không block các task khác.
 *
 * ===================== CÁC LOẠI THÔNG BÁO =====================
 * - MSG_STARTUP:    Gửi khi thiết bị khởi động (xác nhận hệ thống hoạt động)
 * - MSG_FALL_ALERT: Cảnh báo khi phát hiện người bị ngã (khẩn cấp)
 * - MSG_SOS_ALERT:  Cảnh báo khi nhấn nút khẩn cấp SOS
 * - MSG_CANCEL_ALERT: Thông báo khi hủy báo động (người dùng an toàn)
 *
 * ===================== CÁCH SỬ DỤNG =====================
 * 1. Gọi telegram_init(bot_token, chat_id) ở startup
 * 2. Gọi telegram_send_*() ở các sự kiện tương ứng
 * 3. Hàm không blocking - chỉ đẩy message vào queue
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
 * @param bot_token: Token Bot từ BotFather (tạo bot tại @BotFather trên Telegram).
 *                   Định dạng: "1234567890:ABCdefGHIjklMNOpqrsTUVwxyz-1234567"
 * @param chat_id:   ID của chat/người nhận. Có thể lấy từ @userinfobot hoặc
 *                   bằng cách gọi getUpdates sau khi nhắn tin cho bot.
 *                   Định dạng: "1234567890" (số nguyên dạng chuỗi)
 *
 * Hành vi:
 *   - Lưu token và chat_id vào bộ nhớ static
 *   - Tạo FreeRTOS queue (4 phần tử) và background task (telegram_task)
 *   - Gọi hàm này một lần duy nhất ở startup
 */
void telegram_init(const char *bot_token, const char *chat_id);

/*
 * telegram_send_startup - Gửi tin nhắn khởi động.
 * Dùng để xác nhận thiết bị đã khởi động thành công.
 * Non-blocking: message được đẩy vào queue, gửi sau trong background task.
 */
void telegram_send_startup(void);

/*
 * telegram_send_fall_alert - Gửi cảnh báo phát hiện ngã.
 * Gọi từ module MPU6050 khi phát hiện người bị ngã.
 * Non-blocking: message được đẩy vào queue, gửi sau trong background task.
 */
void telegram_send_fall_alert(void);

/*
 * telegram_send_sos_alert - Gửi cảnh báo SOS.
 * Gọi khi nút khẩn cấp được nhấn.
 * Non-blocking: message được đẩy vào queue, gửi sau trong background task.
 */
void telegram_send_sos_alert(void);

/*
 * telegram_send_cancel_alert - Gửi thông báo hủy báo động.
 * Gọi khi người dùng xác nhận an toàn (hủy báo động sai).
 * Non-blocking: message được đẩy vào queue, gửi sau trong background task.
 */
void telegram_send_cancel_alert(void);

/*
 * telegram_is_initialized - Kiểm tra trạng thái khởi tạo.
 *
 * @return: true nếu telegram_init() đã được gọi và khởi tạo thành công,
 *          false nếu chưa khởi tạo.
 *
 * Dùng để kiểm tra trước khi gọi các hàm telegram_send_*().
 */
bool telegram_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEGRAM_H */
