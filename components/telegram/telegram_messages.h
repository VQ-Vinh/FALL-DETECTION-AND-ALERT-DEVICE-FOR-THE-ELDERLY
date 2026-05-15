/**
 * @file telegram_messages.h
 * @brief Định nghĩa các tin nhắn Telegram - DỄ SỬA ĐỔI
 *
 * HƯỚNG DẪN SỬA ĐỔI:
 * Chỉ cần thay đổi nội dung các tin nhắn bên dưới theo nhu cầu.
 * Không cần thay đổi code khác.
 */

#ifndef TELEGRAM_MESSAGES_H
#define TELEGRAM_MESSAGES_H

// ========== TIN NHẮN TELEGRAM ==========
// Index: 0 = Startup, 1 = Fall Alert, 2 = SOS, 3 = Cancel

#define MSG_STARTUP \
    "Thiết bị đã khởi động.\n Sẵn sàng giám sát người thân!"

#define MSG_FALL_ALERT \
    "CẢNH BÁO: Phát hiện người bị ngã!\nVui lòng kiểm tra người thân."

#define MSG_SOS_ALERT \
    "SOS: Nút khẩn cấp được nhấn!\nVui lòng hỗ trợ ngay."

#define MSG_CANCEL_ALERT \
    "THÔNG BÁO: Cảnh báo đã được hủy."

// Danh sách tin nhắn (không cần sửa)
static const char* TELEGRAM_MESSAGES[] = {
    MSG_STARTUP,
    MSG_FALL_ALERT,
    MSG_SOS_ALERT,
    MSG_CANCEL_ALERT
};

#endif // TELEGRAM_MESSAGES_H