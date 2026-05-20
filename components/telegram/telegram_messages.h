/*
 * telegram_messages.h - Nội dung tin nhắn Telegram
 *
 * File DUY NHẤT bạn cần sửa để thay đổi nội dung tin nhắn.
 * Telegram API hỗ trợ UTF-8 nên có thể viết tiếng Việt thoải mái.
 * Dùng \n để xuống dòng. Độ dài tối đa 4096 ký tự.
 *
 * Có 4 loại tin nhắn, đánh index 0-3:
 *   MSG_STARTUP      (0): Gửi khi thiết bị khởi động xong
 *   MSG_FALL_ALERT   (1): Cảnh báo phát hiện té ngã
 *   MSG_SOS_ALERT    (2): Cảnh báo nhấn nút khẩn cấp
 *   MSG_CANCEL_ALERT (3): Báo đã hủy cảnh báo
 *
 * Muốn thêm loại tin nhắn mới thì phải sửa thêm telegram.c.
 */

#ifndef TELEGRAM_MESSAGES_H
#define TELEGRAM_MESSAGES_H

/* Gửi khi boot hoàn tất: xác nhận thiết bị đã online, sẵn sàng giám sát */
#define MSG_STARTUP \
    "Thiết bị đã khởi động.\n Sẵn sàng giám sát người thân!"

/* Gửi khi thuật toán phát hiện té ngã. Tin nhắn quan trọng nhất — cần IN HOA để gây chú ý */
#define MSG_FALL_ALERT \
    "CẢNH BÁO: Phát hiện người bị ngã!\nVui lòng kiểm tra người thân."

/* Gửi khi nhấn nút SOS vật lý. Khác fall_alert: đây là chủ động từ người dùng */
#define MSG_SOS_ALERT \
    "SOS: Nút khẩn cấp được nhấn!\nVui lòng hỗ trợ ngay."

/* Gửi sau fall_alert/sos_alert nếu người dùng xác nhận an toàn — trấn an người thân */
#define MSG_CANCEL_ALERT \
    "THÔNG BÁO: Cảnh báo đã được hủy."

#endif /* TELEGRAM_MESSAGES_H */
