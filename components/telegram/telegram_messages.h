/**
 * @file telegram_messages.h
 * @brief Định nghĩa nội dung các tin nhắn Telegram - NGƯỜI DÙNG CÓ THỂ CHỈNH SỬA
 *
 * ===================== HƯỚNG DẪN SỬ DỤNG =====================
 * Đây là file DUY NHẤT bạn cần sửa để thay đổi nội dung tin nhắn Telegram.
 * KHÔNG cần sửa bất kỳ file nào khác (telegram.c, telegram.h).
 *
 * Cách sửa: Thay đổi chuỗi trong #define thành nội dung bạn muốn.
 *
 * Ví dụ:
 *   #define MSG_STARTUP "May da khoi dong xong!"
 *
 * Lưu ý:
 *   - Nội dung có thể dùng tiếng Việt không dấu (do Telegram API hỗ trợ UTF-8)
 *   - Có thể dùng \n để xuống dòng
 *   - Độ dài tối đa: 4096 ký tự (theo giới hạn Telegram API)
 *   - Nếu muốn thêm loại tin nhắn mới: cần sửa thêm telegram.c
 *
 * ===================== CÁC LOẠI TIN NHẮN =====================
 *   MSG_STARTUP      (Index 0): Tin nhắn khởi động - gửi khi thiết bị bật
 *   MSG_FALL_ALERT   (Index 1): Cảnh báo ngã - gửi khi phát hiện té ngã
 *   MSG_SOS_ALERT    (Index 2): Cảnh báo SOS - gửi khi nhấn nút khẩn cấp
 *   MSG_CANCEL_ALERT (Index 3): Hủy báo động - gửi khi người dùng an toàn
 */

#ifndef TELEGRAM_MESSAGES_H
#define TELEGRAM_MESSAGES_H

/*
 * MSG_STARTUP - Tin nhắn khởi động.
 * Gửi khi thiết bị hoàn tất quá trình khởi động (WiFi, sensor, Telegram).
 * Mục đích: xác nhận thiết bị đang hoạt động và sẵn sàng giám sát.
 */
#define MSG_STARTUP \
    "Thiết bị đã khởi động.\n Sẵn sàng giám sát người thân!"

/*
 * MSG_FALL_ALERT - Cảnh báo phát hiện người bị ngã.
 * Gửi khi thuật toán phát hiện té ngã từ MPU6050.
 * Đây là tin nhắn quan trọng nhất, cần gây chú ý cho người nhận.
 * Nên dùng chữ IN HOA hoặc dấu cảnh báo (!) ở đầu.
 */
#define MSG_FALL_ALERT \
    "CẢNH BÁO: Phát hiện người bị ngã!\nVui lòng kiểm tra người thân."

/*
 * MSG_SOS_ALERT - Cảnh báo SOS từ nút khẩn cấp.
 * Gửi khi người dùng nhấn nút SOS vật lý.
 * Khác với fall_alert: đây là chủ động từ người dùng, không phải tự động.
 */
#define MSG_SOS_ALERT \
    "SOS: Nút khẩn cấp được nhấn!\nVui lòng hỗ trợ ngay."

/*
 * MSG_CANCEL_ALERT - Thông báo hủy báo động.
 * Gửi sau fall_alert hoặc sos_alert nếu người dùng xác nhận an toàn.
 * Mục đích: trấn an người thân rằng không có nguy hiểm thực sự.
 */
#define MSG_CANCEL_ALERT \
    "THÔNG BÁO: Cảnh báo đã được hủy."

#endif /* TELEGRAM_MESSAGES_H */
