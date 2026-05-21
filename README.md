# Phát Hiện Té Ngã Cho Người Cao Tuổi

Thiết bị đeo thắt lưng giúp phát hiện khi người già bị té ngã, tự động hú còi và gửi tin nhắn Telegram đến người thân. Xây dựng trên ESP32-C3 và MPU6050.

## Tính năng

- Phát hiện té ngã qua 3 giai đoạn: rơi → va chạm → nằm yên
- Còi báo + LED nhấp nháy khi có ngã
- Gửi tin nhắn Telegram (Bot API) khi phát hiện ngã
- Dashboard web theo dõi thông số cảm biến
- Nút SOS khẩn cấp, nút Cancel hủy báo giả
- Tự động tắt báo sau 30 giây

## Phần cứng cần

- ESP32-C3 (RISC-V, WiFi)
- MPU6050 (accel + gyro)
- Buzzer, LED, 2 nút nhấn

## Sơ đồ chân

| GPIO | Kết nối |
|------|---------|
| 0 | Buzzer |
| 1 | LED |
| 5 | Nút Cancel (giữ 3s hủy báo) |
| 6 | Nút SOS (giữ 3s kích hoạt) |
| 8 | I2C SDA → MPU6050 |
| 9 | I2C SCL → MPU6050 |

## Build

```bash
idf.py build
idf.py flash
idf.py monitor
```

Cấu hình WiFi và Telegram token qua `idf.py menuconfig`.

## Cách hoạt động

Thiết bị đọc MPU6050 100 lần/giây, chạy state machine 5 trạng thái để phân biệt ngã thật với các chuyển động khác:

- IDLE: bình thường
- FREEFALL: accel dưới 0.5g (đang rơi)
- IMPACT: accel trên 2g (va chạm)
- WAIT_LIE_DOWN: chờ 5s xác nhận nằm yên
- SOS: xác nhận té ngã, báo động

## API web

| Đường dẫn | Trả về |
|-----------|--------|
| `/` | Dashboard HTML |
| `/api/data` | JSON (accel, gyro, roll, pitch, fall_state, ...) |
| `/api/status` | JSON (wifi, alert, uptime) |

## License

MIT
