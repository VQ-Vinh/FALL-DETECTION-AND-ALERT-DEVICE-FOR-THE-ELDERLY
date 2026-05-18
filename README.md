# Hệ Thống Phát Hiện Té Ngã Cho Người Cao Tuổi

Thiết bị IoT phát hiện té ngã dành cho người già, sử dụng cảm biến MPU6050 (6-axis IMU) trên nền ESP32-C3 với ESP-IDF framework. Khi phát hiện té ngã, thiết bị kích hoạt còi báo + LED nhấp nháy, gửi thông báo Telegram, và cung cấp dashboard web thời gian thực.

## Tính Năng Chính

| Tính năng | Mô tả |
|-----------|-------|
| **Phát hiện té ngã** | State machine 5 trạng thái: IDLE → FREEFALL → IMPACT → WAIT_LIE_DOWN → SOS |
| **Cảnh báo tại chỗ** | Buzzer + LED nhấp nháy (tự động tắt sau 30s) |
| **Dashboard Web** | Theo dõi cảm biến thời gian thực qua HTTP (polling 100ms) |
| **Telegram Bot** | Thông báo đẩy đến điện thoại qua Telegram API |
| **Hiệu chuẩn tự động** | 5 giây hiệu chuẩn gyro khi khởi động |
| **Nút vật lý** | Nút SOS (giữ 3s) và nút Cancel (giữ 3s) |
| **Double-buffer** | Cơ chế đồng bộ dữ liệu không dùng mutex giữa task cảm biến và HTTP server |

## Kiến Trúc Hệ Thống

```
┌─────────────────────────────────────────────────────────────────┐
│                         ESP32-C3                                │
│  ┌──────────┐   ┌──────────────┐   ┌─────────────────────────┐│
│  │ MPU6050  │──▶│ mpu6050_task │──▶│ fall_detection_update() ││
│  │ (I2C)    │   │ (100Hz)      │   │ (state machine)         ││
│  └──────────┘   └──────┬───────┘   └──────────┬──────────────┘│
│                        │                       │               │
│                        ▼                       ▼               │
│                 ┌──────────────┐   ┌────────────────────────┐  │
│                 │ Double-      │   │ alert_callback()       │  │
│                 │ Buffer       │   │ ├── buzzer + LED       │  │
│                 │ (sensor data)│   │ └── Telegram queue     │  │
│                 └──────┬───────┘   └────────────────────────┘  │
│                        │                                        │
│  ┌──────────┐   ┌──────▼───────┐                                │
│  │ WiFi STA │──▶│ Web Server   │  Port 80                      │
│  │ + NTP    │   │ GET /        │  HTML dashboard               │
│  └──────────┘   │ GET /api/data│  JSON sensor data             │
│                 │ GET /api/status│ JSON device status          │
│                 └──────────────┘                                │
│  ┌──────────┐   ┌──────────────┐                                │
│  │ Telegram │◀──│ Queue (async)│                                │
│  │ Bot API  │   │ (4 messages) │                                │
│  └──────────┘   └──────────────┘                                │
└─────────────────────────────────────────────────────────────────┘
```

## Phần Cứng

### Linh Kiện

| Linh kiện | Thông số |
|-----------|----------|
| ESP32-C3 | RISC-V, 160MHz, 400KB SRAM, WiFi |
| MPU6050 | 6-axis IMU (Accelerometer ±8g + Gyroscope ±2000°/s) |
| Buzzer | Active buzzer 3.3V |
| LED | LED thường (nhấp nháy báo hiệu) |
| 2× Nút nhấn | Nút Cancel (GPIO 5) + Nút SOS (GPIO 6), pull-up nội |

### Sơ Đồ Chân

| GPIO | Hướng | Chức năng |
|------|-------|-----------|
| 0 | Output | Buzzer |
| 1 | Output | LED |
| 8 | I2C SDA | MPU6050 data |
| 9 | I2C SCL | MPU6050 clock |
| 5 | Input (Pull-up) | Nút Cancel — giữ 3s để hủy báo động giả |
| 6 | Input (Pull-up) | Nút SOS — giữ 3s để kích hoạt khẩn cấp |

### Vị Trí Đặt Cảm Biến
- **Đeo ở**: Thắt lưng (belt)
- **Trục X**: Hướng lên trên
- **Trục Y**: Hướng về phía trước
- **Trục Z**: Nằm ngang

## Thuật Toán Phát Hiện Té Ngã

### State Machine

```
┌───────┐  raw < 0.5g   ┌───────────┐  raw ≥ 2.0g  ┌────────┐  timer 1s  ┌──────────────┐  tilt≥70°
│ IDLE  │ ───────────►  │ FREEFALL  │ ───────────►  │ IMPACT │ ─────────►  │ WAIT_LIE_DOWN│  + gyro<20
└───────┘               └───────────┘               └────────┘            └──────┬───────┘  + 3.5s ổn
     ▲                        │                        │                        │               │
     │                        │  timer 150ms           │                        │               ▼
     └────────────────────────┴────────────────────────┘                        │         ┌─────────────┐
                         (reset về IDLE)          (reset)                      │         │    SOS      │
                                                                               │         │  (báo động)  │
                                                                               │         └─────────────┘
                                                                               │
                                                                               └── tilt<70° (đứng dậy)
                                                                                   → reset về IDLE
```

### Chi Tiết Trạng Thái

| Trạng thái | Giá trị | Ý nghĩa |
|-------------|---------|----------|
| IDLE | 0 | Bình thường, đang theo dõi |
| FREEFALL | 1 | Phát hiện rơi tự do (gia tốc < 0.5g) |
| IMPACT | 2 | Phát hiện va chạm (gia tốc ≥ 2.0g) |
| WAIT_LIE_DOWN | 3 | Đợi xác nhận nằm yên trong 3.5s |
| SOS_TRIGGERED | 4 | Xác nhận té ngã, kích hoạt báo động |

### Ngưỡng Phát Hiện

| Tham số | Giá trị | Giải thích |
|---------|---------|------------|
| Freefall | ≤ 0.5g | Mất trọng lượng khi rơi (raw accel) |
| Impact | ≥ 2.0g | Va chạm mạnh sau khi ngã (raw accel) |
| Góc nằm | ≥ 70° | Tư thế nằm ngang |
| Timeout freefall | 150ms | Thời gian rơi tối đa trước khi reset |
| Delay impact | 1000ms | Chờ 1s sau va chạm mới kiểm tra góc |
| Xác nhận nằm | 3500ms | Cần 3.5s ổn định để xác nhận té ngã |
| Lọc α (fall detect) | 0.5 | 50% cũ + 50% mới |
| Lọc α (roll/pitch) | 0.80 | Complementary filter cho góc |

## API Web

### GET /api/data — Dữ liệu cảm biến

```json
{
  "accel_g": 1.02,
  "gyro": 15,
  "roll": 2.3,
  "pitch": -1.8,
  "ready": 1,
  "wifi_connected": 1,
  "alert_active": 0,
  "uptime": 1234,
  "fall_state": 0,
  "max_tilt": 2.3,
  "filt_accel": 1.01
}
```

### GET /api/status — Trạng thái thiết bị

```json
{
  "wifi_connected": 1,
  "alert_active": 0,
  "error_state": 0,
  "uptime": 1234
}
```

## Cấu Trúc Thư Mục

```
PROJECT---FALL-DETECTION-AND-ALERT-DEVICE-FOR-THE-ELDERLY/
├── main/
│   └── CODE.c                    # Chương trình chính: init, tasks, alert logic
├── components/
│   ├── mpu6050/
│   │   ├── mpu6050.c             # Driver I2C: đọc raw, chuyển đổi, hiệu chuẩn
│   │   ├── mpu6050.h             # API: init, read, convert, calibrate
│   │   ├── mpu6050_constants.h   # Hằng số chung: scale, gravity, alpha
│   │   ├── roll_pitch.c          # Bộ lọc Complementary cho góc Roll/Pitch
│   │   └── roll_pitch.h          # API: init, update, get_roll, get_pitch
│   ├── fall_detection/
│   │   ├── fall_detection.c      # State machine phát hiện té ngã (5 trạng thái)
│   │   └── fall_detection.h      # API: init, update, reset, callback
│   ├── wifi/
│   │   ├── wifi.c                # WiFi Station + NTP time sync + auto reconnect
│   │   └── wifi.h                # Cấu hình SSID/PASS
│   ├── webserver/
│   │   ├── webserver.c           # HTTP server: dashboard HTML + JSON API
│   │   └── webserver.h           # API: start/stop, sensor_data_t struct
│   └── telegram/
│       ├── telegram.c            # Async Telegram Bot: queue-based
│       ├── telegram.h            # API: init, send_startup/fall_alert/sos_alert/cancel_alert
│       └── telegram_messages.h   # Nội dung tin nhắn (dễ sửa)
├── CMakeLists.txt
├── sdkconfig
└── README.md
```

## Cấu Hình

### WiFi (`components/wifi/wifi.h`)

```c
#define WIFI_SSID        "B3 1409"
#define WIFI_PASS        "05122003"
#define WIFI_MAX_RETRY   10        // Số lần retry mỗi batch
#define WIFI_RETRY_INTERVAL_MS 10000  // Retry mỗi 10s
```

### Telegram Bot (`main/CODE.c`)

```c
#define TELEGRAM_BOT_TOKEN  "8659816659:AAEFwAc-LdtDNuVEGbUHt_cpOwP_ilWfSjA"
#define TELEGRAM_CHAT_ID    "-5239342658"
```

### Nội Dung Tin Nhắn (`components/telegram/telegram_messages.h`)

Chỉ cần sửa nội dung các macro, không cần chỉnh code khác:

```c
#define MSG_FALL_ALERT "CẢNH BÁO: Phát hiện người bị ngã!"
```

## Hướng Dẫn Sử Dụng

### Build & Flash

```bash
# Kích hoạt môi trường ESP-IDF (Windows)
%userprofile%\esp\esp-idf\export.ps1

# Build
idf.py build

# Flash
idf.py flash

# Monitor (115200 baud)
idf.py monitor
```

### Quy Trình Hoạt Động

1. **Khởi động**: LED nhấp nháy 5 giây — hiệu chuẩn gyro (giữ thiết bị yên)
2. **Buzzer beep 1s**: Xác nhận hoàn tất hiệu chuẩn
3. **Kết nối WiFi**: Đợi tối đa 30s, dashboard web tại `http://<ESP_IP>`
4. **Theo dõi**: Đọc cảm biến 100Hz, cập nhật dashboard + state machine
5. **Phát hiện té ngã**:
   - Buzzer bật + LED nhấp nháy 1Hz
   - Telegram gửi cảnh báo
   - Tự động tắt sau 30s, reset về IDLE
6. **Hủy báo động giả**: Giữ nút Cancel (GPIO 5) trong 3s
7. **SOS thủ công**: Giữ nút SOS (GPIO 6) trong 3s

### Debug

```bash
# Xem log
idf.py monitor

# Lọc theo tag
idf.py monitor | Select-String "FALL_DETECT"
idf.py monitor | Select-String "MPU6050"
idf.py monitor | Select-String "TELEGRAM"
```

## Luồng Dữ Liệu Chi Tiết

### Double-Buffer (không dùng mutex)

```
MPU6050 Task (100Hz)
  │
  ├── g_sensor_buffers[writer_index] ← ghi data mới
  ├── writer_index = 1 - writer_index  (swap)
  │
  └── HTTP Handler (khi có request)
      └── reader_index = 1 - g_get_writer_index()
          └── memcpy từ g_sensor_buffers[reader_index]
```

Writer và reader không bao giờ truy cập cùng một buffer tại cùng thời điểm → không cần mutex.

### Async Telegram

```
alert_callback() / timer_callback()
  │
  └── xQueueSend(queue, msg_type)    ← non-blocking (từ task/timer)
        │
        ▼
  telegram_task (background)
        │
        └── esp_http_client_perform() ← blocking OK (task riêng)
```

## License

MIT License
