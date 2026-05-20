# Phát Hiện Té Ngã Cho Người Cao Tuổi — Fall Detection Device

Thiết bị đeo thông minh giúp phát hiện té ngã ở người già, gửi cảnh báo tức thời qua còi báo, LED nhấp nháy, và Telegram đến người thân. Được xây dựng trên vi điều khiển **ESP32-C3** và cảm biến **MPU6050**, chạy hệ điều hành thời gian thực **FreeRTOS** (ESP-IDF).

---

## Chuyện gì xảy ra khi một người già bị ngã?

Ngã là một trong những tai nạn nguy hiểm nhất với người cao tuổi. Không phải cú ngã nào cũng gây chấn thương — nhưng nếu nạn nhân nằm đó hàng giờ không ai biết, hậu quả mới thực sự nghiêm trọng.

Thiết bị này được thiết kế để giải quyết vấn đề đó. Đeo ở thắt lưng, nó liên tục theo dõi chuyển động của người dùng. Khi phát hiện một cú ngã thật sự, nó lập tức:

- **Hú còi + nháy LED** để thu hút sự chú ý
- **Gửi tin nhắn Telegram** đến người thân hoặc người chăm sóc
- **Hiển thị trạng thái** trên web dashboard theo thời gian thực

Và nếu chỉ là báo động giả? Người dùng chỉ cần giữ nút Cancel 3 giây.

---

## Tính năng

- **Phát hiện ngã bằng state machine 5 trạng thái**: Nhận diện đúng 3 giai đoạn vật lý của một cú ngã: rơi → va chạm → nằm yên. Nếu thiếu bất kỳ giai đoạn nào, thiết bị bỏ qua — giảm tối đa báo động giả.
- **Cảnh báo tại chỗ**: Buzzer kêu liên tục + LED nhấp nháy 1Hz. Tự động tắt sau 30 giây.
- **Dashboard Web thời gian thực**: Mở trình duyệt, gõ địa chỉ IP của thiết bị, thấy ngay accel, gyro, góc nghiêng, trạng thái. Polling mỗi 100ms.
- **Telegram Bot**: Tin nhắn đẩy đến điện thoại khi phát hiện ngã, khi nhấn SOS, hoặc khi hủy báo động. Nội dung có thể sửa dễ dàng.
- **Hiệu chuẩn tự động**: 5 giây đầu tiên sau khởi động, thiết bị tự hiệu chuẩn gyro — giữ yên là được.
- **Nút vật lý**: SOS (giữ 3s để gọi khẩn cấp), Cancel (giữ 3s để hủy báo động giả).
- **Double-buffer không mutex**: Kỹ thuật đồng bộ dữ liệu giữa task cảm biến và web server mà không cần khóa — nhanh, an toàn, không deadlock.

---

## Cách phát hiện ngã — giải thích đơn giản

### Nguyên lý vật lý

Một cú ngã điển hình có ba giai đoạn:

```
1. RƠI TỰ DO         2. VA CHẠM             3. NẰM YÊN
  ───────              ───────               ─────────
accel ≈ 0g           accel ≥ 2g            góc ≥ 70°
(mất trọng lượng)    (đập xuống sàn)        (nằm ngang, không nhúc nhích)
```

Cảm biến MPU6050 đo hai thứ:
- **Gia tốc (accel)**: Lực tác động lên thiết bị. Bình thường là 1g (trọng lực). Khi rơi tự do, accel ~ 0g. Khi va chạm, accel có thể vọt lên 2-3g.
- **Vận tốc góc (gyro)**: Tốc độ xoay. Khi nằm yên, gyro gần như bằng 0.

### State machine — "bộ não" của thiết bị

Thiết bị không chỉ nhìn vào accel rồi kết luận "ngã". Nó đi qua 5 trạng thái, mỗi trạng thái kiểm tra một điều kiện khác nhau:

```
                    accel < 0.5g                 accel ≥ 2.0g
     ┌───────┐  ──────────────────►  ┌───────────┐ ────────────────►  ┌────────┐
     │ IDLE  │                       │ FREEFALL  │                   │ IMPACT │
     └───┬───┘                       └─────┬─────┘                   └────┬───┘
         ▲                                │                             │
         │          ┌──────────────────────┘                             │
         │          │  timeout 150ms (không có va chạm)                  │
         │          │  → reset về IDLE (lắc tay, cúi người,...)          │
         │          │                                                    │
         │          └─────────────────────┐                 timer 1s     │
         │                                │              ┌───────────────┘
         │                                │              ▼
         │                                │        ┌──────────────┐
         │                                │        │WAIT_LIE_DOWN │
         │                                │        └──────┬───────┘
         │                                │               │
         │                                │    ┌──────────┴──────────────┐
         │                                │    │                         │
         │                                │   góc ≥ 70°                góc < 70°
         │                                │   + gyro < 20°/s            (đã đứng dậy)
         │                                │   + 3.5s ổn định             → IDLE
         │                                │    │
         │                                │    ▼
         │                                │  ┌─────────────┐
         │                                │  │    SOS      │
         │                                │  │ (báo động)  │
         │                                │  └─────────────┘
         │                                │       │
         └────────────────────────────────┴───────┘
                     (reset từ nút Cancel hoặc hết 30s)
```

Mỗi trạng thái có một ý nghĩa:

| State | Tên | Chuyện gì đang xảy ra |
|-------|-----|----------------------|
| 0 | IDLE | Bình thường, accel ~1g, đang theo dõi |
| 1 | FREEFALL | Accel giảm mạnh dưới 0.5g — có thể đang rơi. Đếm ngược 150ms, nếu không có va chạm thì coi là nhiễu |
| 2 | IMPACT | Accel vọt lên trên 2.0g — đã va chạm. Chờ 1s cho cảm biến ổn định |
| 3 | WAIT_LIE_DOWN | Kiểm tra: có nằm yên 3.5s không? Góc ≥ 70°? Gyro thấp? — nếu đủ điều kiện → xác nhận ngã |
| 4 | SOS | Xác nhận té ngã. Báo động bật, chờ người dùng reset |

### Tại sao phải phức tạp vậy?

Chỉ cần một lần vỗ tay mạnh cũng tạo ra accel > 2g. Chỉ cần nằm võng cũng có góc > 70°. Nhưng kết hợp cả ba giai đoạn (rơi → va chạm → nằm yên) thì chỉ có té ngã thật mới làm được. Đó là lý do state machine giúp giảm báo động giả hiệu quả.

### Chi tiết ngưỡng

| Tham số | Giá trị | Giải thích |
|---------|---------|------------|
| Ngưỡng rơi tự do | accel ≤ 0.5g | Mất trọng lượng khi rơi |
| Ngưỡng va chạm | accel ≥ 2.0g | Xung lực đập xuống sàn |
| Góc nằm | ≥ 70° | Tư thế nằm ngang (không thể đứng ở góc này) |
| Timeout rơi | 150ms | Quá 150ms không va chạm → bỏ qua |
| Chờ sau va chạm | 1 giây | Cho cảm biến ổn định sau chấn động |
| Xác nhận nằm | 3.5 giây | Cần nằm yên không cựa quậy đủ 3.5s |
| Bộ lọc accel | α = 0.5 | Cân bằng giữa mượt và phản ứng nhanh |
| Bộ lọc góc | α = 0.8 | Complementary filter cho Roll/Pitch |

---

## Phần cứng

### Linh kiện

| Linh kiện | Chi tiết |
|-----------|----------|
| ESP32-C3 | RISC-V 160MHz, 400KB SRAM, WiFi |
| MPU6050 | 6-axis: Accelerometer ±8g + Gyroscope ±2000°/s |
| Buzzer | Active buzzer 3.3V |
| LED | LED thường, nhấp nháy báo hiệu |
| Nút Cancel | GPIO 5, pull-up nội |
| Nút SOS | GPIO 6, pull-up nội |

### Sơ đồ chân

| GPIO | Hướng | Kết nối |
|------|-------|---------|
| 0 | Output | Buzzer |
| 1 | Output | LED |
| 8 | I2C SDA | MPU6050 - Data line |
| 9 | I2C SCL | MPU6050 - Clock line |
| 5 | Input | Nút Cancel (giữ 3s để hủy báo) |
| 6 | Input | Nút SOS (giữ 3s để kích hoạt) |

### Cách đeo

Thiết bị được thiết kế để **đeo ở thắt lưng (belt)**:
- Trục X: hướng lên trên (theo chiều đứng)
- Trục Y: hướng về phía trước
- Trục Z: nằm ngang

Khi người đứng, trục X thẳng đứng, accel đọc được ~1g. Khi người ngã, accel giảm mạnh (rơi) rồi tăng đột ngột (va chạm), và góc nghiêng sẽ xoay gần 90°.

---

## Cấu trúc thư mục

```
PROJECT---FALL-DETECTION-AND-ALERT-DEVICE-FOR-THE-ELDERLY/
├── main/
│   └── CODE.c              # Chương trình chính: khởi tạo, task, callback
├── components/
│   ├── mpu6050/             # Driver đọc MPU6050 qua I2C
│   │   ├── mpu6050.c        #   Đọc raw, chuyển đổi, hiệu chuẩn
│   │   ├── mpu6050.h
│   │   ├── mpu6050_constants.h
│   │   ├── roll_pitch.c     #   Bộ lọc Complementary cho góc
│   │   └── roll_pitch.h
│   ├── fall_detection/      # State machine phát hiện ngã
│   │   ├── fall_detection.c #   5 trạng thái: IDLE → FREEFALL → IMPACT → WAIT → SOS
│   │   └── fall_detection.h
│   ├── wifi/                # Kết nối WiFi Station + tự động reconnect
│   │   ├── wifi.c
│   │   └── wifi.h
│   ├── webserver/           # HTTP server: dashboard HTML + JSON API
│   │   ├── webserver.c
│   │   └── webserver.h
│   └── telegram/            # Gửi Telegram bất đồng bộ (queue-based)
│       ├── telegram.c
│       ├── telegram.h
│       └── telegram_messages.h   # ← Sửa nội dung tin nhắn ở đây
├── CMakeLists.txt
├── sdkconfig
└── README.md
```

---

## Hướng dẫn build và flash

### Yêu cầu

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/) (framework cho ESP32-C3)
- Python 3.8+ (ESP-IDF đi kèm)

### Build

```bash
# Kích hoạt môi trường ESP-IDF (Windows)
%userprofile%\esp\esp-idf\export.ps1

# Build firmware
idf.py build

# Flash xuống board
idf.py flash

# Monitor serial (baudrate 115200)
idf.py monitor
```

### Debug

```bash
# Xem toàn bộ log
idf.py monitor

# Lọc log theo module
idf.py monitor | Select-String "FALL_DETECT"
idf.py monitor | Select-String "MPU6050"
idf.py monitor | Select-String "TELEGRAM"
idf.py monitor | Select-String "WEB"
```

### Quy trình hoạt động

1. **Khởi động**: LED nhấp nháy 5 giây — đây là lúc hiệu chuẩn gyro. **Giữ thiết bị yên** trong thời gian này.
2. **Buzzer beep 1s**: Báo hiệu hiệu chuẩn hoàn tất.
3. **Kết nối WiFi**: Đèn tắt, thiết bị bắt đầu kết nối. Tối đa 30 giây. Dashboard web tại `http://<địa-chỉ-IP>`.
4. **Theo dõi**: Cảm biến đọc 100 lần mỗi giây, state machine phân tích.
5. **Phát hiện ngã**: Còi kêu + LED nháy + Telegram gửi cảnh báo. Tự động tắt sau 30s.
6. **Hủy báo động giả**: Giữ nút **Cancel** 3 giây.
7. **SOS thủ công**: Giữ nút **SOS** 3 giây.

---

## Cấu hình

### WiFi

Sửa trong `components/wifi/wifi.h`:

```c
#define WIFI_SSID        "Tên WiFi"
#define WIFI_PASS        "Mật khẩu"
#define WIFI_MAX_RETRY   10        // Số lần thử lại
#define WIFI_RETRY_INTERVAL_MS 10000  // Cách nhau 10 giây
```

### Telegram Bot

Có hai chỗ cần cấu hình trong `main/CODE.c`:

```c
#define TELEGRAM_BOT_TOKEN  "123456789:ABCdefGHIjklmNOPqrstUVwxyz"
#define TELEGRAM_CHAT_ID    "-987654321"
```

- **Token**: Lấy từ [@BotFather](https://t.me/BotFather) trên Telegram.
- **Chat ID**: ID của nhóm hoặc người nhận. Có thể lấy từ [@getidsbot](https://t.me/getidsbot).

### Nội dung tin nhắn Telegram

Không cần sửa code. Chỉ cần mở `components/telegram/telegram_messages.h` và thay đổi nội dung:

```c
#define MSG_STARTUP      "Thiết bị đã khởi động.\nSẵn sàng giám sát người thân!"
#define MSG_FALL_ALERT   "CẢNH BÁO: Phát hiện người bị ngã!\nVui lòng kiểm tra người thân."
#define MSG_SOS_ALERT    "SOS: Nút khẩn cấp được nhấn!\nVui lòng hỗ trợ ngay."
#define MSG_CANCEL_ALERT "THÔNG BÁO: Cảnh báo đã được hủy."
```

---

## API Web

Sau khi kết nối WiFi, mở trình duyệt và vào `http://<địa-chỉ-IP>` để xem dashboard.

### `GET /` — Dashboard HTML

Giao diện tối (dark theme) hiển thị 4 thông số cảm biến + trạng thái WiFi + trạng thái ngã. Cập nhật mỗi 100ms.

### `GET /api/data` — Dữ liệu cảm biến (JSON)

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

| Trường | Kiểu | Ý nghĩa |
|--------|------|---------|
| `accel_g` | float | Gia tốc tổng (đơn vị g) |
| `gyro` | float | Vận tốc góc tổng (deg/s) |
| `roll` | float | Góc nghiêng trái-phải (độ) |
| `pitch` | float | Góc nghiêng trước-sau (độ) |
| `ready` | int | 1 = dữ liệu cảm biến đã sẵn sàng |
| `wifi_connected` | int | 1 = đã kết nối WiFi |
| `alert_active` | int | 1 = đang báo động |
| `uptime` | long | Thời gian hoạt động (giây) |
| `fall_state` | int | 0=IDLE, 1=FREEFALL, 2=IMPACT, 3=WAIT, 4=SOS |
| `max_tilt` | float | Góc nghiêng lớn nhất (độ) |
| `filt_accel` | float | Gia tốc sau lọc (g) |

### `GET /api/status` — Trạng thái thiết bị (JSON)

```json
{
  "wifi_connected": 1,
  "alert_active": 0,
  "error_state": 0,
  "uptime": 1234
}
```

---

## Kiến trúc hệ thống

```
                        ┌─────────────────────────────────────────────────┐
                        │                  ESP32-C3                      │
                        │                                                │
┌──────────┐   ┌────────▼───────┐   ┌──────────────────────────┐         │
│ MPU6050  │──▶│ mpu6050_task  │──▶│ fall_detection_update()  │         │
│ (I2C)    │   │ (100Hz)       │   │ (state machine 5 state)  │         │
└──────────┘   └───────┬───────┘   └───────────┬──────────────┘         │
                       │                       │                          │
                       ▼                       ▼                          │
                ┌──────────────┐   ┌────────────────────────┐             │
                │ Double-      │   │ alert_callback()       │             │
                │ Buffer       │   │  ├── buzzer + LED      │             │
                │ (sensor data)│   │  └── Telegram queue     │             │
                └──────┬───────┘   └────────────────────────┘             │
                       │                                                  │
         ┌──────────┐  │  ┌──────────────┐                                │
         │ WiFi STA │──┴──▶ Web Server  │  Port 80                       │
         │ + NTP    │      │ GET /        │ HTML dashboard                 │
         └──────────┘      │ GET /api/data│ JSON sensor data              │
                           │ GET /api/status│ JSON device status          │
                           └──────────────┘                                │
         ┌──────────┐   ┌──────────────┐                                  │
         │ Telegram │◀──│ Queue (async)│  4 messages max                  │
         │ Bot API  │   │ (background) │                                  │
         └──────────┘   └──────────────┘                                  │
                        └─────────────────────────────────────────────────┘
```

### Double-buffer (không dùng mutex)

MPU6050 task ghi dữ liệu vào một trong hai buffer, web server đọc từ buffer còn lại. Không bao giờ đụng nhau, nên không cần mutex:

```
MPU6050 Task (100Hz)
  │
  ├── g_sensor_buffers[writer_index] ← ghi dữ liệu mới
  ├── writer_index = 1 - writer_index   (swap buffer)
  │
  └── HTTP Handler (khi có request)
      └── reader_index = 1 - g_get_writer_index()
          └── đọc buffer an toàn
```

### Telegram bất đồng bộ

Không gửi HTTP ngay trong callback (vì HTTP có thể block). Thay vào đó, callback đẩy message vào queue, một task riêng xử lý sau:

```
alert_callback() / timer
  │
  └── xQueueSend(queue, msg_type)    ← non-blocking (gửi ngay)
        │
        ▼
  telegram_task (background)
        │
        └── esp_http_client_perform() ← blocking OK (task riêng)
```

---

## License

MIT License — bạn có thể sử dụng, sửa đổi, phân phối cho bất kỳ mục đích nào.
