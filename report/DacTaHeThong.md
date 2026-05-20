# ĐẶC TẢ HỆ THỐNG
## Thiết bị Phát hiện Té ngã Cho Người Cao Tuổi

---

## 1. Tổng Quan

### 1.1. Mô tả hệ thống
Hệ thống nhúng IoT phát hiện té ngã tự động, sử dụng cảm biến quán tính MPU6050 (6-axis IMU) trên nền vi điều khiển ESP32-C3. Thiết bị đeo ở thắt lưng, phân tích chuyển động theo thời gian thực (100 Hz) và kích hoạt cảnh báo đa kênh khi phát hiện té ngã.

### 1.2. Mục đích
Phát hiện té ngã ở người cao tuổi sống một mình và thông báo tức thời đến người chăm sóc qua Telegram, còi báo, LED, và web dashboard — giảm thiểu rủi từ "long-lie" (nằm lâu sau ngã).

### 1.3. Đối tượng sử dụng
- Người cao tuổi ≥ 65 tuổi (người đeo thiết bị)
- Người thân/người chăm sóc (nhận cảnh báo)

---

## 2. Yêu Cầu Hệ Thống

### 2.1. Yêu cầu chức năng

| ID | Yêu cầu | Mô tả | Mức ưu tiên |
|----|---------|-------|-------------|
| F01 | Đọc cảm biến MPU6050 | Đọc gia tốc 3 trục (m/s²) và vận tốc góc 3 trục (°/s) qua I2C ở 100Hz | Cao |
| F02 | Phát hiện té ngã | State machine 5 trạng thái: IDLE → FREEFALL → IMPACT → WAIT_LIE_DOWN → SOS | Cao |
| F03 | Cảnh báo tại chỗ | Bật buzzer + LED nhấp nháy (1Hz) khi phát hiện té ngã | Cao |
| F04 | Tự động tắt báo động | Tự động tắt buzzer/LED sau 30 giây (`ALERT_DURATION_MS = 30000`) | Cao |
| F05 | Reset về IDLE sau báo động | Gọi `fall_detection_reset()` khi tắt báo → sẵn sàng phát hiện lần tiếp | Cao |
| F06 | Gửi Telegram | Gửi tin nhắn qua Telegram Bot API khi: khởi động, té ngã, SOS, hủy báo | Cao |
| F07 | Dashboard Web | HTTP server port 80, hiển thị WiFi status, fall state (5 màu), accel (g), gyro, roll, pitch | Trung bình |
| F08 | JSON API | Endpoint `/api/data`: JSON sensor data + fall state `/api/status`: JSON device status | Trung bình |
| F09 | Nút SOS | Giữ 3 giây → kích hoạt báo động khẩn cấp + Telegram SOS | Cao |
| F10 | Nút Cancel | Giữ 3 giây → hủy báo động + Telegram cancel | Cao |
| F11 | Kết nối WiFi | Chế độ Station, WPA2-PSK, retry vô hạn mỗi 10s khi mất kết nối | Cao |
| F12 | Đồng bộ NTP | SNTP pool.ntp.org, múi giờ ICT (UTC+7) | Thấp |
| F13 | Hiệu chuẩn gyro | Two-phase calibration: 500 mẫu (5s) khi khởi động, LED nhấp nháy báo hiệu | Trung bình |
| F14 | CORS headers | Hỗ trợ cross-origin cho API (`Access-Control-Allow-Origin: *`) | Thấp |
| F15 | Xử lý nút nhấn | Interrupt-driven GPIO (ANYEDGE), debounce 50ms, queue-based xử lý | Cao |

### 2.2. Yêu cầu phi chức năng

| ID | Yêu cầu | Mô tả | Chỉ tiêu |
|----|---------|-------|----------|
| NF01 | Tần số lấy mẫu | MPU6050 đọc ở 100Hz | 100 Hz (±2Hz) |
| NF02 | Độ trễ phát hiện freefall | Từ khi rơi đến phát hiện | ≤ 10 ms (1 mẫu) |
| NF03 | Độ trễ phát hiện impact | Từ khi va chạm đến phát hiện | ≤ 10 ms (1 mẫu) |
| NF04 | Thời gian xác nhận ngã | Từ freefall → SOS | ~3.52 s (có chủ đích) |
| NF05 | Thời gian hủy báo | Giữ nút Cancel | 3 s (hold-to-confirm) |
| NF06 | Thời gian SOS | Giữ nút SOS | 3 s (hold-to-confirm) |
| NF07 | Tự động tắt báo | Alert timeout | 30 s |
| NF08 | Tiêu thụ dòng (IDLE) | WiFi + sensor polling | ~85-90 mA |
| NF09 | Tiêu thụ dòng (ALERT) | Buzzer + LED active | ~130-140 mA |
| NF10 | Điện áp hoạt động | USB-C | 5 V |
| NF11 | Giao tiếp I2C | Fast Mode | 400 kHz |
| NF12 | Giao tiếp UART debug | Serial console | 115200 baud |
| NF13 | Polling dashboard | JavaScript fetch() | 100 ms |
| NF14 | HTTP request Telegram | Timeout mỗi request | 15 s |

---

## 3. Đặc Tả Phần Cứng

### 3.1. Sơ đồ khối

```
┌─────────────────────────────────────────────────────────────┐
│                     ESP32-C3 (RISC-V 32-bit, 160MHz)        │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  400 KB SRAM  |  4 MB Flash  |  WiFi 2.4GHz          │  │
│  │  I2C Master (400kHz) | GPIO | FreeRTOS (ESP-IDF)     │  │
│  └───────────────────────────────────────────────────────┘  │
│      │ I2C         │ GPIO0   │ GPIO1  │ GPIO5 │ GPIO6      │
│      ▼             ▼         ▼        ▼       ▼            │
│  ┌────────┐   ┌────────┐ ┌──────┐ ┌─────┐ ┌─────┐         │
│  │MPU6050 │   │ Buzzer │ │ LED  │ │Cancel│ │ SOS │         │
│  │6-axis  │   │Active  │ │+220Ω │ │Button│ │Button│         │
│  └────────┘   └────────┘ └──────┘ └─────┘ └─────┘         │
└─────────────────────────────────────────────────────────────┘
```

### 3.2. Danh sách linh kiện

| STT | Linh kiện | Model/Thông số | Số lượng |
|-----|-----------|----------------|----------|
| 1 | Vi điều khiển | ESP32-C3-DevKit-M-1 (RISC-V, 160MHz, 4MB Flash) | 1 |
| 2 | Cảm biến IMU | MPU6050 (Accel ±8g, Gyro ±2000°/s, I2C 0x68) | 1 |
| 3 | Còi báo | Active Buzzer 3.3V (~2300Hz, ~30mA) | 1 |
| 4 | LED | LED đỏ 5mm + Resistor 220Ω (~6mA) | 1 bộ |
| 5 | Nút nhấn | Tactile switch, thường hở | 2 |
| 6 | Breadboard | 830 điểm + dây Dupont | 1 bộ |

### 3.3. Bảng chân GPIO

| GPIO | I/O | Kết nối | Chức năng | Cấu hình |
|------|-----|---------|-----------|----------|
| 8 | I/O | MPU6050 SDA | Dữ liệu I2C | Open-drain, pull-up 4.7kΩ (trên module) |
| 9 | I/O | MPU6050 SCL | Clock I2C 400kHz | Open-drain, pull-up 4.7kΩ (trên module) |
| 0 | Output | Buzzer (chân +) | Bật/tắt còi | GPIO_MODE_OUTPUT |
| 1 | Output | LED (+), Resistor 220Ω → GND | Báo hiệu trạng thái | GPIO_MODE_OUTPUT |
| 5 | Input | Nút Cancel → GND | Hủy báo động (giữ 3s) | GPIO_INTR_ANYEDGE, Pull-up nội |
| 6 | Input | Nút SOS → GND | SOS thủ công (giữ 3s) | GPIO_INTR_ANYEDGE, Pull-up nội |

### 3.4. Thông số cảm biến MPU6050

| Tham số | Giá trị |
|---------|---------|
| Giao tiếp | I2C (địa chỉ 0x68) |
| Tốc độ bus | 400 kHz (Fast Mode) |
| Điện áp | 3.3 V |
| Dòng tiêu thụ | 3.9 mA |
| Dải đo Accel | ±8 g (cấu hình qua thanh ghi 0x1C = 0x10) |
| Độ phân giải Accel | 4096 LSB/g |
| Dải đo Gyro | ±2000 °/s (cấu hình qua thanh ghi 0x1B = 0x18) |
| Độ phân giải Gyro | 16.4 LSB/°/s |
| Tần số lấy mẫu | 100 Hz (10ms) |
| Dữ liệu đọc | 14 bytes: accel (6) + temp (2) + gyro (6), big-endian |

### 3.5. Công thức chuyển đổi

```
accel_X (m/s²) = raw_X / 4096 × 9.81 - accel_bias[X]
gyro_X (°/s)   = raw_X / 16.4 - gyro_bias[X]
|accel| (g)    = √(ax² + ay² + az²) / 9.81
|gyro| (°/s)   = √(gx² + gy² + gz²)
```

---

## 4. Đặc Tả Phần Mềm

### 4.1. Kiến trúc FreeRTOS

| Task | Priority | Stack | Chu kỳ | Mô tả |
|------|----------|-------|--------|-------|
| `mpu6050_task` | 5 (cao) | 4096 B | 10 ms (100Hz) | Đọc MPU6050 qua I2C, tính góc roll/pitch, cập nhật double-buffer, chạy fall detection state machine |
| `telegram_task` | 4 | 4096 B | Sự kiện (queue) | Chờ queue, gửi HTTP request đến Telegram Bot API (blocking OK) |
| `btn_task` | 3 | 2048 B | Sự kiện (queue) | Nhận GPIO event từ ISR queue, xử lý press/release, quản lý software timer SOS/Cancel |
| `app_main` (main loop) | 1 (thấp) | — | 1000 ms | Kiểm tra WiFi status, xử lý cờ Telegram từ timer callbacks, log heartbeat |

### 4.2. Giao tiếp liên task

```
ISR (GPIO) ─────► GPIO Queue ────► btn_task ──► xTimerStart/Stop
                                                      │
                    Timer Callback ──► Flag ──► main loop ──► telegram_send_*()
                                                                   │
                                                              Queue ────► telegram_task ──► HTTP (Telegram)
```

### 4.3. Software Timer

| Timer | Chu kỳ | Loại | Chức năng |
|-------|--------|------|-----------|
| `alert_timeout` | 30000 ms | One-shot | Tự động tắt báo động |
| `led_blink` | 1000 ms | Auto-reload | LED nhấp nháy 1Hz khi báo động |
| `sos_timer` | 3000 ms | One-shot | Xác nhận giữ nút SOS đủ 3s |
| `cancel_timer` | 3000 ms | One-shot | Xác nhận giữ nút Cancel đủ 3s |

### 4.4. State Machine Phát Hiện Té Ngã

**Ngưỡng phát hiện:**

| Tham số | Ký hiệu | Giá trị | Giải thích |
|---------|---------|---------|------------|
| Freefall | `accel_freefall_abs` | < 0.5 g | Gia tốc giảm mất trọng lượng |
| Impact | `accel_impact_abs` | ≥ 2.0 g | Va chạm sau té ngã |
| Góc nằm | `lying_angle_threshold` | ≥ 70° | max(\|pitch\|, \|roll\|) |
| Ngưỡng gyro tĩnh | — | < 20 °/s | Không cử động |
| Xác nhận accel ổn | — | \|filtered_accel - 1.0\| < 0.3 g | Nằm yên trên mặt đất |
| Timeout freefall | `timeout_freefall` | 150 ms | Reset nếu không có impact |
| Delay impact | `timeout_impact_check` | 1000 ms | Chờ ổn định sau va chạm |
| Xác nhận nằm | `wait_lie_down_time` | 3500 ms | Cần 3.5s ổn định |
| Lọc α (fall) | `filter_alpha` | 0.5 | Low-pass cho lying detection |

**Luồng trạng thái:**

```
        accel_raw < 0.5g          accel_raw ≥ 2.0g
IDLE ─────────────────────► FREEFALL ───────────────► IMPACT
  ▲                            │                        │
  │      timer 150ms           │                        │ timer 1s
  └────────────────────────────┘                        │
                                            ┌───────────┘
                                            ▼
                                    max_tilt ≥ 70° ?
                                   ┌────┴────┐
                                  YES       NO → IDLE
                                   │
                                   ▼
                              WAIT_LIE_DOWN
                              ┌───┴───┐
                         gyro<20   gyro≥20
                         +3.5s      → reset timer
                         │
                         ▼
                     SOS_TRIGGERED
                     (báo động + Telegram)
```

**Xử lý raw vs filtered:**
- **Raw accel** cho freefall và impact: sự kiện nhanh (10-50ms), lọc low-pass gây trễ 2-5 mẫu có thể bỏ lỡ xung impact
- **Filtered accel** cho lying detection: cần ổn định hàng giây, độ trễ lọc không đáng kể

### 4.5. Double-Buffer

```
mpu6050_task (100Hz)
  │
  ├── g_sensor_buffers[writer].total_accel_g = ...
  ├── g_sensor_buffers[writer].total_gyro    = ...
  ├── g_sensor_buffers[writer].roll          = ...
  ├── g_sensor_buffers[writer].pitch         = ...
  ├── g_sensor_buffers[writer].data_ready    = true
  ├── writer = 1 - writer  (swap)
  │
  HTTP Handler (/api/data)
      └── reader = 1 - g_get_writer_index()
          └── memcpy(&local, &g_sensor_buffers[reader], sizeof)
```

Không cần mutex do writer và reader không bao giờ truy cập cùng buffer.

### 4.6. Cấu hình WiFi

| Tham số | Giá trị | Mô tả |
|---------|---------|-------|
| Mode | Station (WPA2-PSK) | Client kết nối router |
| SSID/PASS | Hardcoded trong `wifi.h` | Cần cập nhật trước khi build |
| Retry interval | 10000 ms | Timer one-shot |
| Retry max | 10 lần/batch → reset về 0 (vô hạn) | Retry vô hạn |
| NTP server | pool.ntp.org | SNTP polling |
| Timezone | ICT-7 (UTC+7) | Giờ Việt Nam |
| Tx Power | 40 dBm | Công suất phát tối đa |

### 4.7. Cấu hình Telegram

| Tham số | Giá trị | Mô tả |
|---------|---------|-------|
| API URL | `https://api.telegram.org/bot{token}/sendMessage` | HTTP GET |
| Token | Hardcoded trong `CODE.c` | Bot token từ BotFather |
| Chat ID | Hardcoded trong `CODE.c` | ID người dùng/nhóm |
| Timeout | 15000 ms | Thời gian chờ HTTP |
| SSL | esp_crt_bundle_attach | Xác thực chứng chỉ |
| Queue | 4 messages | Hàng đợi tối đa |
| Async | Queue + dedicated task | Non-blocking |

### 4.8. Web Dashboard & API

| Endpoint | Phương thức | Response | Tần suất |
|----------|------------|----------|----------|
| `/` | GET | HTML dashboard (embedded C-string) | Khi truy cập |
| `/api/data` | GET | JSON sensor + fall state | Polling 100ms |
| `/api/status` | GET | JSON device status | Khi cần |
| `/favicon.ico` | GET | 204 No Content | Trình duyệt |
| `/api/data` | OPTIONS | 204 + CORS headers | Preflight |
| `/api/*` | OPTIONS | 204 + CORS headers | Preflight |

**Response /api/data:**
```json
{
  "accel_g": 1.02,
  "gyro": 3,
  "roll": -1.5,
  "pitch": 0.8,
  "ready": 1,
  "wifi_connected": 1,
  "alert_active": 0,
  "uptime": 1234,
  "fall_state": 0,
  "max_tilt": 2.1,
  "filt_accel": 1.01
}
```

### 4.9. Cấu trúc thư mục

```
main/
  CODE.c                    # Chương trình chính (init, tasks, alert logic)
components/
  mpu6050/
    mpu6050.c               # Driver I2C MPU6050
    mpu6050.h               # Public API
    mpu6050_constants.h     # Scale factors, gravity, alpha
    roll_pitch.c            # Complementary filter
    roll_pitch.h            # Roll/Pitch API
  fall_detection/
    fall_detection.c        # State machine (5 states)
    fall_detection.h        # Public API, config struct, enums
  wifi/
    wifi.c                  # WiFi Station + NTP
    wifi.h                  # SSID/PASS config
  webserver/
    webserver.c             # HTTP server + dashboard
    webserver.h             # sensor_data_t, externs
  telegram/
    telegram.c              # Async queue-based Telegram
    telegram.h              # Public API
    telegram_messages.h     # User-editable messages
```

---

## 5. Giao Diện

### 5.1. Giao diện người dùng (Web Dashboard)

- **Theme**: Dark (`#0f0f1a` background, `#e0e0e0` text)
- **Layout**: Cards layout, responsive (max 480px)
- **WiFi status**: "Connected" (green `#4ecca3`) / "Disconnected" (red `#e94560`)
- **Fall State**: 5 màu — IDLE green, FREEFALL yellow, IMPACT orange, WAIT_LIE_DOWN orange, SOS red
- **Sensor cards**: Accel (g), Gyro (deg/s), Roll (°), Pitch (°) — giá trị cyan `#4fc3f7`
- **Polling**: JavaScript `setInterval(updateData, 100)` → `fetch('/api/data')`
- **Chế độ**: Read-only (observation-only)

### 5.2. Giao diện Telegram Bot

| Sự kiện | Tin nhắn |
|---------|----------|
| Khởi động | "Thiết bị đã khởi động. Sẵn sàng giám sát người thân!" |
| Phát hiện ngã | "CẢNH BÁO: Phát hiện người bị ngã! Vui lòng kiểm tra người thân." |
| SOS | "SOS: Nút khẩn cấp được nhấn! Vui lòng hỗ trợ ngay." |
| Hủy báo động | "THÔNG BÁO: Cảnh báo đã được hủy." |

### 5.3. Giao diện phần cứng (LED)

| Trạng thái | LED |
|------------|-----|
| Hiệu chuẩn (5s) | Nhấp nháy 10Hz (50% duty) |
| IDLE (bình thường) | Tắt |
| WAIT_LIE_DOWN | Nhấp nháy 10Hz (100ms on/off, 50% duty) |
| SOS (báo động) | Nhấp nháy 1Hz (500ms on/off) |
| Lỗi (I2C/WiFi) | Nhấp nháy nhanh 200ms |

### 5.4. Giao diện phần cứng (Buzzer)

| Trạng thái | Buzzer |
|------------|--------|
| Bình thường | Tắt |
| SOS (báo động) | Bật liên tục (~2300Hz) |
| Hết 30s timeout | Tắt |

### 5.5. Giao diện UART (Debug)

- **Tốc độ**: 115200 baud, 8N1
- **Log heartbeat**: `[SYSTEM] Heartbeat - WiFi: OK | Alert: IDLE | Error: NO | Uptime: 123s` (mỗi 1s)
- **Log state machine**: `-> STATE_FREEFALL (accel: 0.32g)` khi chuyển trạng thái
- **Log alert**: `>>> FALL ALERT STARTED <<<` khi báo động

---

## 6. Yêu Cầu Về Hiệu Năng & Ràng Buộc

### 6.1. Ràng buộc phần cứng

- ESP32-C3: SRAM 400 KB, Flash 4 MB
- WiFi: băng tần 2.4 GHz, chuẩn 802.11 b/g/n
- I2C: 400 kHz (Fast Mode), dây ngắn < 10 cm

### 6.2. Ràng buộc phần mềm

- ESP-IDF 5.5.2 (FreeRTOS)
- Trình biên dịch: RISC-V GCC
- HTTP server: `esp_http_server` (thread-safe, event-driven)
- HTTP client: `esp_http_client` (synchronous, blocking)
- SSL: `esp_crt_bundle` (Mozilla CA certificate bundle)

### 6.3. Ràng buộc thời gian thực

- Task MPU6050 priority 5 (cao nhất) → đảm bảo chu kỳ 10ms
- Không gọi blocking API trong timer callbacks (daemon task stack nhỏ)
- ISR tối thiểu: chỉ ghi queue, không log, không xử lý nặng

### 6.4. Giới hạn hệ thống

- Phát hiện: té ngã đột ngột (không bao gồm ngã từ từ / trượt)
- Phạm vi: trong nhà (có WiFi)
- Nguồn: USB-C 5V (chưa có pin sạc tích hợp)
- Chưa có ML/AI, chưa có cloud backend

---

*Tài liệu đặc tả hệ thống — IoT Fall Detection Device*
*Ngày: 19/05/2026*
