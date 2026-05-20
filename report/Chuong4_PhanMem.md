# Chương 4: Thiết Kế Phần Mềm

Chương này trình bày chi tiết thiết kế phần mềm của thiết bị phát hiện té ngã, bao gồm kiến trúc đa nhiệm FreeRTOS, thuật toán xử lý tín hiệu cảm biến, máy trạng thái phát hiện té ngã, dashboard Web và cơ chế kết nối mạng.

## 4.1. Kiến trúc đa nhiệm FreeRTOS

Hệ thống được xây dựng trên nền tảng FreeRTOS (Real-Time Operating System) tích hợp trong ESP-IDF, cho phép thực thi đồng thời nhiều tác vụ với độ ưu tiên khác nhau. Kiến trúc phần mềm gồm bốn tác vụ (task) chính và bốn bộ định thời (software timer) được tổ chức như minh họa trong Hình 4.1.

### 4.1.1. Các tác vụ (Task)

**Bảng 4.1. Các tác vụ FreeRTOS trong hệ thống**

| Tên task | Độ ưu tiên | Kích thước Stack | Chu kỳ | Chức năng |
|----------|------------|------------------|--------|-----------|
| `mpu6050_task` | 5 | 4096 byte | 10 ms (100 Hz) | Đọc cảm biến, phát hiện té ngã |
| `btn_task` | 3 | 2048 byte | Sự kiện | Xử lý nút nhấn từ GPIO queue |
| `telegram_task` (queue-based) | 4 | 4096 byte | Sự kiện | Gửi HTTP request đến Telegram API qua hàng đợi (queue) |
| `main loop` (app_main) | 1 | — | 1 s | Kiểm tra WiFi, xử lý cờ Telegram |

**mpu6050_task** (priority 5) là tác vụ quan trọng nhất, thực hiện vòng lặp với tần số 100 Hz (mỗi 10 ms). Tác vụ này đọc dữ liệu thô từ cảm biến MPU6050 qua giao thức I2C, chuyển đổi sang đơn vị vật lý, tính góc Roll/Pitch bằng bộ lọc bổ sung (complementary filter), cập nhật double-buffer, và chạy máy trạng thái phát hiện té ngã.

**btn_task** (priority 3) chờ dữ liệu từ hàng đợi GPIO event (gpio_queue). Khi có ngắt từ nút nhấn, ISR ghi số hiệu GPIO vào queue; btn_task nhận và xác định thao tác press/release, từ đó khởi động hoặc dừng các software timer tương ứng.

**Main loop** (priority 1) chạy với chu kỳ 1 giây, kiểm tra các cờ `s_sos_telegram_pending` và `s_cancel_telegram_pending` (do timer callback đặt), gọi `telegram_send_sos_alert()` hoặc `telegram_send_cancel_alert()` để đẩy lệnh vào queue, đồng thời giám sát trạng thái kết nối WiFi.

### 4.1.2. Giao tiếp liên tác vụ dùng hàng đợi (Queue)

Hệ thống sử dụng hai hàng đợi để truyền thông tin giữa các tác vụ:

- **GPIO queue** (`gpio_queue`): ISR ngắt nút bấm ghi số hiệu GPIO (5 hoặc 6) vào queue. `btn_task` đọc queue và xử lý. Cơ chế này đảm bảo ISR không thực hiện các tác vụ chặn (blocking) như gọi `ESP_LOGI` hay `xTimerReset`.

- **Telegram queue** (`s_telegram_queue`): Các lời gọi HTTP đến Telegram API có thể chặn tác vụ (blocking), nên việc gửi tin nhắn được thực hiện qua queue. Timer callback (chạy trong ngữ cảnh ISR-like, không thể blocking) đặt cờ `s_sos_telegram_pending` hoặc `s_cancel_telegram_pending`; main loop kiểm tra cờ và gọi `telegram_send_sos_alert()` / `telegram_send_cancel_alert()`. Các hàm `telegram_send_*()` ghi lệnh vào `s_telegram_queue` qua `xQueueSend()`. `telegram_task` đọc queue trong vòng lặp `while(1)` và thực hiện HTTP request đến Telegram API.

### 4.1.3. Software Timer

Bốn software timer được sử dụng để quản lý thời gian:

- **alert_timeout** (30 s, one-shot): Tự động tắt báo động sau 30 giây, tránh trường hợp người dùng không can thiệp.
- **led_blink** (1 s, auto-reload): Tạo hiệu ứng LED nhấp nháy trong thời gian báo động.
- **sos_timer** (3 s, one-shot): Xác nhận giữ nút SOS đủ 3 giây trước khi kích hoạt báo động khẩn cấp.
- **cancel_timer** (3 s, one-shot): Xác nhận giữ nút CANCEL đủ 3 giây trước khi hủy báo động.

Cơ chế hold-to-confirm (giữ 3 giây) được áp dụng cho cả hai nút SOS và CANCEL nhằm tránh kích hoạt nhầm do va chạm vô tình.

## 4.2. Đọc và xử lý dữ liệu cảm biến MPU6050

### 4.2.1. Giao tiếp I2C và giải mã Big-Endian

MPU6050 giao tiếp với ESP32 qua bus I2C với địa chỉ 0x68, tần số 400 kHz (Fast Mode). Dữ liệu cảm biến được lưu trong các thanh ghi 16-bit theo định dạng big-endian (MSB trước, LSB sau). Để đọc toàn bộ dữ liệu accelerometer (6 byte), nhiệt độ (2 byte) và gyroscope (6 byte), vi điều khiển thực hiện giao thức *combined write-read* với một điều kiện khởi tạo lại (repeated start):

```
START → GHI 0xD0 + 0x3B → REPEATED START → ĐỌC 0xD1 + 14 BYTE → STOP
```

Giá trị raw 16-bit được ghép từ hai byte liên tiếp:

```c
int16_t raw_value = (data[0] << 8) | data[1];
```

### 4.2.2. Chuyển đổi sang đơn vị vật lý

Cảm biến được cấu hình ở dải đo ±8 g (accelerometer) và ±2000 °/s (gyroscope), tương ứng với độ nhạy:

- Accelerometer: 4096 LSB/g
- Gyroscope: 16.4 LSB/°/s

Công thức chuyển đổi:

```c
// Chuyển đổi từ raw sang m/s²
void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                           float *x, float *y, float *z) {
    float scale = MPU6050_GRAVITY / MPU6050_ACC_SCALE;  // 9.81 / 4096
    *x = (float)raw_x * scale - accel_bias[0];
    *y = (float)raw_y * scale - accel_bias[1];
    *z = (float)raw_z * scale - accel_bias[2];
}

// Chuyển đổi từ raw sang °/s
void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                          float *x, float *y, float *z) {
    *x = (float)raw_x / MPU6050_GYRO_SCALE - gyro_bias[0];  // 16.4
    *y = (float)raw_y / MPU6050_GYRO_SCALE - gyro_bias[1];
    *z = (float)raw_z / MPU6050_GYRO_SCALE - gyro_bias[2];
}
```

Độ lớn tổng hợp (magnitude) được tính bằng định lý Pythagoras trong không gian ba chiều:

- Gia tốc tổng: `|a| = sqrt(ax² + ay² + az²)` (đơn vị: g, chia cho 9.81)
- Vận tốc góc tổng: `|g| = sqrt(gx² + gy² + gz²)` (đơn vị: °/s)

### 4.2.3. Hiệu chuẩn hai pha (Two-phase calibration)

Do sai số chế tạo, cảm biến MEMS có độ lệch tĩnh (bias) khi đứng yên. Gyroscope lý tưởng đo 0 °/s khi không quay, nhưng thực tế có thể lệch ±10 °/s hoặc hơn. Nếu không hiệu chuẩn, sai số này tích lũy qua tích phân gây *drift* góc.

Quy trình hiệu chuẩn hai pha:

- **Pha 1 — Thu thập**: Lấy 500 mẫu (5 giây ở 100 Hz) khi thiết bị đứng yên. LED nhấp nháy để báo hiệu trạng thái hiệu chuẩn.
- **Pha 2 — Tính bias**: Tính trung bình cộng các mẫu gyroscope thu thập được. Kết quả là giá trị bias cần trừ trong các lần đọc sau.

Accelerometer bias không được tính riêng vì gia tốc trọng trường luôn hiện diện, không thể tách biệt bias và gravity chỉ từ dữ liệu accel.

### 4.2.4. Bộ lọc bổ sung (Complementary Filter) cho Roll/Pitch

Bộ lọc bổ sung kết hợp ưu điểm của hai cảm biến:

- **Accelerometer**: Cho góc tuyệt đối, không drift, nhưng nhiễu khi có gia tốc động (rung, lắc).
- **Gyroscope**: Cho góc mượt, đáp ứng nhanh, nhưng bị drift do tích phân sai số.

Công thức bộ lọc:

```
góc = α × (góc_cũ + gyro × dt) + (1 − α) × góc_accel
```

Với α = 0,80 (80% tin vào gyroscope, 20% tin vào accelerometer). Tần số cắt của bộ lọc:

```
fc = (1 − α) / (2π × Δt) = 0,20 / (2π × 0,01) ≈ 3,18 Hz
```

Tín hiệu accelerometer trên 3,18 Hz (nhiễu rung) bị suy giảm; tín hiệu gyroscope dưới 3,18 Hz (drift chậm) cũng bị suy giảm.

```c
void roll_pitch_update(float accel_x, float accel_y, float accel_z,
                       float gyro_x, float gyro_y, float gyro_z) {
    // Bước 1: Tính dt thực tế từ timestamp
    int64_t now = esp_timer_get_time();
    float dt = (now - prev_timestamp_us) / 1000000.0f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.050f) dt = 0.050f;
    prev_timestamp_us = now;

    // Bước 2: Góc từ accelerometer
    float accel_roll = atan2f(-accel_y, accel_z) * 180.0f / M_PI - 90.0f;
    float accel_pitch = atan2f(-accel_x,
        sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;

    // Bước 3: Tích phân gyroscope
    roll  += gyro_x * dt;
    pitch += gyro_y * dt;

    // Bước 4: Complementary filter
    roll  = ROLL_PITCH_ALPHA * roll  + (1.0f - ROLL_PITCH_ALPHA) * accel_roll;
    pitch = ROLL_PITCH_ALPHA * pitch + (1.0f - ROLL_PITCH_ALPHA) * accel_pitch;
}
```

Giá trị `dt` được tính từ timestamp thực tế (độ phân giải microsecond) thay vì giả định 10 ms cố định, nhằm bù cho sai lệch thời gian do hệ điều hành gây ra (jitter).

### 4.2.5. Cơ chế Double-Buffer

Để tránh xung đột dữ liệu giữa tác vụ ghi (mpu6050_task) và tác vụ đọc (HTTP server) mà không cần khóa mutex, hệ thống sử dụng kỹ thuật double-buffer (ping-pong):

```c
// Mảng hai buffer
sensor_data_t g_sensor_buffers[2];
static uint8_t s_writer_index = 0;

// Writer (mpu6050_task) — ghi vào buffer hiện tại, sau đó swap
void mpu6050_task(void *param) {
    while (1) {
        // ... đọc và xử lý dữ liệu cảm biến ...

        // Ghi vào buffer writer
        g_sensor_buffers[s_writer_index].total_accel_g = total_accel_g;
        g_sensor_buffers[s_writer_index].total_gyro = total_gyro;
        g_sensor_buffers[s_writer_index].roll = roll;
        g_sensor_buffers[s_writer_index].pitch = pitch;
        g_sensor_buffers[s_writer_index].data_ready = true;

        // Swap: writer chuyển sang buffer còn lại
        s_writer_index = 1 - s_writer_index;
    }
}

// Reader (HTTP handler) — đọc từ buffer còn lại
uint8_t reader_index = 1 - g_get_writer_index();
memcpy(&local_data, &g_sensor_buffers[reader_index], sizeof(sensor_data_t));
```

Cơ chế này đảm bảo reader và writer không bao giờ truy cập cùng một buffer tại cùng một thời điểm, loại bỏ hoàn toàn race condition mà không cần mutex.

## 4.3. Thuật toán phát hiện té ngã (State Machine 5 trạng thái)

### 4.3.1. Tổng quan

Thuật toán phát hiện té ngã dựa trên ba đặc điểm vật lý không thể thiếu của một cú ngã thật:

1. **Rơi tự do** (freefall): Gia tốc giảm đột ngột dưới 0,5 g (mất trọng lượng).
2. **Va chạm** (impact): Gia tốc tăng từ 2,0 g trở lên (phản lực từ mặt đất).
3. **Nằm yên** (lie down): Góc nghiêng ≥ 70°, vận tốc góc < 20 °/s trong 3,5 giây.

Máy trạng thái gồm năm trạng thái được tổ chức như sau:

```
                  ┌──────────────────────────────────────────┐
                  │                IDLE (0)                  │
                  │   (accel ~1 g, theo dõi bình thường)    │
                  └──────────┬───────────────────────────────┘
                             │ accel < 0,5 g (raw)
                             ▼
                  ┌──────────────────────────────────────────┐
                  │            FREEFALL (1)                  │
                  │   (mất trọng lượng, timer 150 ms)        │
                  └──────────┬───────────────────────────────┘
                             │ accel ≥ 2,0 g (raw)
                             ▼
                  ┌──────────────────────────────────────────┐
                  │            IMPACT (2)                    │
                  │   (va chạm, timer 1 s)                   │
                  └──────────┬───────────────────────────────┘
                             │ timer fire, max_tilt ≥ 70°
                             ▼
                  ┌──────────────────────────────────────────┐
                  │        WAIT_LIE_DOWN (3)                 │
                  │   (xác nhận nằm yên 3,5 s)               │
                  │   kiểm tra: tilt + gyro + accel          │
                  └──────────┬───────────────────────────────┘
                             │ ổn định 3,5 s
                             ▼
                  ┌──────────────────────────────────────────┐
                  │        SOS_TRIGGERED (4)                 │
                  │   (báo động đã kích hoạt, chờ reset)     │
                  └──────────────────────────────────────────┘
```

### 4.3.2. Chi tiết từng trạng thái

**IDLE (0):** Trạng thái bình thường. Hệ thống liên tục theo dõi gia tốc tổng (raw_accel). Nếu raw_accel giảm dưới ngưỡng 0,5 g, chuyển sang FREEFALL.

**FREEFALL (1):** Phát hiện rơi tự do. Một timer 150 ms được khởi động. Nếu có va chạm (raw_accel ≥ 2,0 g) trong thời gian này, chuyển sang IMPACT và dừng timer. Nếu timer hết mà không có impact, quay về IDLE (cơ chế chống nhiễu cho các chuyển động như lắc tay, cúi người nhanh).

**IMPACT (2):** Phát hiện va chạm. Timer 1 s được khởi động; trong thời gian này, cảm biến ổn định sau chấn động. Khi timer hết, kiểm tra góc nghiêng lớn nhất (max_tilt = max(|pitch|, |roll|)). Nếu max_tilt ≥ 70° (nạn nhân nằm), chuyển WAIT_LIE_DOWN. Nếu max_tilt < 70° (nạn nhân đã đứng dậy), reset về IDLE.

**WAIT_LIE_DOWN (3):** Xác nhận nằm yên trong 3,5 giây. Ba điều kiện phải đồng thời thỏa mãn:

- max_tilt ≥ 70° (góc nằm)
- gyro_dps < 20 °/s (không cử động)
- |filtered_accel − 1,0| < 0,3 g (cơ thể ổn định trên mặt đất)

Nếu có chuyển động (gyro ≥ 20 °/s), bộ đếm `stable_start` bị reset. Nếu góc giảm dưới 70°, reset về IDLE. Nếu cả ba điều kiện duy trì liên tục đủ 3,5 s, chuyển sang SOS_TRIGGERED.

**SOS_TRIGGERED (4):** Đã xác nhận té ngã. Gọi callback `fall_alert_callback()` để gửi Telegram và bật báo động. Chỉ có thể thoát khi gọi `fall_detection_reset()` từ bên ngoài (nút CANCEL).

### 4.3.3. Raw so với Filtered

Một điểm thiết kế quan trọng: **raw accel** được dùng cho phát hiện freefall và impact, trong khi **filtered accel** được dùng cho lying detection. Lý do:

Freefall và impact là các sự kiện rất nhanh (10–50 ms). Bộ lọc low-pass với α = 0,5 làm trễ tín hiệu khoảng 20–50 ms (2–5 mẫu ở 100 Hz), có thể bỏ lỡ hoàn toàn xung impact:

- Xung impact 20 ms với raw_accel = 2,5 g
- Filtered_accel sau 10 ms: 0,5 × 1,0 + 0,5 × 2,5 = 1,75 g (chưa vượt ngưỡng 2,0 g)
- Filtered_accel sau 20 ms: 0,5 × 1,75 + 0,5 × 2,5 = 2,125 g (đã muộn)

Ngược lại, lying detection yêu cầu ổn định trong hàng giây, độ trễ 20–50 ms là không đáng kể. Filter giúp loại bỏ nhiễu tần số cao, tránh dao động ngưỡng.

### 4.3.4. Mã nguồn máy trạng thái

```c
void fall_detection_update(float accel_g, float gyro_dps, float pitch, float roll) {
    // Lọc low-pass cả bốn kênh
    filtered_accel = low_pass_filter(accel_g, filtered_accel, config.filter_alpha);
    filtered_pitch = low_pass_filter(pitch, filtered_pitch, config.filter_alpha);
    filtered_roll  = low_pass_filter(roll, filtered_roll, config.filter_alpha);
    filtered_gyro  = low_pass_filter(gyro_dps, filtered_gyro, config.filter_alpha);

    float max_tilt = fmaxf(fabsf(filtered_pitch), fabsf(filtered_roll));
    float raw_accel = accel_g;

    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        switch (current_state) {
            case STATE_IDLE:
                if (raw_accel < config.accel_freefall_abs) {
                    current_state = STATE_FREEFALL;
                    start_timer(&s_freefall_timer,
                        freefall_timeout_callback, config.timeout_freefall);
                }
                break;

            case STATE_FREEFALL:
                if (raw_accel >= config.accel_impact_abs) {
                    stop_timer(&s_freefall_timer);
                    current_state = STATE_IMPACT;
                    start_timer(&s_impact_timer,
                        impact_timeout_callback, config.timeout_impact_check);
                }
                break;

            case STATE_IMPACT:
                // Chờ timer impact fire
                break;

            case STATE_WAIT_LIE_DOWN:
                if (max_tilt >= config.lying_angle_threshold) {
                    if (gyro_dps < 20.0f) {
                        if (stable_start == 0) {
                            stable_start = xTaskGetTickCount();
                        } else if ((xTaskGetTickCount() - stable_start) *
                                    portTICK_PERIOD_MS >= config.wait_lie_down_time) {
                            if (fabsf(filtered_accel - 1.0f) < 0.3f) {
                                current_state = STATE_SOS_TRIGGERED;
                                alert_triggered = true;
                                if (alert_callback != NULL) {
                                    xSemaphoreGive(s_state_mutex);
                                    alert_callback();
                                    // Lấy lại mutex sau callback
                                    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                                }
                            }
                        }
                    } else {
                        stable_start = 0;
                    }
                } else {
                    stable_start = 0;
                    current_state = STATE_IDLE;
                }
                break;

            case STATE_SOS_TRIGGERED:
                break;
        }
        xSemaphoreGive(s_state_mutex);
    }
}
```

## 4.4. Dashboard Web & JSON API

### 4.4.1. Kiến trúc HTTP Server

HTTP server được xây dựng trên thư viện `esp_http_server` của ESP-IDF, hoạt động ở cổng 80 với sáu URI handler được đăng ký:

**Bảng 4.2. Các URI handler của HTTP server**

| URI | Phương thức | Chức năng |
|-----|------------|-----------|
| `/` | GET | Trả về trang dashboard HTML |
| `/api/data` | GET | Trả về dữ liệu cảm biến dạng JSON |
| `/api/status` | GET | Trả về trạng thái thiết bị dạng JSON |
| `/favicon.ico` | GET | Trả về 204 No Content |
| `/api/data` | OPTIONS | CORS preflight |
| `/api/*` | OPTIONS | CORS preflight (wildcard) |

### 4.4.2. Dashboard HTML nhúng

Toàn bộ trang dashboard được lưu dưới dạng hằng chuỗi C (`static const char HTML_PAGE[]`). Đây là lựa chọn thiết kế có chủ đích nhằm:

- **Single-binary deployment**: Toàn bộ firmware là một binary duy nhất, không phụ thuộc vào SPIFFS (SPI Flash File System).
- **Độ tin cậy cao**: Không lo file HTML bị hỏng, xóa nhầm hoặc lỗi đọc flash.
- **Kích thước nhỏ**: Trang HTML chỉ khoảng 2 KB.

Giao diện dashboard gồm hai thẻ trạng thái (WiFi và Fall State) và bốn thẻ cảm biến (Acceleration, Gyroscope, Roll, Pitch) với theme tối.

### 4.4.3. JavaScript Polling và cập nhật DOM

Trang dashboard sử dụng JavaScript `fetch()` để gọi API `/api/data` mỗi 100 ms:

```javascript
function updateData() {
    fetch('/api/data')
        .then(function(r) { return r.json(); })
        .then(function(d) {
            var s = d.fall_state || 0;
            document.getElementById('accel_g').textContent = d.accel_g.toFixed(2);
            document.getElementById('gyro').textContent = d.gyro.toFixed(0);
            document.getElementById('roll').textContent = d.roll.toFixed(1);
            document.getElementById('pitch').textContent = d.pitch.toFixed(1);

            var w = document.getElementById('wifi');
            if (d.wifi_connected) {
                w.textContent = 'Connected';
                w.style.color = '#4ecca3';
            } else {
                w.textContent = 'Disconnected';
                w.style.color = '#e94560';
            }

            var e = document.getElementById('fall_state');
            e.textContent = SN[s] || 'UNKNOWN';
            e.style.color = SC[s] || '#fff';
            e.style.background = SL[s] || '#1a1a2e';
        });
}
setInterval(updateData, 100);
```

Màu sắc trạng thái fall được mã hóa bằng năm màu:

- IDLE (0): Xanh lá `#4ecca3` — an toàn
- FREEFALL (1): Vàng `#ffc107` — cảnh báo mức thấp
- IMPACT (2): Cam `#ff9800` — cảnh báo mức trung bình
- WAIT_LIE_DOWN (3): Cam `#ff9800` — đang xác nhận
- SOS (4): Đỏ `#e94560` — khẩn cấp

### 4.4.4. JSON API /api/data

Endpoint `/api/data` trả về dữ liệu dạng JSON với các trường:

```json
{
    "accel_g":       1.02,   // Gia tốc tổng hợp (g)
    "gyro":          3,      // Vận tốc góc tổng hợp (°/s)
    "roll":          -1.5,   // Góc Roll (°)
    "pitch":         0.8,    // Góc Pitch (°)
    "ready":         1,      // Dữ liệu cảm biến sẵn sàng
    "wifi_connected":1,      // Kết nối WiFi
    "alert_active":  0,      // Trạng thái báo động
    "uptime":        1234,   // Thời gian hoạt động (giây)
    "fall_state":    0,      // Trạng thái máy phát hiện ngã (0–4)
    "max_tilt":      2.1,    // Góc nghiêng lớn nhất (°)
    "filt_accel":    1.01    // Gia tốc đã lọc (g)
}
```

### 4.4.5. CORS Headers

Để hỗ trợ truy cập từ các ứng dụng web bên ngoài (cross-origin), HTTP server thiết lập các header CORS:

```c
static void set_cors_headers(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}
```

### 4.4.6. Luồng xử lý request tại data_get_handler

Khi HTTP server nhận request `GET /api/data`, hàm `data_get_handler` thực hiện theo thứ tự:

1. **Đọc double-buffer**: Tính `reader_index = 1 - g_get_writer_index()`, copy `sensor_data_t` từ buffer vào biến local bằng `memcpy`.
2. **Lấy fall detection state**: Gọi `fall_detection_get_state_internal()` (trả về 0-4), `fall_detection_get_max_tilt_internal()`, `fall_detection_get_filtered_accel_internal()`.
3. **Kiểm tra WiFi**: Gọi `wifi_is_connected()` từ module wifi.
4. **Build JSON response**: Dùng `snprintf` với format string chứa 11 trường JSON.
5. **Gửi response**: Gọi `send_json_response()` với CORS headers + content-type `application/json`.

```c
// Mã nguồn tóm lược xử lý API /api/data
static esp_err_t data_get_handler(httpd_req_t *req) {
    char buf[256];
    uint8_t fall_state = 0;
    float fall_max_tilt = 0.0f, fall_filtered_accel = 0.0f;

    // Bước 1: Đọc double-buffer
    sensor_data_t local_data;
    uint8_t reader_index = 1 - g_get_writer_index();
    memcpy(&local_data, &g_sensor_buffers[reader_index], sizeof(sensor_data_t));

    // Bước 2: Lấy fall detection state
    fall_state = fall_detection_get_state_internal();
    fall_max_tilt = fall_detection_get_max_tilt_internal();
    fall_filtered_accel = fall_detection_get_filtered_accel_internal();

    // Bước 3: WiFi status
    bool wifi_ok = wifi_is_connected();

    // Bước 4: Build JSON
    snprintf(buf, sizeof(buf),
        "{\"accel_g\":%.2f,\"gyro\":%.0f,\"roll\":%.1f,\"pitch\":%.1f,"
        "\"ready\":%d,\"wifi_connected\":%d,\"alert_active\":%d,\"uptime\":%lu,"
        "\"fall_state\":%d,\"max_tilt\":%.1f,\"filt_accel\":%.2f}",
        local_data.total_accel_g, local_data.total_gyro,
        local_data.roll, local_data.pitch, local_data.data_ready ? 1 : 0,
        wifi_ok ? 1 : 0, (fall_state == 4) ? 1 : 0, system_uptime_sec,
        fall_state, fall_max_tilt, fall_filtered_accel);

    // Bước 5: Gửi response
    return send_json_response(req, buf, strlen(buf));
}
```

Trường `alert_active` được tính bằng `(fall_state == 4) ? 1 : 0` — chỉ bằng 1 khi state machine ở trạng thái SOS_TRIGGERED.

---

## 4.5. Hệ thống gửi thông báo Telegram Bot

### 4.5.1. Kiến trúc async queue

Telegram Bot API yêu cầu gửi HTTP GET request đến máy chủ `api.telegram.org`. Quá trình này có thể block từ vài trăm mili giây đến vài giây (tùy tốc độ mạng). Do đó, hệ thống không thể gọi trực tiếp HTTP trong `mpu6050_task` (100Hz) hoặc trong timer callbacks (stack nhỏ). Giải pháp là kiến trúc **queue-based asynchronous**:

```
mpu6050_task / btn_task / timer callback
    │
    └── telegram_send_fall_alert() / telegram_send_sos_alert() / ...
            │
            └── xQueueSend(s_telegram_queue, &msg_type, 0)   ← non-blocking
                    │
                    ▼
            telegram_task (background, priority 4)
                    │
                    └── xQueueReceive(s_telegram_queue, ..., portMAX_DELAY)
                            │
                            └── send_telegram_message(TELEGRAM_MESSAGES[msg_type])
                                    │
                                    └── esp_http_client_perform()  ← blocking OK
```

**Các thành phần:**

- **`s_telegram_queue`**: FreeRTOS queue, sức chứa 4 message, mỗi message là `telegram_msg_type_t` (enum 4 giá trị).
- **`telegram_task`**: Task riêng chạy `while(1)`, đợi queue, gọi HTTP. Không ảnh hưởng đến các task khác khi bị block.
- **`telegram_send_*()`**: Các hàm public non-blocking, chỉ push vào queue, không đợi kết quả.

### 4.5.2. Luồng xử lý tin nhắn SOS và Cancel

Một ngoại lệ: timer callbacks (`sos_timer_callback`, `cancel_timer_callback`) chạy trong FreeRTOS daemon task, không thể gọi `xQueueSend` trực tiếp (daemon task stack rất nhỏ, không nên dùng queue API phức tạp). Do đó:

```
Timer callback (daemon task)
    │
    └── Đặt flag: s_sos_telegram_pending = true
            │
            ▼
    main loop (app_main, 1Hz)
            │
            └── if (flag) { flag = false; telegram_send_sos_alert(); }
                    │
                    └── xQueueSend(s_telegram_queue, ...)
                            │
                            ▼
                    telegram_task gửi HTTP
```

### 4.5.3. Cấu trúc message queue

```c
typedef enum {
    TELEGRAM_MSG_STARTUP,       // 0: Thiết bị khởi động
    TELEGRAM_MSG_FALL_ALERT,    // 1: Phát hiện té ngã
    TELEGRAM_MSG_SOS,           // 2: SOS thủ công
    TELEGRAM_MSG_CANCEL         // 3: Hủy báo động
} telegram_msg_type_t;
```

Mảng ánh xạ:

```c
static const char* TELEGRAM_MESSAGES[] = {
    MSG_STARTUP,       // "Thiết bị đã khởi động..."
    MSG_FALL_ALERT,    // "CẢNH BÁO: Phát hiện người bị ngã!"
    MSG_SOS_ALERT,     // "SOS: Nút khẩn cấp được nhấn!"
    MSG_CANCEL_ALERT   // "THÔNG BÁO: Cảnh báo đã được hủy."
};
```

Nội dung tin nhắn được định nghĩa trong file `telegram_messages.h` riêng — người dùng có thể chỉnh sửa mà không cần can thiệp code logic.

### 4.5.4. Gửi HTTP request

Hàm `send_telegram_message()` thực hiện các bước:

1. **URL Encode**: Mã hóa nội dung tin nhắn — chuyển ký tự đặc biệt (dấu cách, newline, Unicode) thành `%XX`.

```c
static void url_encode(const char *src, char *dst, int dst_size) {
    const char *hex = "0123456789ABCDEF";
    int j = 0;
    for (int i = 0; src[i] && j < dst_size - 3; i++) {
        unsigned char c = src[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            dst[j++] = c;                       // Giữ nguyên ký tự an toàn
        } else if (c == ' ') {
            dst[j++] = '%'; dst[j++] = '2'; dst[j++] = '0';  // Dấu cách → %20
        } else if (c == '\n') {
            dst[j++] = '%'; dst[j++] = '0'; dst[j++] = 'A';  // Xuống dòng → %0A
        } else {
            dst[j++] = '%'; dst[j++] = hex[c >> 4]; dst[j++] = hex[c & 0x0F];
        }
    }
    dst[j] = '\0';
}
```

2. **Build URL**: `https://api.telegram.org/bot{TOKEN}/sendMessage?chat_id={CHAT_ID}&text={MESSAGE}`

3. **Cấu hình HTTP client**:

```c
esp_http_client_config_t config = {
    .url = url,                          // URL đã build
    .method = HTTP_METHOD_GET,           // GET request
    .event_handler = http_event_handler, // Xử lý response
    .timeout_ms = 15000,                 // Timeout 15s
    .crt_bundle_attach = esp_crt_bundle_attach,  // SSL certificate bundle
};
```

4. **Thực thi và xử lý kết quả**:

```c
esp_err_t err = esp_http_client_perform(client);
if (err == ESP_OK) {
    int status = esp_http_client_get_status_code(client);
    if (status == 200) {
        // Thành công: Telegram đã nhận tin nhắn
    } else {
        // Thất bại: log status code
    }
}
```

### 4.5.5. Xử lý response

HTTP event handler `http_event_handler` được đăng ký trong cấu hình client. Khi có sự kiện `HTTP_EVENT_ON_DATA` (dữ liệu response đến), handler copy response vào buffer tĩnh `s_response_buffer`:

```c
static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA: {
        int len = evt->data_len;
        if (len > (int)sizeof(s_response_buffer) - 1) {
            len = sizeof(s_response_buffer) - 1;  // Tránh overflow
        }
        memcpy(s_response_buffer, evt->data, len);
        s_response_buffer[len] = '\0';
        break;
    }
    case HTTP_EVENT_ERROR:
        ESP_LOGE(TAG, "HTTP error");  // Lỗi kết nối
        break;
    default:
        break;
    }
    return ESP_OK;
}
```

### 4.5.6. Bảo mật SSL/TLS

ESP-IDF hỗ trợ SSL/TLS qua thư viện mbedTLS:

- **Certificate Bundle**: `esp_crt_bundle_attach` nhúng sẵn ~130 chứng chỉ gốc (root CA) từ Mozilla. Dùng để xác thực chứng chỉ của `api.telegram.org` mà không cần lưu riêng.
- **Yêu cầu trong CMakeLists**: component `telegram` REQUIRES `mbedtls`, `esp_http_client`, `cjson`.
- **Kết nối an toàn**: Toàn bộ dữ liệu gửi đến Telegram được mã hóa TLS 1.2/1.3.

### 4.5.7. Khởi tạo

```c
void telegram_init(const char *bot_token, const char *chat_id) {
    strncpy(s_bot_token, bot_token, sizeof(s_bot_token) - 1);
    strncpy(s_chat_id, chat_id, sizeof(s_chat_id) - 1);
    s_initialized = true;

    // Tạo queue 4 message
    s_telegram_queue = xQueueCreate(4, sizeof(telegram_msg_type_t));
    // Tạo task background, priority 4, stack 4096
    xTaskCreate(telegram_task, "telegram_task", 4096, NULL, 4, &s_telegram_task_handle);

    ESP_LOGI(TAG, "Telegram init: chat_id=%s", s_chat_id);
}
```

Queue sức chứa 4 message tương ứng 4 loại tin nhắn (startup, fall, sos, cancel) — đủ cho hầu hết kịch bản.

---

## 4.6. Kết nối WiFi và đồng bộ NTP

### 4.6.1. Chế độ Station và cơ chế kết nối

Thiết bị hoạt động ở chế độ WiFi Station (WPA2-PSK), kết nối đến router gia đình. Quá trình kết nối được điều khiển bởi hệ thống sự kiện (event-driven) của ESP-IDF, đăng ký handler cho cả hai event base `WIFI_EVENT` và `IP_EVENT`:

| Sự kiện | Hành động |
|---------|-----------|
| `WIFI_EVENT_STA_START` | Gọi `esp_wifi_connect()` lần đầu |
| `WIFI_EVENT_STA_CONNECTED` | Đặt `s_wifi_connected = true`, dừng retry timer |
| `WIFI_EVENT_STA_DISCONNECTED` | Đặt `s_wifi_connected = false`, khởi động retry timer 10s |
| `IP_EVENT_STA_GOT_IP` | In địa chỉ IP, khởi tạo SNTP, đồng bộ thời gian |

Khi nhận được IP qua DHCP, handler in địa chỉ IP và bắt đầu đồng bộ NTP:

```c
} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "WiFi CONNECTED - IP: " IPSTR, IP2STR(&ip_event->ip_info.ip));
    s_retry_num = 0;
    initialize_sntp();
    vTaskDelay(pdMS_TO_TICKS(2000));
    sync_time();
}
```

### 4.6.2. Cơ chế Retry vô hạn

Khi mất kết nối (`WIFI_EVENT_STA_DISCONNECTED`), một software timer one-shot 10 giây được khởi động. Khi timer cháy, gọi `esp_wifi_connect()` thử lại:

```c
static void wifi_retry_timer_callback(TimerHandle_t xTimer) {
    if (!s_wifi_connected) {
        if (s_retry_num >= WIFI_MAX_RETRY) {
            s_retry_num = 0;  // Reset → retry vô hạn
        }
        ESP_LOGI(TAG, "WiFi retry, attempting reconnect...");
        esp_wifi_connect();
        s_retry_num++;
    }
}
```

Bộ đếm `s_retry_num` được reset về 0 sau mỗi batch `WIFI_MAX_RETRY` lần retry, đảm bảo thiết bị luôn cố gắng kết nối lại bất kể thời gian mất mạng kéo dài.

### 4.6.3. Đồng bộ thời gian NTP

Sau khi có địa chỉ IP, hệ thống khởi tạo SNTP client với server `pool.ntp.org` và múi giờ ICT (UTC+7):

```c
static void initialize_sntp(void) {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}
```

Quá trình đồng bộ kiểm tra thời gian trong 10 lần × 500 ms = 5 giây. Thời gian được coi là hợp lệ nếu lớn hơn mốc 01/01/2021 (unix timestamp 1609451200). Sau khi đồng bộ thành công, thời gian được in ra console theo định dạng `YYYY-MM-DD HH:MM:SS` (giờ Việt Nam).

---
**Tổng kết Chương 4:** Chương này đã trình bày kiến trúc phần mềm của thiết bị phát hiện té ngã dựa trên FreeRTOS với bốn tác vụ (mpu6050_task, btn_task, telegram_task, main loop) và bốn software timer (alert_timeout, led_blink, sos_timer, cancel_timer). Thuật toán phát hiện té ngã sử dụng máy trạng thái năm trạng thái kết hợp raw accel cho sự kiện nhanh và filtered accel cho xác nhận ổn định. Dữ liệu cảm biến được xử lý qua bộ lọc bổ sung (complementary filter) với α = 0,80 và lưu trữ trong double-buffer để tránh xung đột liên tác vụ. Dashboard Web (HTML nhúng + JavaScript polling 100ms) và JSON API hai endpoint `/api/data` và `/api/status` cung cấp giao diện giám sát thời gian thực. Hệ thống Telegram sử dụng kiến trúc queue-based async với task riêng, đảm bảo non-blocking cho toàn hệ thống. Kết nối WiFi sử dụng cơ chế event-driven với retry vô hạn và đồng bộ NTP tự động.
