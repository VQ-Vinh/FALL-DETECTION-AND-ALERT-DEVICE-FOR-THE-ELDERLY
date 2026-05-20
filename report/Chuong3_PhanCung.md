# Chương 3: Thiết Kế Phần Cứng Hệ Thống

---

## 3.1. Sơ đồ khối tổng thể

Hệ thống phần cứng của thiết bị phát hiện té ngã được tổ chức theo mô hình tập trung, trong đó ESP32-C3 đóng vai trò là vi điều khiển trung tâm, giao tiếp với các khối ngoại vi thông qua các chuẩn giao tiếp I2C và GPIO. Sơ đồ khối tổng thể được mô tả như sau:

```
                                    ┌─────────────────────────┐
                                    │      Telegram API /     │
                                    │     Web Dashboard       │
                                    └──────────┬──────────────┘
                                               │  Internet
                                               │  (WiFi)
                    ┌──────────────────────────┴──────────────────────────┐
                    │                    ESP32-C3                         │
                    │  ┌────────────────────────────────────────────────┐  │
                    │  │   CPU: RISC-V 32-bit 160 MHz                  │  │
                    │  │   Bộ nhớ: 400 KB SRAM, 4 MB Flash             │  │
                    │  └────────────────────────────────────────────────┘  │
                    └──────┬───────┬──────┬──────┬───────────────────────┘
                      I2C  │       │GPIO0 │GPIO1 │  GPIO5       GPIO6
                           │       │      │      │     │            │
                    ┌──────┴──┐  ┌──┴──┐ ┌─┴──┐ ┌─┴─────────┐ ┌───┴────────┐
                    │MPU6050  │  │Buzzer│ │LED │ │Nút Cancel │ │Nút SOS     │
                    │(SDA/SCL)│  │      │ │    │ │(Input)    │ │(Input)     │
                    └─────────┘  └─────┘ └────┘ └───────────┘ └────────────┘
```

**Chức năng từng khối:**

- **ESP32-C3:** Vi điều khiển trung tâm của hệ thống. Thực hiện đọc dữ liệu cảm biến MPU6050 qua I2C, xử lý thuật toán phát hiện té ngã dựa trên gia tốc và vận tốc góc, điều khiển còi báo và đèn LED, đọc trạng thái nút nhấn, và truyền thông qua WiFi đến Telegram API hoặc Web Dashboard.

- **MPU6050:** Cảm biến quán tính 6 trục (gia tốc kế 3 trục + con quay hồi chuyển 3 trục). Cung cấp dữ liệu gia tốc tức thời (g) và vận tốc góc (°/s) theo ba trục X, Y, Z. Dữ liệu được truyền về ESP32-C3 qua chuẩn I2C ở tốc độ 400 kHz.

- **Buzzer:** Còi báo động chủ động (active buzzer), được kích hoạt ở mức cao 3,3 V từ GPIO 0. Khi phát hiện té ngã, ESP32-C3 sẽ xuất tín hiệu PWM hoặc mức logic cao để phát âm thanh cảnh báo.

- **LED:** Đèn báo hiệu trạng thái hoạt động của thiết bị. LED được nối tiếp với điện trở hạn dòng 220 Ω và điều khiển bởi GPIO 1. Các chế độ nháy khác nhau biểu thị trạng thái khởi tạo, hoạt động bình thường, phát hiện té ngã và kết nối WiFi.

- **Nút Cancel (GPIO 5):** Nút nhấn dùng để hủy báo động sai. Khi người dùng giữ nút trong 3 giây, thiết bị sẽ xác nhận không có té ngã và quay lại trạng thái giám sát.

- **Nút SOS (GPIO 6):** Nút nhấn khẩn cấp thủ công. Người dùng có thể giữ nút 3 giây để kích hoạt báo động khẩn cấp ngay cả khi không té ngã, hữu ích trong các tình huống cần trợ giúp y tế.

- **WiFi → Internet → Telegram API / Web Dashboard:** ESP32-C3 kết nối đến mạng WiFi hiện có, gửi thông báo qua Telegram Bot API và cập nhật trạng thái lên Web Dashboard. Đây là khối giao tiếp không dây, cho phép người thân hoặc trung tâm chăm sóc nhận được cảnh báo kịp thời.

---

## 3.2. Thiết kế và kết nối phần cứng

### 3.2.1. Linh kiện

Danh sách linh kiện sử dụng trong hệ thống được trình bày trong bảng sau:

| STT | Linh kiện | Số lượng | Chức năng |
|-----|-----------|----------|-----------|
| 1 | ESP32-C3-DevKit | 1 | Vi điều khiển trung tâm, xử lý thuật toán và kết nối WiFi |
| 2 | MPU6050 | 1 | Cảm biến gia tốc 3 trục và con quay hồi chuyển 3 trục |
| 3 | Buzzer 3,3 V | 1 | Còi báo động âm thanh khi phát hiện té ngã |
| 4 | LED đỏ 5 mm + Điện trở 220 Ω | 1 bộ | Đèn báo hiệu trạng thái thiết bị |
| 5 | Nút nhấn thường hở | 2 | Cancel (hủy báo động) và SOS (khẩn cấp thủ công) |
| 6 | Breadboard + Dây nối Dupont | 1 bộ | Kết nối và thử nghiệm mạch |

### 3.2.2. Sơ đồ kết nối chân

| ESP32-C3 | Linh kiện | Chức năng |
|----------|-----------|-----------|
| GPIO 8 (I2C SDA) | MPU6050 SDA | Đường truyền dữ liệu I2C |
| GPIO 9 (I2C SCL) | MPU6050 SCL | Đường xung clock I2C (400 kHz) |
| GPIO 0 | Buzzer (chân điều khiển) | Bật/tắt còi báo động |
| GPIO 1 | LED + Resistor 220 Ω → GND | Nhấp nháy báo hiệu trạng thái |
| GPIO 5 | Nút Cancel → GND (pull-up nội) | Hủy báo động (giữ 3 giây) |
| GPIO 6 | Nút SOS → GND (pull-up nội) | Kích hoạt SOS thủ công (giữ 3 giây) |
| 3,3 V | MPU6050 VCC, Buzzer VCC | Cấp nguồn cho các linh kiện |
| GND | Tất cả linh kiện (GND chung) | Tham chiếu điện áp chung |

### 3.2.3. Giải thích thiết kế

#### Lựa chọn vi điều khiển

ESP32-C3 được chọn làm vi điều khiển trung tâm nhờ các ưu điểm: kiến trúc RISC-V 32-bit tiết kiệm năng lượng, tích hợp WiFi và Bluetooth 5 (LE), bộ nhớ SRAM 400 KB đủ cho thuật toán xử lý tín hiệu cảm biến, cùng với khả năng lập trình qua Arduino Framework và ESP-IDF. So với ESP32 thế hệ cũ, ESP32-C3 tiêu thụ dòng điện thấp hơn (~50 mA khi hoạt động), phù hợp với các thiết bị đeo.

#### Giao tiếp MPU6050 qua I2C

MPU6050 giao tiếp với ESP32-C3 qua chuẩn I2C ở tốc độ 400 kHz (Fast Mode). Địa chỉ I2C mặc định của MPU6050 là 0x68. Việc sử dụng I2C giúp tiết kiệm chân GPIO so với giao tiếp SPI, đồng thời đảm bảo tốc độ đọc dữ liệu đủ nhanh (tối đa 1.000 mẫu/giây cho gia tốc kế) phục vụ thuật toán phát hiện té ngã thời gian thực. Hai điện trở kéo lên 4,7 kΩ được gắn trên đường SDA và SCL (có sẵn trên module MPU6050).

#### Thiết kế nút nhấn với pull-up nội

Cả hai nút nhấn Cancel và SOS được đấu nối theo cấu hình: một chân nối với GPIO tương ứng, chân còn lại nối xuống GND. ESP32-C3 được lập trình kích hoạt điện trở pull-up nội (khoảng 45 kΩ) cho các chân GPIO 5 và GPIO 6. Khi không nhấn, GPIO đọc mức cao; khi nhấn, GPIO đọc mức thấp. Giải pháp này giúp tiết kiệm linh kiện ngoài (không cần điện trở kéo lên rời).

#### LED báo hiệu

LED đỏ được nối tiếp với điện trở hạn dòng 220 Ω giữa GPIO 1 và GND. Giá trị điện trở được tính toán: với điện áp rơi trên LED khoảng 2 V và dòng mong muốn 10 mA, ta có R = (3,3 - 2) / 0,01 ≈ 130 Ω. Chọn 220 Ω để đảm bảo an toàn và giảm độ sáng vừa phải, dòng thực tế khoảng 6 mA.

#### Buzzer

Buzzer sử dụng là loại active buzzer 3,3 V, chỉ cần đặt mức logic cao ở chân điều khiển là phát âm thanh tần số cố định (~2.300 Hz). Không cần tín hiệu PWM phức tạp, giúp đơn giản hóa chương trình điều khiển.

#### Vị trí đeo thiết bị

Thiết bị được thiết kế để đeo ở thắt lưng người dùng. Khi đeo, trục X của MPU6050 hướng thẳng đứng lên trên, trục Y hướng ngang và trục Z hướng về phía trước. Cách bố trí này giúp thuật toán dễ dàng phân biệt các tư thế đứng, ngồi, nằm dựa trên thành phần gia tốc trọng trường trên từng trục. Khi người dùng đứng thẳng, gia tốc trên trục X xấp xỉ +1 g; khi ngã, các thành phần gia tốc biến đổi đột ngột theo cả ba trục, tạo cơ sở cho thuật toán phát hiện.

---

## 3.3. Nguyên lý hoạt động tổng thể

Hệ thống hoạt động theo chu trình: đầu tiên, ESP32-C3 khởi tạo MPU6050 và kết nối WiFi. Sau đó, vi điều khiển liên tục đọc dữ liệu từ MPU6050 ở tần số 100 Hz (chu kỳ 10 ms). Mỗi mẫu dữ liệu bao gồm gia tốc theo ba trục (ax, ay, az) và vận tốc góc (gx, gy, gz). Thuật toán phát hiện té ngã được thực thi trên từng mẫu dữ liệu, dựa trên ngưỡng gia tốc tổng hợp và phát hiện va chạm.

Khi phát hiện té ngã, ESP32-C3 thực hiện đồng thời các hành động: bật buzzer phát âm thanh liên tục, nháy LED với chu kỳ 1000 ms (1 Hz — 500 ms bật, 500 ms tắt), gửi thông báo qua Telegram API. Hệ thống khởi động timer tự động tắt báo động sau 30 giây (hằng số `ALERT_DURATION_MS`). Trong khoảng thời gian này, người dùng có thể giữ nút Cancel trong 3 giây để hủy báo động giả. Nếu không có phản hồi, timer sẽ cháy và gọi hàm `stop_fall_alert()`: tắt buzzer, dừng nhấp nháy LED, gọi `fall_detection_reset()` để đưa máy trạng thái phát hiện ngã về trạng thái IDLE, sẵn sàng phát hiện lần ngã tiếp theo. Riêng trạng thái lỗi (mất kết nối I2C cảm biến hoặc WiFi) mới sử dụng LED nhấp nháy nhanh với chu kỳ 200 ms để báo hiệu, không kích hoạt còi.
