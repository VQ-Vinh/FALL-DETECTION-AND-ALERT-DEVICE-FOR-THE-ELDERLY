# Chương 5: Kết Quả & Đánh Giá

## 5.1. Kết quả phần cứng

Sau quá trình lắp ráp và kiểm tra, thiết bị phát hiện té ngã đã được hoàn thiện ở dạng nguyên mẫu (prototype) trên breadboard. Toàn bộ linh kiện được kết nối theo sơ đồ chân đã trình bày tại Chương 3, sử dụng bo mạch phát triển ESP32-C3-DevKit-M-1 làm khối xử lý trung tâm. Cảm biến MPU6050 được giao tiếp qua chuẩn I2C (SCL - GPIO9, SDA - GPIO8) và cấp nguồn 3.3V từ chân ra của ESP32-C3. Các thiết bị ngoại vi bao gồm còi báo (buzzer) kết nối qua GPIO0, đèn LED chỉ thị qua GPIO1, và hai nút nhấn vật lý tại GPIO5 (Cancel) và GPIO6 (SOS) sử dụng điện trở kéo lên nội (GPIO_PULLUP_ENABLE).

[Hình 5.1: Sơ đồ nguyên mẫu trên breadboard]

Nhằm phục vụ mục đích đeo trên thắt lưng (belt-wearing), các linh kiện được bố trí gọn trên một breadboard mini, với ESP32-C3 đặt ở trung tâm và MPU6050 được hàn cố định lên một board mở rộng nhỏ để giảm thiểu rung động cơ học không mong muốn. Cảm biến được định hướng sao cho trục Y của MPU6050 thẳng đứng khi người dùng đứng, giúp việc xác định góc nghiêng Roll và Pitch chính xác hơn khi xử lý thuật toán. Toàn bộ hệ thống được cấp nguồn qua cổng USB-C của ESP32-C3, cho phép sử dụng linh hoạt với adapter nguồn 5V hoặc pin dự phòng di động.

[Hình 5.2: Prototype hoàn chỉnh – mặt trên và mặt dưới]

## 5.2. Kết quả phần mềm

### 5.2.1. Dashboard Web

Dashboard web được triển khai trên chính ESP32-C3 thông qua thư viện HTTP Server (esp_http_server), truy cập tại địa chỉ `http://<dia_chi_ip_esp32>/`. Giao diện sử dụng theme tối (dark theme) với bố cục thẻ (cards) responsive, tương thích với cả màn hình máy tính và thiết bị di động. Dashboard hiển thị bốn thành phần chính:

- **Trạng thái WiFi**: hiển thị "Connected" với nền xanh lá hoặc "Disconnected" với nền đỏ.
- **Trạng thái ngã (Fall State)**: thể hiện trạng thái hiện tại của bộ máy trạng thái với mã màu tương ứng — IDLE (xanh lá), FREEFALL (vàng), IMPACT (cam), WAIT_LIE_DOWN (cam), SOS (đỏ).
- **Giá trị cảm biến**: gia tốc tổng hợp (g), tốc độ góc (độ/giây), góc Roll và Pitch (độ).
- **Thời gian hoạt động (Uptime)**: số giây hệ thống đã chạy kể từ khi khởi động.

[Hình 5.3: Giao diện dashboard web]

Trang web sử dụng JavaScript để gửi request fetch() tới endpoint `/api/data` mỗi 100ms, cập nhật giá trị cảm biến và trạng thái theo thời gian thực mà không cần tải lại trang. Dashboard hoạt động ở chế độ chỉ đọc (read-only), không có khả năng điều khiển từ xa. Các endpoint API bao gồm:
- `/api/data` – trả về dữ liệu cảm biến và trạng thái dưới dạng JSON.
- `/api/status` – trả về trạng thái thiết bị (WiFi, uptime) dưới dạng JSON.
- `/` – trang dashboard chính (HTML/CSS/JS nhúng).

### 5.2.2. Telegram Bot thông báo

Hệ thống sử dụng Telegram Bot API để gửi thông báo đến người chăm sóc từ xa. Bot được khởi tạo thông qua token từ BotFather và gửi tin nhắn tới chat ID đã cấu hình sẵn. Các thông báo được định nghĩa với nội dung như sau:

```
Thiết bị đã khởi động. Sẵn sàng giám sát người thân!
```
(Gửi khi ESP32-C3 khởi động và kết nối WiFi thành công)

```
CẢNH BÁO: Phát hiện người bị ngã!
```
(Gửi khi hệ thống xác nhận té ngã sau giai đoạn WAIT_LIE_DOWN)

```
SOS: Nút khẩn cấp được nhấn!
```
(Gửi khi nút nhấn khẩn cấp được giữ trong 3 giây)

```
THÔNG BÁO: Cảnh báo đã được hủy.
```
(Gửi khi cảnh báo được hủy qua nút Cancel)

[Hình 5.4: Ví dụ tin nhắn Telegram từ thiết bị]

### 5.2.3. Nhật ký hệ thống (log)

Hệ thống ghi log chi tiết qua cổng Serial ở tốc độ 115200 baud, phục vụ quá trình debug và giám sát thời gian thực. Dưới đây là một đoạn log điển hình:

```
=========================================
    FALL DETECTION SYSTEM v1.0
    ESP32-C3 + MPU6050
=========================================
Initializing MPU6050...
MPU6050 connected successfully!
Calibrating...
Calibration done (offsets: ax=+12  ay=-34  az=+8  gx=-1  gy=+2  gz=0)
WiFi CONNECTED - IP: 192.168.1.42
Telegram Bot started OK
System ready!

[SYSTEM] Heartbeat - WiFi: OK | Alert: IDLE | Error: NO | Uptime: 12s
[SYSTEM] Heartbeat - WiFi: OK | Alert: IDLE | Error: NO | Uptime: 62s
[SENSOR] ax=0.02 ay=0.01 az=1.02 | gx=0.5 gy=-0.3 gz=0.1 | Roll=0.2 Pitch=-1.1
-> STATE_FREEFALL (accel: 0.32g)
-> STATE_IMPACT (accel: 2.15g)
-> STATE_WAIT_LIE_DOWN (roll=85.2 pitch=12.4)
-> STATE_SOS (lie down detected)
>>> FALL ALERT STARTED <<<
Telegram message sent: FALL ALERT
[SYSTEM] Heartbeat - WiFi: OK | Alert: FALL | Error: NO

[CANCEL] Button pressed
>>> FALSE ALARM CANCELLED <<<
Telegram message sent: CANCELLED
-> STATE_IDLE
```

Log hệ thống cung cấp thông tin chi tiết về chuyển đổi trạng thái kèm giá trị cảm biến tương ứng, giúp kỹ thuật viên đánh giá chính xác hành vi của thuật toán.

## 5.3. Đánh giá hiệu năng

### 5.3.1. Độ trễ phát hiện té ngã

Độ trễ của hệ thống được đánh giá dựa trên tần số lấy mẫu và các ngưỡng thời gian trong thuật toán:

| Giai đoạn | Điều kiện | Độ trễ lý thuyết |
|-----------|-----------|-------------------|
| Lấy mẫu cảm biến | 100Hz → chu kỳ 10ms | 10ms |
| Phát hiện rơi tự do (FREEFALL) | Gia tốc toàn phần < 0.5g | ≤ 10ms (mẫu tiếp theo) |
| Phát hiện va chạm (IMPACT) | Gia tốc toàn phần ≥ 2.0g | ≤ 10ms (mẫu tiếp theo) |
| Chờ nằm yên (WAIT_LIE_DOWN) | 3.5 giây ổn định | 3500ms (cố định) |
| **Tổng thời gian phát hiện** | Từ khi rơi đến SOS | **~3.52 – 3.54 giây** |

Thời gian chờ 3.5 giây ở trạng thái WAIT_LIE_DOWN là một thiết kế có chủ đích nhằm dung hòa giữa độ nhạy và độ đặc hiệu. Khoảng thời gian này đủ dài để loại bỏ các tín hiệu nhiễu do người dùng ngồi xuống nhanh hoặc nằm xuống có chủ ý, nhưng đủ ngắn để đảm bảo cảnh báo được gửi kịp thời trong tình huống té ngã thực tế.

Độ trễ gửi thông báo Telegram phụ thuộc vào độ trễ mạng WiFi và thời gian xử lý của Telegram API, thường dao động từ 500ms đến 2 giây trong điều kiện mạng ổn định.

### 5.3.2. Tỷ lệ phát hiện đúng

Hệ thống sử dụng cơ chế xác thực ba giai đoạn (freefall → impact → lying), giúp loại bỏ phần lớn các trường hợp dương tính giả (false positive). Phân tích lý thuyết:

- **Rơi tự do (gia tốc < 0.5g)**: sử dụng ngưỡng thấp đảm bảo không bỏ sót sự kiện rơi. Gia tốc trung bình khi rơi tự do tiến về 0g nếu không có lực cản không khí, gần với 0.3g – 0.4g trong điều kiện thực tế.
- **Va chạm (gia tốc ≥ 2.0g)**: ngưỡng này cao hơn biên độ gia tốc thông thường (tối đa ~1.5g khi vận động mạnh), giúp tránh phát hiện nhầm khi người dùng chạy nhảy hay leo cầu thang.
- **Nằm yên (max_tilt = max(|Pitch|, |Roll|) ≥ 70° và tốc độ góc < 20°/s)**: kiểm tra đồng thời góc nghiêng và sự ổn định về góc, đảm bảo người dùng thực sự nằm ở tư thế không bình thường.
- **Xác nhận gia tốc ≈ 1g**: kiểm tra gia tốc tổng hợp xấp xỉ 1g khi nằm yên, xác nhận cảm biến vẫn hoạt động chính xác ở trạng thái tĩnh.

Thuật toán xử lý trên dữ liệu thô (raw acceleration) thay vì dữ liệu đã lọc để tránh làm trễ tín hiệu va chạm. Trong trường hợp các hoạt động hàng ngày như cúi người nhặt đồ, hệ thống có thể chuyển sang FREEFALL trong thời gian ngắn nhưng không thỏa mãn điều kiện va chạm (gia tốc < 2.0g) nên sẽ quay về IDLE sau 150ms timeout.

### 5.3.3. Tiêu thụ năng lượng

Mức tiêu thụ năng lượng của hệ thống được ước tính như sau:

| Linh kiện | Dòng điện (mA) | Ghi chú |
|-----------|---------------|---------|
| ESP32-C3 (WiFi bật, polling) | ~80 | Ở chế độ active |
| MPU6050 | ~3.9 | Ở chế độ hoạt động bình thường |
| Buzzer | ~30 | Chỉ kích hoạt khi có cảnh báo |
| LED chỉ thị | ~10 | Nhấp nháy tùy trạng thái |
| **Tổng (trung bình)** | **~100** | Khi không có buzzer/LED tối đa |

Trong điều kiện hoạt động bình thường (trạng thái IDLE), mức tiêu thụ đo được khoảng 85–90mA. Khi có cảnh báo với buzzer và LED hoạt động, dòng có thể tăng lên 130–140mA nhưng chỉ trong thời gian ngắn (tối đa 30 giây timeout). Thiết bị hoạt động ở điện áp 5V qua cổng USB-C, phù hợp với các nguồn pin dự phòng 5000mAh thông dụng (cho thời gian hoạt động liên tục khoảng 50–55 giờ).

[Hình 5.5: Đồ thị dòng tiêu thụ theo các trạng thái hoạt động]

---

# Chương 6: Kết Luận & Hướng Phát Triển

## 6.1. Kết luận

Đề tài "Thiết bị phát hiện té ngã cho người cao tuổi" đã đạt được các mục tiêu đề ra ban đầu. Một thiết bị IoT hoàn chỉnh đã được xây dựng dựa trên vi điều khiển ESP32-C3 và cảm biến quán tính MPU6050, có khả năng phát hiện sự kiện té ngã và thông báo tức thời đến người chăm sóc qua nhiều kênh khác nhau. Cụ thể:

1. **Phát hiện té ngã**: Thuật toán 5 trạng thái (IDLE – FREEFALL – IMPACT – WAIT_LIE_DOWN – SOS) được phát triển dựa trên phân tích gia tốc và tốc độ góc, giúp phát hiện chính xác sự kiện té ngã với cơ chế xác thực ba giai đoạn nhằm giảm thiểu cảnh báo sai.

2. **Cảnh báo đa kênh**: Hệ thống cảnh báo đồng thời qua ba kênh — còi báo và đèn LED tại chỗ, dashboard web hiển thị trạng thái trực quan, và Telegram Bot gửi thông báo đến thiết bị di động của người chăm sóc. Điều này đảm bảo cảnh báo đến được người nhận ngay cả khi họ không ở gần thiết bị.

3. **Nút nhấn vật lý**: Nút khẩn cấp (SOS) và nút hủy (Cancel) được thiết kế với cơ chế nhấn giữ 3 giây nhằm tránh thao tác nhầm, phù hợp với đặc điểm tâm lý và thể trạng của người cao tuổi.

4. **Vận hành tự động**: Bộ đếm thời gian 30 giây giúp hệ thống tự động trở về trạng thái giám sát IDLE sau khi phát cảnh báo, đảm bảo thiết bị luôn sẵn sàng cho sự kiện tiếp theo mà không cần can thiệp thủ công.

## 6.2. Hạn chế

Bên cạnh các kết quả đạt được, hệ thống hiện tại còn tồn tại một số hạn chế cần được khắc phục trong các phiên bản tiếp theo:

1. **Chưa có quản lý năng lượng (power management)**: Hệ thống hoạt động liên tục ở chế độ active với WiFi luôn bật, dẫn đến tiêu thụ năng lượng tương đối cao (~100mA). Chưa có cơ chế chuyển sang chế độ ngủ sâu (deep sleep) để tiết kiệm pin khi không phát hiện chuyển động trong thời gian dài.

2. **Phụ thuộc vào kết nối WiFi**: Khả năng cảnh báo từ xa hoàn toàn phụ thuộc vào kết nối WiFi. Khi mạng không khả dụng hoặc mất kết nối, hệ thống chỉ có thể cảnh báo tại chỗ qua còi và đèn LED, làm giảm đáng kể hiệu quả giám sát.

3. **Sử dụng cảm biến đơn (single sensor)**: Hệ thống chỉ dựa trên một cảm biến MPU6050 duy nhất để phát hiện té ngã. Việc kết hợp thêm các cảm biến khác như khí áp kế (barometer) để đo độ cao, từ kế (magnetometer) để hỗ trợ định hướng, hoặc camera thị giác máy tính sẽ cải thiện độ chính xác và giảm tỷ lệ cảnh báo sai.

4. **Chưa được thử nghiệm trên đối tượng thực tế**: Thiết bị mới chỉ được thử nghiệm trên đối tượng khỏe mạnh trong điều kiện mô phỏng tình huống té ngã với đệm bảo hộ. Chưa có cơ hội thử nghiệm trên người cao tuổi thực tế để đánh giá độ tin cậy trong điều kiện sử dụng hàng ngày.

5. **Chưa có lưu trữ dữ liệu đám mây**: Các dữ liệu cảm biến và sự kiện mới chỉ được lưu tạm thời trên RAM của ESP32, chưa được lưu trữ lên nền tảng đám mây để phục vụ phân tích dài hạn và xây dựng mô hình học máy.

## 6.3. Hướng phát triển

Dựa trên các kết quả đã đạt được cũng như các hạn chế hiện tại, đề tài có thể được phát triển theo các hướng sau:

1. **Chế độ ngủ sâu và đánh thức bằng chuyển động**: Tận dụng khả năng deep sleep của ESP32-C3 và chân ngắt chuyển động (motion interrupt) của MPU6050, hệ thống có thể chuyển sang chế độ ngủ vi năng (~10µA) và tự động đánh thức khi phát hiện chuyển động mạnh. Giải pháp này kéo dài thời gian hoạt động trên pin từ vài chục giờ lên đến vài tháng.

2. **Edge Machine Learning (TinyML)**: Triển khai mô hình học máy nhẹ như Decision Tree, Random Forest hoặc mạng CNN tối giản lên chính ESP32-C3 sử dụng TensorFlow Lite Micro. Các mô hình này có thể phân loại hoạt động (đi, chạy, ngồi, nằm, té ngã) trực tiếp trên thiết bị với độ chính xác cao hơn so với phương pháp ngưỡng thủ công.

3. **Nền tảng đám mây (Cloud Backend)**: Xây dựng hệ thống lưu trữ dữ liệu trên AWS IoT Core, Azure IoT Hub hoặc dịch vụ tương tự. Dữ liệu cảm biến và sự kiện được đồng bộ lên đám mây, cho phép truy xuất lịch sử, phân tích xu hướng và xây dựng dashboard quản lý tập trung cho nhiều thiết bị.

4. **Tích hợp GPS**: Bổ sung module GPS (ví dụ NEO-6M hoặc SIM7000) cho phép xác định vị trí của người dùng ngoài trời, hỗ trợ công tác cứu hộ khi có sự kiện té ngã xảy ra bên ngoài khuôn viên nhà.

5. **Đa cảm biến (Multi-sensor Fusion)**: Kết hợp thêm cảm biến khí áp (BMP280/BME280) để phát hiện thay đổi độ cao đột ngột khi té ngã, và cảm biến từ trường (magnetometer) để cải thiện độ chính xác của phép đo góc nghiêng thông qua bộ lọc bổ sung (sensor fusion).

6. **Ứng dụng di động**: Phát triển ứng dụng di động (Android/iOS) thay thế Telegram Bot, cung cấp giao diện giám sát trực quan hơn với biểu đồ thời gian thực, lịch sử sự kiện, và các tùy chọn cấu hình linh hoạt như thay đổi ngưỡng cảm biến, thời gian chờ, và danh sách người nhận thông báo.
