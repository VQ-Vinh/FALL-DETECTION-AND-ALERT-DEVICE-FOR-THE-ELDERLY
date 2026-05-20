# BÁO CÁO ĐỒ ÁN IoT

**Đề tài:** Thiết bị phát hiện té ngã cho người cao tuổi
**Vi điều khiển:** ESP32-C3
**Cảm biến:** MPU6050 (IMU 6-axis)

---

## Chương 1: Giới Thiệu

### 1.1. Tính cấp thiết của đề tài

Theo thống kê của Tổ chức Y tế Thế giới (WHO), mỗi năm có khoảng 646.000 người tử vong do té ngã trên toàn cầu, khiến té ngã trở thành nguyên nhân tử vong thứ hai do tai nạn thương tích, chỉ sau tai nạn giao thông. Đối với người cao tuổi trên 65 tuổi, té ngã là nguyên nhân hàng đầu dẫn đến chấn thương nghiêm trọng và nhập viện. Khoảng 28–35% người trên 65 tuổi trải qua ít nhất một lần té ngã mỗi năm, và tỷ lệ này gia tăng theo độ tuổi.

Một vấn đề nghiêm trọng khác là thời gian nằm trên sàn sau khi té ngã (long-lie) — nếu người bị té không được phát hiện và hỗ trợ trong vòng 1 giờ, nguy cơ biến chứng (viêm phổi, hạ thân nhiệt, mất nước) tăng cao đáng kể. Đối với người cao tuổi sống một mình — vốn là xu hướng ngày càng phổ biến tại các đô thị lớn ở Việt Nam — việc không được phát hiện kịp thời sau té ngã là một rủi ro lớn.

Sự phát triển của Internet vạn vật (IoT) và công nghệ cảm biến vi cơ điện tử (MEMS) đã mở ra khả năng xây dựng các thiết bị đeo thông minh với chi phí thấp, có khả năng phát hiện té ngã tự động và gửi cảnh báo tức thời đến người thân hoặc trung tâm hỗ trợ. Vi điều khiển ESP32-C3 của Espressif là một lựa chọn phù hợp nhờ giá thành rẻ, kích thước nhỏ gọn, tích hợp sẵn WiFi và Bluetooth LE cùng hệ sinh thái phát triển ESP-IDF mạnh mẽ dựa trên FreeRTOS.

Đề tài "Thiết bị phát hiện té ngã cho người cao tuổi" được đề xuất với mục tiêu tạo ra một giải pháp khả thi, giá rẻ, có thể triển khai tại hộ gia đình, giúp giảm thiểu rủi ro cho người cao tuổi sống một mình.

### 1.2. Mục tiêu của đồ án

Đồ án hướng đến các mục tiêu cụ thể sau:

1. **Thiết kế và chế tạo thiết bị đeo** có khả năng phát hiện té ngã tự động dựa trên cảm biến quán tính MPU6050, sử dụng vi điều khiển ESP32-C3 làm trung tâm xử lý.

2. **Xây dựng thuật toán phát hiện té ngã** dựa trên mô hình máy trạng thái (state machine) với 5 trạng thái: Idle → Freefall → Impact → Lying → Alert, phân tích tín hiệu gia tốc và vận tốc góc từ MPU6050.

3. **Triển khai hệ thống cảnh báo đa kênh:** còi báo động (buzzer), đèn LED nhấp nháy, gửi tin nhắn Telegram qua Bot API, và hiển thị trạng thái qua giao diện web HTTP trên thiết bị.

4. **Xây dựng cơ chế hủy báo động giả** (false alarm cancellation) bằng nút nhấn vật lý, cho phép người dùng hủy cảnh báo trong thời gian 30 giây, kết hợp tự động reset nếu không phát hiện thêm tác động.

### 1.3. Phạm vi và giới hạn

**Phạm vi:**
- Phát hiện các cú ngã đột ngột điển hình (forward fall, backward fall, lateral fall, syncope fall).
- Hoạt động trong môi trường trong nhà với phủ sóng WiFi.
- Thiết bị đeo ở vị trí thắt lưng hoặc ngực.

**Giới hạn:**
- Chưa áp dụng các phương pháp học máy (Machine Learning) hoặc trí tuệ nhân tạo (AI) cho bài toán phát hiện té ngã.
- Chưa xây dựng hệ thống cloud backend lưu trữ dữ liệu lịch sử.
- Phụ thuộc vào kết nối WiFi để gửi cảnh báo Telegram — khi mất kết nối, chỉ còn cảnh báo cục bộ (còi + LED).
- Không bao gồm phát hiện té ngã từ từ (gradual fall) hoặc trượt ngã (slip fall).
- Thiết bị chưa được thử nghiệm lâm sàng trên đối tượng người cao tuổi thực tế; các thử nghiệm được thực hiện trên tình nguyện viên trẻ với mô phỏng động tác ngã.

### 1.4. Phương pháp thực hiện

Đồ án được triển khai theo quy trình gồm các bước sau:

1. **Nghiên cứu lý thuyết:** Khảo sát các nghiên cứu hiện có về đặc điểm té ngã của người cao tuổi, nguyên lý hoạt động của cảm biến MEMS, các thuật toán phát hiện té ngã dựa trên ngưỡng gia tốc và vận tốc góc, tổng quan giao thức I2C và HTTP.

2. **Thiết kế phần cứng:** Xây dựng sơ đồ khối hệ thống, lựa chọn linh kiện (ESP32-C3 Super Mini, MPU6050, buzzer, LED, nút nhấn), thiết kế sơ đồ kết nối chân và sơ đồ nguyên lý, tính toán nguồn cấp và các điện trở kéo lên I2C.

3. **Lập trình firmware:** Phát triển phần mềm nhúng trên nền tảng ESP-IDF (Espressif IoT Development Framework) với hệ điều hành thời gian thực FreeRTOS, cấu trúc đa tác vụ (multitasking) bao gồm tác vụ đọc cảm biến MPU6050 qua I2C, tác vụ xử lý nút nhấn, tác vụ gửi Telegram, tác vụ webserver HTTP.

4. **Kiểm thử từng module:** Kiểm tra riêng kết nối I2C với MPU6050, kiểm tra đọc dữ liệu gia tốc và vận tốc góc, kiểm tra phản hồi buzzer/LED, kiểm tra gửi HTTP request đến Telegram Bot API, kiểm tra web server.

5. **Tích hợp và kiểm thử toàn hệ thống:** Kết hợp tất cả module, thực hiện các kịch bản té ngã mô phỏng, đánh giá độ nhạy (sensitivity) và độ đặc hiệu (specificity), hiệu chỉnh ngưỡng phát hiện.

6. **Đánh giá kết quả:** Phân tích dữ liệu thu được, đánh giá hiệu năng hệ thống (thời gian phát hiện, tỷ lệ phát hiện đúng, tỷ lệ báo động giả), đề xuất các hướng cải tiến.

---

## Chương 2: Cơ Sở Lý Thuyết & Công Nghệ

### 2.1. Đặc điểm và hành vi té ngã

Té ngã (fall) được định nghĩa là hiện tượng một người mất thăng bằng và tiếp xúc ngoài ý muốn với mặt đất hoặc bề mặt thấp hơn. Nghiên cứu của Noury và cộng sự (2008) đã phân tích tín hiệu gia tốc trong các cú ngã điển hình và mô tả ba pha chính như sau:

**Pha 1 — Freefall (rơi tự do):**
Trong khoảng thời gian 200–400 ms đầu tiên, cơ thể ở trạng thái rơi tự do, cảm biến ghi nhận gia tốc tổng (resultant acceleration) giảm mạnh về xấp xỉ 0g (g = 9.81 m/s²). Tín hiệu accel trên cả ba trục đều tiến gần về 0. Đây là pha đặc trưng nhất của một cú ngã đột ngột và có thể dùng làm dấu hiệu nhận biết ban đầu.

**Pha 2 — Impact (va chạm):**
Khi cơ thể tiếp xúc với mặt đất, gia tốc tổng tăng đột ngột trong khoảng 20–50 ms với biên độ từ 2g đến 5g (có thể lên đến 8–10g đối với bề mặt cứng). Đồng thời, vận tốc góc (angular velocity) ghi nhận sự thay đổi đột ngột trên cả ba trục. Đỉnh gia tốc (peak acceleration) và thời gian va chạm là các thông số quan trọng để phân biệt té ngã với các hoạt động hàng ngày như nhảy, chạy, hoặc vỗ tay mạnh.

**Pha 3 — Lying (nằm yên):**
Sau va chạm, cơ thể nằm yên trên mặt đất, gia tốc ổn định ở 1g trên một trục (theo phương trọng lực) và xấp xỉ 0g trên hai trục còn lại. Góc nghiêng của thân người so với phương thẳng đứng vượt quá 70° — đây là dấu hiệu quan trọng để phân biệt té ngã với các tác động có chủ đích như đấm, đập tay.

Nghiên cứu của Ge và cộng sự chỉ ra rằng các cú ngã ở người cao tuổi thường có thời gian freefall ngắn hơn và biên độ thấp hơn so với người trẻ do chiều cao thấp hơn và phản xạ bảo vệ kém hơn. Do đó, ngưỡng phát hiện cần được hiệu chỉnh phù hợp.

### 2.2. Tổng quan hệ thống IoT

Kiến trúc Internet vạn vật (IoT) điển hình bao gồm ba lớp (tier-3 architecture):

1. **Perception Layer (lớp cảm nhận):** Bao gồm các thiết bị cảm biến như MPU6050, cùng với vi điều khiển ESP32-C3 thực hiện thu thập và xử lý sơ bộ dữ liệu. Đây là nơi diễn ra quá trình chuyển đổi tín hiệu vật lý thành tín hiệu điện số.

2. **Network Layer (lớp mạng):** Đảm nhận truyền dữ liệu giữa thiết bị đầu cuối và máy chủ thông qua các giao thức truyền thông. Trong đồ án này, ESP32-C3 kết nối WiFi ở chế độ Station (client), gửi HTTP GET request đến Telegram Bot API qua internet.

3. **Application Layer (lớp ứng dụng):** Cung cấp giao diện tương tác cho người dùng cuối. Đồ án triển khai hai kênh: web dashboard trên thiết bị (HTTP server trên ESP32-C3) và Telegram Bot (tin nhắn văn bản đến điện thoại).

Đồ án áp dụng mô hình **IoT edge computing**: quá trình phát hiện té ngã — vốn là lõi xử lý chính — được thực hiện ngay tại thiết bị đầu cuối (edge device), không phụ thuộc vào kết nối cloud. Việc xử lý tại chỗ giúp giảm độ trễ phát hiện, tăng độ tin cậy, và phù hợp với yêu cầu thời gian thực. Chỉ các thông báo cảnh báo mới được gửi lên Telegram, giảm băng thông và tiết kiệm năng lượng.

### 2.3. Cảm biến MPU6050

#### 2.3.1. Giới thiệu tổng quan

MPU6050 là cảm biến quán tính (Inertial Measurement Unit — IMU) 6 bậc tự do do InvenSense (nay thuộc TDK) sản xuất, tích hợp:
- **Accelerometer 3 trục:** đo gia tốc tĩnh (trọng lực) và gia tốc động theo các phương X, Y, Z.
- **Gyroscope 3 trục:** đo vận tốc góc (tốc độ quay) quanh các trục X, Y, Z (roll, pitch, yaw).
- **Digital Motion Processor (DMP):** bộ xử lý chuyển động tích hợp, có thể thực hiện các thuật toán fusion (kết hợp accel và gyro) như tính góc Euler hoặc quaternion, giảm tải cho MCU chính.

Cảm biến giao tiếp qua bus I2C với địa chỉ mặc định là 0x68 (hoặc 0x69 nếu chân AD0 nối lên VCC). Điện áp hoạt động 2.375–3.46 V (thường dùng 3.3 V), phù hợp để kết nối trực tiếp với ESP32-C3.

#### 2.3.2. Dải đo và cấu hình

**Accelerometer** có thể cấu hình bốn dải đo với độ phân giải tương ứng:

| Dải đo | Độ phân giải | Hệ số chuyển đổi (LSB/g) |
|--------|-------------|--------------------------|
| ±2g    | 16-bit      | 16384                    |
| ±4g    | 16-bit      | 8192                     |
| ±8g    | 16-bit      | 4096                     |
| ±16g   | 16-bit      | 2048                     |

**Gyroscope** có thể cấu hình bốn dải đo:

| Dải đo (°/s) | Độ phân giải | Hệ số chuyển đổi (LSB/°/s) |
|-------------|-------------|----------------------------|
| ±250        | 16-bit      | 131.0                      |
| ±500        | 16-bit      | 65.5                       |
| ±1000       | 16-bit      | 32.8                       |
| ±2000       | 16-bit      | 16.4                       |

**Cấu hình trong đồ án:** Accelerometer ±8g (4096 LSB/g), Gyroscope ±2000°/s (16.4 LSB/°/s). Dải đo accel ±8g được chọn để có độ phân giải đủ cao trong khi vẫn bao phủ được biên độ gia tốc té ngã (2–5g). Dải đo gyro ±2000°/s, cao nhất có thể, nhằm ghi nhận các chuyển động quay nhanh trong quá trình ngã.

#### 2.3.3. Công thức chuyển đổi dữ liệu thô

Dữ liệu từ MPU6050 được đọc dưới dạng số nguyên 16-bit có dấu (little-endian) cho mỗi trục. Giá trị thực được tính như sau:

**Gia tốc (m/s²):**

```
accel_X (m/s²) = raw_accel_X / ACCEL_SCALE × 9.81
```

Trong đó `ACCEL_SCALE` = 4096 LSB/g (với cấu hình ±8g).

**Vận tốc góc (°/s):**

```
gyro_X (°/s) = raw_gyro_X / GYRO_SCALE
```

Trong đó `GYRO_SCALE` = 16.4 LSB/°/s (với cấu hình ±2000°/s).

**Gia tốc tổng (Resultant Acceleration — RA):**

```
RA = √(accel_X² + accel_Y² + accel_Z²)
```

Giá trị RA là đại lượng quan trọng nhất dùng trong thuật toán phát hiện té ngã:
- Khi đứng yên: RA ≈ 1g (9.81 m/s²).
- Khi rơi tự do: RA ≈ 0g.
- Khi va chạm: RA > 2g.

#### 2.3.4. Giao tiếp I2C

I2C (Inter-Integrated Circuit) là bus giao tiếp nối tiếp đồng bộ hai dây do Philips (nay là NXP) phát triển:
- **SCL (Serial Clock):** tín hiệu xung nhịp do master (ESP32-C3) tạo ra.
- **SDA (Serial Data):** tín hiệu dữ liệu hai chiều.

Giao thức I2C hoạt động với các frame: START condition → địa chỉ slave 7-bit + bit R/W → ACK → dữ liệu 8-bit + ACK → ... → STOP condition. Tốc độ bus cấu hình ở mức Standard (100 kHz) hoặc Fast (400 kHz). Đồ án sử dụng tốc độ 400 kHz (Fast Mode) để đảm bảo đọc dữ liệu 6 trục với tần suất cao.

Điện trở kéo lên (pull-up) 4.7 kΩ được sử dụng trên cả hai đường SDA và SCL, phù hợp với chiều dài bus ngắn trong thiết bị đeo.

### 2.4. Hệ điều hành thời gian thực FreeRTOS trên ESP-IDF

FreeRTOS là hệ điều hành thời gian thực (RTOS) mã nguồn mở phổ biến cho vi điều khiển, được Espressif tích hợp sâu trong ESP-IDF. Các khái niệm cốt lõi được sử dụng trong đồ án gồm:

**Task (tác vụ):**
- Đơn vị thực thi độc lập, mỗi task có stack riêng và priority riêng.
- Task được tạo bằng API `xTaskCreatePinnedToCore()`.
- Bộ lập lịch (scheduler) của FreeRTOS thực hiện preemptive scheduling dựa trên priority: task có priority cao hơn được ưu tiên chạy trước.

**Queue (hàng đợi):**
- Cơ chế giao tiếp liên task (inter-task communication) an toàn.
- API: `xQueueCreate()`, `xQueueSend()`, `xQueueReceive()`.
- Project sử dụng queue để truyền dữ liệu cảm biến từ task MPU6050 sang task Telegram.

**Software Timer (bộ định thời phần mềm):**
- Tạo bằng API `xTimerCreate()` với callback chạy trong timer service task.
- Dùng để đếm lùi thời gian hủy báo động (10 giây) và tự động reset.

**Semaphore / Mutex:**
- `xSemaphoreCreateBinary()`, `xSemaphoreGive()`, `xSemaphoreTake()`.
- Dùng trong đồ án để đồng bộ sự kiện nút nhấn (button press event).

**Cấu trúc task trong đồ án:**

| Task | Priority | Chức năng | Chu kỳ |
|------|----------|-----------|--------|
| `mpu6050_task` | 5 (cao nhất) | Đọc MPU6050 qua I2C, tính RA, chạy state machine | 10 ms (100 Hz) |
| `btn_task` | 3 | Quét nút nhấn (debounce phần mềm), báo sự kiện | Sự kiện (Event) |
| `telegram_task` | 4 | Gửi HTTP request đến Telegram API (queue-based, background) | Khi có cảnh báo |
| `main_loop` | 1 (thấp nhất) | Kiểm tra WiFi status, xử lý cờ Telegram | 1000 ms (1 Hz) |

Priority cao nhất được gán cho task MPU6050 nhằm đảm bảo không bỏ lỡ mẫu dữ liệu trong các pha chuyển tiếp nhanh (freefall ~200 ms). Task Telegram được ưu tiên cao hơn task nút nhấn để đảm bảo cảnh báo được gửi đi nhanh nhất có thể.

### 2.5. Giao thức HTTP và Telegram Bot API

#### 2.5.1. Telegram Bot API

Telegram Bot API là giao diện lập trình ứng dụng cho phép các chương trình tự động (bot) gửi và nhận tin nhắn trên nền tảng Telegram. API hoạt động qua giao thức HTTP với phương thức GET hoặc POST, trả về dữ liệu định dạng JSON.

**Endpoint sử dụng:**

```
https://api.telegram.org/bot{TOKEN}/sendMessage?chat_id={CHAT_ID}&text={MESSAGE}
```

Trong đó:
- `{TOKEN}`: mã xác thực bot do BotFather cấp khi tạo bot.
- `{CHAT_ID}`: định danh cuộc trò chuyện (có thể là ID người dùng hoặc ID nhóm).
- `{MESSAGE}`: nội dung tin nhắn, cần được URL-encode.

#### 2.5.2. ESP HTTP Client

ESP-IDF cung cấp thư viện `esp_http_client` với các API:
- `esp_http_client_init()` — khởi tạo cấu trúc client.
- `esp_http_client_set_url()` — thiết lập URL đích.
- `esp_http_client_perform()` — thực thi request, trả về mã HTTP và dữ liệu response.
- `esp_http_client_cleanup()` — giải phóng tài nguyên.

#### 2.5.3. Bảo mật SSL/TLS

Telegram API yêu cầu kết nối HTTPS qua cổng 443. ESP-IDF hỗ trợ SSL/TLS với:
- **ESP-TLS:** wrapper thư viện mbedTLS.
- **Certificate Bundle (ESP x509 Certificate Bundle):** tập hợp chứng chỉ gốc (root certificates) được nhúng sẵn, dùng để xác thực máy chủ Telegram mà không cần lưu trữ riêng chứng chỉ.

Cấu hình kết nối:
- Timeout 15 giây cho mỗi HTTP request.
- Bộ đệm nội dung tin nhắn có kích thước 256 ký tự, đủ để gửi thông tin cảnh báo.

#### 2.5.4. Web dashboard

ESP32-C3 chạy HTTP server (dùng `esp_http_server` library) phục vụ một dashboard dạng chỉ-đọc (read-only) với giao diện card layout tối màu (dark theme), hiển thị:
- Trạng thái kết nối WiFi.
- Trạng thái Fall state với 5 màu sắc tương ứng 5 trạng thái máy trạng thái.
- Gia tốc (Acceleration) đơn vị g.
- Vận tốc góc (Gyroscope) đơn vị °/s.
- Góc Roll và Pitch đơn vị °.

Dashboard sử dụng JavaScript `fetch()` với chu kỳ polling 100 ms để cập nhật dữ liệu từ REST endpoint của ESP32-C3. Giao diện chỉ mang tính quan sát, không có các nút điều khiển. Trang web nhẹ, phù hợp với tài nguyên hạn chế của ESP32-C3 (ROM 4 MB, RAM 400 KB).
