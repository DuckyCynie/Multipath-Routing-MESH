# Multipath Routing MESH

Hệ thống mạng mesh không dây gồm 3 loại nút ESP32, triển khai định tuyến đa đường và đo đạc các chỉ số lý thuyết thông tin trên phần cứng thực.

---

## Sơ đồ hệ thống

```
                  ┌─────────────┐
                  │  Root Node  │
                  │  (Gateway)  │
                  └──────┬──────┘
                         │ painlessMesh (Wi-Fi 2.4 GHz)
           ┌─────────────┴─────────────┐
           │                           │
    ┌──────┴──────┐             ┌──────┴──────┐
    │  Relay A    │             │  Relay B    │
    │  (PATH_A)   │             │  (PATH_B)   │
    └──────┬──────┘             └──────┬──────┘
           └─────────────┬─────────────┘
                  ┌──────┴──────┐
                  │ Sensor Node │
                  │  (DHT11)    │
                  └─────────────┘
```

Tất cả nút giao tiếp qua một mạng mesh chung, không có điểm trung tâm cố định.

---

## Vai trò từng nút

**Sensor Node** — thu thập và phát dữ liệu

Đọc nhiệt độ và độ ẩm từ cảm biến DHT11 mỗi 5 giây. Theo dõi trạng thái sống/chết của từng Relay qua tín hiệu HEARTBEAT. Tuỳ trạng thái mạng mà chọn chế độ gửi phù hợp (xem phần Thuật toán bên dưới).

**Relay Node** — trung chuyển và đo tín hiệu

Phát HEARTBEAT định kỳ để Sensor biết mình còn sống. Khi nhận được gói dữ liệu từ Sensor, đọc cường độ tín hiệu Wi-Fi (RSSI) tại thời điểm đó, gắn vào gói rồi chuyển tiếp lên Root.

**Root Node** — xử lý, tính toán, hiển thị

Điều phối toàn bộ mạng: phát ANNOUNCE để các nút khác biết địa chỉ của mình, gửi PING định kỳ để đo độ trễ, nhận dữ liệu từ hai Relay rồi ghép lại, tính toán các chỉ số và hiển thị lên hai màn hình OLED.

---

## Thuật toán định tuyến đa đường

Sensor duy trì bộ đếm thời gian cho mỗi Relay. Nếu không nhận được HEARTBEAT trong vòng 6 giây, Relay đó bị coi là mất kết nối.

```
Nếu cả hai Relay còn sống:
    Gửi song song — PATH_A chỉ mang nhiệt độ
                  — PATH_B chỉ mang độ ẩm
    (Data Striping: chia nhỏ dữ liệu ra hai kênh độc lập)

Nếu chỉ còn một Relay:
    Gửi toàn bộ nhiệt độ + độ ẩm qua Relay còn lại

Nếu cả hai mất kết nối:
    Bỏ gói, ghi cảnh báo
```

Root ghép hai nửa lại theo số thứ tự gói (sequence number). Nếu sau 8 giây chỉ nhận được một nửa, xử lý với dữ liệu có sẵn thay vì chờ mãi.

---

## Công thức tính toán

**Độ trễ (RTT)**

Root gửi gói PING có nhúng timestamp. Sensor nhận được thì phản hồi PONG giữ nguyên timestamp đó. Root tính:

    RTT = thời điểm nhận PONG - timestamp trong gói

**Packet Loss**

    Loss (%) = (số PING đã gửi - số PONG nhận được) / số PING đã gửi × 100

**Dung lượng kênh truyền — Shannon-Hartley**

Relay đọc RSSI (dBm) của kết nối Wi-Fi và gắn vào gói tin. Root dùng giá trị đó để ước tính dung lượng kênh lý thuyết:

    SNR (dB)     = RSSI - noise_floor        (noise_floor = -95 dBm)
    SNR (linear) = 10 ^ (SNR_dB / 10)
    C            = B × log₂(1 + SNR_linear)  (B = 20 MHz)

Kết quả C tính bằng bits/s, cho biết giới hạn lý thuyết tối đa của kênh truyền.

**Entropy Shannon của dữ liệu cảm biến**

Root thu thập các mẫu nhiệt độ và độ ẩm theo thời gian, phân vào 20 bin theo phân phối thực tế, rồi tính:

    H(X) = - Σ p(x) × log₂(p(x))   (bits)

Entropy càng cao, dữ liệu cảm biến càng đa dạng và khó dự đoán.

---

## Hiển thị kết quả

Root Node có hai màn hình OLED 128×64 trên hai bus I²C riêng biệt:

| Màn hình | Nội dung |
|---|---|
| OLED 1 (SDA=21, SCL=22) | Nhiệt độ, độ ẩm, trạng thái đường truyền |
| OLED 2 (SDA=18, SCL=19) | RTT, RSSI, packet loss, dung lượng kênh, entropy |

---

## Công nghệ sử dụng

- **Vi điều khiển:** ESP32 DOIT DevKit V1
- **Thư viện mesh:** painlessMesh + AsyncTCP
- **Cảm biến:** DHT11 (nhiệt độ + độ ẩm)
- **Màn hình:** Adafruit SSD1306 (I²C)
- **Build system:** PlatformIO, framework Arduino

---

## Cấu hình và nạp firmware

Trước khi nạp cho Relay, chỉnh hai dòng trong `relay_node/src/main.cpp`:

    RELAY_PATH = "PATH_A"  và  RELAY_NAME = "RELAY_1"   (cho Relay thứ nhất)
    RELAY_PATH = "PATH_B"  và  RELAY_NAME = "RELAY_2"   (cho Relay thứ hai)

Nạp firmware:

    cd <tên_node>
    pio run --target upload
    pio device monitor --baud 115200

---

## Liên quan môn học

Bài Tập Lớn môn Lý Thuyết Thông Tin 2026. Hệ thống đo thực tế trên phần cứng các chỉ số: độ trễ (RTT), packet loss, dung lượng kênh (Shannon-Hartley), entropy nguồn tin.
