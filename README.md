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

# Tổng hợp lý thuyết — Ước lượng đặc tính kênh truyền WiFi Mesh

## 1. Mô hình suy hao tín hiệu — Log-distance Path Loss

**Mục đích:** Ước lượng RSSI tại khoảng cách d từ điểm tham chiếu d₀.

$$RSSI(d) = RSSI(d_0) - 10n \cdot \log_{10}\left(\frac{d}{d_0}\right) \quad \text{[dBm]}$$

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| RSSI(d₀) | −40 dBm | RSSI tại d₀ = 1m (chuẩn IEEE 802.11) |
| n | 2.0 / 2.8 / 3.5 | Path loss exponent (free space / indoor / dense) |
| d₀ | 1 m | Khoảng cách tham chiếu |

---

## 2. SNR — Tỉ số tín hiệu trên nhiễu

**Mục đích:** Chuyển đổi từ RSSI sang đại lượng quyết định chất lượng kênh.

$$SNR_{dB}(d) = RSSI(d) - NF$$

$$SNR_{linear}(d) = 10^{SNR_{dB}/10}$$

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| NF | −95 dBm | Noise floor WiFi 2.4GHz |
| SNR ≥ 25 dB | — | Đạt tốc độ 54 Mbps (802.11g) |
| SNR ≥ 10 dB | — | Ngưỡng kết nối tối thiểu |

---

## 3. Dung lượng kênh — Shannon-Hartley

**Mục đích:** Tính giới hạn lý thuyết tối đa của lượng thông tin có thể truyền qua kênh, tức giá trị cực đại của lượng tin tương hỗ I(X;Y).

$$C = B \cdot \log_2(1 + SNR_{linear}) \quad \text{[bits/s]}$$

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| B | 20 MHz | Băng thông kênh WiFi 2.4GHz |
| SNR_linear | từ công thức (2) | Tỉ số tín hiệu/nhiễu tuyến tính |

> **Lưu ý:** C là giới hạn trên lý thuyết. Throughput thực tế luôn nhỏ hơn C do overhead giao thức, CSMA/CA, retransmission.

---

## 4. Tỉ lệ lỗi bit — BER (BPSK-AWGN)

**Mục đích:** Ước lượng xác suất lỗi mỗi bit trong kênh AWGN với điều chế BPSK.

$$BER = Q\left(\sqrt{2 \cdot SNR_{linear}}\right) = \frac{1}{2} \cdot \text{erfc}\left(\sqrt{SNR_{linear}}\right)$$

Trong đó hàm Q và erfc liên hệ:
$$Q(x) = \frac{1}{2} \cdot \text{erfc}\left(\frac{x}{\sqrt{2}}\right)$$

---

## 5. Tỉ lệ lỗi gói — PER (BER → PER)

**Mục đích:** Chuyển đổi từ BER sang xác suất mất gói tin (Packet Error Rate).

$$PER = 1 - (1 - BER)^L$$

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| L | 200 bits (~25 bytes) | Độ dài gói tin ước tính |
| PER | [0, 1] | Xác suất gói bị lỗi |

---

## 6. Entropy Shannon H(X)

**Mục đích:** Đo lượng thông tin trung bình của nguồn dữ liệu cảm biến.

$$H(X) = -\sum_{i=1}^{N} p(x_i) \cdot \log_2 p(x_i) \quad \text{[bits]}$$

**Phương pháp tính trong hệ thống:**
- Dữ liệu nhiệt độ và độ ẩm được bin hóa thành 20 bins
- Xác suất p(xᵢ) = số mẫu trong bin i / tổng số mẫu
- H(X) tích lũy và ổn định dần theo thời gian

| Trường hợp | Entropy | Ý nghĩa |
|---|---|---|
| H(X) = 0 | Nguồn hoàn toàn xác định | Không có thông tin |
| H(X) = log₂N | Phân phối đều | Thông tin tối đa |

---

## 7. RTT lý thuyết — Mô hình Retransmission

**Mục đích:** Ước lượng Round-Trip Time khi tính đến ảnh hưởng của packet error.

$$RTT(d) = RTT_{base} + N_{hop} \cdot E[retx] \cdot t_{retx}$$

Số lần retransmit trung bình theo phân phối hình học:

$$E[retx] = \frac{PER}{1 - PER}$$

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| RTT_base | 12 ms | RTT cơ bản (4 hop × 3ms processing) |
| N_hop | 4 | Số hop PING/PONG: Sensor→R1→Root→R1→Sensor |
| t_retx | 10 ms | Thời gian mỗi lần retransmit (backoff + resend) |

---

## 8. Throughput ứng dụng lý thuyết

**Mục đích:** Ước lượng throughput thực tế tầng ứng dụng IoT sau khi tính packet loss.

$$Thr_{app}(d) = \frac{L_{payload} \times 8}{T_{send}} \times (1 - PER(d)) \quad \text{[bps]}$$

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| L_payload | 30 bytes | Kích thước payload DATA message |
| T_send | 5 s | Chu kỳ gửi của Sensor |
| Thr_max | 48 bps | Throughput tối đa khi PER = 0 |

---

## 9. Chuỗi phụ thuộc các công thức

```
d (khoảng cách)
    │
    ▼ Công thức (1): Log-distance Path Loss
RSSI(d)
    │
    ▼ Công thức (2): SNR = RSSI - NF
SNR(d)
    ├──▶ Công thức (3): C = B·log₂(1+SNR)    → Dung lượng kênh
    ├──▶ Công thức (4): BER = Q(√(2·SNR))    → Tỉ lệ lỗi bit
    │         │
    │         ▼ Công thức (5): PER = 1-(1-BER)^L
    │        PER(d)
    │         ├──▶ Công thức (7): RTT = f(PER)      → Trễ
    │         └──▶ Công thức (8): Thr = Thr_max×(1-PER) → Throughput
    │
    └──▶ Công thức (6): H(X) = -Σp·log₂p     → Entropy nguồn
```

---

## 10. Tham số hệ thống sử dụng

| Tham số | Giá trị | Nguồn |
|---|---|---|
| B (bandwidth) | 20 MHz | IEEE 802.11g/n 2.4GHz |
| NF (noise floor) | −95 dBm | Điển hình WiFi 2.4GHz |
| RSSI(d₀=1m) | −40 dBm | Chuẩn IEEE 802.11 |
| n (free space) | 2.0 | Lý thuyết Friis |
| n (indoor) | 2.8 | ITU-R P.1238 |
| n (dense indoor) | 3.5 | ITU-R P.1238 |
| L_packet | 200 bits | Ước tính payload DATA |
| T_send | 5 s | Cấu hình Sensor node |
| Bins entropy | 20 | Tham số histogram |

Bài Tập Lớn môn Lý Thuyết Thông Tin 2026. Hệ thống đo thực tế trên phần cứng các chỉ số: độ trễ (RTT), packet loss, dung lượng kênh (Shannon-Hartley), entropy nguồn tin.
