// =====================================================
// ROOT NODE — Gateway với 2 OLED + Metrics đầy đủ

#include <Arduino.h>
#include "painlessMesh.h"
#include <Wire.h>
// Wire1 là I2C bus thứ 2 — ESP32 hỗ trợ sẵn
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ── Shannon-Hartley constants ────────────────────────
// C = B · log2(1 + SNR)
// B  = 20 MHz (WiFi 2.4GHz standard channel bandwidth)
// NF = -95 dBm (typical WiFi 2.4GHz noise floor)
#define WIFI_BANDWIDTH_HZ   20000000.0f   // 20 MHz
#define NOISE_FLOOR_DBM     (-95)

#define MESH_PREFIX   "IoT_MESH"
#define MESH_PASSWORD "12345678"
#define MESH_PORT     5555

#define ANNOUNCE_INTERVAL  10000
#define OLED_UPDATE_INTERVAL 2000
#define ENTROPY_BINS       20     // số bin để tính histogram entropy

// OLED 128x64 I2C
#define SCREEN_W   128
#define SCREEN_H   64
#define OLED_ADDR  0x3C   // cả 2 màn dùng 0x3C vì khác bus

Scheduler            userScheduler;
painlessMesh         mesh;

// OLED1: bus Wire  (SDA=21, SCL=22) — Sensor data
// OLED2: bus Wire1 (SDA=18, SCL=19) — Metrics
TwoWire              Wire1_bus = TwoWire(1);
Adafruit_SSD1306     display1(SCREEN_W, SCREEN_H, &Wire,      -1);
Adafruit_SSD1306     display2(SCREEN_W, SCREEN_H, &Wire1_bus, -1);

bool oled1Ready = false;
bool oled2Ready = false;

// ── Metrics theo path ────────────────────────────────
struct PathMetrics {
    // Packet loss
    uint32_t received = 0;
    uint32_t lastSeq  = 0;
    uint32_t lost     = 0;

    // RTT (ms)
    uint32_t rttSum   = 0;
    uint32_t rttCount = 0;
    uint32_t rttMin   = UINT32_MAX;
    uint32_t rttMax   = 0;

    // RSSI (dBm)
    int32_t  rssiSum  = 0;
    int8_t   rssiLast = 0;

    // Channel capacity C = B·log₂(1+SNR)  [bits/s]
    // Đây là max mutual information I(X;Y) theo Shannon-Hartley
    float    capacity = 0;      // bits/s
    float    capacityMbps = 0;  // Mbps để dễ đọc
    uint32_t lastReceiveTime = 0;  // millis() lần cuối nhận gói



    // Throughput thực tế — dùng timestamp gói đầu và gói cuối
    uint32_t thrFirstTs  = 0;    // millis() gói đầu tiên
    uint32_t thrLastTs   = 0;    // millis() gói gần nhất
    uint32_t thrBytes    = 0;    // tổng bytes tích lũy
    float    throughputBps = 0;  // bps

#define PATH_TIMEOUT 10000  // ms không nhận gói → coi path dead
    float activeCapacityMbps() {
        if (lastReceiveTime == 0) return 0;
        if (millis() - lastReceiveTime > PATH_TIMEOUT) return 0;
        return capacityMbps;
    }
    bool isAlive() {
        return (lastReceiveTime > 0) &&
               (millis() - lastReceiveTime < PATH_TIMEOUT);
    }

    // Dữ liệu sensor mới nhất
    float    lastTemp = 0;
    float    lastHum  = 0;

    float avgRTT()  { return rttCount ? (float)rttSum / rttCount : 0; }
    float avgRSSI() { return received ? (float)rssiSum / received : 0; }
    float lossRate(){ return (received + lost) ? (float)lost / (received + lost) * 100.0f : 0; }
};

PathMetrics pmA, pmB;

// ── Giá trị OLED1 — cập nhật khi có data, task đọc độc lập ──
float   oled1Temp = NAN;
float   oled1Hum  = NAN;
String  oled1Path = "---";
uint32_t oled1LastUpdate = 0;   // millis() lần cuối nhận data

// ── RTT đo thật bằng PING/PONG ──────────────────────
struct RTTStats {
    uint32_t count  = 0;
    uint32_t sum    = 0;
    uint32_t minVal = UINT32_MAX;
    uint32_t maxVal = 0;
    uint32_t last   = 0;

    void add(uint32_t rtt) {
        count++;
        sum += rtt;
        last = rtt;
        if (rtt < minVal) minVal = rtt;
        if (rtt > maxVal) maxVal = rtt;
    }
    float avg() { return count ? (float)sum / count : 0; }
    uint32_t mn() { return minVal == UINT32_MAX ? 0 : minVal; }
} rttStats;

uint32_t pingId  = 0;
uint32_t pingTs  = 0;   // timestamp lúc gửi PING gần nhất
bool     waitingPong = false;
uint32_t pingLost = 0;

#define PING_INTERVAL 10000   // ms

// ── Dedup table ──────────────────────────────────────
// Khi cả 2 relay đều sống, cùng 1 seqNum sẽ đến 2 lần
// Lưu seqNum lần cuối đã xử lý để bỏ qua bản sao
#define DEDUP_SIZE 32
struct DedupEntry { String sensorID; uint32_t seqNum; };
DedupEntry dedupTable[DEDUP_SIZE];
uint8_t    dedupIdx = 0;

bool isDuplicate(const String &sensorID, uint32_t seqNum) {
    for (int i = 0; i < DEDUP_SIZE; i++) {
        if (dedupTable[i].sensorID == sensorID &&
            dedupTable[i].seqNum   == seqNum) {
            return true;
        }
    }
    // Chưa có — thêm vào bảng (circular buffer)
    dedupTable[dedupIdx] = { sensorID, seqNum };
    dedupIdx = (dedupIdx + 1) % DEDUP_SIZE;
    return false;
}

// ── Entropy Shannon ──────────────────────────────────
// Gộp cả temp và humidity vào histogram để tính H(X)
struct EntropyCalc {
    float tempMin =  100, tempMax = -100;
    float humMin  =  100, humMax  = -100;
    uint32_t tempHist[ENTROPY_BINS] = {0};
    uint32_t humHist [ENTROPY_BINS] = {0};
    uint32_t count = 0;

    void addSample(float temp, float hum) {
        count++;
        // Cập nhật range
        if (temp < tempMin) tempMin = temp;
        if (temp > tempMax) tempMax = temp;
        if (hum  < humMin)  humMin  = hum;
        if (hum  > humMax)  humMax  = hum;

        // Bin hóa
        if (tempMax > tempMin) {
            int b = (int)((temp - tempMin) / (tempMax - tempMin) * (ENTROPY_BINS - 1));
            b = constrain(b, 0, ENTROPY_BINS - 1);
            tempHist[b]++;
        }
        if (humMax > humMin) {
            int b = (int)((hum - humMin) / (humMax - humMin) * (ENTROPY_BINS - 1));
            b = constrain(b, 0, ENTROPY_BINS - 1);
            humHist[b]++;
        }
    }

    // H(X) = -Σ p·log2(p) bits
    float calcEntropy(uint32_t* hist) {
        if (count < 2) return 0;
        float h = 0;
        for (int i = 0; i < ENTROPY_BINS; i++) {
            if (hist[i] == 0) continue;
            float p = (float)hist[i] / count;
            h -= p * log2f(p);
        }
        return h;
    }

    float tempEntropy() { return calcEntropy(tempHist); }
    float humEntropy()  { return calcEntropy(humHist);  }
} entropy;

// ── OLED display ─────────────────────────────────────
// OLED1: cập nhật liên tục khi có DATA mới — nhiệt độ + độ ẩm
// OLED2: cập nhật theo task 5s — metrics tính toán

// ── Init OLED với retry ──────────────────────────────
bool tryInitOLED1() {
    if (!display1.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return false;
    display1.ssd1306_command(SSD1306_DISPLAYON);   // tường minh bật màn
    delay(10);
    display1.clearDisplay();
    display1.setTextSize(1);
    display1.setTextColor(SSD1306_WHITE);
    display1.setCursor(0, 20);
    display1.println("  OLED1: Sensor Data");
    display1.display();
    return true;
}

bool tryInitOLED2() {
    if (!display2.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return false;
    display2.ssd1306_command(SSD1306_DISPLAYON);   // tường minh bật màn
    delay(10);
    display2.clearDisplay();
    display2.setTextSize(1);
    display2.setTextColor(SSD1306_WHITE);
    display2.setCursor(0, 20);
    display2.println("  OLED2: Metrics");
    display2.display();
    return true;
}

void drawOLED1() {
    display1.ssd1306_command(SSD1306_DISPLAYON);
    bool aAlive = pmA.isAlive();
    bool bAlive = pmB.isAlive();
    bool multipath = aAlive && bAlive;

    // Timeout: không nhận data quá 10s → báo mất tín hiệu
    bool dataFresh = (oled1LastUpdate > 0) &&
                     (millis() - oled1LastUpdate < 10000);

    display1.clearDisplay();
    display1.setTextColor(SSD1306_WHITE);

    // Dòng 1: trạng thái path
    if (multipath && dataFresh) {
        display1.fillRect(0, 0, 128, 10, SSD1306_WHITE);
        display1.setTextColor(SSD1306_BLACK);
        display1.setCursor(2, 1);
        display1.print("** MULTIPATH A+B **");
    } else {
        display1.setTextColor(SSD1306_WHITE);
        display1.setCursor(2, 1);
        if      (!dataFresh)  display1.print("!! NO DATA");
        else if (aAlive)      display1.print("SINGLE  PATH_A");
        else if (bAlive)      display1.print("SINGLE  PATH_B");
        else                  display1.print("NO PATH");
    }
    display1.setTextColor(SSD1306_WHITE);

    // Dòng 2-3: nhiệt độ + độ ẩm font lớn
    display1.setTextSize(2);
    display1.setCursor(0, 14);
    if (!isnan(oled1Temp) && dataFresh)
        display1.printf("%.1fC", oled1Temp);
    else
        display1.print("--.-C");

    display1.setCursor(0, 34);
    if (!isnan(oled1Hum) && dataFresh)
        display1.printf("%.1f%%", oled1Hum);
    else
        display1.print("--.- %");

    // Dòng 4: packet count
    display1.setTextSize(1);
    display1.setCursor(0, 56);
    display1.printf("A:%u B:%u", pmA.received, pmB.received);

    display1.display();
}

void drawOLED2() {
    display2.ssd1306_command(SSD1306_DISPLAYON);
    bool aAlive = pmA.isAlive();
    bool bAlive = pmB.isAlive();
    bool multipath = aAlive && bAlive;

    float lossRate = (pmA.received + pmA.lost + pmB.received + pmB.lost) > 0 ?
                     (float)(pmA.lost + pmB.lost) /
                     (pmA.received + pmA.lost + pmB.received + pmB.lost) * 100.0f : 0;

    display2.clearDisplay();
    display2.setTextColor(SSD1306_WHITE);

    // Header — đảo màu khi multipath
    if (multipath) {
        display2.fillRect(0, 0, 128, 10, SSD1306_WHITE);
        display2.setTextColor(SSD1306_BLACK);
        display2.setCursor(2, 1);
        display2.print("MULTIPATH  A+B ACTIVE");
    } else {
        display2.setCursor(2, 1);
        display2.printf("SINGLE PATH_%s",
                        aAlive ? "A" : (bAlive ? "B" : "?"));
    }
    display2.setTextColor(SSD1306_WHITE);

    display2.setTextSize(1);
    display2.setCursor(0, 12);
    display2.printf("RTT: %u ms\n", rttStats.last);
    display2.printf("SNR A:%.0f B:%.0fdB\n",
                    pmA.avgRSSI() - NOISE_FLOOR_DBM,
                    pmB.avgRSSI() - NOISE_FLOOR_DBM);
    display2.printf("LOSS:%.1f%%\n", lossRate);
    display2.printf("C_A:%.1fM C_B:%.1fM\n",
                    pmA.activeCapacityMbps(), pmB.activeCapacityMbps());
    display2.printf("THR A:%.1f B:%.1f bps\n",
                    pmA.throughputBps, pmB.throughputBps);
    display2.printf("H(T):%.2f H(H):%.2f b\n",
                    entropy.tempEntropy(), entropy.humEntropy());
    display2.display();
}

#define OLED_RETRY_INTERVAL 5000   // ms thử lại nếu OLED chưa sáng
uint32_t oledLastRetry = 0;

Task oledTask(OLED_UPDATE_INTERVAL, TASK_FOREVER, []() {
    uint32_t now = millis();

    // Thử init lại OLED nếu chưa ready (mỗi 5s)
    if ((!oled1Ready || !oled2Ready) &&
        (now - oledLastRetry > OLED_RETRY_INTERVAL)) {
        oledLastRetry = now;
        if (!oled1Ready) {
            oled1Ready = tryInitOLED1();
        }
        if (!oled2Ready) {
            oled2Ready = tryInitOLED2();
        }
    }

    if (oled1Ready) drawOLED1();
    if (oled2Ready) drawOLED2();
});

// ── Ping task ────────────────────────────────────────
Task pingTask(PING_INTERVAL, TASK_FOREVER, []() {
    // Nếu PING trước chưa có PONG → đếm lost
    if (waitingPong) {
        pingLost++;
    }

    pingId++;
    pingTs       = millis();
    waitingPong  = true;

    // Broadcast PING — Relay sẽ forward xuống Sensor
    String pingMsg = "PING|" + String(pingId) + "|" + String(pingTs);
    mesh.sendBroadcast(pingMsg);


});

// ── Announce task ────────────────────────────────────
Task announceTask(ANNOUNCE_INTERVAL, TASK_FOREVER, []() {
    mesh.sendBroadcast("ANNOUNCE|" + String(mesh.getNodeId()));

});

// ── Tính channel capacity Shannon-Hartley ───────────
// C = B · log₂(1 + SNR_linear)
// SNR_dB    = RSSI - noise_floor
// SNR_linear = 10^(SNR_dB / 10)
float calcCapacity(int8_t rssi_dbm) {
    float snr_db     = (float)rssi_dbm - NOISE_FLOOR_DBM;
    if (snr_db < 0) snr_db = 0;
    float snr_linear = powf(10.0f, snr_db / 10.0f);
    float C          = WIFI_BANDWIDTH_HZ * log2f(1.0f + snr_linear);
    return C;   // bits/s
}


// ── Reassembly buffer ───────────────────────────────
// Khi striping: PATH_A mang temp, PATH_B mang hum
// Root cần nhận đủ 2 nửa cùng seqNum mới xử lý
#define REASM_SIZE 16

struct ReasmEntry {
    String   sensorID  = "";
    uint32_t seqNum    = 0;
    float    temp      = NAN;
    float    hum       = NAN;
    uint32_t tsA       = 0;    // timestamp từ PATH_A
    uint32_t tsB       = 0;    // timestamp từ PATH_B
    int8_t   rssiA     = 0;
    int8_t   rssiB     = 0;
    bool     hasA      = false;
    bool     hasB      = false;
    uint32_t firstSeen = 0;    // millis() — để timeout entry cũ
};

ReasmEntry reasmBuf[REASM_SIZE];
uint8_t    reasmIdx = 0;

#define REASM_TIMEOUT 8000   // ms — nếu 8s không có nửa còn lại thì xử lý nửa có được

ReasmEntry* findOrCreate(const String &sensorID, uint32_t seqNum) {
    // Tìm entry đã tồn tại
    for (int i = 0; i < REASM_SIZE; i++) {
        if (reasmBuf[i].sensorID == sensorID &&
            reasmBuf[i].seqNum   == seqNum) {
            return &reasmBuf[i];
        }
    }
    // Tạo mới — ghi đè entry cũ nhất (circular)
    ReasmEntry* e    = &reasmBuf[reasmIdx];
    reasmIdx         = (reasmIdx + 1) % REASM_SIZE;
    *e               = ReasmEntry();
    e->sensorID      = sensorID;
    e->seqNum        = seqNum;
    e->firstSeen     = millis();
    return e;
}

// ── Parse và xử lý DATA ─────────────────────────────
// Striping format  PATH_A: DATA|<id>|PATH_A|<seq>|<temp>|<ts>|RSSI:<r>|DEST:<d>
void handleData(uint32_t from, String &msg) {

    // Kiểm tra DEST
    int destIdx = msg.indexOf("|DEST:");
    if (destIdx < 0) return;
    uint32_t destId = strtoul(msg.substring(destIdx + 6).c_str(), NULL, 10);
    if (destId != mesh.getNodeId()) return;

    // Lấy payload (bỏ |DEST:...)
    String payload = msg.substring(0, destIdx);

    // Tách RSSI
    int rssiIdx = payload.lastIndexOf("|RSSI:");
    int8_t rssi = 0;
    if (rssiIdx >= 0) {
        rssi    = (int8_t)payload.substring(rssiIdx + 6).toInt();
        payload = payload.substring(0, rssiIdx);
    }

    // Tách tất cả fields bằng cách split theo '|'
    // Striping: DATA|id|PATH_A|seq|temp|ts       → 6 tokens (idx 0..5)
    // Single:   DATA|id|path|seq|temp|hum|ts     → 7 tokens (idx 0..6)
    String tokens[8];
    int    nTok = 0;
    {
        int start = 0;
        for (int i = 0; i <= payload.length() && nTok < 8; i++) {
            if (i == payload.length() || payload[i] == '|') {
                tokens[nTok++] = payload.substring(start, i);
                start = i + 1;
            }
        }
    }
    // tokens: [0]=DATA [1]=sensorID [2]=pathID [3]=seq [4]=val [5]=val2orTs [6]=ts
    if (nTok < 6) { return; }

    String   sensorID     = tokens[1];
    String   pathID       = tokens[2];
    uint32_t seqNum       = strtoul(tokens[3].c_str(), NULL, 10);

    // Striping: 6 tokens → DATA|id|path|seq|val|ts
    // Single:   7 tokens → DATA|id|path|seq|temp|hum|ts
    bool isSinglePath = (nTok >= 7);

    float    val    = tokens[4].toFloat();
    uint32_t sentTs = 0;

    if (isSinglePath) {
        sentTs = strtoul(tokens[6].c_str(), NULL, 10);
    } else {
        sentTs = strtoul(tokens[5].c_str(), NULL, 10);
    }



    PathMetrics *pm = nullptr;
    if      (pathID == "PATH_A") pm = &pmA;
    else if (pathID == "PATH_B") pm = &pmB;
    else return;

    // Cập nhật metrics path
    pm->rssiLast     = rssi;
    pm->rssiSum     += rssi;
    pm->capacity         = calcCapacity(rssi);
    pm->capacityMbps     = pm->capacity / 1e6f;
    pm->lastReceiveTime  = millis();
    if (pm->lastSeq > 0 && seqNum > pm->lastSeq + 1)
        pm->lost += seqNum - pm->lastSeq - 1;
    pm->lastSeq = seqNum;
    pm->received++;

    // ── Reassembly: ghép 2 nửa lại ──────────────────
    // isSinglePath đã được xác định ở trên bằng nTok >= 7
    float temp = NAN, hum = NAN;

    if (isSinglePath) {
        // Single path: tokens[4]=temp, tokens[5]=hum, tokens[6]=ts
        temp = val;              // tokens[4]
        hum  = tokens[5].toFloat();  // tokens[5]

    } else {
        // Striping mode
        ReasmEntry *e = findOrCreate(sensorID, seqNum);
        if (pathID == "PATH_A") {
            e->hasA  = true; e->temp = val; e->tsA = sentTs; e->rssiA = rssi;
        } else {
            e->hasB  = true; e->hum  = val; e->tsB = sentTs; e->rssiB = rssi;
        }

        // Chờ đủ 2 nửa hoặc timeout
        bool timeout = (millis() - e->firstSeen) > REASM_TIMEOUT;
        if (!e->hasA || !e->hasB) {
            if (timeout) {
                // Xử lý nửa có được
                temp = e->hasA ? e->temp : NAN;
                hum  = e->hasB ? e->hum  : NAN;

                e->sensorID = ""; // xóa entry
            } else {
                return; // chờ nửa còn lại
            }
        } else {
            temp = e->temp;
            hum  = e->hum;

            e->sensorID = ""; // xóa entry
        }
    }

    // ── Xử lý dữ liệu đã đủ ─────────────────────────
    if (!isnan(temp)) { pmA.lastTemp = temp; pmB.lastTemp = temp; }
    if (!isnan(hum))  { pmA.lastHum  = hum;  pmB.lastHum  = hum; }

    if (!isnan(temp) && !isnan(hum)) entropy.addSample(temp, hum);

    // 1 dòng duy nhất — tránh burst Serial làm overflow Web Serial buffer
    Serial.printf("DATA|%s|%u|T=%.1f|H=%.1f|"
                  "RSSI_A=%d|CAP_A=%.2f|RSSI_B=%d|CAP_B=%.2f|"
                  "LOSS_A=%.1f|LOSS_B=%.1f|"
                  "THR_A=%.2f|THR_B=%.2f|"
                  "RTT=%u|H(T)=%.3f|H(H)=%.3f|SAMPLES=%u\n",
                  sensorID.c_str(), seqNum,
                  isnan(temp)?0.0f:temp, isnan(hum)?0.0f:hum,
                  pmA.rssiLast, pmA.activeCapacityMbps(),
                  pmB.rssiLast, pmB.activeCapacityMbps(),
                  pmA.lossRate(), pmB.lossRate(),
                  pmA.throughputBps, pmB.throughputBps,
                  rttStats.last,
                  entropy.tempEntropy(), entropy.humEntropy(),
                  entropy.count);

    // Cập nhật global state cho OLED1 task
    if (!isnan(temp)) oled1Temp = temp;
    if (!isnan(hum))  oled1Hum  = hum;
    oled1Path       = (!isnan(temp)&&!isnan(hum)) ? "A+B" :
                      (!isnan(temp)) ? "PATH_A" : "PATH_B";
    oled1LastUpdate = millis();

    // Throughput tích lũy — tính sau khi dữ liệu đã được xử lý thành công
    // Throughput — tích lũy bytes, tính từ gói đầu đến gói cuối
    {
        uint32_t now2 = millis();
        uint32_t plen = payload.length();
        bool hasTemp = !isnan(temp);
        bool hasHum  = !isnan(hum);

        auto updateThr = [&](PathMetrics* p) {
            if (p->thrFirstTs == 0) p->thrFirstTs = now2;
            p->thrLastTs = now2;
            p->thrBytes += plen;
            uint32_t span = p->thrLastTs - p->thrFirstTs;
            if (span > 1000) {  // tính sau ít nhất 1s
                p->throughputBps = (float)p->thrBytes * 8000.0f / (float)span;
            }
        };

        if (hasTemp && hasHum) {
            updateThr(&pmA);
            updateThr(&pmB);
        } else if (hasTemp) {
            updateThr(&pmA);
        } else if (hasHum) {
            updateThr(&pmB);
        }


    }
}

void handlePong(String &msg) {
    // Format: PONG|<pingId>|<ts>
    // Sensor gửi sendSingle thẳng Root nên không cần DEST check
    int p1 = msg.indexOf('|');
    int p2 = msg.indexOf('|', p1 + 1);
    if (p1 < 0 || p2 < 0) return;

    uint32_t id = strtoul(msg.substring(p1 + 1, p2).c_str(), NULL, 10);
    uint32_t ts = strtoul(msg.substring(p2 + 1).c_str(), NULL, 10);

    // Chỉ tính RTT nếu đúng pingId đang chờ
    if (id != pingId) return;

    uint32_t rtt = millis() - ts;
    rttStats.add(rtt);
    waitingPong = false;


}

void receivedCallback(uint32_t from, String &msg) {
    if (msg.startsWith("DATA")) handleData(from, msg);
    if (msg.startsWith("PONG")) handlePong(msg);
}

void setup() {
    Serial.begin(115200);

    // Init mesh TRƯỚC — WiFi stack reset I2C nếu init sau
    mesh.setDebugMsgTypes(ERROR);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);

    // Init OLED SAU khi WiFi đã ổn định
    delay(500);   // chờ WiFi stack settle hoàn toàn
    Wire.begin(25, 26);       // OLED1: GPIO25(SDA) / GPIO26(SCL)
    Wire1_bus.begin(18, 19);  // OLED2: GPIO18(SDA) / GPIO19(SCL)
    delay(100);               // chờ I2C bus ổn định

    oled1Ready = tryInitOLED1();
    oled2Ready = tryInitOLED2();
    // Nếu chưa sáng thì oledTask sẽ retry mỗi 5s

    userScheduler.addTask(pingTask);
    userScheduler.addTask(announceTask);
    userScheduler.addTask(oledTask);
    pingTask.enable();
    announceTask.enable();
    oledTask.enable();

    Serial.printf("\n[ROOT] Started  nodeId=%u\n", mesh.getNodeId());
}

void loop() { mesh.update(); }