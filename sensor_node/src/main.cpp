// =====================================================
// SENSOR NODE — Multipath 
//
// Khi cả 2 relay sống: gửi song song qua cả PATH_A và PATH_B
// Khi 1 relay chết:    gửi qua relay còn lại
// Khi cả 2 chết:       log cảnh báo, bỏ gói
//
// lib_deps: painlessMesh, AsyncTCP, DHT sensor library
// =====================================================

#include <Arduino.h>
#include "painlessMesh.h"
#include <DHT.h>

// ── CẤU HÌNH NODE ────────────────────────────────────
#define SENSOR_ID     "S1"
#define DHT_PIN       4
#define DHT_TYPE      DHT11      // đổi DHT22 nếu cần
// ─────────────────────────────────────────────────────

#define MESH_PREFIX        "IoT_MESH"
#define MESH_PASSWORD      "12345678"
#define MESH_PORT          5555

#define SEND_INTERVAL      5000   // ms
#define FAILOVER_TIMEOUT   6000   // ms — relay coi là chết sau bao lâu mất HB

Scheduler   userScheduler;
painlessMesh mesh;
DHT         dht(DHT_PIN, DHT_TYPE);

// ── Relay info ───────────────────────────────────────
struct RelayInfo {
    String   pathID;
    uint32_t nodeId   = 0;
    uint32_t lastSeen = 0;
    RelayInfo(const char* p) : pathID(p), nodeId(0), lastSeen(0) {}

    bool alive() {
        return (nodeId != 0) &&
               (millis() - lastSeen < FAILOVER_TIMEOUT);
    }
};

RelayInfo relayA("PATH_A");
RelayInfo relayB("PATH_B");

uint32_t seqNum    = 0;
uint32_t rootNodeId = 0;   // học từ ANNOUNCE, dùng để gửi PONG thẳng về Root

// ── Chọn relay active (dùng chung cho sendTask và receivedCallback) ──
RelayInfo* activeRelay() {
    bool aAlive = relayA.alive();
    bool bAlive = relayB.alive();
    if (aAlive) return &relayA;
    if (bAlive) return &relayB;
    return nullptr;
}

// ── Send task ────────────────────────────────────────
Task sendTask(SEND_INTERVAL, TASK_FOREVER, []() {

    bool aAlive = relayA.alive();
    bool bAlive = relayB.alive();

    if (!aAlive && !bAlive) {
        Serial.println("[SENSOR] No relay available — DATA dropped");
        return;
    }

    // Đọc sensor
    float temp = dht.readTemperature();
    float hum  = dht.readHumidity();

    if (isnan(temp) || isnan(hum)) {
        Serial.println("[SENSOR] DHT read failed, skip");
        return;
    }

    seqNum++;
    uint32_t ts = millis();

    if (aAlive && bAlive) {
        // ── Data striping: chia đôi dữ liệu qua 2 kênh độc lập ──
        // PATH_A chỉ mang nhiệt độ (temp)
        // PATH_B chỉ mang độ ẩm    (hum)
        // Root ghép cả 2 lại mới có đủ thông tin
        // Format: DATA|<id>|<path>|<seq>|<val>|<ts>
        String msgA = "DATA|" + String(SENSOR_ID) + "|PATH_A|" +
                      String(seqNum) + "|" +
                      String(temp, 1) + "|" +    // chỉ temp
                      String(ts);

        String msgB = "DATA|" + String(SENSOR_ID) + "|PATH_B|" +
                      String(seqNum) + "|" +
                      String(hum, 1) + "|" +     // chỉ hum
                      String(ts);

        bool okA = mesh.sendSingle(relayA.nodeId, msgA);
        bool okB = mesh.sendSingle(relayB.nodeId, msgB);

        Serial.printf("[SENSOR] STRIPING seq=%u  PATH_A=T(%.1f)%s  PATH_B=H(%.1f)%s\n",
                      seqNum, temp, okA?"OK":"FAIL", hum, okB?"OK":"FAIL");

    } else {
        // ── Single path: gửi cả temp+hum qua relay còn sống ──
        RelayInfo* relay = aAlive ? &relayA : &relayB;

        String msg = "DATA|" + String(SENSOR_ID) + "|" +
                     relay->pathID + "|" +
                     String(seqNum) + "|" +
                     String(temp, 1) + "|" + String(hum, 1) + "|" +
                     String(ts);

        bool ok = mesh.sendSingle(relay->nodeId, msg);

        Serial.printf("[SENSOR] SINGLE seq=%u via %s  T=%.1f H=%.1f  ok=%d\n",
                      seqNum, relay->pathID.c_str(), temp, hum, ok);
    }
});

// ── Callback ─────────────────────────────────────────
void receivedCallback(uint32_t from, String &msg) {

    if (msg.startsWith("HEARTBEAT|")) {
        String pathID = msg.substring(10);
        if (pathID == relayA.pathID) {
            relayA.nodeId   = from;
            relayA.lastSeen = millis();
        } else if (pathID == relayB.pathID) {
            relayB.nodeId   = from;
            relayB.lastSeen = millis();
        }
        return;
    }

    // Học rootNodeId từ ANNOUNCE
    if (msg.startsWith("ANNOUNCE|")) {
        uint32_t id = strtoul(msg.substring(9).c_str(), NULL, 10);
        if (id != 0) rootNodeId = id;
        return;
    }

    // Nhận PING từ Root → trả PONG thẳng về Root bằng sendSingle
    // Không qua relay để tránh vòng lặp broadcast
    // Format PING: PING|<pingId>|<ts>
    // Format PONG: PONG|<pingId>|<ts>
    if (msg.startsWith("PING|")) {
        if (rootNodeId == 0) {
            Serial.println("[SENSOR] PING received but Root unknown, skip");
            return;
        }
        String pong = msg;
        pong.replace("PING|", "PONG|");
        bool ok = mesh.sendSingle(rootNodeId, pong);
        Serial.printf("[SENSOR] PONG sent to Root %u  ok=%d\n",
                      rootNodeId, ok);
    }
}

void setup() {
    Serial.begin(115200);
    dht.begin();
    mesh.setDebugMsgTypes(ERROR);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    userScheduler.addTask(sendTask);
    sendTask.enable();
    Serial.printf("\n[SENSOR %s] Started  nodeId=%u\n",
                  SENSOR_ID, mesh.getNodeId());
}

void loop() { mesh.update(); }