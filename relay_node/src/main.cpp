// =====================================================
// RELAY NODE — Forward + HEARTBEAT
//
// Relay1: RELAY_PATH = "PATH_A", RELAY_NAME = "RELAY_1"
// Relay2: RELAY_PATH = "PATH_B", RELAY_NAME = "RELAY_2"
//
// Thêm RSSI vào gói trước khi forward để Root đo SNR
// =====================================================

#include <Arduino.h>
#include "painlessMesh.h"
#include "esp_wifi.h"   // để đọc RSSI

// ── CẤU HÌNH NODE ────────────────────────────────────
#define RELAY_PATH    "PATH_A"
#define RELAY_NAME    "RELAY_1"
// ─────────────────────────────────────────────────────

#define MESH_PREFIX   "IoT_MESH"
#define MESH_PASSWORD "12345678"
#define MESH_PORT     5555

#define HEARTBEAT_INTERVAL 3000

Scheduler   userScheduler;
painlessMesh mesh;

uint32_t rootNodeId = 0;
uint32_t fwdOK  = 0;
uint32_t fwdFail = 0;

// ── Heartbeat ────────────────────────────────────────
Task heartbeatTask(HEARTBEAT_INTERVAL, TASK_FOREVER, []() {
    mesh.sendBroadcast("HEARTBEAT|" + String(RELAY_PATH));
});

// ── Lấy RSSI của kết nối WiFi hiện tại ──────────────
int8_t getRSSI() {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) return ap.rssi;
    return 0;
}

// ── Forward lên Root ─────────────────────────────────
// Gắn thêm |RSSI:<val>|DEST:<rootId> vào cuối
void forwardToRoot(uint32_t from, String &msg) {
    if (rootNodeId == 0) {
        Serial.printf("[%s] Root unknown, skipped\n", RELAY_NAME);
        return;
    }

    int8_t rssi = getRSSI();

    // Format forward: <original>|RSSI:<rssi>|DEST:<rootId>
    String fwdMsg = msg +
                    "|RSSI:" + String(rssi) +
                    "|DEST:" + String(rootNodeId);

    mesh.sendBroadcast(fwdMsg);
    fwdOK++;

    Serial.printf("[%s] FWD #%u from=%u rssi=%d\n",
                  RELAY_NAME, fwdOK, from, rssi);
}

// ── Callback ─────────────────────────────────────────
void receivedCallback(uint32_t from, String &msg) {

    if (msg.startsWith("ANNOUNCE|")) {
        uint32_t id = strtoul(msg.substring(9).c_str(), NULL, 10);
        if (id != 0 && id != rootNodeId) {
            rootNodeId = id;
            Serial.printf("[%s] Root learned: %u\n", RELAY_NAME, rootNodeId);
        }
        return;
    }

    // Forward DATA từ Sensor (chưa có DEST)
    if (msg.startsWith("DATA") && msg.indexOf("DEST:") < 0) {
        forwardToRoot(from, msg);
        return;
    }
}

void setup() {
    Serial.begin(115200);
    mesh.setDebugMsgTypes(ERROR);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    userScheduler.addTask(heartbeatTask);
    heartbeatTask.enable();
    Serial.printf("\n[%s] Started  path=%s  nodeId=%u\n",
                  RELAY_NAME, RELAY_PATH, mesh.getNodeId());
}

void loop() { mesh.update(); }