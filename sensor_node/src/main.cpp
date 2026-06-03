#include <Arduino.h>
#include "painlessMesh.h"

#define MESH_PREFIX     "IoT_MESH"
#define MESH_PASSWORD   "12345678"
#define MESH_PORT       5555

Scheduler userScheduler;
painlessMesh mesh;

uint32_t sensorValue = 0;

// =====================
// SEND SENSOR DATA
// =====================

Task sensorTask(
    2000,
    TASK_FOREVER,
    []() {

        sensorValue++;

        String dataMsg =
            "DATA|" +
            String(sensorValue);

        mesh.sendBroadcast(
            dataMsg
        );

        Serial.println("\nDATA SENT");
        Serial.println(dataMsg);
    }
);

// =====================
// RECEIVE PING
// =====================

void receivedCallback(
    uint32_t from,
    String &msg
) {

    if (msg.startsWith("PING")) {

        Serial.println("\nPING RECEIVED");

        String pongMsg =
            msg;

        pongMsg.replace(
            "PING",
            "PONG"
        );

        mesh.sendBroadcast(
            pongMsg
        );

        Serial.println("PONG SENT");
    }
}

void setup() {

    Serial.begin(115200);

    mesh.setDebugMsgTypes(ERROR);

    mesh.init(
        MESH_PREFIX,
        MESH_PASSWORD,
        &userScheduler,
        MESH_PORT
    );

    mesh.onReceive(
        &receivedCallback
    );

    userScheduler.addTask(
        sensorTask
    );

    sensorTask.enable();

    Serial.println("\nSENSOR STARTED");
}

void loop() {

    mesh.update();
}