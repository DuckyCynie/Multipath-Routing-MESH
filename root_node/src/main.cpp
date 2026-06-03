#include <Arduino.h>
#include "painlessMesh.h"

#define MESH_PREFIX     "IoT_MESH"
#define MESH_PASSWORD   "12345678"
#define MESH_PORT       5555

Scheduler userScheduler;
painlessMesh mesh;

uint32_t pingPacketID = 0;

uint32_t sentPings = 0;
uint32_t receivedPongs = 0;

Task pingTask(
    5000,
    TASK_FOREVER,
    []() {

        pingPacketID++;

        sentPings++;

        String pingMsg =
            "PING|" +
            String(pingPacketID) +
            "|" +
            String(millis());

        mesh.sendBroadcast(pingMsg);

        Serial.println("\n====================");
        Serial.println("PING SENT");
        Serial.println(pingMsg);
    }
);

void receivedCallback(
    uint32_t from,
    String &msg
) {

    // =====================
    // SENSOR DATA
    // =====================

    if (msg.startsWith("DATA")) {

        Serial.println("\n====================");
        Serial.println("SENSOR DATA RECEIVED");

        Serial.printf("FROM: %u\n",
                      from);

        Serial.printf("DATA: %s\n",
                      msg.c_str());
    }

    // =====================
    // PONG RTT
    // =====================

    if (msg.startsWith("PONG")) {

        receivedPongs++;

        int firstSep =
            msg.indexOf("|");

        int secondSep =
            msg.indexOf("|",
            firstSep + 1);

        String idStr =
            msg.substring(
                firstSep + 1,
                secondSep
            );

        String timeStr =
            msg.substring(
                secondSep + 1
            );

        uint32_t packetID =
            idStr.toInt();

        uint32_t sentTime =
            timeStr.toInt();

        uint32_t rtt =
            millis() - sentTime;

        uint32_t lost =
            sentPings - receivedPongs;

        Serial.println("\n====================");
        Serial.println("PONG RECEIVED");

        Serial.printf("FROM : %u\n",
                      from);

        Serial.printf("ID   : %u\n",
                      packetID);

        Serial.printf("RTT  : %lu ms\n",
                      rtt);

        Serial.printf("LOSS : %u\n",
                      lost);
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
        pingTask
    );

    pingTask.enable();

    Serial.println("\nROOT STARTED");
}

void loop() {

    mesh.update();
}