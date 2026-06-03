#include <Arduino.h>
#include "painlessMesh.h"

#define NODE_NAME      "RELAY_2"

#define MESH_PREFIX     "IoT_MESH"
#define MESH_PASSWORD   "12345678"
#define MESH_PORT       5555

Scheduler userScheduler;
painlessMesh mesh;

void receivedCallback(
    uint32_t from,
    String &msg
) {

    Serial.println("\n====================");

    Serial.printf("[%s]\n",
                  NODE_NAME);

    Serial.printf("FROM: %u\n",
                  from);

    Serial.printf("MSG : %s\n",
                  msg.c_str());
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

    Serial.println("\n====================");

    Serial.printf("%s STARTED\n",
                  NODE_NAME);

    Serial.printf("NODE ID: %u\n",
                  mesh.getNodeId());
}

void loop() {

    mesh.update();
}