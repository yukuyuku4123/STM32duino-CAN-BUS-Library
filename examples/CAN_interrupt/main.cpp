#include "Arduino.h"
#include "BUS.h"

#define CANID 0x0123
#define CANID2 0x0456

BUS Bus1;

void receiveEvent() {
    Serial.println("Message received!");

    Serial.print("Received ID: 0x");
    Serial.print(Bus1.CAN_Rx_Msg.id, HEX);
    Serial.print(", Data: ");
    for (int i = 0; i < Bus1.CAN_Rx_Msg.len; i++) {
        Serial.print("0x");
        Serial.print(Bus1.CAN_Rx_Msg.data[i], HEX);
        if (i < Bus1.CAN_Rx_Msg.len - 1) Serial.print(", ");
    }
    Serial.println();

}

void transmitEvent() {
    Serial.println("Message transmitted successfully!");
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Initializing CAN bus...");

    if (Bus1.begin(1000000) != 0) {
        Serial.println("CAN Initialization Failed!");
        while (1);
    }

    Bus1.setID(CANID, CANID2);

    Bus1.onReceive(receiveEvent);
    Bus1.onTransmit(transmitEvent);

    Serial.println("CAN Initialized Successfully!");
    Serial.println("Ready to send/receive messages...");
}

void loop() {
    static uint32_t lastSendTime = 0;
    if (millis() - lastSendTime > 2000) {
        lastSendTime = millis();

        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        Bus1.write(CANID, data, sizeof(data));
        Serial.println("Message sent!");
    }
}