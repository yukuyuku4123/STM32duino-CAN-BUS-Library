#include "Arduino.h"
#include "BUS.h"

#define CANID 0x0123

BUS Bus1;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    if (Bus1.begin(1000000) != 0) {
        Serial.println("BUS Initialization Failed!");
        while (1);
    }

    if (Bus1.setID(CANID, 0x0000) != 0) {
        Serial.println("BUS ID Configuration Failed!");
        while (1);
    }

    Serial.println("BUS Initialized!");
}

void loop() {
    if (Bus1.available() > 0) {
        Bus1.read();
        Serial.print("Received ID: ");
        Serial.print("0x");
        Serial.print(Bus1.CAN_Rx_Msg.id, HEX);
        Serial.print(" Data: ");
        for (int i = 0; i < Bus1.CAN_Rx_Msg.len; i++) {
            Serial.print("0x");
            Serial.print(Bus1.CAN_Rx_Msg.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}