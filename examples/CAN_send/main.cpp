#include "Arduino.h"
#include "BUS.h"

#define CANID 0x0123
#define CANID2 0x0456

BUS Bus1;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (Bus1.begin(1000000) != 0) {
    Serial.println("CAN Initialization Failed!");
    while (1);
  }
  Serial.println("CAN Initialized!");
}

void loop() {
  uint8_t txData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0xA5};

  Bus1.write(CANID, txData, 8);
  Serial.print("Sent ID: ");
  Serial.print("0x");
  Serial.println(CANID, HEX);
  Serial.print("Data: ");
  for (int i = 0; i < 8; i++) {
      Serial.print("0x");
      Serial.print(txData[i], HEX);
      Serial.print(" ");
  }
  Serial.println();

  delay(1000);

  Bus1.write(CANID2, txData, 8);
  Serial.print("Sent ID: ");
  Serial.print("0x");
  Serial.println(CANID2, HEX);
  Serial.print("Data: ");
  for (int i = 0; i < 8; i++) {
      Serial.print("0x");
      Serial.print(txData[i], HEX);
      Serial.print(" ");
  }
  Serial.println();

  delay(1000);
}

