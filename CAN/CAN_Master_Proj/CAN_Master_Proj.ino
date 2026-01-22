#include <SPI.h>
#include <mcp_can.h>

// CAN bus setup
#define CAN_CS 10   // Chip Select pin for MCP2515
MCP_CAN CAN(CAN_CS);

// Example variables (adjust to your project)
int motorSpeed = 100;
int motorID = 1;5
int sensorValue = 0;

void setup() {
  Serial.begin(9600);

  // Initialize CAN bus
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("Master: CAN init ok!");
  } else {
    Serial.println("Master: CAN init fail!");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
}

void loop() {
  // -------- Send command to Slave --------
  byte txData[2];
  txData[0] = motorID;
  txData[1] = motorSpeed;
  CAN.sendMsgBuf(0x100, 0, 2, txData); // ID 0x100 = Master→Slave

  delay(100); // give slave time to respond

  // -------- Receive response from Slave --------
  unsigned char len = 0;
  unsigned char rxBuf[8];
  unsigned long rxId;

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&rxId, &len, rxBuf); 
    if (rxId == 0x200) {                 // Slave→Master response
      sensorValue = rxBuf[0];
      Serial.print("Received sensor value: ");
      Serial.println(sensorValue);
    }
  }

  delay(500);
}
