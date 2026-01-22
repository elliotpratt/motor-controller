#include <SPI.h>
#include <mcp_can.h>

// CAN bus setup
#define CAN_CS 10   // Chip Select pin for MCP2515
MCP_CAN CAN(CAN_CS);

// Example variables (adjust to your project)
int motorSpeed = 0;
int motorID = 0;
int currentSensorValue = 50; // placeholder sensor data

void setup() {
  Serial.begin(9600);

  // Initialize CAN bus
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("Slave: CAN init ok!");
  } else {
    Serial.println("Slave: CAN init fail!");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
}

void loop() {
  unsigned char len = 0;
  unsigned char rxBuf[8];
  unsigned long rxId;

  // -------- Listen for Master commands --------
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&rxId, &len, rxBuf); 

    if (rxId == 0x100) { // Master→Slave command
      motorID = rxBuf[0];
      motorSpeed = rxBuf[1];

      Serial.print("Received command: Motor ");
      Serial.print(motorID);
      Serial.print(" -> Speed ");
      Serial.println(motorSpeed);

      // Run motor control function here...
      // motorControl(motorID, motorSpeed);

      // -------- Send response back to Master --------
      byte resp[1];
      resp[0] = currentSensorValue; // return sensor feedback
      CAN.sendMsgBuf(0x200, 0, 1, resp);
    }
  }

  // update sensor value periodically (example)
  currentSensorValue = (currentSensorValue + 1) % 100;

  delay(100);
}
