#include <fortytwo_can.h>

// Set up CAN0 instance to run on pins 4 (tx) and 5 (rx)
FORTYTWO_CAN can(ID_CAN0, 4, 5);

void setup() {
  uint8_t ret;

  SerialUSB.begin(115200);

  ret = can.begin(MCP_ANY, CAN_125KBPS); // MCP_ANY doesn't initialize filters
  if(ret == CAN_OK) {
    SerialUSB.println("CAN initialized successfully");
  }
  else {
    SerialUSB.print("Error initializing CAN, error code ");
    SerialUSB.println(ret);
  }
}

void loop() {
  uint8_t ret;
  uint32_t id;
  uint8_t len;
  uint8_t buf[8];
  uint8_t i;

  // Receive a message
  ret = can.readMsgBuf(&id, &len, buf);
  
  if (ret == CAN_OK) {
      SerialUSB.print("Got a message from: ");
      SerialUSB.print(id);
      SerialUSB.print("  Length: ");
      SerialUSB.print(len);
      SerialUSB.print("  Data: ");
      for (i = 0; i < len; i++) {
          SerialUSB.print(buf[i], HEX);
      }
      SerialUSB.println("");      
  }
}