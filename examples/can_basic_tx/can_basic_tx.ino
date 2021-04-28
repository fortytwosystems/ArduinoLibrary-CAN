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

  id = 0x64;
  len = 8;
  for(int i = 0; i < 8; i++) {
    buf[i] = random(0x00, 0xFF);
  }
  
  // Transmit a random message
  can.sendMsgBuf(id, 0, len, buf);

  delay(1000);
}