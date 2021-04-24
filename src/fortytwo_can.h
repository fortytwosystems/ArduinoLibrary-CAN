
// #ifdef ARDUINO_ARCH_SAMC
#ifndef FORTYTWO_CAN_H_
#define FORTYTWO_CAN_H_

#include <samc21.h>

class FORTYTWO_CAN {
public:
    FORTYTWO_CAN(uint8_t cantx, uint8_t canrx, uint8_t group);
    
private:

}

#endif
// #endif