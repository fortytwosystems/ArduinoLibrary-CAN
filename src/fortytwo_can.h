
#ifdef ARDUINO_ARCH_SAMC

#ifndef FORTYTWO_CAN_H_
#define FORTYTWO_CAN_H_

#include <samc21.h>
#include "mcan.h"
#include "mcan_helper.h"
#include "mcp_can_dfs.h"

#define MAX_CHAR_IN_MESSAGE CAN_MAX_CHAR_IN_MESSAGE

#include <inttypes.h>
#include <string.h>

/* size of our custom Rx and Tx Buffer Elements, in words */
#define RAM_BUF_SIZE                  (MCAN_RAM_BUF_HDR_SIZE + 64u / 4)

#define RAM_ARRAY_SIZE_FILT_STD       (8u)
#define RAM_ARRAY_SIZE_FILT_EXT       (8u)
#define RAM_FIFO_SIZE_RX0             (12u)
/* no Rx FIFO 1 in our Message RAM */
#define RAM_ARRAY_SIZE_RX             (4u)
/* no Tx Event FIFO in our Message RAM */
#define RAM_ARRAY_SIZE_TX             (0u)
#define RAM_FIFO_SIZE_TX              (4u)


/* size of our custom Message RAM, in words */
#define MSG_RAM_SIZE      ( \
                            RAM_ARRAY_SIZE_FILT_STD * MCAN_RAM_FILT_STD_SIZE \
                            + RAM_ARRAY_SIZE_FILT_EXT * MCAN_RAM_FILT_EXT_SIZE \
                            + RAM_FIFO_SIZE_RX0 * RAM_BUF_SIZE \
                            + RAM_ARRAY_SIZE_RX * RAM_BUF_SIZE \
                            + RAM_ARRAY_SIZE_TX * RAM_BUF_SIZE \
                            + RAM_FIFO_SIZE_TX * RAM_BUF_SIZE )

#define MSG_LEN_1_CAN     8
#define MSG_LEN_1_CAN_FD  64
#define MSG_LEN_2_CAN     7
#define MSG_LEN_2_CAN_FD  48
#define MSG_ID_ALLOW_ALL_MASK     0x000ul     /* bits 0 & 1 are don't care */
#define RX_BUFFER_0       0
#define RX_BUFFER_1       1
#define FILTER_0          0
#define FILTER_1          1

struct frame_desc {
    uint32_t id;
    uint8_t data[64];
    uint8_t len;
    uint8_t buf_idx;
};

typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t ext;
    uint8_t buffer[MAX_CHAR_IN_MESSAGE];
    uint8_t ret;
} mcp_can_buf;


class FORTYTWO_CAN {
public:
    FORTYTWO_CAN(uint8_t canid, uint8_t pintx, uint8_t pinrx);
    uint8_t begin(uint8_t idmodeset, uint32_t speedset);
    uint8_t initMask(uint8_t num, uint8_t ext, uint32_t ulData);              // Initilize Mask(s)
    uint8_t initMask(uint8_t num, uint32_t ulData);                          // Initilize Mask(s)
    uint8_t initFilt(uint8_t num, uint8_t ext, uint32_t ulData);              // Initilize Filter(s)
    uint8_t initFilt(uint8_t num, uint32_t ulData); // Initilize Filter(s)
    uint8_t setMode(uint8_t opMode);                                        // Set operational mode
    uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);      // Send message to transmit buffer
    uint8_t sendMsgBuf(uint32_t id, uint8_t len, uint8_t *buf);
    uint8_t readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t *buf);
    uint8_t readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf);
    uint8_t checkReceive(void);
    uint8_t checkError(void);
    uint8_t getError(void);
    uint8_t errorCountRX(void);
    uint8_t errorCountTX(void);
    uint8_t enOneShotTX(void);
    uint8_t disOneShotTX(void);

    struct mcan_set mcan;
    volatile bool rx_ded_buffer_data;
private:
    uint8_t _pinrx;
    uint8_t _pintx;
    uint8_t _canid;
    uint8_t _idmode;

    uint32_t mcan_msg_ram[MSG_RAM_SIZE] __attribute__((aligned(4)));
    uint32_t mcan_msg_ram_size = ARRAY_SIZE(mcan_msg_ram);

    uint8_t _mode;
    uint8_t _cs;
};

#endif
#endif