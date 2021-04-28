
#ifdef ARDUINO_ARCH_SAMC

#include "fortytwo_can.h"
#include "Arduino.h"
#include "wiring_private.h"

FORTYTWO_CAN *fortytwo_can_objects[2];

FORTYTWO_CAN::FORTYTWO_CAN(uint8_t canid, uint8_t pintx, uint8_t pinrx) 
    : _canid(canid) {

    _pintx = pintx;
    _pinrx = pinrx;

    if(_canid == ID_CAN0) {
        fortytwo_can_objects[0] = this;
    } else if(_canid == ID_CAN1) {
        fortytwo_can_objects[1] = this;
    }
};

uint8_t FORTYTWO_CAN::begin(uint8_t idmode, uint32_t speedset) {
    uint8_t ret;
    _idmode = idmode;
    
    if((_canid != ID_CAN0) && (_canid != ID_CAN1)) {
        return CAN_FAIL;
    }

    const struct mcan_config config = {
        id: _canid,
        
        regs: ((_canid == ID_CAN0) ? CAN0 : CAN1),
        
        msg_ram: mcan_msg_ram,
        
        array_size_filt_std: RAM_ARRAY_SIZE_FILT_STD,
        array_size_filt_ext: RAM_ARRAY_SIZE_FILT_EXT,
        
        fifo_size_rx0: RAM_FIFO_SIZE_RX0,
        fifo_size_rx1: 0,
        
        array_size_rx: RAM_ARRAY_SIZE_RX,
        
        fifo_size_tx_evt : 0,
        array_size_tx: RAM_ARRAY_SIZE_TX,
        fifo_size_tx: RAM_FIFO_SIZE_TX,

        buf_size_rx_fifo0 : 64,
        buf_size_rx_fifo1 : 0,
        buf_size_rx : 64,
        buf_size_tx : 64,

        /*
        using values from AT6493 (SAMC21 app note); the plus values are to add on what the MCAN driver subtracts back off
        */
        bit_rate: speedset,
        quanta_before_sp : 10 + 2,
        quanta_after_sp : 3 + 1,

        /*
        AT6493 (SAMC21 app note) 'fast' values were unhelpfully the same as normal speed; these are for double (1MBit)
                the maximum peripheral clock of 48MHz on the SAMC21 does restrict us from very high rates
        */
        bit_rate_fd: speedset,
        quanta_before_sp_fd : 10 + 2,
        quanta_after_sp_fd : 3 + 1,

        quanta_sync_jump : 3 + 1,
        quanta_sync_jump_fd : 3 + 1,
    };

    if((g_APinDescription[_pintx].ulPinAttribute >> PIO_COM & 0x1) || (g_APinDescription[_pintx].ulPinAttribute >> PIO_CAN & 0x1)) {
        pinPeripheral(_pintx, PIO_COM);
    }
    else {
        return CAN_FAILINIT;
    }
    if((g_APinDescription[_pinrx].ulPinAttribute >> PIO_COM & 0x1) || (g_APinDescription[_pinrx].ulPinAttribute >> PIO_CAN & 0x1)) {
        pinPeripheral(_pinrx, PIO_COM);
    }
    else {
        return CAN_FAILINIT;
    }

    switch (config.id) {
        case ID_CAN0:
            GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
            MCLK->AHBMASK.reg |= MCLK_AHBMASK_CAN0;
            NVIC_EnableIRQ(CAN0_IRQn);
            break;
        case ID_CAN1:
            GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
            MCLK->AHBMASK.reg |= MCLK_AHBMASK_CAN1;
            NVIC_EnableIRQ(CAN1_IRQn);
            break;
        default:
            break;
    }

    if (mcan_configure_msg_ram(&config, &mcan_msg_ram_size)) {
        SerialUSB.println("RAM configuration succeeded");
    } else {
        SerialUSB.println("Failed");
        return CAN_FAIL;
    }

    ret = mcan_initialize(&mcan, &config);
    if (ret == 0) {
        SerialUSB.println("CAN initialized");
    } else {
        SerialUSB.print("CAN init failed, code ");
        SerialUSB.println(ret);
        return CAN_FAIL;
    }

    mcan_set_tx_queue_mode(&mcan);
    if (_mode == MCP_LOOPBACK) {
        mcan_loopback_on(&mcan);
    } else {
        mcan_loopback_off(&mcan);
    }

    mcan_set_mode(&mcan, MCAN_MODE_CAN);
    mcan_enable(&mcan);

    // Enable chip standby
    mcan_enable_rx_array_flag(&mcan, 0);

    // MCP_ANY means filters don't matter
    if (_idmode == MCP_ANY) {
        initMask(FILTER_0, (CAN_EXT_MSG_ID | MSG_ID_ALLOW_ALL_MASK));
        initMask(FILTER_1, (CAN_STD_MSG_ID | MSG_ID_ALLOW_ALL_MASK));
    }

    if (mcan_is_enabled(&mcan)) {
        SerialUSB.println("MCAN is enabled!");
        return CAN_OK;
    }

    SerialUSB.println("Something went wrong!!!!!!!!");

    return CAN_FAIL;
};

uint8_t FORTYTWO_CAN::initMask(uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint32_t id;

    if (ext) {
        id = 0x1fffffff;
        ulData &= id;
        id |= CAN_EXT_MSG_ID;
        if (ext >= mcan.cfg.array_size_filt_ext) {
            return MCP2515_FAIL;
        }
    } else {
        id = 0x7ff;
        ulData &= id;
        id |= CAN_STD_MSG_ID;
        if (ext >= mcan.cfg.array_size_filt_std) {
            return MCP2515_FAIL;
        }
    }
    
    mcan_filter_id_mask(&mcan, 0, num, id, ulData);
    return MCP2515_OK;
};

uint8_t FORTYTWO_CAN::initMask(uint8_t num, uint32_t ulData)
{
    return initMask(num, ((ulData & CAN_EXT_MSG_ID) == CAN_EXT_MSG_ID), ulData);
};

// Initilize Filter(s)
uint8_t FORTYTWO_CAN::initFilt(uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint32_t mask;
    if (ext) {
        mask = 0x1fffffff;
        if (ext >= mcan.cfg.array_size_filt_ext) {
            return MCP2515_FAIL;
        }
    } else {
        mask = 0x7ff;
        if (ext >= mcan.cfg.array_size_filt_std) {
            return MCP2515_FAIL;
        }
    }
    ulData &= mask;
    mcan_filter_id_mask(&mcan, 0, num, ulData, mask);
    return MCP2515_OK;
};

// Initilize Filter(s)
uint8_t FORTYTWO_CAN::initFilt(uint8_t num, uint32_t ulData)
{
    return initFilt(num, ((ulData & CAN_EXT_MSG_ID) == CAN_EXT_MSG_ID), ulData);
}; 

// Set operational mode
uint8_t FORTYTWO_CAN::setMode(uint8_t opMode)
{
    if ((opMode == MCP_LOOPBACK) && (_mode != MCP_LOOPBACK)) {
        _mode = opMode;
        mcan_disable(&mcan);
        mcan_reconfigure(&mcan);
        mcan_loopback_on(&mcan);
        mcan_enable(&mcan);
    } else if ((opMode == MCP_NORMAL) && (_mode != MCP_NORMAL)) {
        _mode = opMode;
        mcan_disable(&mcan);
        mcan_reconfigure(&mcan);
        mcan_loopback_off(&mcan);
        mcan_enable(&mcan);
    } else {
        return MCP2515_FAIL;
    }
    return MCP2515_OK;
};                                        

// Send message to transmit buffer
uint8_t FORTYTWO_CAN::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    uint8_t ret = 0xff;
    if (!mcan_is_enabled(&mcan)) {
        return CAN_CTRLERROR;
    }
    if (ext) {
        id |= CAN_EXT_MSG_ID;
    }
    ret = mcan_enqueue_outgoing_msg(&mcan, id, len, buf);
    if (ret != 0xFF) {
        return CAN_OK;
    }
    return CAN_FAILTX;
};      

// Send message to transmit buffer
uint8_t FORTYTWO_CAN::sendMsgBuf(uint32_t id, uint8_t len, uint8_t *buf)
{
    return sendMsgBuf(id, 1, len, buf);
};                 

// Read message from receive buffer
uint8_t FORTYTWO_CAN::readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t *buf)
{
    struct mcan_msg_info msg;
    msg.data = buf;
    msg.data_len = 8;
    uint8_t fifo_entries;
    if (mcan_is_tx_complete(&mcan)) {
        mcan_clear_tx_flag(&mcan);
    }
    fifo_entries = mcan_dequeue_received_msg(&mcan, 0, &msg);
    if (fifo_entries > 0) {
        *id = mcan_get_id(msg.id);
        *len = msg.data_len;
        *ext = (msg.id & CAN_EXT_MSG_ID) == CAN_EXT_MSG_ID;
        return CAN_OK;
    }
    return CAN_NOMSG;
};   

// Read message from receive buffer
uint8_t FORTYTWO_CAN::readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf)
{
    uint8_t ext;
    return readMsgBuf(id, &ext, len, buf);
};               

// Check for received data
uint8_t FORTYTWO_CAN::checkReceive(void)
{
    return (uint8_t)mcan_rx_fifo_data(&mcan, 0);
};                                           

// Check for errors
uint8_t FORTYTWO_CAN::checkError(void)
{
    return 0;
};                                             

// Check for errors
uint8_t FORTYTWO_CAN::getError(void)
{
    return 0;
};                                               

// Get error count
uint8_t FORTYTWO_CAN::errorCountRX(void)
{
    return 0;
};                                           

// Get error count
uint8_t FORTYTWO_CAN::errorCountTX(void)
{
    return 0;
};                                           

// Enable one-shot transmission
uint8_t FORTYTWO_CAN::enOneShotTX(void)
{
    return 0;
};  

// Disable one-shot transmission                                
uint8_t FORTYTWO_CAN::disOneShotTX(void)
{
    return 0;
};                                           


void CAN0_Handler(void)
{
    SerialUSB.println("A");
    if (mcan_rx_array_data(&(fortytwo_can_objects[0]->mcan))) {
        mcan_clear_rx_array_flag(&(fortytwo_can_objects[0]->mcan));
        fortytwo_can_objects[0]->rx_ded_buffer_data = true;
        Serial.println("Got a Packet 0!");
    }
}
void CAN1_Handler(void)
{
    SerialUSB.println("B");
    if (mcan_rx_array_data(&(fortytwo_can_objects[1]->mcan))) {
        mcan_clear_rx_array_flag(&(fortytwo_can_objects[1]->mcan));
        fortytwo_can_objects[1]->rx_ded_buffer_data = true;
        Serial.println("Got a Packet 1!");
    }
}

#endif