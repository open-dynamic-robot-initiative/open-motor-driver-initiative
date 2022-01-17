//-----------------------------------------------------------------------
//                          INCLUDES
//-----------------------------------------------------------------------

#include "communication.h"
//#include "communication_cm.h"
//#include "checksum.h"
//#include "lsb_msb_macros.h"
//#include "uomodri_hal_config_handlers.h"

//-----------------------------------------------------------------------
//                      GLOBAL VARIABLES
//-----------------------------------------------------------------------
// Global dual buffer declarations
// Put them in the RAMG so DMA can reach them
#pragma DATA_SECTION(dualBufferW, "ramgs0");
DualBufferW dualBufferW =
{
 .ptrDmaCh  = (volatile struct CH_REGS *) DMA_COM_SPI_TX_BASE_ADDR,//(&dmaCfgList[0].dmaChBase),
 .index     = 0,
 //     ComPacketW comPackets[2];
};
#pragma DATA_SECTION(dualBufferR, "ramgs0");
DualBufferR dualBufferR =
{
 .ptrDmaCh  = (volatile struct CH_REGS *) DMA_COM_SPI_RX_BASE_ADDR,//(&dmaCfgList[1].dmaChBase),
 .index     = 0,
 //  ComPacketR comPackets[2];
};
#pragma DATA_SECTION(readData, "MSGRAM_CPU_TO_CM")
uint32_t readData[10];
//IPC_MessageQueue_t msgQueue;

// Debug
uint32_t comError = 0;
uint32_t comSuccess = 0;

//DualBufferW dualBufferW;
//DualBufferR dualBufferR;

//-----------------------------------------------------------------------
//                      INITIALIZATION ROUTINE
//-----------------------------------------------------------------------

// Communication initialization routine
void SPI_initCommunication(const DMA_TypeDef* p_dmaCfg)
{
    if(p_dmaCfg != NULL)
    {
        uint32_t dmaChBase = p_dmaCfg->dmaChBase;

        while(dmaChBase != 0xFFFFFFFF)
        {
//            // DMA register pointers initialization
//            if(p_dmaCfg->dmaChBase == COM_SPI_TX_DMA_BASE_ADDR)
//                dualBufferW.ptrDmaCh = (volatile struct CH_REGS*)p_dmaCfg->dmaChBase;
//            else if(p_dmaCfg->dmaChBase == COM_SPI_RX_DMA_BASE_ADDR)
//                dualBufferR.ptrDmaCh = (volatile struct CH_REGS*)p_dmaCfg->dmaChBase;
            // Start channel
            DMA_startChannel(dmaChBase);
            dmaChBase = (++p_dmaCfg)->dmaChBase;
        }
    }

    return;
}

//-----------------------------------------------------------------------
//                      WRITE PACKETS ROUTINES
//-----------------------------------------------------------------------

// Returns a pointer to the packet that isn't sent to the master board
inline ComPacketWPtr SPI_getAvailableWPacket()
{
    return (dualBufferW.comPackets + !dualBufferW.index); // !index corresponds to the unused packet index
}

// Compute CRC and swap Tx packets after the unused one has been updated
inline void SPI_pushWPacket()
{
    ComPacketWPtr availablePacketAddress;

    // Swap DualBuffer index and compute new DMA begin address
    dualBufferW.index = !dualBufferW.index; // Toggle index
    availablePacketAddress = dualBufferW.comPackets + dualBufferW.index;

    // Compute packet CRC
    uint32_t crc = com_crc32((uint16_t *)availablePacketAddress, DMA_COM_SPI_TX_RX_8BIT_LENGTH - 2); // The last two words are CRC fields
    // Update packet CRC
    availablePacketAddress->crc_msb = MSB_32(crc);
    availablePacketAddress->crc_lsb = LSB_32(crc);

    // Wait for end of transfer before changing addresses
    while (dualBufferW.ptrDmaCh->CONTROL.bit.TRANSFERSTS);

    // Change source addresses
    EALLOW;
    dualBufferW.ptrDmaCh->SRC_BEG_ADDR_SHADOW = (uint32_t)availablePacketAddress;
    dualBufferW.ptrDmaCh->SRC_ADDR_SHADOW = (uint32_t)availablePacketAddress;
    EDIS;
}

//-----------------------------------------------------------------------
//                      READ PACKETS ROUTINES
//-----------------------------------------------------------------------

// Returns a pointer to the packet that doesn't receive master board data
// If the CRC of the returned packet isn't correct, return (ComPacketRPtr)0xFFFFFFFF pointer instead.
// The user MUST check if the returned pointer is (ComPacketRPtr)0xFFFF before using it.
inline ComPacketRPtr SPI_getAvailableRPacket()
{
    ComPacketRPtr availablePacketAddress;
    ComPacketRPtr currentPacketAddress;

    // Compute address of the packet currently used for the comunication
    currentPacketAddress = dualBufferR.comPackets + dualBufferR.index;

    // Check if packet have been read already
    if (currentPacketAddress->mode.bit.rsvd == 1)
    {
        return RPACKET_ERROR;
    }

    // Swap DualBuffer index and compute new DMA begin address
    dualBufferR.index = !dualBufferR.index; // Toggle index
    availablePacketAddress = dualBufferR.comPackets + dualBufferR.index;

    // Wait for end of transfer before changing addresses
    while (dualBufferR.ptrDmaCh->CONTROL.bit.TRANSFERSTS);

    // Change destination address
    EALLOW;
    dualBufferR.ptrDmaCh->DST_BEG_ADDR_SHADOW = (uint32_t)availablePacketAddress;
    dualBufferR.ptrDmaCh->DST_ADDR_SHADOW = (uint32_t)availablePacketAddress;
    EDIS;

    // Compute CRC
    uint32_t crc = com_crc32((uint16_t *)currentPacketAddress, DMA_COM_SPI_TX_RX_8BIT_LENGTH - 2); // The last two words are CRC fields

    // Mark packet as read
    currentPacketAddress->mode.bit.rsvd = 1;

    // If CRC isn't correct, return error pointer
    if (MSB_32(crc) != currentPacketAddress->crc_msb || LSB_32(crc) != currentPacketAddress->crc_lsb)
    {
        comError++;

        return RPACKET_ERROR;
    }
    else
    {
        comSuccess++;

        return currentPacketAddress;
    }
}

// Function definitions
/**
 * @brief       Extract commands coming from Master to be performed by motor_foc
 * @param p_msg Pointer on the message received from Master board
 * @param p_cmd Pointer on the command structure (actions to be realized by motor_foc)
 * @return      Is message received from Master board valid or not ?
 */
bool com_cmdMsgExtract(cmd_msg_t* p_msg, cmd_uomodri_t* p_cmd)
{
//    uint32_t addr_offset = (uint32_t)p_msg + sizeof(rx_payload_t);
    bool rx_msg_valid = false;

//    while((uint32_t)p_msg < addr_offset)
//    COM_crc32((uint16_t *)p_msg, DMA_COM_SPI_TX_RX_16BIT_LENGTH);
//                GCRC_writeData(GCRC_BASE, *(uint32_t *)p_msg++, GCRC_DATATYPE_32BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.modeReg, GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.positionRef[MOTOR_1], GCRC_DATATYPE_32BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.positionRef[MOTOR_2], GCRC_DATATYPE_32BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.velocityRef[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.velocityRef[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.iqRef[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.iqRef[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.kpCoeff[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.kpCoeff[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.kdCoeff[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.kdCoeff[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.iSat[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.iSat[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.index, GCRC_DATATYPE_16BIT);

//    if(com_crc32((uint16_t *)p_msg, sizeof(cmd_payload_t)) == p_msg->crc)//GCRC_readResult(GCRC_BASE) == p_msg->crc)
    uint32_t crc = com_crc32((uint16_t *)p_msg, DMA_COM_SPI_TX_RX_16BIT_PAYLOAD_LENGTH);
    if((MSB_32(crc) == p_msg->crc.u16_msb) && (LSB_32(crc) == p_msg->crc.u16_lsb))
    {
        // run crc check
        p_cmd->cmdReg.bit.TIMEOUT   = p_msg->cmdReg.bit.TIMEOUT;
        p_cmd->cmdReg.bit.EI2OC     = p_msg->cmdReg.bit.EI2OC;
        p_cmd->cmdReg.bit.EI1OC     = p_msg->cmdReg.bit.EI1OC;
        p_cmd->cmdReg.bit.EPRE      = p_msg->cmdReg.bit.EPRE;
        p_cmd->cmdReg.bit.EM2       = p_msg->cmdReg.bit.EM2;
        p_cmd->cmdReg.bit.EM1       = p_msg->cmdReg.bit.EM1;
        p_cmd->cmdReg.bit.ES        = p_msg->cmdReg.bit.ES;
//        p_cmd->positionRef[MOTOR_1] = (float32_t)(FUS_16_TO_32(p_msg->positionRef[MOTOR_1][0], p_msg->positionRef[MOTOR_1][1]) * POSITION_LSB);
        p_cmd->positionRef[MOTOR_1] = (float32_t)(FUS_16_TO_32(p_msg->positionRef[MOTOR_1].u16_msb, p_msg->positionRef[MOTOR_1].u16_lsb) * POSITION_LSB);
//        p_cmd->positionRef[MOTOR_2] = (float32_t)(FUS_16_TO_32(p_msg->positionRef[MOTOR_2][0], p_msg->positionRef[MOTOR_2][1]) * POSITION_LSB);
        p_cmd->positionRef[MOTOR_2] = (float32_t)(FUS_16_TO_32(p_msg->positionRef[MOTOR_2].u16_msb, p_msg->positionRef[MOTOR_2].u16_lsb) * POSITION_LSB);
        p_cmd->velocityRef[MOTOR_1] = (float32_t)(p_msg->velocityRef[MOTOR_1]   * VELOCITY_LSB);
        p_cmd->velocityRef[MOTOR_2] = (float32_t)(p_msg->velocityRef[MOTOR_2]   * VELOCITY_LSB);
        p_cmd->iqRef[MOTOR_1]       = (float32_t)(p_msg->iqRef[MOTOR_1]         * IQ_LSB);
        p_cmd->iqRef[MOTOR_2]       = (float32_t)(p_msg->iqRef[MOTOR_2]         * IQ_LSB);
        p_cmd->kpCoeff[MOTOR_1]     = (float32_t)(p_msg->kpCoeff[MOTOR_1]       * KP_LSB);
        p_cmd->kpCoeff[MOTOR_2]     = (float32_t)(p_msg->kpCoeff[MOTOR_2]       * KP_LSB);
        p_cmd->kdCoeff[MOTOR_1]     = (float32_t)(p_msg->kdCoeff[MOTOR_1]       * KD_LSB);
        p_cmd->kdCoeff[MOTOR_2]     = (float32_t)(p_msg->kdCoeff[MOTOR_2]       * KD_LSB);
        p_cmd->iSat[MOTOR_1]        = (float32_t)(((p_msg->iSat & 0x00FF) >> 8) * CURRENT_SAT_LSB);
        p_cmd->iSat[MOTOR_2]        = (float32_t)((p_msg->iSat & 0xFF)          * CURRENT_SAT_LSB);
        rx_msg_valid                = true;
    }

    return(rx_msg_valid);
}

/**
 * @brief       Create/update the status message returned to Master using motor_foc data structure.
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message to create/update for Master
 * @return      Is message fully created/updated using motor_1 \& motor_2 data structure.
 */
bool com_statusMsgCreate(FOC_STRUCT* p_foc, cmd_uomodri_t* p_cmd, status_msg_t* p_msg)
{
//    uint32_t* p_crc = (uint32_t *)p_msg;
    int32_t pos         = (int32_t)(p_foc->motor_enc.thetaAbsolute / POSITION_LSB);
    bool    tx_msg_end  = false;

    p_msg->statusReg.bit.SE                 = p_cmd->cmdReg.bit.ES;                                         // System enabled
    p_msg->statusReg.bit.ERROR_CODE         = 0x00;                                                         //p_cmdcomInterface.canError;
    if(p_foc->motor_id == MOTOR_1)
    {
        // Update status field - MOTOR_1
        p_msg->statusReg.bit.M1E            = p_cmd->cmdReg.bit.EM1;                                        // MOTOR_1 enabled
        p_msg->statusReg.bit.M1R            = (p_foc->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_1 ready
        p_msg->statusReg.bit.IDX1T          = p_foc->motor_enc.indexToggle;                                 // MOTOR_1 index toggle
        p_msg->statusReg.bit.IDX1D          = p_foc->motor_enc.indexDetected;                               // MOTOR_1 index detected
        // Update time stamp in milliseconds - MOTOR_1
//        p_msg->payload.timeStamp            = (uint16_t)(p_msg->time[MOTOR_1] * 1000.0f);
        // Update motor position, velocity, Iq, coil resistance and alternative aquisition - MOTOR_1
        p_msg->position[MOTOR_1].u16_msb    = MSB_32(pos);//0xAAAA;//
        p_msg->position[MOTOR_1].u16_lsb    = LSB_32(pos);//0x5555;//
        p_msg->velocity[MOTOR_1]            = (int16_t)(p_foc->motor_enc.speedEst.speedMech / VELOCITY_LSB);//0x1234;//
        p_msg->iq[MOTOR_1]                  = (int16_t)(p_foc->iq / IQ_LSB);//0x5678;//
        p_msg->coilRes[MOTOR_1]             = (uint16_t)(p_foc->resistanceEst / RESISTANCE_LSB);//0x9ABC;//
        p_msg->adcSamples[MOTOR_1]          = (uint16_t)(p_foc->motor_acq.auxAcq / VOLTAGE_LSB);//0xDEF0;//
        tx_msg_end                          = false;
    }
    else if(p_foc->motor_id == MOTOR_2)
    {
        // Update status field - MOTOR_2
        p_msg->statusReg.bit.M2E            = p_cmd->cmdReg.bit.EM2;                                        // MOTOR_2 enabled
        p_msg->statusReg.bit.M2R            = (p_foc->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_2 ready
        p_msg->statusReg.bit.IDX2T          = p_foc->motor_enc.indexToggle;                                 // MOTOR_2 index toggle
        p_msg->statusReg.bit.IDX2D          = p_foc->motor_enc.indexDetected;                               // MOTOR_2 index detected
        // Update time stamp in milliseconds - MOTOR_2
//        p_msg->payload.timeStamp            = (uint16_t)(p_msg->time[MOTOR_2] * 1000.0f);
        // Update motor position, velocity, Iq, coil resistance and alternative aquisition - MOTOR_2
        p_msg->position[MOTOR_2].u16_msb    = MSB_32(pos);//0xAAAA;//
        p_msg->position[MOTOR_2].u16_lsb    = LSB_32(pos);//0x5555;//
        p_msg->velocity[MOTOR_2]            = (int16_t)(p_foc->motor_enc.speedEst.speedMech / VELOCITY_LSB);//0x1234;//
        p_msg->iq[MOTOR_2]                  = (int16_t)(p_foc->iq / IQ_LSB);// 0x5678;//
        p_msg->coilRes[MOTOR_2]             = (uint16_t)(p_foc->resistanceEst / RESISTANCE_LSB);//0x9ABC;//
        p_msg->adcSamples[MOTOR_2]          = (uint16_t)(p_foc->motor_acq.auxAcq / VOLTAGE_LSB);//0xDEF0;//
        // Compute CRC
        uint32_t crc = com_crc32((uint16_t *)p_msg, DMA_COM_SPI_TX_RX_16BIT_PAYLOAD_LENGTH);
        p_msg->crc.u16_msb                  = MSB_32(crc);
        p_msg->crc.u16_lsb                  = LSB_32(crc);
        tx_msg_end                          = true;
    }

//    while((uint32_t)p_crc < ((uint32_t)p_msg + sizeof(tx_payload_t)))
//        GCRC_writeData(GCRC_BASE, *p_crc++, GCRC_DATATYPE_32BIT);
//    p_msg->crc = GCRC_readResult(GCRC_BASE);

    return(tx_msg_end);
}

uint32_t com_crc32 (uint16_t* buf, uint16_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  while (len--)
    {
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ MSB_16(*buf)) & 0xFF];
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ LSB_16(*buf)) & 0xFF];
      buf++;
    }
  return crc;
}

//void com_bgcrc32_init(void)
//{
//    // Unlock the BGCRC configuration in case it is locked CFG-1
//    BGCRC_unlockRegister(BGCRC_CPU_BASE, BGCRC_REG_CTRL1 | BGCRC_REG_WD_CFG | BGCRC_REG_INTEN | BGCRC_REG_SEED);
//    // Unlock the BGCRC configuration in case it is locked CFG-2
//    BGCRC_unlockRegister(BGCRC_CPU_BASE, BGCRC_REG_EN | BGCRC_REG_CTRL2 | BGCRC_REG_START_ADDR | BGCRC_REG_GOLDEN | BGCRC_REG_WD_MIN | BGCRC_REG_WD_MAX);
//    // Enable NMI and set the emulation mode as soft.
//
//    // The CRC module and the watchdog stops immediately on debug suspend
//    BGCRC_setConfig(BGCRC_CPU_BASE, BGCRC_NMI_ENABLE, BGCRC_EMUCTRL_SOFT);
//    // Set the emulation mode of the BGCRC to free run.
//    BGCRC_setConfig(BGCRC_CPU_BASE, BGCRC_NMI_DISABLE, BGCRC_EMUCTRL_FREE);
//
//    // Configure the CRC Watchdog
//    // Watchdog window - Min : DATA_SIZE, Max : DATA_SIZE + 1
//    BGCRC_enableWatchdog(BGCRC_CPU_BASE);
//    BGCRC_setWatchdogWindow(BGCRC_CPU_BASE, DATA_SIZE, DATA_SIZE + 1);
//    // Configure the region
//    BGCRC_setRegion(BGCRC_CPU_BASE, (uint32_t)data, BGCRC_SIZE_KBYTES(1), BGCRC_CRC_MODE);
//    // Set the seed value same as used in SW computation
//    BGCRC_setSeedValue(BGCRC_CPU_BASE, seed);
//    // Initialize the golden value with previously computed CRC using software
//    BGCRC_setGoldenCRCValue(BGCRC_CPU_BASE, crc32);
//    // Enable interrupts
//    // No need to enable the NMIs as all the NMIs are enabled by default
//    BGCRC_enableInterrupt(BGCRC_CPU_BASE, BGCRC_TEST_DONE | BGCRC_CRC_FAIL | BGCRC_UNCORR_ERR | BGCRC_CORR_ERR | BGCRC_WD_UNDERFLOW | BGCRC_WD_OVERFLOW);
//    // Lock Register configuration for CFG1
//    BGCRC_lockRegister(BGCRC_CPU_BASE, BGCRC_REG_CTRL1 | BGCRC_REG_WD_CFG | BGCRC_REG_INTEN | BGCRC_REG_SEED);
//    // Commit Register Lock for CFG1
//    BGCRC_commitRegisterLock(BGCRC_CPU_BASE, BGCRC_REG_CTRL1 | BGCRC_REG_WD_CFG | BGCRC_REG_INTEN | BGCRC_REG_SEED);
//    BGCRC_start(BGCRC_CPU_BASE);
//    // Lock Register configuration for CFG2
//    BGCRC_lockRegister(BGCRC_CPU_BASE, BGCRC_REG_EN | BGCRC_REG_CTRL2 | BGCRC_REG_START_ADDR | BGCRC_REG_GOLDEN | BGCRC_REG_WD_MIN | BGCRC_REG_WD_MAX);
//}

/**
 * @brief       Message created/updates for debug
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
//void com_msgDbgTx(FOC_STRUCT* p_foc, uint8_t* p_msg)
//{
//    float32_t       arrayTmp[4];
//    static uint32_t CntMsg = 0;
//    //    uint8_t     aMsgCreate[MSG_SIZE] = {'a', 'b', 'c', 'd'};
//    uint8_t         *pspl = NULL, *pCntMsg = NULL, i = 0;
//
//    arrayTmp[0] = p_foc->motor_acq.ia + 1.0;
//    arrayTmp[1] = p_foc->motor_acq.ib + 1.0;
//    arrayTmp[2] = p_foc->motor_acq.ic + 1.0;
//    arrayTmp[3] = p_foc->motor_enc.thetaMech[NEW];
//    //arrayTmp[4] = p_foc->motor_enc.thetaElec;
//    pCntMsg     = (uint8_t *)&CntMsg;
//    pspl        = (uint8_t *)&arrayTmp[0];
//
//    for(i = 0; i < DBG_TX_MSG_SIZE; i++)
//    {
//        if(i < 4)
//            *p_msg = 'a' + i; //aMsgCreate[i] = 'a' + i;
//        else if(i < 8)
//            *p_msg = *pCntMsg++; //aMsgCreate[i] = *pCntMsg++;
//        else if(i < DBG_TX_MSG_SIZE)
//            *p_msg = *pspl++; //aMsgCreate[i] = *pspl++;
//        p_msg++;
//    }
//    CntMsg++;
//}

