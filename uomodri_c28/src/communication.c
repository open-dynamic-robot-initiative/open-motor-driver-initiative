/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#ifdef __TMS320C28XX_VCRC__
#include "vcu2/vcu2_crc.h"
#endif
#include "device.h"

#include "config/uomodri_math.h"
#include "uomodri_user_defines.h"
#include "communication.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void COM_msgSwapBytes(uint16_t*, uint16_t*, uint16_t);
uint32_t COM_vCRC32(uint16_t*, uint16_t);

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief           Master to slave message extraction function
 * @param[in]       p_msg       Pointer on the message received from Master board
 * @param[out]      p_cmd_m1    Pointer on the command structure used for MOTOR_1 control(actions to be realized by motor_foc).
 * @param[out]      p_cmd_m2    Pointer on the command structure used for MOTOR_2 control(actions to be realized by motor_foc).
 * @param[in]       crc         Computed value of CRC.
 * @return          Is message received from Master board valid or not ?
 */
void COM_msgExtract(mst2slv_msg_t* p_msg, cmd_t* p_cmd_m1, cmd_t* p_cmd_m2)
{
    int32_t pos_s32                     = (int32_t)FUS_16_TO_32(p_msg->position[MOTOR_1].u16_msb, p_msg->position[MOTOR_1].u16_lsb);
    cmd_reg_u* p_cmd_reg                = &p_cmd_m1->cmdField;
    p_cmd_m1->posRef                    = (float32_t)(pos_s32                   * POSITION_LSB      * FM_ROUND2RAD);
    p_cmd_m1->velRef                    = (float32_t)(p_msg->velocity[MOTOR_1]  * VELOCITY_LSB      * FM_KRPM2RADPS);
    p_cmd_m1->iqff                      = (float32_t)(p_msg->current[MOTOR_1]   * IQ_LSB);
    p_cmd_m1->kpCoeff                   = (float32_t)(p_msg->kpCoeff[MOTOR_1]   * KP_LSB            * FM_RAD2ROUND);
    p_cmd_m1->kdCoeff                   = (float32_t)(p_msg->kdCoeff[MOTOR_1]   * KD_LSB            * FM_RADPS2KRPM);
    float32_t isat                      = (float32_t)(MSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
    p_cmd_m1->iSat                      = (isat < MOTOR1_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR1_CURRENT_CMD_SAT_MAX);
    p_cmd_m1->iSat                      = (p_cmd_m1->iSat > 0.0f)               ? (p_cmd_m1->iSat)  : (MOTOR1_CURRENT_CMD_SAT_MAX);
    p_cmd_m1->timeoutRef                = (uint16_t)p_msg->mode.TIMEOUT;
    p_cmd_m1->cptTimeout                = 0U;
    p_cmd_m1->index                     = p_msg->index;
    p_cmd_reg->encOffsetEnable          = (bool_t)p_msg->mode.EI1OC;
    p_cmd_reg->rollOverEnable           = (bool_t)p_msg->mode.EPRE;
    p_cmd_reg->motorEnable              = (bool_t)p_msg->mode.EM1;
    p_cmd_reg->systemEnable             = (bool_t)p_msg->mode.ES;

    pos_s32                             = (int32_t)FUS_16_TO_32(p_msg->position[MOTOR_2].u16_msb, p_msg->position[MOTOR_2].u16_lsb);
    p_cmd_reg                           = &p_cmd_m2->cmdField;
    p_cmd_m2->posRef                    = (float32_t)(pos_s32                   * POSITION_LSB      * FM_ROUND2RAD);
    p_cmd_m2->velRef                    = (float32_t)(p_msg->velocity[MOTOR_2]  * VELOCITY_LSB      * FM_KRPM2RADPS);
    p_cmd_m2->iqff                      = (float32_t)(p_msg->current[MOTOR_2]   * IQ_LSB);
    p_cmd_m2->kpCoeff                   = (float32_t)(p_msg->kpCoeff[MOTOR_2]   * KP_LSB            * FM_RAD2ROUND);
    p_cmd_m2->kdCoeff                   = (float32_t)(p_msg->kdCoeff[MOTOR_2]   * KD_LSB            * FM_RADPS2KRPM);
    isat                                = (float32_t)(LSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
    p_cmd_m2->iSat                      = (isat < MOTOR2_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR2_CURRENT_CMD_SAT_MAX);
    p_cmd_m2->iSat                      = (p_cmd_m2->iSat > 0.0f)               ? (p_cmd_m2->iSat)  : (MOTOR2_CURRENT_CMD_SAT_MAX);
    p_cmd_m2->timeoutRef                = (uint16_t)p_msg->mode.TIMEOUT;
    p_cmd_m2->cptTimeout                = 0U;
    p_cmd_m2->index                     = p_msg->index;
    p_cmd_reg->encOffsetEnable          = (bool_t)p_msg->mode.EI2OC;
    p_cmd_reg->rollOverEnable           = (bool_t)p_msg->mode.EPRE;
    p_cmd_reg->motorEnable              = (bool_t)p_msg->mode.EM2;
    p_cmd_reg->systemEnable             = (bool_t)p_msg->mode.ES;

    return;
}

/**
 * @brief           Slave to Master message creation/update. IT Call.
 * @param[in]       *p_motor    Pointer on current data structure used for control.
 * @param[inout]    *p_msg      Pointer on the message to transmit to the Master board.
 * @param[in]       tmr_cnt     Current value of the timer counter.
 */
bool_t COM_msgCreate(motor_t* p_motor, slv2mst_msg_t* p_msg, uint32_t tmr_cnt)
{
    foc_t* p_foc                        = &p_motor->foc;
    encoder_t* p_enc                    = &p_foc->enc;
    error_reg_u motor_err               = p_motor->error;
    cmd_reg_u motor_cmd                 = {.all = p_foc->cmd.cmdField.all};
    float32_t theta_index_comp          = (motor_cmd.encOffsetEnable) ? (p_enc->thetaIndexAbsolute) : (0.0f);
    float32_t pos_offset                = p_enc->thetaAbsolute - theta_index_comp;
    int32_t pos                         = (int32_t)(pos_offset                  / POSITION_LSB      * FM_RAD2ROUND);
    uint16_t motor_id                   = p_motor->id;
    bool_t align_done                   = (p_motor->state == MOTOR_STATE_READY) || (p_motor->state == MOTOR_STATE_STOP);

    p_msg->position[motor_id].u16_msb   = MSB_32(pos);
    p_msg->position[motor_id].u16_lsb   = LSB_32(pos);
    p_msg->velocity[motor_id]           = (int16_t)(p_enc->speed.speedMech[0]   / VELOCITY_LSB      * FM_RADPS2KRPM);
    p_msg->current[motor_id]            = (int16_t)(p_foc->iq                   / IQ_LSB);
    p_msg->coilRes[motor_id]            = (uint16_t)(p_foc->resEst              / RESISTANCE_LSB);
    p_msg->adcSamples[motor_id]         = (uint16_t)(p_foc->acq.vExt            / VOLTAGE_LSB);

    if(motor_id == MOTOR_1)
    {
        // Update status field - MOTOR_1
        p_msg->status.M1E               = motor_cmd.motorEnable;    // MOTOR_1 enabled
        p_msg->status.M1R               = align_done;               // MOTOR_1 ready
        p_msg->status.IDX1T             = p_enc->flags.indexToggle; // MOTOR_1 index toggle
        p_msg->status.IDX1D             = p_enc->flags.indexDetect; // MOTOR_1 index detected

        p_msg->status.ERROR_CODE        = (motor_err.com_timeout)   ? (STATUS_ERROR_SPI_RECV_TIMEOUT)   : (STATUS_ERROR_NO_ERROR);
        p_msg->status.ERROR_CODE        = (motor_err.pos_rollover)  ? (STATUS_ERROR_POS_ROLLOVER)       : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.drv_fault)     ? (STATUS_ERROR_DRV_FAULT)          : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.enc_indexErr)  ? (STATUS_ERROR_ENCODER1)           : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.enc_alignErr)  ? (STATUS_ERROR_ENCODER1)           : (p_msg->status.ERROR_CODE);

        return(false);
    }
    else
    {
        // Update status field - MOTOR_2
        p_msg->status.M2E               = motor_cmd.motorEnable;    // MOTOR_2 enabled
        p_msg->status.M2R               = align_done;               // MOTOR_2 ready
        p_msg->status.IDX2T             = p_enc->flags.indexToggle; // MOTOR_2 index toggle
        p_msg->status.IDX2D             = p_enc->flags.indexDetect; // MOTOR_2 index detected

        p_msg->status.SE                = motor_cmd.systemEnable;   // System enabled
        p_msg->status.STATUS_RSV1       = 0;
        p_msg->lastCmdIndex             = p_foc->cmd.index;
        p_msg->timeStamp                = (uint16_t)tmr_cnt;
        p_msg->status.ERROR_CODE        = (motor_err.com_timeout)   ? (STATUS_ERROR_SPI_RECV_TIMEOUT)   : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.pos_rollover)  ? (STATUS_ERROR_POS_ROLLOVER)       : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.drv_fault)     ? (STATUS_ERROR_DRV_FAULT)          : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.enc_indexErr)  ? (STATUS_ERROR_ENCODER2)           : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.enc_alignErr)  ? (STATUS_ERROR_ENCODER2)           : (p_msg->status.ERROR_CODE);

        return(true);
    }
}

/**
 * @brief           Initialize the command structure for a motor
 * @param[in]       p_cmd   Pointer on the command structure (actions to be realized by motor_foc)
 */
void COM_initCmdStruct(cmd_t* p_cmd)
{
    p_cmd->posRef               = 0.0f;
    p_cmd->velRef               = 0.0f;
    p_cmd->iqff                 = 0.0f;
    p_cmd->kpCoeff              = 0.0f;
    p_cmd->kdCoeff              = 0.0f;
//    p_cmd->iSat                 = 0.0f;
    p_cmd->timeoutRef           = 0U;
    p_cmd->cptTimeout           = 0U;
    p_cmd->index                = 0U;
    p_cmd->cmdField.all         = 0U;

    return;
}

/**
 * @brief           Swap bytes position in a message.
 * @param[in]       *p_msg  Pointer on the message to swap.
 * @param[out]      *p_swap Pointer on the swapped message.
 * @param[in]       length  Number of elements to swap.
 */
void COM_msgSwapBytes(uint16_t* p_msg, uint16_t* p_swap, uint16_t length)
{
    uint16_t i;

    for(i = 0; i < length; i++)
        *p_swap++ = (*p_msg << 8) | (*p_msg++ >> 8);

    return;
}

/**
 * @brief           Software cyclic redundancy check (CRC) using the VCRC assembly code accelerator.
 * @param[in]       *p_msg  Pointer on the message to compute.
 * @param[in]       length  Number of elements necessary for CRC computation.
 * @return          New CRC result.
 */
uint32_t COM_vCRC32(uint16_t* p_msg, uint16_t length)
{
    CRC_Obj crc;
    // Initialize the CRC object
    crc.seedValue   = CRC_INIT_CRC32;
    crc.nMsgBytes   = 2 * length;
    crc.parity      = CRC_parity_even;
    crc.crcResult   = 0;
    crc.pMsgBuffer  = (uint16_t *)p_msg;
    crc.init        = (void (*)(void *))CRC_init32Bit;
    crc.run         = (void (*)(void *))CRC_run32BitPoly1;
    // Run the CRC module with the VCRC assembly code
    crc.init(&crc);
    crc.run(&crc);
    return(crc.crcResult);
}

/*
 * @brief           Compute the CRC 32bits with bytes swapping first.
 * @param[in]       *p_msg  Pointer on the message to swap.
 * @param[in]       length  Number of elements to be swapped.
 * return           The 32bit CRC result.
 */
uint32_t COM_msgCRC(uint16_t* p_msg, uint16_t length)
{
    uint16_t swap[COM_MSG_EXTRACT_TX_RX_MAX_SIZE];
    COM_msgSwapBytes(p_msg, swap, length);
    uint32_t crcResult  = COM_vCRC32(swap, length);
    return(crcResult);
}

/*
 * @brief           Read & concatenate the data available in the SCI peripheral.
 * @param[in]       sci-base    SCI peripheral identifier.
 * @param[out]      *p_msg_rx   Pointer on the RS485 reception structure.
 */
inline void RS485_msgReceive(uint32_t sci_base, rs485_rx_t* p_msg_rx)
{
//    uint16_t fifo_rx_status = (uint16_t)SCI_getRxFIFOStatus(sci_base);
    uint16_t fifo_rx_status = (uint16_t)((HWREGH(sci_base + SCI_O_FFRX) & SCI_FFRX_RXFFST_M) >> SCI_FFRX_RXFFST_S);
    while(fifo_rx_status--)
    {
        uint16_t* p_msg     = &p_msg_rx->msg.raw[p_msg_rx->index >> 1];
//        uint16_t tmp        = SCI_readCharNonBlocking(sci_base);
        uint16_t tmp        = (uint16_t)(HWREGH(sci_base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M);
        *p_msg              = (p_msg_rx->index & 0x0001) ? (*p_msg | (tmp & 0x00FF)) : ((tmp << 8) & 0xFF00);
        p_msg_rx->index++;
    }

    return;
}

/*
 * @brief           Compute the CRC 32bits with bytes swapping first.
 * @param[in]       *p_msg  Pointer on the message to swap.
 * @param[in]       length  Number of elements to swap.
 * return           The 32bit CRC result.
 */
inline void RS485_msgTransmit(uint32_t sci_base, rs485_tx_t* p_msg_tx)
{
//    uint16_t fifo_tx_status = (uint16_t)(SCI_FIFO_TX16 - SCI_getTxFIFOStatus(sci_base));
    SCI_TxFIFOLevel fifo_tx_status  = SCI_FIFO_TX16;
    fifo_tx_status                 -= (SCI_TxFIFOLevel)((HWREGH(sci_base + SCI_O_FFTX) & SCI_FFTX_TXFFST_M) >> SCI_FFTX_TXFFST_S);

    while((uint16_t)fifo_tx_status && ((2 * COM_MSG_RS485_TX_16BIT_FULL_LENGTH) - p_msg_tx->index))//p_msg_tx->remaining)
    {
        uint16_t* p_msg = &p_msg_tx->msg[p_msg_tx->msg_cr].raw[p_msg_tx->index >> 1];
//        SCI_writeCharNonBlocking(sci_base, (((p_msg_tx->index >> 1) ? (*p_msg): (*p_msg >> 8)) & 0x00FF));
        HWREGH(sci_base + SCI_O_TXBUF) = ((p_msg_tx->index & 0x0001) ? (*p_msg): (*p_msg >> 8)) & 0x00FF;
        p_msg_tx->index++;
        fifo_tx_status--;
    }

    return;
}

/*
 * @brief           Test if SPI bus is used as communication bus.
 * @param[in]       *p_spi_rx_msg   Pointer on the message received by DMA.
 * return           The communication bus selected.
 */
void COM_checkSPIBus(spi_rx_msg_t* p_spi_rx_msg, com_bus_e* p_com_bus)
{
    static spi_test_state spi_state = COM_SPI_TEST_STATE_IDLE;
    uint32_t spi_msg_crc            = 0;
    bool_t spi_rx_status            = DMA_getTransferStatusFlag(DMA_COM_SPI_RX_BASE_ADDR);
    bool_t spi_cs_pin               = COM_SPI_CS_READ;

    switch(spi_state)
    {
    case COM_SPI_TEST_STATE_IDLE:
    default:
        spi_state = (spi_cs_pin)    ? (spi_state) : (COM_SPI_TEST_STATE_CS_ACTIVE);
        break;

    case COM_SPI_TEST_STATE_CS_ACTIVE:
        spi_state = (!spi_rx_status)? (spi_state) : (COM_SPI_TEST_STATE_RX_PROGRESS);
        spi_state = (!spi_cs_pin)   ? (spi_state) : (COM_SPI_TEST_STATE_IDLE);
        break;

    case COM_SPI_TEST_STATE_RX_PROGRESS:
        spi_state = (spi_rx_status) ? (spi_state) : (COM_SPI_TEST_STATE_RX_END);
        spi_state = (!spi_cs_pin)   ? (spi_state) : (COM_SPI_TEST_STATE_IDLE);
        break;

    case COM_SPI_TEST_STATE_RX_END:
        spi_state = (!spi_cs_pin)   ? (spi_state) : (COM_SPI_TEST_STATE_CS_INACTIVE);
        break;

    case COM_SPI_TEST_STATE_CS_INACTIVE:
        spi_msg_crc = COM_msgCRC((uint16_t *)&p_spi_rx_msg->data, COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH);
        if((p_spi_rx_msg->crc.u16_msb == MSB_32(spi_msg_crc)) && (p_spi_rx_msg->crc.u16_lsb == LSB_32(spi_msg_crc)))
            *p_com_bus = COM_USE_SPI_BUS;
        spi_state = COM_SPI_TEST_STATE_IDLE;
        break;
    }

    return;
}

void COM_checkCANFDBus(com_bus_e* p_com_bus)
{
    // Get the interrupt status.
    uint32_t mcan_it_status = MCAN_getIntrStatus(MCANA_DRIVER_BASE);
    // Check a valid receive.
    if(mcan_it_status & CANFD_IT_TEST_VALID)
    {
        MCAN_RxNewDataStatus newData = {.statusLow = 0, .statusHigh = 0};
        // Read the NewData registers
        MCAN_getNewDataStatus(MCANA_DRIVER_BASE, &newData);
        //  Clearing the NewData registers
        MCAN_clearNewDataStatus(MCANA_DRIVER_BASE, &newData);
        *p_com_bus = COM_USE_CANFD_BUS;
    }
    // Receive something but wrong.
    else if(mcan_it_status)
    {
        // Shutdown CAN-FD transceiver.
        CANFD_SHDN_MODE_SET;
        // Switch input mux to RS485.
        CANFD_RS485_MUX_SEL_SET;
    }
    // Clear the interrupt status.
    MCAN_clearIntrStatus(MCANA_DRIVER_BASE, mcan_it_status);

    return;
}



//void SCI_rxDisable(uint32_t sci_base)
//{
//    SCI_resetRxFIFO(sci_base);
//    SCI_clearOverflowStatus(sci_base);
//    SCI_clearInterruptStatus(sci_base, SCI_INT_RXERR | SCI_INT_RXFF);
//    SCI_disableRxModule(sci_base);
//}

//void SCI_rxEnable(uint32_t sci_base)
//{
//    SCI_resetRxFIFO(sci_base);
//    SCI_clearOverflowStatus(sci_base);
//    SCI_clearInterruptStatus(sci_base, SCI_INT_RXERR | SCI_INT_RXFF);
//    SCI_enableRxModule(sci_base);
//}

//void SCI_txDisable(uint32_t sci_base)
//{
//    SCI_resetTxFIFO(sci_base);
//    SCI_clearInterruptStatus(sci_base, SCI_INT_TXRDY | SCI_INT_TXFF);
//    SCI_disableInterrupt(sci_base, SCI_INT_TXFF);
////    SCI_disableTxModule(sci_base);
//}

//void SCI_txEnable(uint32_t sci_base)
//{
//    SCI_resetTxFIFO(sci_base);
//    SCI_clearInterruptStatus(sci_base, SCI_INT_TXRDY | SCI_INT_TXFF);
//    SCI_enableInterrupt(sci_base, SCI_INT_TXFF);
////    SCI_enableTxModule(sci_base);
//}

//! \brief Write status message to the corresponding transmission mailbox.
//inline void CAN_setStatusMsg(can_status_msg_u status_msg)
//{
//    ECAN_MBOXES ECanaMboxes;
//
//    ECanaMboxes.MBOX15.MDL.byte.BYTE0 = statusmsg.all;
//
//    return;
//}
//
//
////! \brief Write motor data of motor 1 to the corresponding transmission
////!     mailboxes.
////!
////! \param current_iq  The current Iq of the motor in A.
////! \param encoder_position  Current position of the motor in mechanical
////!                          revolutions (mrev).
////! \param velocity  Velocity of the motor in krpm.
//inline void CAN_setDataMotor1(_iq current_iq, _iq position, _iq velocity)
//{
//    ECanaMboxes.MBOX14.MDL.all = current_iq;
//    ECanaMboxes.MBOX13.MDL.all = position;
//    ECanaMboxes.MBOX12.MDL.all = velocity;
//
//    return;
//}
//
//
////! \brief Write motor data of motor 2 to the corresponding transmission
////!     mailboxes.
////!
////! \param current_iq  The current Iq of the motor in A.
////! \param encoder_position  Current position of the motor in mechanical
////!                          revolutions (mrev).
////! \param velocity  Velocity of the motor in krpm.
//inline void CAN_setDataMotor2(_iq current_iq, _iq encoder_position,
//        _iq velocity)
//{
//    ECanaMboxes.MBOX14.MDH.all = current_iq;
//    ECanaMboxes.MBOX13.MDH.all = encoder_position;
//    ECanaMboxes.MBOX12.MDH.all = velocity;
//
//    return;
//}
//
//
////! \brief Write readings of ADCINA6 and B6 to the corresponding transmission
////!     mailbox.
////!
////! \param adcin_a6 Result of ADCINA6
////! \param adcin_b6 Result of ADCINB6
//inline void CAN_setAdcIn6Values(_iq adcin_a6, _iq adcin_b6)
//{
//    ECanaMboxes.MBOX11.MDL.all = adcin_a6;
//    ECanaMboxes.MBOX11.MDH.all = adcin_b6;
//}
//
//
////! \brief Write encoder index position to the corresponding transmission
////!     mailbox.
////!
////! \param mtrNum Number of the motor (either 0 or 1).
////! \param index_position Position of the motor when the index occured [mrev].
//inline void CAN_setEncoderIndex(uint16_t mtrNum, _iq index_position)
//{
//    ECanaMboxes.MBOX10.MDH.byte.BYTE4 = mtrNum & 0xFF;
//    ECanaMboxes.MBOX10.MDL.all = index_position;
//}
//
//
////! \brief Get the last command message that was received.
////!
////! \returns Last received command message.
//inline CAN_Command_t CAN_getCommand()
//{
//    CAN_Command_t cmd;
//
//    cmd.id = ECanaMboxes.MBOX0.MDH.all;
//    cmd.value = ECanaMboxes.MBOX0.MDL.all;
//
//    return cmd;
//}
//
//
////! \brief Get the last Iq reference that was received.
////!
////! \param mtrNum Number of the motor
////! \returns Last received IqRef value for the specified motor.
//inline _iq CAN_getIqRef(uint16_t mtrNum)
//{
//    if (mtrNum == HAL_MTR1) {
//        return ECanaMboxes.MBOX1.MDL.all;
//    } else {
//        return ECanaMboxes.MBOX1.MDH.all;
//    }
//}
//
//
////! \brief Send data of the specified mailboxes
////!
////! To specify the mailboxes, use the CAN_MBOX_OUT_xy defines for this. Example:
////! to send current, position and velocity of motor 1 call
////! `CAN_send(CAN_MBOX_OUT_IqPos_mtr1 | CAN_MBOX_OUT_SPEED_mtr1);`
////!
////! \param mailboxes  A bitmap specifying the mailboxes to be send.
//inline void CAN_send(uint32_t mailboxes)
//{
//    // Always access whole register, not bitfield.  Therefore no shadow register
//    // should be necessary.
//
//    // Set TRS for all specified mailboxes
//    ECanaRegs.CANTRS.all |= mailboxes;
//    // Wait for all TAn bits of specified mailboxes to be set
////  while ((ECanaRegs.CANTA.all & mailboxes) != mailboxes);
////  // Clear all TAn (we have to set it to 1 so it becomes 0...)
////  ECanaRegs.CANTA.all = mailboxes;
////  // wait for all TAn bits to become zero
////  while ((ECanaRegs.CANTA.all & mailboxes) != 0);
//
//    return;
//}
//
//
////! \brief Abort pending transmissions of the specified mailboxes.
////!
////! \param mailboxes  A bitmap specifying the mailboxes to be send.
//inline void CAN_abort(uint32_t mailboxes)
//{
//    // request transmission reset
//    ECanaRegs.CANTRR.all |= mailboxes;
//    // wait until TRS is cleared
//    //while ((ECanaRegs.CANTRS.all & mailboxes) != 0);
//
//    return;
//}
//
//
////! \brief Check if received message pending flag is set.
////!
////! Check if the "received message pending" flag is set for the specified
////! mailbox.  To clear the flag, use CAN_clearReceivedMessagePending().
////!
////! \param mailbox_mask Bitmask that specifies the mailbox.  If you specify more
////!     than one mailbox in the mask, this function will return true if a
////!     message is pending in at least one of the mailboxes.
////! \returns True if a new message is pending in the specified mailbox.
//inline bool CAN_checkReceivedMessagePending(uint32_t mailbox_mask)
//{
//    return ECanaRegs.CANRMP.all & mailbox_mask;
//}
//
//
////! \brief Clear received message pending flag.
////!
////! \param mailbox_mask Bitmask that specifies the mailbox.  If you specify more
////!     than one mailbox in the mask, the flag is reset for each of them.
//inline void CAN_clearReceivedMessagePending(uint32_t mailbox_mask)
//{
//    // reset bit (have to write a 1 to get a 0)
//    ECanaRegs.CANRMP.all = mailbox_mask;
//}
//
//
////! \brief Check if new message arrived and acknowledge if yes.
////!
////! Check if the "received message pending" flag for the specified mailbox.  If
////! yes, the message is acknowledged (i.e. the flag is reset).
////!
////! \param mailbox_mask Bitmask that specifies the mailbox.  If you specify more
////!     than one mailbox in the mask, this function will return true if a
////!     message is pending in at least one of the mailboxes.
////! \returns True if a new message is pending in the specified mailbox.
//inline bool CAN_checkAndClearRMP(uint32_t mailbox_mask)
//{
//    if (CAN_checkReceivedMessagePending(mailbox_mask))
//    {
//        CAN_clearReceivedMessagePending(mailbox_mask);
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}
//
//
////! \brief Check if a message is waiting for transmission
////!
////! Check if a message is waiting for transmission (i.e. the TRS bit is set) in
////! the specified mailbox.  If more then one mailbox is specified, it is checked
////! if at least one of them has a pending message.
////!
////! \param mailbox_mask Bitmask that specifies the mailbox(es).  See the
////!     CAN_MBOX_OUT_* defines.
////! \returns True if a message is waiting for transmission in one of the
////!     specified mailboxes.
//inline bool CAN_checkTransmissionPending(uint32_t mailbox_mask)
//{
//    return ECanaRegs.CANTRS.all & mailbox_mask;
//}
