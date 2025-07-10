/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#ifdef __TMS320C28XX_VCRC__
#include "vcu2/vcu2_crc.h"
#endif
#include "string.h"
#include "device.h"
#include "board.h"

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
    int32_t dim = 0;

    // Extract motor 1 command:
    for (dim = 0; dim < ROBOT_STATE_DIMS; dim++)
    {
        p_cmd_m1->linCtrlAlpha[dim] = p_msg->linCtrlAlphas[0][dim];
    }

    p_cmd_m1->linCtrlBeta = p_msg->linCtrlBetas[0];

    float32_t isat                      = (float32_t)(MSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
    p_cmd_m1->iSat                      = (isat < MOTOR1_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR1_CURRENT_CMD_SAT_MAX);
//    p_cmd_m1->iSat                      = (p_cmd_m1->iSat > 0.0f)               ? (p_cmd_m1->iSat)  : (MOTOR1_CURRENT_CMD_SAT_MAX);

    p_cmd_m1->timeoutRef                = (uint16_t)p_msg->mode.TIMEOUT;
    p_cmd_m1->cptTimeout                = 0U;
    p_cmd_m1->index                     = p_msg->index;

    cmd_reg_u* p_cmd_reg                = &p_cmd_m1->cmdField;
    p_cmd_reg->encOffsetEnable          = (bool_t)p_msg->mode.EI1OC;
    p_cmd_reg->rollOverEnable           = (bool_t)p_msg->mode.EPRE;
    p_cmd_reg->motorEnable              = (bool_t)p_msg->mode.EM1;
    p_cmd_reg->systemEnable             = (bool_t)p_msg->mode.ES;

    // Extract motor 2 command:
    for (dim = 0; dim < ROBOT_STATE_DIMS; dim++)
    {
        p_cmd_m2->linCtrlAlpha[dim] = p_msg->linCtrlAlphas[1][dim];
    }

    p_cmd_m2->linCtrlBeta = p_msg->linCtrlBetas[1];

    isat                                = (float32_t)(LSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
    p_cmd_m2->iSat                      = (isat < MOTOR2_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR2_CURRENT_CMD_SAT_MAX);
//    p_cmd_m2->iSat                      = (p_cmd_m2->iSat > 0.0f)               ? (p_cmd_m2->iSat)  : (MOTOR2_CURRENT_CMD_SAT_MAX);

    p_cmd_m2->timeoutRef                = (uint16_t)p_msg->mode.TIMEOUT;
    p_cmd_m2->cptTimeout                = 0U;
    p_cmd_m2->index                     = p_msg->index;

    p_cmd_reg                           = &p_cmd_m2->cmdField;
    p_cmd_reg->encOffsetEnable          = (bool_t)p_msg->mode.EI2OC;
    p_cmd_reg->rollOverEnable           = (bool_t)p_msg->mode.EPRE;
    p_cmd_reg->motorEnable              = (bool_t)p_msg->mode.EM2;
    p_cmd_reg->systemEnable             = (bool_t)p_msg->mode.ES;

    return;
}

/**
 * @brief           Slave to Master message creation/update. IT Call.
 * @param[in]       *p_motor     Pointer on current data structure used for control.
 * @param[inout]    *p_msg       Pointer on the message to transmit to the Master board.
 * @param[in]       *p_tmr_cnt   Pointer on the current value of the timer counter.
 * @param[in]       *p_fsi_state (if daisy chain enabled) Pointer to the state of the FSI chain.
 */
#if DAISY_CHAIN_ENABLE
void COM_msgCreate(motor_t* p_motor, slv2mst_msg_t* p_msg, uint32_t* p_tmr_cnt, volatile fsi_comm_state_e* p_fsi_state)
#else
void COM_msgCreate(motor_t* p_motor, slv2mst_msg_t* p_msg, uint32_t* p_tmr_cnt)
#endif
{
    foc_t* p_foc                        = &p_motor->foc;
    encoder_t* p_enc                    = &p_foc->enc;
    error_reg_u motor_err               = p_motor->error;
    cmd_reg_u motor_cmd                 = {.all = p_foc->cmd.cmdField.all};
    float32_t pos_rad                   = (motor_cmd.encOffsetEnable) ? (p_enc->thetaFromIndex) : (p_enc->thetaAbsolute);
    int32_t pos                         = (int32_t)(pos_rad                  / POSITION_LSB      * FM_RAD2ROUND);
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

//        return(false);
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
        p_msg->timeStamp                = (uint16_t)*p_tmr_cnt;
        p_msg->status.ERROR_CODE        = (motor_err.com_timeout)   ? (STATUS_ERROR_SPI_RECV_TIMEOUT)   : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.pos_rollover)  ? (STATUS_ERROR_POS_ROLLOVER)       : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.drv_fault)     ? (STATUS_ERROR_DRV_FAULT)          : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.enc_indexErr)  ? (STATUS_ERROR_ENCODER2)           : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.enc_alignErr)  ? (STATUS_ERROR_ENCODER2)           : (p_msg->status.ERROR_CODE);

//        return(true);
    }

#if DAISY_CHAIN_ENABLE
    // Mark whether there was an FSI error:
    p_msg->status.ERROR_CODE = (*p_fsi_state == COMM_ERROR) ? (STATUS_ERROR_FSI) : (p_msg->status.ERROR_CODE);
#endif

    return;
}

/**
 * @brief           Initialize the command structure for a motor
 * @param[in]       p_cmd       Pointer on the command structure (actions to be realized by motor_foc)
 * @param[in]       motor_id    Motor identifier.
 */
void COM_initCmdStruct(cmd_t* p_cmd, uint16_t motor_id)
{
    p_cmd->linCtrlBeta          = 0.0f;
    memset(p_cmd->linCtrlAlpha, 0, ROBOT_STATE_DIMS * sizeof(float32_t));

    p_cmd->iSat                 = (motor_id == MOTOR_1) ? \
            (MOTOR1_CURRENT_CMD_SAT_MAX) : \
            (MOTOR2_CURRENT_CMD_SAT_MAX);

    p_cmd->timeoutRef           = 0U;
    p_cmd->cptTimeout           = 0U;
    p_cmd->index                = 0U;
    p_cmd->cmdField.all         = 0U;
    p_cmd->periphField.all      = 0U;

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
 * @brief           Read & concatenate the data available in the SCI peripheral. MSB first.
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
        *p_msg              = (p_msg_rx->index & 1U) ? (*p_msg | (tmp & LSB_BYTE_MASK)) : ((tmp << 8) & MSB_BYTE_MASK);
        p_msg_rx->index++;
    }

    return;
}

/*
 * @brief           Fill the SCI peripheral Tx FIFO with message to be sent.
 * @param[in]       sci-base    SCI peripheral identifier.
 * @param[out]      *p_msg_tx   Pointer on the RS485 transmission structure.
 */
inline void RS485_msgTransmit(uint32_t sci_base, rs485_tx_t* p_msg_tx)
{
//    uint16_t fifo_tx_status = (uint16_t)(SCI_FIFO_TX16 - SCI_getTxFIFOStatus(sci_base));
    SCI_TxFIFOLevel fifo_tx_status  = SCI_FIFO_TX16;
    fifo_tx_status                 -= (SCI_TxFIFOLevel)((HWREGH(sci_base + SCI_O_FFTX) & SCI_FFTX_TXFFST_M) >> SCI_FFTX_TXFFST_S);

    while((uint16_t)fifo_tx_status && ((2 * COM_MSG_RS485_TX_16BIT_FULL_LENGTH) - p_msg_tx->index))//p_msg_tx->remaining)
    {
//        uint16_t* p_msg = &p_msg_tx->msg[p_msg_tx->msg_cr].raw[p_msg_tx->index >> 1];
//        HWREGH(sci_base + SCI_O_TXBUF) = ((p_msg_tx->index & 0x0001) ? (*p_msg): (*p_msg >> 8)) & 0x00FF;
        uint16_t* p_msg = &p_msg_tx->msg_8b[p_msg_tx->msg_cr][p_msg_tx->index];
        HWREGH(sci_base + SCI_O_TXBUF) = *p_msg;// & 0x00FF;
        p_msg_tx->index++;
        fifo_tx_status--;
    }

    return;
}

/*
 * @brief           Split 16bits words into 8bits words for RS485.
 * @param[inout]    *p_msg_tx   Pointer on the RS485 transmission structure.
 */
void RS485_msgWordToBytes(rs485_tx_t* p_msg_tx)
{
    uint16_t i, msg_active = p_msg_tx->msg_nx;
    for(i = 0; i < COM_MSG_RS485_TX_16BIT_FULL_LENGTH; i++)
    {
        p_msg_tx->msg_8b[msg_active][2 * i]         = p_msg_tx->msg[msg_active].raw[i];
        p_msg_tx->msg_8b[msg_active][2 * i]         = (p_msg_tx->msg_8b[msg_active][2 * i] >> 8) & 0x00FF;
        p_msg_tx->msg_8b[msg_active][(2 * i) + 1]   = p_msg_tx->msg[msg_active].raw[i];
        p_msg_tx->msg_8b[msg_active][(2 * i) + 1]   = p_msg_tx->msg_8b[msg_active][(2 * i) + 1] & 0x00FF;
    }

    return;
}

/*
 * @brief           Read & concatenate the data available in the MCAN peripheral. MSB first.
 * @param[out]      *p_msg_rx   Pointer on the MCAN buffer reception structure.
 */
inline void CANFD_msgBytesToWord(canfd_rx_t* p_msg_rx)
{
//    uint16_t* p_buf = &p_msg_rx->buf.data[0];
//    uint16_t* p_msg = &p_msg_rx->bytes2short[0];
//    uint16_t index  = 0;
//
//    while(index < MCAN_MAX_PAYLOAD_BYTES)
//    {
//        *p_msg  = ((index++) & 1U) ? (*p_msg | (*p_buf & LSB_BYTE_MASK)) : ((*p_buf << 8) & MSB_BYTE_MASK);
//        p_buf   = &p_msg_rx->buf.data[index];
//        p_msg   = &p_msg_rx->bytes2short[index >> 1];
//    }
//
//    return;

    uint16_t i, msg_build = 0;
    for(i = 0; i < COM_MSG_CANFD_TX_RX_8BIT_FULL_LENGTH; i = i + 2)
    {
        msg_build                   = (p_msg_rx->buf.data[i] << 8) & 0xFF00;
        msg_build                  |= (p_msg_rx->buf.data[i + 1] & 0x00FF);
        p_msg_rx->msg_16b[i >> 1]   = msg_build;
    }
}

/*
 * @brief           Write & concatenate the data available in the MCAN peripheral. MSB first.
 * @param[out]      *p_msg_rx   Pointer on the MCAN buffer reception structure.
 */
inline void CANFD_msgWordToBytes(canfd_tx_t* p_msg_tx)
{
//    uint16_t* p_msg_8b  = &p_msg_tx->buf.data[0];
//    uint16_t* p_msg_16b = &p_msg_tx->msg_16b[0];
//    uint16_t index  = 0;
//
//    while(index < MCAN_MAX_PAYLOAD_BYTES)
//    {
//        *p_buf  = (((index++) & 1U) ? (*p_msg) : (*p_msg >> 8)) & LSB_BYTE_MASK;
//        p_buf   = &p_msg_tx->buf.data[index];
//        p_msg   = &p_msg_tx->short2bytes[index >> 1];
//    }

    uint16_t i;
    for(i = 0; i < COM_MSG_CANFD_TX_RX_16BIT_FULL_LENGTH; i++)
    {
        p_msg_tx->buf.data[2 * i]       = p_msg_tx->msg_16b[i];
        p_msg_tx->buf.data[2 * i]       = (p_msg_tx->buf.data[2 * i] >> 8) & 0x00FF;
        p_msg_tx->buf.data[(2 * i) + 1] = p_msg_tx->msg_16b[i];
        p_msg_tx->buf.data[(2 * i) + 1] = p_msg_tx->msg_16b[(2 * i) + 1] & 0x00FF;
    }

    return;
}
