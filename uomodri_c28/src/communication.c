/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "uomodri_user_defines.h"
#include "communication.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief           Master to slave message extraction function
 * @param[in]       p_msg       Pointer on the message received from Master board
 * @param[out]      p_cmd_m1    Pointer on the command structure used for MOTOR_1 control(actions to be realized by motor_foc).
 * @param[out]      p_cmd_m2    Pointer on the command structure used for MOTOR_2 control(actions to be realized by motor_foc).
 * @return          Is message received from Master board valid or not ?
 */
bool_t COM_msgExtract(mst2slv_msg_t* p_msg,  cmd_t* p_cmd_m1,  cmd_t* p_cmd_m2)
{
    uint32_t crc                        = COM_crc32((uint16_t *) p_msg, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
    if((MSB_32(crc) == p_msg->crc.u16_msb) && (LSB_32(crc) == p_msg->crc.u16_lsb))
    {
        int32_t pos_s32                 = (int32_t)FUS_16_TO_32(p_msg->position[MOTOR_1].u16_msb, p_msg->position[MOTOR_1].u16_lsb);
        cmd_reg_t* p_cmd_reg            = &p_cmd_m1->enableReg.bit;
        p_cmd_m1->posRef                = (float32_t)(pos_s32                   * POSITION_LSB      * FM_ROUND2RAD);
        p_cmd_m1->velRef                = (float32_t)(p_msg->velocity[MOTOR_1]  * VELOCITY_LSB      * FM_KRPM2RADPS);
        p_cmd_m1->iqff                  = (float32_t)(p_msg->current[MOTOR_1]   * IQ_LSB);
        p_cmd_m1->kpCoeff               = (float32_t)(p_msg->kpCoeff[MOTOR_1]   * KP_LSB            * FM_RAD2ROUND);
        p_cmd_m1->kdCoeff               = (float32_t)(p_msg->kdCoeff[MOTOR_1]   * KD_LSB            * FM_RADPS2KRPM);
        float32_t isat                  = (float32_t)(MSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
        p_cmd_m1->iSat                  = (isat < MOTOR1_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR1_CURRENT_CMD_SAT_MAX);
        p_cmd_m1->iSat                  = (p_cmd_m1->iSat > 0.0f)               ? (p_cmd_m1->iSat)  : (MOTOR1_CURRENT_CMD_SAT_MAX);
        p_cmd_m1->timeoutRef            = (uint16_t)p_msg->mode.bit.TIMEOUT;
        p_cmd_m1->cptTimeout            = 0U;
        p_cmd_m1->index                 = p_msg->index;
        p_cmd_reg->encOffsetEnable      = (bool_t)p_msg->mode.bit.EI1OC;
        p_cmd_reg->rollOverEnable       = (bool_t)p_msg->mode.bit.EPRE;
        p_cmd_reg->motorEnable          = (bool_t)p_msg->mode.bit.EM1;
        p_cmd_reg->systemEnable         = (bool_t)p_msg->mode.bit.ES;

        pos_s32                         = (int32_t)FUS_16_TO_32(p_msg->position[MOTOR_2].u16_msb, p_msg->position[MOTOR_2].u16_lsb);
        p_cmd_reg                       = &p_cmd_m2->enableReg.bit;
        p_cmd_m2->posRef                = (float32_t)(pos_s32                   * POSITION_LSB      * FM_ROUND2RAD);
        p_cmd_m2->velRef                = (float32_t)(p_msg->velocity[MOTOR_2]  * VELOCITY_LSB      * FM_KRPM2RADPS);
        p_cmd_m2->iqff                  = (float32_t)(p_msg->current[MOTOR_2]   * IQ_LSB);
        p_cmd_m2->kpCoeff               = (float32_t)(p_msg->kpCoeff[MOTOR_2]   * KP_LSB            * FM_RAD2ROUND);
        p_cmd_m2->kdCoeff               = (float32_t)(p_msg->kdCoeff[MOTOR_2]   * KD_LSB            * FM_RADPS2KRPM);
        isat                            = (float32_t)(LSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
        p_cmd_m2->iSat                  = (isat < MOTOR2_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR2_CURRENT_CMD_SAT_MAX);
        p_cmd_m2->iSat                  = (p_cmd_m2->iSat > 0.0f)               ? (p_cmd_m2->iSat)  : (MOTOR2_CURRENT_CMD_SAT_MAX);
        p_cmd_m2->timeoutRef            = (uint16_t)p_msg->mode.bit.TIMEOUT;
        p_cmd_m2->cptTimeout            = 0U;
        p_cmd_m2->index                 = p_msg->index;
        p_cmd_reg->encOffsetEnable      = (bool_t)p_msg->mode.bit.EI2OC;
        p_cmd_reg->rollOverEnable       = (bool_t)p_msg->mode.bit.EPRE;
        p_cmd_reg->motorEnable          = (bool_t)p_msg->mode.bit.EM2;
        p_cmd_reg->systemEnable         = (bool_t)p_msg->mode.bit.ES;

        return(true);
    }

    return(false);
}

/**
 * @brief       	Master to slave message extraction function to be performed by CLA - TO BE TESTED
 * @param[in]       p_msg       Pointer on the extraction structure
 * @param[out]      p_cmd_m1    Pointer on the command structure used for MOTOR_1 control(actions to be realized by motor_foc).
 * @param[out]      p_cmd_m2    Pointer on the command structure used for MOTOR_2 control(actions to be realized by motor_foc).
 * @return          Is message received from Master board valid or not ?
 */
bool_t COM_msgExtract_cla(mst2slv_msg_cla_t* p_msg, cmd_t* p_cmd_m1, cmd_t* p_cmd_m2)
{
    float32_t   isat            = 0.0f;
    bool_t      msg_rx_valid    = false;

    switch(p_msg->msg_rx_state)
    {
    case COM_RX_STATE_MODE:
        p_cmd_m1->enableReg.bit.encOffsetEnable = (bool_t)((p_msg->msg_rx & MSG_RX_EI1OC)   ? (true): (false));
        p_cmd_m1->enableReg.bit.rollOverEnable  = (bool_t)((p_msg->msg_rx & MSG_RX_EPRE)    ? (true): (false));
        p_cmd_m1->enableReg.bit.motorEnable     = (bool_t)((p_msg->msg_rx & MSG_RX_EM1)     ? (true): (false));
        p_cmd_m1->enableReg.bit.systemEnable    = (bool_t)((p_msg->msg_rx & MSG_RX_ES)      ? (true): (false));
        p_cmd_m1->timeoutRef                    = (uint16_t)p_msg->msg_rx & MSG_RX_TIMEOUT;
        p_cmd_m2->enableReg.bit.encOffsetEnable = (bool_t)((p_msg->msg_rx & MSG_RX_EI2OC)   ? (true): (false));
        p_cmd_m2->enableReg.bit.rollOverEnable  = (bool_t)((p_msg->msg_rx & MSG_RX_EPRE)    ? (true): (false));
        p_cmd_m2->enableReg.bit.motorEnable     = (bool_t)((p_msg->msg_rx & MSG_RX_EM2)     ? (true): (false));
        p_cmd_m2->enableReg.bit.systemEnable    = (bool_t)((p_msg->msg_rx & MSG_RX_ES)      ? (true): (false));
        p_cmd_m2->timeoutRef                    = (uint16_t)p_msg->msg_rx & MSG_RX_TIMEOUT;
        p_msg->msg_crc32                        = UINT32_MAX;
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_POS_MSB_MOT1:
    case COM_RX_STATE_POS_MSB_MOT2:
        p_msg->msg_tmp      = p_msg->msg_rx;
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_POS_LSB_MOT1:
        p_cmd_m1->posRef    = (float32_t)((int32_t)FUS_16_TO_32(p_msg->msg_tmp, p_msg->msg_rx) * POSITION_LSB * FM_ROUND2RAD);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_POS_LSB_MOT2:
        p_cmd_m2->posRef    = (float32_t)((int32_t)FUS_16_TO_32(p_msg->msg_tmp, p_msg->msg_rx) * POSITION_LSB * FM_ROUND2RAD);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_VEL_MOT1:
        p_cmd_m1->velRef    = (float32_t)(p_msg->msg_rx * VELOCITY_LSB * FM_KRPM2RADPS);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_VEL_MOT2:
        p_cmd_m2->velRef    = (float32_t)(p_msg->msg_rx * VELOCITY_LSB * FM_KRPM2RADPS);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_IQ_MOT1:
        p_cmd_m1->iqff      = (float32_t)(p_msg->msg_rx * IQ_LSB);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_IQ_MOT2:
        p_cmd_m2->iqff      = (float32_t)(p_msg->msg_rx * IQ_LSB);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_KP_MOT1:
        p_cmd_m1->kpCoeff   = (float32_t)(p_msg->msg_rx * KP_LSB * FM_RAD2ROUND);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_KP_MOT2:
        p_cmd_m2->kpCoeff   = (float32_t)(p_msg->msg_rx * KP_LSB * FM_RAD2ROUND);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_KD_MOT1:
        p_cmd_m1->kdCoeff   = (float32_t)(p_msg->msg_rx * KD_LSB * FM_RADPS2KRPM);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_KD_MOT2:
        p_cmd_m2->kdCoeff   = (float32_t)(p_msg->msg_rx * KD_LSB * FM_RADPS2KRPM);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_ISAT_MOT12:
        isat                = (float32_t)(MSB_16(p_msg->msg_rx) * CURRENT_SAT_LSB);
        p_cmd_m1->iSat      = (isat < MOTOR1_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR1_CURRENT_CMD_SAT_MAX);
        p_cmd_m1->iSat      = (p_cmd_m1->iSat > 0.0f)               ? (p_cmd_m1->iSat)  : (MOTOR1_CURRENT_CMD_SAT_MAX);
        isat                = (float32_t)(LSB_16(p_msg->msg_rx) * CURRENT_SAT_LSB);
        p_cmd_m2->iSat      = (isat < MOTOR2_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR2_CURRENT_CMD_SAT_MAX);
        p_cmd_m2->iSat      = (p_cmd_m2->iSat > 0.0f)               ? (p_cmd_m2->iSat)  : (MOTOR2_CURRENT_CMD_SAT_MAX);
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_INDEX:
        p_cmd_m1->index     = p_msg->msg_rx;
        p_cmd_m2->index     = p_msg->msg_rx;
        COM_crc32_cla(p_msg);
        break;

    case COM_RX_STATE_CRC_LSB:
        p_msg->msg_tmp      = p_msg->msg_rx;
        break;

    case COM_RX_STATE_CRC_MSB:
        msg_rx_valid        = (p_msg->msg_crc32 == FUS_16_TO_32(p_msg->msg_tmp, p_msg->msg_rx)) ? (true) : (false);
        break;
    }

    return (msg_rx_valid);
}

/**
 * @brief	        Slave to Master message creation/update using motor_foc data structure.
 * @param[in]       p_motor_m1  Pointer on data structure used for MOTOR_1 control.
 * @param[in]       p_motor_m2  Pointer on data structure used for MOTOR_1 control.
 * @param[inout]    p_msg       Pointer on the message to transmit to the Master board
 */
void COM_msgCreate(motor_t* p_motor_m1, motor_t* p_motor_m2, slv2mst_msg_t* p_msg)
{
    foc_t* p_foc                        = p_motor_m1->p_motorFOC;
    motor_state_e state                 = p_motor_m1->motor_state;
    error_reg_u* p_err[2]               = {&p_motor_m1->motor_error, &p_motor_m2->motor_error};
    encoder_t* p_enc                    = &p_foc->motor_enc;
    cmd_reg_t* p_cmd_reg                = &p_foc->motor_cmd.enableReg.bit;
    float32_t pos_offset                = p_enc->thetaAbsolute - ((p_cmd_reg->encOffsetEnable) ? (p_enc->thetaIndex) : (0.0f));
    int32_t pos                         = (int32_t)(pos_offset                  / POSITION_LSB      * FM_RAD2ROUND);
    bool_t align_done                   = (state == MOTOR_STATE_READY) || (state == MOTOR_STATE_STOP);
    // Update status field - MOTOR_1
    p_msg->status.bit.M1E               = p_cmd_reg->motorEnable;   // MOTOR_1 enabled
    p_msg->status.bit.M1R               = align_done;               // MOTOR_1 ready
    p_msg->status.bit.IDX1T             = p_enc->indexToggle;       // MOTOR_1 index toggle
    p_msg->status.bit.IDX1D             = p_enc->indexDetect;       // MOTOR_1 index detected
    p_msg->position[MOTOR_1].u16_msb    = MSB_32(pos);
    p_msg->position[MOTOR_1].u16_lsb    = LSB_32(pos);
    p_msg->velocity[MOTOR_1]            = (int16_t)(p_enc->speed.speedMech[0]   / VELOCITY_LSB      * FM_RADPS2KRPM);
    p_msg->current[MOTOR_1]             = (int16_t)(p_foc->iq                   / IQ_LSB);
    p_msg->coilRes[MOTOR_1]             = (uint16_t)(p_foc->resEst              / RESISTANCE_LSB);
    p_msg->adcSamples[MOTOR_1]          = (uint16_t)(p_foc->motor_acq.vExt      / VOLTAGE_LSB);

    p_msg->status.bit.SE                = p_cmd_reg->systemEnable;  // System enabled
    p_msg->status.bit.STATUS_RSV1       = 0;
    p_msg->lastCmdIndex                 = p_foc->motor_cmd.index;
    p_msg->timeStamp                    = (uint16_t)CPUTimer_getTimerCount(CPU_TIMER_0_BASE);

    p_foc                               = p_motor_m2->p_motorFOC;
    state                               = p_motor_m2->motor_state;
    p_enc                               = &p_foc->motor_enc;
    p_cmd_reg                           = &p_foc->motor_cmd.enableReg.bit;
    pos_offset                          = p_enc->thetaAbsolute - ((p_cmd_reg->encOffsetEnable) ? (p_enc->thetaIndex) : (0.0f));
    pos                                 = (int32_t)(pos_offset                  / POSITION_LSB      * FM_RAD2ROUND);
    align_done                          = (state == MOTOR_STATE_READY) || (state == MOTOR_STATE_STOP);
    // Update status field - MOTOR_2
    p_msg->status.bit.M2E               = p_cmd_reg->motorEnable;   // MOTOR_2 enabled
    p_msg->status.bit.M2R               = align_done;               // MOTOR_2 ready
    p_msg->status.bit.IDX2T             = p_enc->indexToggle;       // MOTOR_2 index toggle
    p_msg->status.bit.IDX2D             = p_enc->indexDetect;       // MOTOR_2 index detected
    p_msg->position[MOTOR_2].u16_msb    = MSB_32(pos);
    p_msg->position[MOTOR_2].u16_lsb    = LSB_32(pos);
    p_msg->velocity[MOTOR_2]            = (int16_t)(p_enc->speed.speedMech[0]   / VELOCITY_LSB      * FM_RADPS2KRPM);
    p_msg->current[MOTOR_2]             = (int16_t)(p_foc->iq                   / IQ_LSB);
    p_msg->coilRes[MOTOR_2]             = (uint16_t)(p_foc->resEst              / RESISTANCE_LSB);
    p_msg->adcSamples[MOTOR_2]          = (uint16_t)(p_foc->motor_acq.vExt      / VOLTAGE_LSB);

#if 0
    if((!p_err[MOTOR_1]->all) && (!p_err[MOTOR_2]->all))
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_NO_ERROR;
    else if((p_err[MOTOR_1]->bit.pos_rollover) || (p_err[MOTOR_2]->bit.pos_rollover))
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_POS_ROLLOVER;
    else if((p_err[MOTOR_1]->bit.drv_fault) || (p_err[MOTOR_2]->bit.drv_fault))
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_DRV_NFAULT;
    else if(p_err[MOTOR_1]->bit.enc_mismatch)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_ENCODER1;
    else if(p_err[MOTOR_2]->bit.enc_mismatch)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_ENCODER2;
    else if((p_err[MOTOR_1]->bit.com_timeout) || (p_err[MOTOR_2]->bit.com_timeout))
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_SPI_RECV_TIMEOUT;
#elif 0
    bool_t no_err                       = (!p_err[MOTOR_1]->all)            && (!p_err[MOTOR_2]->all);
    bool_t rollover_err                 = p_err[MOTOR_1]->bit.pos_rollover  || p_err[MOTOR_2]->bit.pos_rollover;
    bool_t drv_fault_err                = p_err[MOTOR_1]->bit.drv_fault     || p_err[MOTOR_2]->bit.drv_fault;
    bool_t timeout_err                  = p_err[MOTOR_1]->bit.com_timeout   || p_err[MOTOR_2]->bit.com_timeout;
    bool_t enc1_err                     = p_err[MOTOR_1]->bit.enc_mismatch;
    bool_t enc2_err                     = p_err[MOTOR_2]->bit.enc_mismatch;

    if(no_err)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_NO_ERROR;
    else
    {
        if(rollover_err)
            p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_POS_ROLLOVER;
        else
        {
            if(drv_fault_err)
                p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_DRV_NFAULT;
            else
            {
                if(enc1_err)
                    p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_ENCODER1;
                else
                {
                    if(enc2_err)
                        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_ENCODER2;
                    else
                    {
                        if(timeout_err)
                            p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_SPI_RECV_TIMEOUT;
                    }
                }
            }
        }
    }
#else
    bool_t no_err                       = (!p_err[MOTOR_1]->all)            && (!p_err[MOTOR_2]->all);
    bool_t rollover_err                 = p_err[MOTOR_1]->bit.pos_rollover  || p_err[MOTOR_2]->bit.pos_rollover;
    bool_t drv_fault_err                = p_err[MOTOR_1]->bit.drv_fault     || p_err[MOTOR_2]->bit.drv_fault;
    bool_t timeout_err                  = p_err[MOTOR_1]->bit.com_timeout   || p_err[MOTOR_2]->bit.com_timeout;
    bool_t enc1_err                     = p_err[MOTOR_1]->bit.enc_mismatch;
    bool_t enc2_err                     = p_err[MOTOR_2]->bit.enc_mismatch;

    if(no_err)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_NO_ERROR;
    else if(rollover_err)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_POS_ROLLOVER;
    else if(drv_fault_err)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_DRV_NFAULT;
    else if(enc1_err)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_ENCODER1;
    else if(enc2_err)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_ENCODER2;
    else if(timeout_err)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_SPI_RECV_TIMEOUT;
#endif
    // Compute CRC
    uint32_t crc = COM_crc32((uint16_t *)p_msg, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
    p_msg->crc.u16_msb                  = LSB_32(crc);              // required for Solo
    p_msg->crc.u16_lsb                  = MSB_32(crc);

    return;
}

/**
 * @brief           Reset the command structure for a motor
 * @param[in]       p_cmd   Pointer on the command structure (actions to be realized by motor_foc)
 */
void COM_resetCmdStruct(cmd_t* p_cmd)
{
    p_cmd->posRef               = 0.0f;
    p_cmd->velRef               = 0.0f;
    p_cmd->iqff                 = 0.0f;
    p_cmd->kpCoeff              = 0.0f;
    p_cmd->kdCoeff              = 0.0f;
    p_cmd->iSat                 = 0.0f;
    p_cmd->timeoutRef           = 0U;
    p_cmd->cptTimeout           = 0U;
    p_cmd->index                = 0U;
    p_cmd->enableReg.all        = 0U;

    return;
}

/**
 * @brief           Reset the extraction structure associated with COM_msgExtract_cla
 * @param[in]       p_msg       Pointer on the extraction structure
 */
void COM_resetMsgExtractStruct(mst2slv_msg_cla_t* p_msg)
{
    p_msg->msg_rx_state = COM_RX_STATE_MODE;
    p_msg->msg_crc32    = UINT32_MAX;
    p_msg->msg_rx       = 0U;
    p_msg->msg_tmp      = 0U;

    return;
}

/**
 * @brief       	Software cyclic redundancy check (CRC) using predefined array.
 * @param[in]       p_buf   Pointer on the array to be CRCed.
 * @param[in]       len     Length of the array
 * @return
 */
uint32_t COM_crc32(uint16_t* p_buf, uint16_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  while (len--)
    {
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ MSB_16(*p_buf)) & 0xFF];
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ LSB_16(*p_buf)) & 0xFF];
      p_buf++;
    }

  return(crc);
}

/**
 * @brief           Software cyclic redundancy check (CRC) using predefined array. Used with COM_msgExtract_cla.
 *                  CRC computed at each new state
 * @param[in]       data    Value to be added into CRC.
 * @param[inout]    crc     Current state of CRC.
 * @return          New computed value of CRC.
 */
uint32_t COM_crc32_cla(mst2slv_msg_cla_t* p_msg)
{
    p_msg->msg_crc32    = (p_msg->msg_crc32 << 8) ^ crc32_table[((p_msg->msg_crc32 >> 24) ^ MSB_16(p_msg->msg_rx)) & 0xFF];
    p_msg->msg_crc32    = (p_msg->msg_crc32 << 8) ^ crc32_table[((p_msg->msg_crc32 >> 24) ^ LSB_16(p_msg->msg_rx)) & 0xFF];

    return(p_msg->msg_crc32);
}
