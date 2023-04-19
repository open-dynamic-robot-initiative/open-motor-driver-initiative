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
        cmd_reg_u* p_cmd_reg            = &p_cmd_m1->cmdArray;
        p_cmd_m1->posRef                = (float32_t)(pos_s32                   * POSITION_LSB      * FM_ROUND2RAD);
        p_cmd_m1->velRef                = (float32_t)(p_msg->velocity[MOTOR_1]  * VELOCITY_LSB      * FM_KRPM2RADPS);
        p_cmd_m1->iqff                  = (float32_t)(p_msg->current[MOTOR_1]   * IQ_LSB);
        p_cmd_m1->kpCoeff               = (float32_t)(p_msg->kpCoeff[MOTOR_1]   * KP_LSB            * FM_RAD2ROUND);
        p_cmd_m1->kdCoeff               = (float32_t)(p_msg->kdCoeff[MOTOR_1]   * KD_LSB            * FM_RADPS2KRPM);
        float32_t isat                  = (float32_t)(MSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
        p_cmd_m1->iSat                  = (isat < MOTOR1_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR1_CURRENT_CMD_SAT_MAX);
        p_cmd_m1->iSat                  = (p_cmd_m1->iSat > 0.0f)               ? (p_cmd_m1->iSat)  : (MOTOR1_CURRENT_CMD_SAT_MAX);
        p_cmd_m1->timeoutRef            = (uint16_t)p_msg->mode.TIMEOUT;
        p_cmd_m1->cptTimeout            = 0U;
        p_cmd_m1->index                 = p_msg->index;
        p_cmd_reg->encOffsetEnable      = (bool_t)p_msg->mode.EI1OC;
        p_cmd_reg->rollOverEnable       = (bool_t)p_msg->mode.EPRE;
        p_cmd_reg->motorEnable          = (bool_t)p_msg->mode.EM1;
        p_cmd_reg->systemEnable         = (bool_t)p_msg->mode.ES;

        pos_s32                         = (int32_t)FUS_16_TO_32(p_msg->position[MOTOR_2].u16_msb, p_msg->position[MOTOR_2].u16_lsb);
        p_cmd_reg                       = &p_cmd_m2->cmdArray;
        p_cmd_m2->posRef                = (float32_t)(pos_s32                   * POSITION_LSB      * FM_ROUND2RAD);
        p_cmd_m2->velRef                = (float32_t)(p_msg->velocity[MOTOR_2]  * VELOCITY_LSB      * FM_KRPM2RADPS);
        p_cmd_m2->iqff                  = (float32_t)(p_msg->current[MOTOR_2]   * IQ_LSB);
        p_cmd_m2->kpCoeff               = (float32_t)(p_msg->kpCoeff[MOTOR_2]   * KP_LSB            * FM_RAD2ROUND);
        p_cmd_m2->kdCoeff               = (float32_t)(p_msg->kdCoeff[MOTOR_2]   * KD_LSB            * FM_RADPS2KRPM);
        isat                            = (float32_t)(LSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
        p_cmd_m2->iSat                  = (isat < MOTOR2_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR2_CURRENT_CMD_SAT_MAX);
        p_cmd_m2->iSat                  = (p_cmd_m2->iSat > 0.0f)               ? (p_cmd_m2->iSat)  : (MOTOR2_CURRENT_CMD_SAT_MAX);
        p_cmd_m2->timeoutRef            = (uint16_t)p_msg->mode.TIMEOUT;
        p_cmd_m2->cptTimeout            = 0U;
        p_cmd_m2->index                 = p_msg->index;
        p_cmd_reg->encOffsetEnable      = (bool_t)p_msg->mode.EI2OC;
        p_cmd_reg->rollOverEnable       = (bool_t)p_msg->mode.EPRE;
        p_cmd_reg->motorEnable          = (bool_t)p_msg->mode.EM2;
        p_cmd_reg->systemEnable         = (bool_t)p_msg->mode.ES;

        return(true);
    }

    return(false);
}

/**
 * @brief	        Slave to Master message creation/update using motor_foc data structure.
 * @param[in]       p_motor_m1  Pointer on data structure used for MOTOR_1 control.
 * @param[in]       p_motor_m2  Pointer on data structure used for MOTOR_1 control.
 * @param[inout]    p_msg       Pointer on the message to transmit to the Master board
 */
void COM_msgCreate(motor_t* p_motor_m1, motor_t* p_motor_m2, slv2mst_msg_t* p_msg)
{
#if (CLA1_CORE_ENABLE)
    foc_t* p_foc                        = (foc_t *)p_motor_m1->p_foc.ptr;
#else
    foc_t* p_foc                        = p_motor_m1->p_foc;
#endif
    encoder_t* p_enc                    = &p_foc->motor_enc;
    cmd_reg_u cmd_reg                   = {.all = p_foc->motor_cmd.cmdArray.all};
    float32_t pos_offset                = p_enc->thetaAbsolute - ((cmd_reg.encOffsetEnable) ? (p_enc->thetaIndex) : (0.0f));
    int32_t pos                         = (int32_t)(pos_offset                  / POSITION_LSB      * FM_RAD2ROUND);
    uint8_t state                       = p_motor_m1->motor_state;
    bool_t align_done                   = (state == MOTOR_STATE_READY) || (state == MOTOR_STATE_STOP);
    // Update status field - MOTOR_1
    p_msg->status.M1E                   = cmd_reg.motorEnable;      // MOTOR_1 enabled
    p_msg->status.M1R                   = align_done;               // MOTOR_1 ready
    p_msg->status.IDX1T                 = p_enc->indexToggle;       // MOTOR_1 index toggle
    p_msg->status.IDX1D                 = p_enc->indexDetect;       // MOTOR_1 index detected
    p_msg->position[MOTOR_1].u16_msb    = MSB_32(pos);
    p_msg->position[MOTOR_1].u16_lsb    = LSB_32(pos);
    p_msg->velocity[MOTOR_1]            = (int16_t)(p_enc->speed.speedMech[0]   / VELOCITY_LSB      * FM_RADPS2KRPM);
    p_msg->current[MOTOR_1]             = (int16_t)(p_foc->iq                   / IQ_LSB);
    p_msg->coilRes[MOTOR_1]             = (uint16_t)(p_foc->resEst              / RESISTANCE_LSB);
    p_msg->adcSamples[MOTOR_1]          = (uint16_t)(p_foc->motor_acq.vExt      / VOLTAGE_LSB);

    p_msg->status.SE                    = cmd_reg.systemEnable; // System enabled
    p_msg->status.STATUS_RSV1           = 0;
    p_msg->lastCmdIndex                 = p_foc->motor_cmd.index;
    p_msg->timeStamp                    = (uint16_t)CPUTimer_getTimerCount(CPU_TIMER_0_BASE);

#if (CLA1_CORE_ENABLE)
    p_foc                               = (foc_t *)p_motor_m2->p_foc.ptr;
#else
    p_foc                               = p_motor_m2->p_foc;
#endif
    p_enc                               = &p_foc->motor_enc;
    cmd_reg.all                         = p_foc->motor_cmd.cmdArray.all;
    pos_offset                          = p_enc->thetaAbsolute - ((cmd_reg.encOffsetEnable) ? (p_enc->thetaIndex) : (0.0f));
    pos                                 = (int32_t)(pos_offset                  / POSITION_LSB      * FM_RAD2ROUND);
    state                               = p_motor_m2->motor_state;
    align_done                          = (state == MOTOR_STATE_READY) || (state == MOTOR_STATE_STOP);
    // Update status field - MOTOR_2
    p_msg->status.M2E                   = cmd_reg.motorEnable;      // MOTOR_2 enabled
    p_msg->status.M2R                   = align_done;               // MOTOR_2 ready
    p_msg->status.IDX2T                 = p_enc->indexToggle;       // MOTOR_2 index toggle
    p_msg->status.IDX2D                 = p_enc->indexDetect;       // MOTOR_2 index detected
    p_msg->position[MOTOR_2].u16_msb    = MSB_32(pos);
    p_msg->position[MOTOR_2].u16_lsb    = LSB_32(pos);
    p_msg->velocity[MOTOR_2]            = (int16_t)(p_enc->speed.speedMech[0]   / VELOCITY_LSB      * FM_RADPS2KRPM);
    p_msg->current[MOTOR_2]             = (int16_t)(p_foc->iq                   / IQ_LSB);
    p_msg->coilRes[MOTOR_2]             = (uint16_t)(p_foc->resEst              / RESISTANCE_LSB);
    p_msg->adcSamples[MOTOR_2]          = (uint16_t)(p_foc->motor_acq.vExt      / VOLTAGE_LSB);

    error_reg_u* p_err[2]               = {&p_motor_m1->motor_error, &p_motor_m2->motor_error};
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_1]->com_timeout) ? (STATUS_ERROR_SPI_RECV_TIMEOUT)   : (STATUS_ERROR_NO_ERROR);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_2]->com_timeout) ? (STATUS_ERROR_SPI_RECV_TIMEOUT)   : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_1]->pos_rollover)? (STATUS_ERROR_POS_ROLLOVER)       : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_2]->pos_rollover)? (STATUS_ERROR_POS_ROLLOVER)       : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_1]->drv_fault)   ? (STATUS_ERROR_DRV_FAULT)          : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_2]->drv_fault)   ? (STATUS_ERROR_DRV_FAULT)          : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_1]->enc_mismatch)? (STATUS_ERROR_ENCODER1)           : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_2]->enc_mismatch)? (STATUS_ERROR_ENCODER2)           : (p_msg->status.ERROR_CODE);

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
    p_cmd->cmdArray.all         = 0U;

    return;
}

/**
 * @brief           Reset the extraction structure associated with COM_msgExtract_cla
 * @param[in]       p_msg       Pointer on the extraction structure
 */
//void COM_resetMsgExtractStruct(mst2slv_msg_cla_t* p_msg)
//{
//    p_msg->msg_rx_state = COM_RX_STATE_MODE;
//    p_msg->msg_crc32    = UINT32_MAX;
//    p_msg->msg_rx       = 0U;
//    p_msg->msg_tmp      = 0U;
//
//    return;
//}

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
//uint32_t COM_crc32_cla(mst2slv_msg_cla_t* p_msg)
//{
//    p_msg->msg_crc32    = (p_msg->msg_crc32 << 8) ^ crc32_table[((p_msg->msg_crc32 >> 24) ^ MSB_16(p_msg->msg_rx)) & 0xFF];
//    p_msg->msg_crc32    = (p_msg->msg_crc32 << 8) ^ crc32_table[((p_msg->msg_crc32 >> 24) ^ LSB_16(p_msg->msg_rx)) & 0xFF];
//
//    return(p_msg->msg_crc32);
//}
