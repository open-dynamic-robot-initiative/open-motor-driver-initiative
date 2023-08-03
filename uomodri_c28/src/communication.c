/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdlib.h>
#include "vcu2/vcu2_crc.h"

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
bool_t COM_msgExtract(mst2slv_msg_t* p_msg, mst2slv_msg_t* p_swap, cmd_t* p_cmd_m1, cmd_t* p_cmd_m2)
{
    COM_msgSwapEndian((uint16_t *)p_msg, (uint16_t *)p_swap, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
    uint32_t crc                        = COM_vcrc32((uint16_t *)p_swap, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
    if((MSB_32(crc) == p_msg->crc.u16_msb) && (LSB_32(crc) == p_msg->crc.u16_lsb))
    {
        int32_t pos_s32                 = (int32_t)FUS_16_TO_32(p_msg->position[MOTOR_1].u16_msb, p_msg->position[MOTOR_1].u16_lsb);
        cmd_reg_u* p_cmd_reg            = &p_cmd_m1->cmdBits;
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
        p_cmd_reg                       = &p_cmd_m2->cmdBits;
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

bool_t COM_msgExtractOnTheFly(mst2slv_msg_otf_t* p_msg, cmd_t* p_cmd_m1, cmd_t* p_cmd_m2, uint16_t msg_raw)
{
    float32_t   isat            = 0.0f;
    bool_t      msg_rx_valid    = false;

    switch(p_msg->msg_rx_state)
    {
    case COM_RX_STATE_MODE:
    default:
        p_cmd_m1->timeoutRef                = (uint16_t)(msg_raw & MSG_RX_TIMEOUT_MASK);
        p_cmd_m1->cmdBits.encOffsetEnable   = (bool_t)(msg_raw & MSG_RX_EI1OC_MASK);
        p_cmd_m1->cmdBits.motorEnable       = (bool_t)(msg_raw & MSG_RX_EM1_MASK);
        p_cmd_m1->cmdBits.rollOverEnable    = (bool_t)(msg_raw & MSG_RX_EPRE_MASK);
        p_cmd_m1->cmdBits.systemEnable      = (bool_t)(msg_raw & MSG_RX_ES_MASK);
        p_cmd_m2->cmdBits.encOffsetEnable   = (bool_t)(msg_raw & MSG_RX_EI2OC_MASK);
        p_cmd_m2->cmdBits.motorEnable       = (bool_t)(msg_raw & MSG_RX_EM2_MASK);
        p_cmd_m2->cmdBits.rollOverEnable    = p_cmd_m1->cmdBits.rollOverEnable;
        p_cmd_m2->cmdBits.systemEnable      = p_cmd_m1->cmdBits.systemEnable;
        p_cmd_m2->timeoutRef                = p_cmd_m1->timeoutRef;
        p_msg->msg_crc32                    = UINT32_MAX;
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_POS_MSB_MOT1;
        break;

    case COM_RX_STATE_POS_MSB_MOT1:
        p_msg->msg_tmp                      = msg_raw;
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_POS_LSB_MOT1;//(p_msg->msg_rx_state == COM_RX_STATE_POS_MSB_MOT1) ? (COM_RX_STATE_POS_LSB_MOT1) : (COM_RX_STATE_POS_LSB_MOT2);
        break;

    case COM_RX_STATE_POS_LSB_MOT1:
        p_cmd_m1->posRef                    = (float32_t)((int32_t)FUS_16_TO_32(msg_raw, p_msg->msg_tmp) * POSITION_LSB * FM_ROUND2RAD);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_POS_MSB_MOT2;
        break;

    case COM_RX_STATE_POS_MSB_MOT2:
        p_msg->msg_tmp                      = msg_raw;
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_POS_LSB_MOT2;//(p_msg->msg_rx_state == COM_RX_STATE_POS_MSB_MOT1) ? (COM_RX_STATE_POS_LSB_MOT1) : (COM_RX_STATE_POS_LSB_MOT2);
        break;

    case COM_RX_STATE_POS_LSB_MOT2:
        p_cmd_m2->posRef                    = (float32_t)((int32_t)FUS_16_TO_32(msg_raw, p_msg->msg_tmp) * POSITION_LSB * FM_ROUND2RAD);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_VEL_MOT1;
        break;

    case COM_RX_STATE_VEL_MOT1:
        p_cmd_m1->velRef                    = (float32_t)(msg_raw * VELOCITY_LSB * FM_KRPM2RADPS);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_VEL_MOT2;
        break;

    case COM_RX_STATE_VEL_MOT2:
        p_cmd_m2->velRef                    = (float32_t)(msg_raw * VELOCITY_LSB * FM_KRPM2RADPS);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_IQ_MOT1;
        break;

    case COM_RX_STATE_IQ_MOT1:
        p_cmd_m1->iqff                      = (float32_t)(msg_raw * IQ_LSB);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_IQ_MOT2;
        break;

    case COM_RX_STATE_IQ_MOT2:
        p_cmd_m2->iqff                      = (float32_t)(msg_raw * IQ_LSB);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_KP_MOT1;
        break;

    case COM_RX_STATE_KP_MOT1:
        p_cmd_m1->kpCoeff                   = (float32_t)(msg_raw * KP_LSB * FM_RAD2ROUND);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_KP_MOT2;
        break;

    case COM_RX_STATE_KP_MOT2:
        p_cmd_m2->kpCoeff                   = (float32_t)(msg_raw * KP_LSB * FM_RAD2ROUND);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_KD_MOT1;
        break;

    case COM_RX_STATE_KD_MOT1:
        p_cmd_m1->kdCoeff                   = (float32_t)(msg_raw * KD_LSB * FM_RADPS2KRPM);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_KD_MOT2;
        break;

    case COM_RX_STATE_KD_MOT2:
        p_cmd_m2->kdCoeff                   = (float32_t)(msg_raw * KD_LSB * FM_RADPS2KRPM);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_ISAT_MOT12;
        break;

    case COM_RX_STATE_ISAT_MOT12:
        isat                                = (float32_t)(MSB_16(msg_raw) * CURRENT_SAT_LSB);
        p_cmd_m1->iSat                      = (isat < MOTOR1_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR1_CURRENT_CMD_SAT_MAX);
        p_cmd_m1->iSat                      = (p_cmd_m1->iSat > 0.0f)               ? (p_cmd_m1->iSat)  : (MOTOR1_CURRENT_CMD_SAT_MAX);
        isat                                = (float32_t)(LSB_16(msg_raw) * CURRENT_SAT_LSB);
        p_cmd_m2->iSat                      = (isat < MOTOR2_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR2_CURRENT_CMD_SAT_MAX);
        p_cmd_m2->iSat                      = (p_cmd_m2->iSat > 0.0f)               ? (p_cmd_m2->iSat)  : (MOTOR2_CURRENT_CMD_SAT_MAX);
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_INDEX;
        break;

    case COM_RX_STATE_INDEX:
        p_cmd_m1->index                     = msg_raw;
        p_cmd_m2->index                     = msg_raw;
        p_msg->msg_crc32                    = COM_crc32((uint32_t)p_msg->msg_crc32, (uint16_t *)&msg_raw, (uint16_t)1U);
        p_msg->msg_rx_state                 = COM_RX_STATE_CRC_LSB;
        break;

    case COM_RX_STATE_CRC_LSB:
        p_msg->msg_tmp                      = msg_raw;
        p_msg->msg_rx_state                 = COM_RX_STATE_CRC_MSB;
        break;

    case COM_RX_STATE_CRC_MSB:
        msg_rx_valid                        = (p_msg->msg_crc32 == FUS_16_TO_32(p_msg->msg_tmp, msg_raw)) ? (true) : (false);
        p_msg->msg_rx_state                 = COM_RX_STATE_MODE;
        break;
    }

    return (msg_rx_valid);
}

/**
 * @brief           Slave to Master message creation/update.
 * @param[in]       *p_motor_m1 Pointer on data structure used for MOTOR_1 control.
 * @param[in]       *p_motor_m2 Pointer on data structure used for MOTOR_2 control.
 * @param[inout]    *p_msg      Pointer on the message to transmit to the Master board.
 * @param[inout]    *p_swap     Pointer on the temporary message necessary for CRC computation.
 */
void COM_msgCreate(motor_t* p_motor_m1, motor_t* p_motor_m2, slv2mst_msg_t* p_msg, slv2mst_msg_t* p_swap)
{
    foc_t* p_foc                        = &p_motor_m1->foc;
    encoder_t* p_enc                    = &p_foc->enc;
    cmd_reg_u cmd_reg                   = {.all = p_foc->cmd.cmdBits.all};
    float32_t theta_index_comp          = (cmd_reg.encOffsetEnable) ? (p_enc->thetaIndexAbsolute) : (0.0f);
    float32_t pos_offset                = p_enc->thetaAbsolute - theta_index_comp;
    int32_t pos                         = (int32_t)(pos_offset                  / POSITION_LSB      * FM_RAD2ROUND);
    bool_t align_done                   = (p_motor_m1->state == MOTOR_STATE_READY) || (p_motor_m1->state == MOTOR_STATE_STOP);
    // Update status field - MOTOR_1
    p_msg->status.M1E                   = cmd_reg.motorEnable;      // MOTOR_1 enabled
    p_msg->status.M1R                   = align_done;               // MOTOR_1 ready
    p_msg->status.IDX1T                 = p_enc->flags.indexToggle; // MOTOR_1 index toggle
    p_msg->status.IDX1D                 = p_enc->flags.indexDetect; // MOTOR_1 index detected
    p_msg->position[MOTOR_1].u16_msb    = MSB_32(pos);
    p_msg->position[MOTOR_1].u16_lsb    = LSB_32(pos);
    p_msg->velocity[MOTOR_1]            = (int16_t)(p_enc->speed.speedMech[0]   / VELOCITY_LSB      * FM_RADPS2KRPM);
    p_msg->current[MOTOR_1]             = (int16_t)(p_foc->iq                   / IQ_LSB);
    p_msg->coilRes[MOTOR_1]             = (uint16_t)(p_foc->resEst              / RESISTANCE_LSB);
    p_msg->adcSamples[MOTOR_1]          = (uint16_t)(p_foc->acq.vExt            / VOLTAGE_LSB);

    p_msg->status.SE                    = cmd_reg.systemEnable; // System enabled
    p_msg->status.STATUS_RSV1           = 0;
    p_msg->lastCmdIndex                 = p_foc->cmd.index;
    p_msg->timeStamp                    = (uint16_t)CPUTimer_getTimerCount(CPU_TIMER_0_BASE);

    p_foc                               = &p_motor_m2->foc;
    p_enc                               = &p_foc->enc;
    cmd_reg.all                         = p_foc->cmd.cmdBits.all;
    theta_index_comp                    = (cmd_reg.encOffsetEnable) ? (p_enc->thetaIndexAbsolute) : (0.0f);
    pos_offset                          = p_enc->thetaAbsolute - theta_index_comp;
    pos                                 = (int32_t)(pos_offset                  / POSITION_LSB      * FM_RAD2ROUND);
    align_done                          = (p_motor_m2->state == MOTOR_STATE_READY) || (p_motor_m2->state == MOTOR_STATE_STOP);
    // Update status field - MOTOR_2
    p_msg->status.M2E                   = cmd_reg.motorEnable;      // MOTOR_2 enabled
    p_msg->status.M2R                   = align_done;               // MOTOR_2 ready
    p_msg->status.IDX2T                 = p_enc->flags.indexToggle; // MOTOR_2 index toggle
    p_msg->status.IDX2D                 = p_enc->flags.indexDetect; // MOTOR_2 index detected
    p_msg->position[MOTOR_2].u16_msb    = MSB_32(pos);
    p_msg->position[MOTOR_2].u16_lsb    = LSB_32(pos);
    p_msg->velocity[MOTOR_2]            = (int16_t)(p_enc->speed.speedMech[0]   / VELOCITY_LSB      * FM_RADPS2KRPM);
    p_msg->current[MOTOR_2]             = (int16_t)(p_foc->iq                   / IQ_LSB);
    p_msg->coilRes[MOTOR_2]             = (uint16_t)(p_foc->resEst              / RESISTANCE_LSB);
    p_msg->adcSamples[MOTOR_2]          = (uint16_t)(p_foc->acq.vExt            / VOLTAGE_LSB);

    error_reg_u* p_err[2]               = {&p_motor_m1->error, &p_motor_m2->error};
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_1]->com_timeout) ? (STATUS_ERROR_SPI_RECV_TIMEOUT)   : (STATUS_ERROR_NO_ERROR);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_2]->com_timeout) ? (STATUS_ERROR_SPI_RECV_TIMEOUT)   : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_1]->pos_rollover)? (STATUS_ERROR_POS_ROLLOVER)       : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_2]->pos_rollover)? (STATUS_ERROR_POS_ROLLOVER)       : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_1]->drv_fault)   ? (STATUS_ERROR_DRV_FAULT)          : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_2]->drv_fault)   ? (STATUS_ERROR_DRV_FAULT)          : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_1]->enc_indexErr)? (STATUS_ERROR_ENCODER1)           : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_2]->enc_indexErr)? (STATUS_ERROR_ENCODER2)           : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_1]->enc_alignErr)? (STATUS_ERROR_ENCODER1)           : (p_msg->status.ERROR_CODE);
    p_msg->status.ERROR_CODE            = (p_err[MOTOR_2]->enc_alignErr)? (STATUS_ERROR_ENCODER2)           : (p_msg->status.ERROR_CODE);

    // Compute CRC
    COM_msgSwapEndian((uint16_t *)p_msg, (uint16_t *)p_swap, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
    uint32_t crc                        = COM_vcrc32((uint16_t *)p_swap, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
    p_msg->crc.u16_msb                  = LSB_32(crc);              // required for Solo
    p_msg->crc.u16_lsb                  = MSB_32(crc);

    return;
}

/**
 * @brief           Slave to Master message creation/update. IT Call.
 * @param[in]       *p_motor    Pointer on current data structure used for control.
 * @param[inout]    *p_msg      Pointer on the message to transmit to the Master board.
 * @param[inout]    *p_swap     Pointer on the temporary message necessary for CRC computation.
 */
bool_t COM_msgCreateIT(motor_t* p_motor, slv2mst_msg_t* p_msg, slv2mst_msg_t* p_swap)
{
    foc_t* p_foc                        = &p_motor->foc;
    encoder_t* p_enc                    = &p_foc->enc;
    error_reg_u motor_err               = p_motor->error;
    cmd_reg_u motor_cmd                 = {.all = p_foc->cmd.cmdBits.all};
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
        p_msg->timeStamp                = (uint16_t)CPUTimer_getTimerCount(CPU_TIMER_0_BASE);

        p_msg->status.ERROR_CODE        = (motor_err.com_timeout)   ? (STATUS_ERROR_SPI_RECV_TIMEOUT)   : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.pos_rollover)  ? (STATUS_ERROR_POS_ROLLOVER)       : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.drv_fault)     ? (STATUS_ERROR_DRV_FAULT)          : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.enc_indexErr)  ? (STATUS_ERROR_ENCODER2)           : (p_msg->status.ERROR_CODE);
        p_msg->status.ERROR_CODE        = (motor_err.enc_alignErr)  ? (STATUS_ERROR_ENCODER2)           : (p_msg->status.ERROR_CODE);

        // Compute CRC
        COM_msgSwapEndian((uint16_t *)p_msg, (uint16_t *)p_swap, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
        uint32_t crc                    = COM_vcrc32((uint16_t *)p_swap, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
        p_msg->crc.u16_msb              = LSB_32(crc);              // required for Solo
        p_msg->crc.u16_lsb              = MSB_32(crc);

        return(true);
    }
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
    p_cmd->cmdBits.all          = 0U;

    return;
}

/**
 * @brief           Reset the extraction structure associated with COM_msgExtract_cla
 * @param[in]       p_msg       Pointer on the extraction structure
 */
void COM_resetMsgExtractOnTheFlyStruct(mst2slv_msg_otf_t* p_msg)
{
    p_msg->msg_rx_state = COM_RX_STATE_MODE;
    p_msg->msg_crc32    = UINT32_MAX;
    p_msg->msg_tmp      = 0U;

    return;
}

/**
 * @brief       	Software cyclic redundancy check (CRC) using predefined array.
 * @param[in]       p_buf   Pointer on the array to be CRCed.
 * @param[in]       len     Length of the array
 * @return
 */
uint32_t COM_crc32(uint32_t seed, uint16_t* p_buf, uint16_t len)
{
  uint32_t crc = seed;
  while(len--)
    {
      crc = (crc << 8) ^ crc32_table_normal[((crc >> 24) ^ MSB_16(*p_buf)) & 0xFF];
      crc = (crc << 8) ^ crc32_table_normal[((crc >> 24) ^ LSB_16(*p_buf)) & 0xFF];
      p_buf++;
    }

  return(crc);
}


/**
 * @brief           Endianness conversion. Swap bytes in a message.
 * @param[in]       *p_msg  Pointer on the message to swap.
 * @param[out]      *p_swap Pointer on the swapped message.
 * @param[in]       length  Number of elements to be swapped.
 */
void COM_msgSwapEndian(uint16_t* p_msg, uint16_t* p_swap, uint16_t length)
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
uint32_t COM_vcrc32(uint16_t* p_msg, uint16_t length)
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
