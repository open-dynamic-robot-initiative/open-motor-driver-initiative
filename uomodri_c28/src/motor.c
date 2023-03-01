/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "motor.h"
#include "encoder.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       	Command for the 3 phases of the ePWM
 * @param[out]      p_motor Pointer on the associated motor structure
 * @param[in]       cmd_a   Normalized command on PWM channel A (between 0 and 1)
 * @param[in]       cmd_b   Normalized command on PWM channel B (between 0 and 1)
 * @param[in]       cmd_c   Normalized command on PWM channel C (between 0 and 1)
 */
inline void MOT_runCommand(motor_t* p_motor, float32_t cmd_a, float32_t cmd_b, float32_t cmd_c)
{
    *(p_motor->p_motorChAReg) = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT);
    *(p_motor->p_motorChBReg) = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT);
    *(p_motor->p_motorChCReg) = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT);

    return;
}

/**
 * @brief           Force a hard stop on the 3 phases of the ePWM (low side active)
 * @param[out]      p_motor Pointer on the associated motor structure
 */
inline void MOT_stopCommand(motor_t* p_motor)
{
    *(p_motor->p_motorChAReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorChBReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorChCReg) = (uint16_t)PWM_TIMEBASE_CNT;

    return;
}

/**
 * @brief           Control of the motor states
 * @param[inout]    p_motor Pointer on the associated motor structure
 * @param[in]       p_cmd   Pointer on the command structure driving motor
 */
inline bool_t MOT_runControl(motor_t* p_motor)
{
    static float32_t enc_theta[2] = {MOTOR1_THETA_ALIGN_MAX, MOTOR2_THETA_ALIGN_MAX};

    uint8_t         id          = p_motor->motor_id;
    foc_t*          p_foc       = p_motor->p_motorFOC;
//    error_reg_u*    p_err       = &p_motor->motor_error;
    error_reg_u     err         = {.all = 0};
    encoder_t*      p_enc       = &p_foc->motor_enc;
    cmd_t*          p_cmd       = &p_foc->motor_cmd;
    cmd_reg_t       en_bit      = p_cmd->enableReg.bit;
    p_enc->indexOffset          = en_bit.encOffsetEnable;

    // Read the phases current, voltage & Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Read encoder position
    ENC_getPosition(p_enc);
    // Estimate speed
    ENC_getSpeed(p_enc);
    // Rollover test & index
    ENC_getTheta(p_enc);

    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->p_motorHalCfg->p_pwmCntCmp[0]->epwmBase);

#if 1
    err.bit.drv_fault           = !GPIO_readPin(p_motor->p_motorDRV->p_drvCfgHandler->gpioNumber_FAULT);
    err.bit.pos_rollover        = p_enc->rollOverError && en_bit.rollOverEnable;
    err.bit.com_timeout         = (p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef;
    err.bit.enc_mismatch        = p_enc->indexDetect && p_enc->indexError;
    p_motor->motor_state        = (err.all)                         ? (MOTOR_STATE_ERROR)       : (p_motor->motor_state);
    p_motor->motor_error.all    = err.all;
//    p_err->bit.drv_fault        = !GPIO_readPin(p_motor->p_motorDRV->p_drvCfgHandler->gpioNumber_FAULT);
//    p_err->bit.pos_rollover     = p_enc->rollOverError && en_bit.rollOverEnable;
//    p_err->bit.com_timeout      = (p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef;
//    p_err->bit.enc_mismatch     = p_enc->indexDetect && p_enc->indexError;
//    p_motor->motor_state        = (p_err->all)                      ? (MOTOR_STATE_ERROR)       : (p_motor->motor_state);
#else
    p_motor->motor_error.bit.drv_fault      = !GPIO_readPin(p_motor->p_motorDRV->p_drvCfgHandler->gpioNumber_FAULT);
    p_motor->motor_error.bit.pos_rollover   = p_enc->rollOverError && en_bit.rollOverEnable;
    p_motor->motor_error.bit.com_timeout    = (p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef;
    p_motor->motor_error.bit.enc_mismatch   = p_enc->indexDetect && p_enc->indexError;
    p_motor->motor_state        = (p_motor->motor_error.all)        ? (MOTOR_STATE_ERROR)       : (p_motor->motor_state);
#endif

    switch(p_motor->motor_state)
    {
    case MOTOR_STATE_INIT:
    default:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (en_bit.motorEnable)              ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ALIGN_UP:
        p_motor->itCnt          = 0U;
        p_foc->idRef           += (id == MOTOR_1)                   ? (MOTOR1_CURRENT_ALIGN_INC): (MOTOR2_CURRENT_ALIGN_INC); // Increment for current alignment procedure
        p_foc->iqRef            = 0.0f;
        enc_theta[id]          -= (id == MOTOR_1)                   ? (MOTOR1_THETA_ALIGN_DEC)  : (MOTOR2_THETA_ALIGN_DEC); // Decrement for position alignment procedure
        p_motor->motor_state    = (p_foc->idRef < p_foc->iAlignMax) ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_ALIGN_FIX);
        p_motor->motor_state    = (en_bit.motorEnable)              ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        (p_motor->motor_state  != MOTOR_STATE_ALIGN_UP)             ? (ENC_resetStruct(p_enc))  : (p_enc->thetaElec = enc_theta[id]);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ALIGN_FIX:
        p_motor->itCnt         += 1U; // wait 2s
        p_foc->idRef            = p_foc->iAlignMax; // Maintain alignment current
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (p_motor->itCnt < (2 * PWM_FREQ)) ? (MOTOR_STATE_ALIGN_FIX)   : (MOTOR_STATE_READY);
        p_motor->motor_state    = (en_bit.motorEnable)              ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPosVel);
        p_motor->motor_state    = (en_bit.motorEnable)              ? (MOTOR_STATE_READY)       : (MOTOR_STATE_STOP);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (!en_bit.motorEnable)             ? (MOTOR_STATE_STOP)        : (MOTOR_STATE_READY);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ERROR:
        p_motor->itCnt          = p_motor->itCnt;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = MOTOR_STATE_ERROR;
//        ENC_resetPeriph(p_enc);
//        FOC_resetStruct(p_foc);
//        FOC_runControl(p_foc);
        MOT_stopCommand(p_motor);
        break;
    }

    return(true);
}
