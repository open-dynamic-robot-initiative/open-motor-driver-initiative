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
    *(p_motor->p_cmdReg[CHANNEL_1]) = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT);
    *(p_motor->p_cmdReg[CHANNEL_2]) = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT);
    *(p_motor->p_cmdReg[CHANNEL_3]) = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT);

    return;
}

/**
 * @brief           Force a hard stop on the 3 phases of the ePWM (low side active)
 * @param[out]      p_motor Pointer on the associated motor structure
 */
inline void MOT_stopCommand(motor_t* p_motor)
{
    *(p_motor->p_cmdReg[CHANNEL_1]) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_cmdReg[CHANNEL_2]) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_cmdReg[CHANNEL_3]) = (uint16_t)PWM_TIMEBASE_CNT;

    return;
}

/**
 * @brief           Control of the motor states
 * @param[inout]    p_motor Pointer on the associated motor structure
 * @param[in]       p_cmd   Pointer on the command structure driving motor
 */
inline bool_t MOT_runControl(motor_t* p_motor)
{
    foc_t*              p_foc           = p_motor->p_foc;
    error_reg_u*        p_err           = &p_motor->motor_error;
    encoder_t*          p_enc           = &p_foc->motor_enc;
    cmd_t*              p_cmd           = &p_foc->motor_cmd;
    uint8_t*            p_state         = &p_motor->motor_state;
    float32_t           tmp             = 0.0f;
    cmd_reg_u           cmd_reg         = {.all = p_cmd->cmdArray.all};
    uint8_t             motor_id        = p_motor->motor_id;
    bool_t              idref_max       = false;
    p_enc->flags.indexOffset            = cmd_reg.encOffsetEnable;

    //  Read currents in 3 phases + Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Read encoder position
    ENC_getPosition(p_enc);
    // Estimate speed
    ENC_getSpeed(p_enc);
    // Rollover test & index
    ENC_getTheta(p_enc);

    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->epwmBaseAddr[CHANNEL_1]);
    p_err->drv_fault            = !GPIO_readPin(p_motor->drvFaultNum)                           || p_err->drv_fault;
    p_err->pos_rollover         = (p_enc->flags.rollOverError && cmd_reg.rollOverEnable)        || p_err->pos_rollover;
    p_err->com_timeout          = (p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef  || p_err->com_timeout;
    p_err->enc_mismatch         = (p_enc->flags.indexDetect && p_enc->flags.indexError)         || p_err->enc_mismatch;
    p_err->enc_failure          = p_enc->flags.phaseError                                       || p_err->enc_failure;
    *p_state                    = (p_err->all)                      ?(MOTOR_STATE_ERROR)        :(*p_state);

    switch(*p_state)
    {
    case MOTOR_STATE_INIT:
    default:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->posAlign         = (motor_id == MOTOR_1)             ?(MOTOR1_POSITION_ALIGN_MAX):(MOTOR2_POSITION_ALIGN_MAX);
        *p_state                = (cmd_reg.motorEnable)             ?(MOTOR_STATE_ALIGN_MOVE)   :(MOTOR_STATE_INIT);
        *p_state                = (cmd_reg.systemEnable)            ?(*p_state)                 :(MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ALIGN_MOVE:
        p_motor->itCnt          = 0U;
        p_foc->idRef           += (motor_id == MOTOR_1)             ?(MOTOR1_CURRENT_ALIGN_INC) :(MOTOR2_CURRENT_ALIGN_INC);
        p_foc->iqRef            = 0.0f;
        p_foc->posAlign        -= (motor_id == MOTOR_1)             ?(MOTOR1_POSITION_ALIGN_DEC):(MOTOR2_POSITION_ALIGN_DEC);
        tmp                     = (motor_id == MOTOR_1)             ?(MOTOR1_CURRENT_ALIGN_MAX) :(MOTOR2_CURRENT_ALIGN_MAX);
        idref_max               = (p_foc->idRef >= tmp)             ?(true)                     :(false);
        tmp                     = 2.0f * ((motor_id == MOTOR_1)     ?(MOTOR1_ENC_RESOLUTION_RAD):(MOTOR2_ENC_RESOLUTION_RAD));
        p_enc->flags.phaseError = idref_max && (fabsf(p_enc->thetaMech[NEW]) < tmp) ?(true)     :(false);
        *p_state                = (idref_max)                       ?(MOTOR_STATE_ALIGN_FIX)    :(MOTOR_STATE_ALIGN_MOVE);
        *p_state                = (cmd_reg.motorEnable)             ?(*p_state)                 :(MOTOR_STATE_INIT);
        *p_state                = (cmd_reg.systemEnable)            ?(*p_state)                 :(MOTOR_STATE_INIT);
        (*p_state != MOTOR_STATE_ALIGN_MOVE)                        ?(ENC_resetStruct(p_enc))   :(p_enc->thetaElec = p_foc->posAlign);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ALIGN_FIX:
        p_motor->itCnt         += 1U; // wait 2s
        p_foc->idRef            = (motor_id == MOTOR_1)             ?(MOTOR1_CURRENT_ALIGN_MAX) :(MOTOR2_CURRENT_ALIGN_MAX);
        p_foc->iqRef            = 0.0f;
        *p_state                = (p_motor->itCnt >= (2 * PWM_FREQ))?(MOTOR_STATE_READY)        :(MOTOR_STATE_ALIGN_FIX);
        *p_state                = (cmd_reg.motorEnable)             ?(*p_state)                 :(MOTOR_STATE_INIT);
        *p_state                = (cmd_reg.systemEnable)            ?(*p_state)                 :(MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPosVel);
        *p_state                = (cmd_reg.motorEnable)             ?(MOTOR_STATE_READY)        :(MOTOR_STATE_STOP);
        *p_state                = (cmd_reg.systemEnable)            ?(*p_state)                 :(MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        *p_state                = (cmd_reg.motorEnable)             ?(MOTOR_STATE_READY)        :(MOTOR_STATE_STOP);
        *p_state                = (cmd_reg.systemEnable)            ?(*p_state)                 :(MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ERROR:
        p_motor->itCnt          = p_motor->itCnt;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        *p_state                = MOTOR_STATE_ERROR;
        MOT_stopCommand(p_motor);
        break;
    }

    return(true);
}
