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
#if (USE_STATE_MACHIN_4_ELECTRONCS_ENGINEERS)
    foc_t*              p_foc           = p_motor->p_foc;
    error_reg_u*        p_err           = &p_motor->motor_error;
    encoder_t*          p_enc           = &p_foc->motor_enc;
    cmd_t*              p_cmd           = &p_foc->motor_cmd;
    uint8_t*            p_state         = &p_motor->motor_state;
    uint64_t            tmp             = 0UL;
    float32_t           diff            = 0.0f;
    float32_t           err[2]          = {0.0f, 0.0f};
    cmd_reg_u           cmd_reg         = {.all = p_cmd->cmdArray.all};
    uint8_t             motor_id        = p_motor->motor_id;
    bool_t              evt_max         = false;
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
    p_err->enc_failure          = p_enc->flags.encError                                         || p_err->enc_failure;
    *p_state                    = (p_err->all)              ? (MOTOR_STATE_ERROR)           : (*p_state);

    switch(*p_state)
    {
    case MOTOR_STATE_START:
    default:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->initPos          = 0.0f;
        *p_state                = (cmd_reg.motorEnable)     ? (MOTOR_STATE_INIT_FW)         : (MOTOR_STATE_START);
        *p_state                = (cmd_reg.systemEnable)    ? (*p_state)                    : (MOTOR_STATE_START);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_INIT_FW:
        tmp                     = (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_EVT)    : (MOTOR2_POSITION_INIT_EVT);
        evt_max                 = (p_motor->itCnt > tmp)    ? (true)                        : (false);
        p_motor->itCnt          = (evt_max)                 ? (0U)                          : (p_motor->itCnt + 1);
        p_foc->initPosEnc[FW]   = (evt_max)                 ? (p_enc->thetaAbsolute)        : (p_foc->initPosEnc[FW]);
        p_foc->idRef           += (motor_id == MOTOR_1)     ? (MOTOR1_CURRENT_INIT_STEP)    : (MOTOR2_CURRENT_INIT_STEP);
        p_foc->iqRef            = 0.0f;
        p_foc->initPos         -= (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_STEP)   : (MOTOR2_POSITION_INIT_STEP);
        p_enc->thetaElec        = p_foc->initPos;
        *p_state                = (evt_max)                 ? (MOTOR_STATE_INIT_REV)        : (MOTOR_STATE_INIT_FW);
        *p_state                = (cmd_reg.motorEnable)     ? (*p_state)                    : (MOTOR_STATE_START);
        *p_state                = (cmd_reg.systemEnable)    ? (*p_state)                    : (MOTOR_STATE_START);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_REV:
        tmp                     = (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_EVT)    : (MOTOR2_POSITION_INIT_EVT);
        evt_max                 = (p_motor->itCnt > tmp)    ? (true)                        : (false);
        p_motor->itCnt          = (evt_max)                 ? (0U)                          : (p_motor->itCnt + 1);
        p_foc->initPosEnc[REV]  = (evt_max)                 ? (p_enc->thetaAbsolute)        : (p_foc->initPosEnc[REV]);
        p_foc->idRef            = (motor_id == MOTOR_1)     ? (MOTOR1_CURRENT_INIT_MAX)     : (MOTOR2_CURRENT_INIT_MAX);
        p_foc->iqRef            = 0.0f;
        p_foc->initPos         += (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_STEP)   : (MOTOR2_POSITION_INIT_STEP);
        p_enc->thetaElec        = p_foc->initPos;
        p_motor->motor_state    = (evt_max)                 ? (MOTOR_STATE_INIT_TEST)       : (MOTOR_STATE_INIT_REV);
        p_motor->motor_state    = (cmd_reg.motorEnable)     ? (*p_state)                    : (MOTOR_STATE_START);
        p_motor->motor_state    = (cmd_reg.systemEnable)    ? (*p_state)                    : (MOTOR_STATE_START);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_TEST:
        diff                    = p_foc->initPosEnc[REV] - p_foc->initPosEnc[FW];
        err[MIN]                = (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_ERR_MIN): (MOTOR2_POSITION_INIT_ERR_MIN);
        err[MAX]                = (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_ERR_MAX): (MOTOR2_POSITION_INIT_ERR_MAX);
        p_enc->flags.encError   = (diff > err[MIN])         ? (false)                       : (true);
        p_enc->flags.encError   = (diff < err[MAX])         ? (p_enc->flags.encError)       : (true);
        p_motor->itCnt          = 0U;
        p_foc->idRef            = (motor_id == MOTOR_1)     ? (MOTOR1_CURRENT_INIT_MAX)     : (MOTOR2_CURRENT_INIT_MAX);
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (cmd_reg.motorEnable)     ? (MOTOR_STATE_INIT_FIX)        : (MOTOR_STATE_START);
        p_motor->motor_state    = (cmd_reg.systemEnable)    ? (*p_state)                    : (MOTOR_STATE_START);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_FIX:
        tmp                     = (motor_id == MOTOR_1)     ? (MOTOR1_FIX_INIT_EVT)         : (MOTOR2_FIX_INIT_EVT);
        evt_max                 = (p_motor->itCnt > tmp)    ? (true)                        : (false);
        p_motor->itCnt         += 1U;
        p_foc->idRef            = (motor_id == MOTOR_1)     ? (MOTOR1_CURRENT_INIT_MAX)     : (MOTOR2_CURRENT_INIT_MAX);
        p_foc->iqRef            = 0.0f;
        *p_state                = (evt_max)                 ? (MOTOR_STATE_READY)           : (MOTOR_STATE_INIT_FIX);
        *p_state                = (cmd_reg.motorEnable)     ? (*p_state)                    : (MOTOR_STATE_START);
        *p_state                = (cmd_reg.systemEnable)    ? (*p_state)                    : (MOTOR_STATE_START);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

     case MOTOR_STATE_READY:
         p_motor->itCnt         += 1U;
         p_foc->idRef            = 0.0f;
         p_foc->iqRef            = FOC_runPD(&p_foc->pdPosVel);
         *p_state                = (cmd_reg.motorEnable)     ? (MOTOR_STATE_READY)           : (MOTOR_STATE_STOP);
         *p_state                = (cmd_reg.systemEnable)    ? (*p_state)                    : (MOTOR_STATE_START);
         FOC_runControl(p_foc);
         MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
         break;

     case MOTOR_STATE_STOP:
         p_motor->itCnt         += 1U;
         p_foc->idRef            = 0.0f;
         p_foc->iqRef            = 0.0f;
         *p_state                = (cmd_reg.motorEnable)     ? (MOTOR_STATE_READY)           : (MOTOR_STATE_STOP);
         *p_state                = (cmd_reg.systemEnable)    ? (*p_state)                    : (MOTOR_STATE_START);
         MOT_stopCommand(p_motor);
         break;

     case MOTOR_STATE_ERROR:
         p_motor->itCnt          = p_motor->itCnt;
         p_foc->idRef            = 0.0f;
         p_foc->iqRef            = 0.0f;
         *p_state                = MOTOR_STATE_ERROR;
         MOT_stopCommand(p_motor);
         break;
     }
#elif (USE_STATE_MACHIN_4_COMPUTER_SCIENTISTS)
    foc_t*              p_foc       = p_motor->p_foc;
    error_reg_u*        p_err       = &p_motor->motor_error;
    encoder_t*          p_enc       = &p_foc->motor_enc;
    cmd_t*              p_cmd       = &p_foc->motor_cmd;
    uint8_t*            p_state     = &p_motor->motor_state;
    uint64_t            tmp         = 0UL;
    float32_t           diff        = 0.0f;
    float32_t           err[2]      = {0.0f, 0.0f};
    cmd_reg_u           cmd_reg     = {.all = p_cmd->cmdArray.all};
    uint8_t             motor_id    = p_motor->motor_id;
    p_enc->flags.indexOffset        = cmd_reg.encOffsetEnable;

    //  Read currents in 3 phases + Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Read encoder position
    ENC_getPosition(p_enc);
    // Estimate speed
    ENC_getSpeed(p_enc);
    // Rollover test & index
    ENC_getTheta(p_enc);

    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb              = EPWM_getTimeBaseCounterValue(p_motor->epwmBaseAddr[CHANNEL_1]);
    p_err->drv_fault                = !GPIO_readPin(p_motor->drvFaultNum)                           || p_err->drv_fault;
    p_err->pos_rollover             = (p_enc->flags.rollOverError && cmd_reg.rollOverEnable)        || p_err->pos_rollover;
    p_err->com_timeout              = (p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef  || p_err->com_timeout;
    p_err->enc_mismatch             = (p_enc->flags.indexDetect && p_enc->flags.indexError)         || p_err->enc_mismatch;
    p_err->enc_failure              = p_enc->flags.encError                                         || p_err->enc_failure;
    if(p_err->all)
    *p_state                        = MOTOR_STATE_ERROR;

    switch(*p_state)
    {
    case MOTOR_STATE_START:
    default:
        p_motor->itCnt              = 0U;
        p_foc->idRef                = 0.0f;
        p_foc->iqRef                = 0.0f;
        p_foc->initPos              = 0.0f;
        if(!cmd_reg.motorEnable || !cmd_reg.systemEnable)
            *p_state                = MOTOR_STATE_START;
        else
            *p_state                = MOTOR_STATE_INIT_FW;
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_INIT_FW:
        if(motor_id == MOTOR_1)
        {
            tmp                     = MOTOR1_POSITION_INIT_EVT;
            p_foc->idRef           += MOTOR1_CURRENT_INIT_STEP;
            p_foc->initPos         -= MOTOR1_POSITION_INIT_STEP;
        }
        else
        {
            tmp                     = MOTOR2_POSITION_INIT_EVT;
            p_foc->idRef           += MOTOR2_CURRENT_INIT_STEP;
            p_foc->initPos         -= MOTOR2_POSITION_INIT_STEP;
        }
        if(p_motor->itCnt > tmp)
        {
            p_motor->itCnt          = 0U;
            p_foc->initPosEnc[FW]   = p_enc->thetaAbsolute;
            *p_state                = MOTOR_STATE_INIT_REV;
        }
        else
        {
            p_motor->itCnt++;
            *p_state                = MOTOR_STATE_INIT_FW;
        }
        p_foc->iqRef                = 0.0f;
        p_enc->thetaElec            = p_foc->initPos;

        if(!cmd_reg.motorEnable || !cmd_reg.systemEnable)
            *p_state                = MOTOR_STATE_START;
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_REV:
        if(motor_id == MOTOR_1)
        {
            tmp                     = MOTOR1_POSITION_INIT_EVT;
            p_foc->idRef            = MOTOR1_CURRENT_INIT_MAX;
            p_foc->initPos         += MOTOR1_POSITION_INIT_STEP;
        }
        else
        {
            tmp                     = MOTOR2_POSITION_INIT_EVT;
            p_foc->idRef            = MOTOR2_CURRENT_INIT_MAX;
            p_foc->initPos         += MOTOR2_POSITION_INIT_STEP;
        }
        if(p_motor->itCnt > tmp)
        {
            p_motor->itCnt          = 0U;
            p_foc->initPosEnc[REV]  = p_enc->thetaAbsolute;
            *p_state                = MOTOR_STATE_INIT_TEST;
        }
        else
        {
            p_motor->itCnt++;
            *p_state                = MOTOR_STATE_INIT_REV;
        }
        p_foc->iqRef                = 0.0f;
        p_enc->thetaElec            = p_foc->initPos;
        if(!cmd_reg.motorEnable || !cmd_reg.systemEnable)
            *p_state                = MOTOR_STATE_START;
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_TEST:
        if(motor_id == MOTOR_1)
        {
            err[MIN]                = MOTOR1_POSITION_INIT_ERR_MIN;
            err[MAX]                = MOTOR1_POSITION_INIT_ERR_MAX;
            p_foc->idRef            = MOTOR1_CURRENT_INIT_MAX;
        }
        else
        {
            err[MIN]                = MOTOR2_POSITION_INIT_ERR_MIN;
            err[MAX]                = MOTOR2_POSITION_INIT_ERR_MAX;
            p_foc->idRef            = MOTOR2_CURRENT_INIT_MAX;
        }
        p_motor->itCnt              = 0U;
        p_foc->iqRef                = 0.0f;
        diff                        = p_foc->initPosEnc[REV] - p_foc->initPosEnc[FW];
        if((diff > err[MIN]) && (diff < err[MAX]))
            p_enc->flags.encError   = false;
        else
            p_enc->flags.encError   = true;
        if(!cmd_reg.motorEnable || !cmd_reg.systemEnable)
            *p_state                = MOTOR_STATE_START;
        else
            *p_state                = MOTOR_STATE_INIT_FIX;
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_FIX:
        if(motor_id == MOTOR_1)
        {
            tmp                     = MOTOR1_FIX_INIT_EVT;
            p_foc->idRef            = MOTOR1_CURRENT_INIT_MAX;
        }
        else
        {
            tmp                     = MOTOR2_FIX_INIT_EVT;
            p_foc->idRef            = MOTOR2_CURRENT_INIT_MAX;
        }
        p_motor->itCnt             += 1U;
        p_foc->iqRef                = 0.0f;
        if(!cmd_reg.motorEnable || !cmd_reg.systemEnable)
            *p_state                = MOTOR_STATE_START;
        else if(p_motor->itCnt > tmp)
            *p_state                = MOTOR_STATE_READY;
        else
            *p_state                = MOTOR_STATE_INIT_FIX;
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt             += 1U;
        p_foc->idRef                = 0.0f;
        p_foc->iqRef                = FOC_runPD(&p_foc->pdPosVel);
        if(!cmd_reg.systemEnable)
            *p_state                = MOTOR_STATE_START;
        else if(!cmd_reg.motorEnable)
            *p_state                = MOTOR_STATE_STOP;
        else
            *p_state                = MOTOR_STATE_READY;
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt             += 1U;
        p_foc->idRef                = 0.0f;
        p_foc->iqRef                = 0.0f;
        if(!cmd_reg.systemEnable)
            *p_state                = MOTOR_STATE_START;
        else if(!cmd_reg.motorEnable)
            *p_state                = MOTOR_STATE_STOP;
        else
            *p_state                = MOTOR_STATE_READY;
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ERROR:
        p_motor->itCnt              = p_motor->itCnt;
        p_foc->idRef                = 0.0f;
        p_foc->iqRef                = 0.0f;
        *p_state                    = MOTOR_STATE_ERROR;
        MOT_stopCommand(p_motor);
        break;
    }
#endif


    return(true);
}
