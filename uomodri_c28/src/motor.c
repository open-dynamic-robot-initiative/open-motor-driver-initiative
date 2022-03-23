/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "motor.h"
//#include "uomodri_prj_config_handlers.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Command for the 3 phases of the ePWM
 * @param[out]  p_motor Pointer on the associated motor structure
 * @param[in]   cmd_a   Normalized command on PWM channel A (between 0 and 1)
 * @param[in]   cmd_b   Normalized command on PWM channel B (between 0 and 1)
 * @param[in]   cmd_c   Normalized command on PWM channel C (between 0 and 1)
 */
void MOT_runCommand(motor_t* p_motor, float32_t cmd_a, float32_t cmd_b, float32_t cmd_c)
{
    *(p_motor->p_motorChAReg) = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT);
    *(p_motor->p_motorChBReg) = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT);
    *(p_motor->p_motorChCReg) = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT);

    return;
}

/**
 * @brief       Force a hard stop on the 3 phases of the ePWM (low side active)
 * @param[out]  p_motor Pointer on the associated motor structure
 */
void MOT_stopCommand(motor_t* p_motor)
{
    *(p_motor->p_motorChAReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorChBReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorChCReg) = (uint16_t)PWM_TIMEBASE_CNT;

    return;
}

inline void MOT_resistorEstimation(motor_t* p_motor)
{

    foc_t*      p_foc   = p_motor->p_motorFOC;
    float32_t   resEst  = (p_foc->uq - (p_foc->motor_enc.speed.speedElec * p_foc->motor_cfg.ke)) / p_foc->iq;

    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(p_foc->iq) > FOC_RE_CURRENT_THRESHOLD)
        UOMODRI_RES_ESTIM_FLT(p_motor->statorResEstFlt, p_motor->statorResEst, resEst);

    return;
}
