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

void MOT_resetAlignStruct(align_t* p_align)
{
    p_align->state          = ALIGN_COUNT_UP;
    p_align->cnt            = 0;
    p_align->cntMax         = PWM_FREQ;
    p_align->idAlign        = 0.0f;

    return;
}

// Return true if calibration is finished
void MOT_runAlign(align_t* p_align)
{
    switch (p_align->state)
    {
    case ALIGN_COUNT_UP:
        // Increment alignment current
        p_align->idAlign   += 1e-5;
        p_align->state      = (p_align->idAlign >= p_align->idAlignRef) ? (ALIGN_STAY) : (ALIGN_COUNT_UP);
        break;

    case ALIGN_STAY:
        // Maintain alignment current
        p_align->state      = (++p_align->cnt >= p_align->cntMax) ? (ALIGN_COUNT_DOWN) : (ALIGN_STAY);
        break;

    case ALIGN_COUNT_DOWN:
        // Decrement alignment current
        p_align->idAlign   -= 1e-5;
//        p_align->state      = (p_align->idAlign <= 0.0f) ? (ALIGN_DONE) : (ALIGN_COUNT_DOWN);
        if(p_align->idAlign <= 0.0f)
        {
            p_align->cnt    = 0;
            p_align->idAlign= 0.0f;
            p_align->state  = ALIGN_DONE;
        }
        else
            p_align->state  = ALIGN_COUNT_DOWN;
        break;

    case ALIGN_DONE:
    default:
        p_align->state      = ALIGN_DONE;
        p_align->cnt        = 0;
        p_align->cntMax     = PWM_FREQ;
        p_align->idAlign    = 0.0f;
    }

    return;
}

// Set the motor phases in security mode
//void MOT_emergencyStop(MOTOR_STRUCT* p_motor)
//{
//    // Turn off control loop
//    p_motor->p_motorFOC->enabled = false;
////    motor[MOTOR_1].p_motorFOC->enabled  = false;
////    motor[MOTOR_2].p_motorFOC->enabled  = false;
//
//    // Disable ePWM interrupts
//    Interrupt_disableInCPU(INTERRUPT_CPU_INT3);
//    // Tie phases to ground
//    MOT_stopCommand(p_motor);
////    MOT_stopCommand(&motor[MOTOR_1]);
////    MOT_stopCommand(&motor[MOTOR_2]);
//    // Reset foc structures
//    FOC_resetStruct(p_motor->p_motorFOC);
////    FOC_resetStruct(motor[MOTOR_1].p_motorFOC);
////    FOC_resetStruct(motor[MOTOR_2].p_motorFOC);
//
//    return;
//}

inline void MOT_resistorEstimation(motor_t* p_motor)
{

    foc_t*      p_foc   = p_motor->p_motorFOC;
    float32_t   resEst  = (p_foc->uq - (p_foc->motor_enc.speed.speedElec * p_foc->motor_cfg.ke)) / p_foc->iq;

    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(p_foc->iq) > FOC_RE_CURRENT_THRESHOLD)
        UOMODRI_RES_ESTIM_FLT(p_motor->statorResEstFilter, p_motor->statorResEst, resEst);

    return;
}
