/*
 * motor.c
 *
 *  Created on: 20 mai 2021
 *      Author: jmanhes
 */

#include "motor.h"
#include "communication_interface.h"
#include "uomodri_prj_config_handlers.h"

//extern ComInterface comInterface;
//extern MOTOR_STRUCT motor[2];

//static void LPF_setBandwidth_simple(FILTER_PARAMS* p_flt, float32_t freq, float32_t bw)
//{
//    p_flt->a = 1.0 / (1.0 + 2.0 * M_PI * bw / freq);
//    p_flt->b = 1.0 - p_flt->a;
//}
//
//#define LPF_FILTER(filt, x, y)     (x) = filt.b * (y) + filt.a * (x);



//// Read current measures
//inline void MOTOR_getCurrent(MOTOR_MEASURES* p_measure)
//{
//    p_measure->ia       = (float32_t)(*p_measure->iaMeasReg) * ADC_CURRENT_SCALE;
//    p_measure->ib       = (float32_t)(*p_measure->ibMeasReg) * ADC_CURRENT_SCALE;
//    p_measure->ic       = (float32_t)(*p_measure->icMeasReg) * ADC_CURRENT_SCALE;
//    p_measure->MeasNew  = true;
//}
//
//inline void MOTOR_getVoltage(MOTOR_MEASURES* p_measure)
//{
//    p_measure->va   = (float32_t)(*p_measure->vaMeasReg)   * ADC_VOLTAGE_SCALE;
//    p_measure->vb   = (float32_t)(*p_measure->vbMeasReg)   * ADC_VOLTAGE_SCALE;
//    p_measure->vc   = (float32_t)(*p_measure->vcMeasReg)   * ADC_VOLTAGE_SCALE;
//    p_measure->vbus = (float32_t)(*p_measure->vBusMeasReg) * ADC_VOLTAGE_SCALE;
//    // Get bus voltage (minimum is one to avoid dividing by zero later)
//    p_measure->vbus = __fmax(p_measure->vbus, 1.0f);
//}

void MOT_runCommand(MOTOR_STRUCT* p_motor, float32_t CmdChA, float32_t CmdChB, float32_t CmdChC)
{
    *(p_motor->pwmChAReg) = (uint16_t)((1.0 - CmdChA) * PWM_TIMEBASE_CNT);
    *(p_motor->pwmChBReg) = (uint16_t)((1.0 - CmdChB) * PWM_TIMEBASE_CNT);
    *(p_motor->pwmChCReg) = (uint16_t)((1.0 - CmdChC) * PWM_TIMEBASE_CNT);

    return;
}

void MOT_stopCommand(MOTOR_STRUCT* p_motor)
{
    *(p_motor->pwmChAReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->pwmChBReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->pwmChCReg) = (uint16_t)PWM_TIMEBASE_CNT;
}

void MOTOR_interruptClearFlags(void)
{
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(InterruptList[ADC_IT_MOTOR_1].intPeriphBase, (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_1].intPeriphSrc);
    ADC_clearInterruptStatus(InterruptList[ADC_IT_MOTOR_1].intPeriphBase, (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_1].intPeriphSrc);
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(InterruptList[ADC_IT_MOTOR_2].intPeriphBase, (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_2].intPeriphSrc);
    ADC_clearInterruptStatus(InterruptList[ADC_IT_MOTOR_2].intPeriphBase, (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_2].intPeriphSrc);
    // Clear the event trigger interrupt flag
//    EPWM_clearEventTriggerInterruptFlag(InterruptList[PWM_IT_MOTOR_1].intPeriphBase);
//    EPWM_clearEventTriggerInterruptFlag(InterruptList[PWM_IT_MOTOR_2].intPeriphBase);
    // Clear the interrupt flag associated to XINT1 & XINT2
    XBAR_clearInputFlag(XBAR_INPUT_FLG_INPUT4);
    XBAR_clearInputFlag(XBAR_INPUT_FLG_INPUT5);
}

void MOT_resetAlignStruct(MOTOR_ALIGN* p_align)
{
    p_align->state      = ALIGN_COUNT_UP;
    p_align->counter    = 0;
    p_align->idAlign    = 0.0f;
}

// Return true if calibration is finished
void MOT_runAlign(MOTOR_ALIGN* p_align)
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
        p_align->state      = (++p_align->counter >= p_align->max) ? (ALIGN_COUNT_DOWN) : (ALIGN_STAY);
        break;

    case ALIGN_COUNT_DOWN:
        // Decrement alignment current
        p_align->idAlign   -= 1e-5;
//        p_align->state      = (p_align->idAlign <= 0.0f) ? (ALIGN_DONE) : (ALIGN_COUNT_DOWN);
        if(p_align->idAlign <= 0.0f)
        {
            p_align->counter = 0;
            p_align->idAlign = 0.0f;
            p_align->state = ALIGN_DONE;
        }
        else
            p_align->state = ALIGN_COUNT_DOWN;
        break;

    case ALIGN_DONE:
    default:
        p_align->state      = ALIGN_DONE;
        p_align->counter    = 0;
        p_align->idAlign    = 0.0f;
    }

    return;
}

// Set the motor phases in security mode
void MOT_emergencyStop(void)
{
    // Turn off control loop
    comInterface.systemEnabled          = false;
    motor[MOTOR_1].motorFOC->enabled    = false;
    motor[MOTOR_2].motorFOC->enabled    = false;

    // Disable ePWM interrupts
    Interrupt_disableInCPU(INTERRUPT_CPU_INT3);
    // Tie phases to ground
    MOT_stopCommand(&motor[MOTOR_1]);
    MOT_stopCommand(&motor[MOTOR_2]);
    // Reset foc structures
    FOC_resetStruct(motor[MOTOR_1].motorFOC);
    FOC_resetStruct(motor[MOTOR_2].motorFOC);

}

inline void MOT_resistorEstimation(MOTOR_STRUCT* p_motor)
{

    FOC*        p_foc   = p_motor->motorFOC;
    float32_t   resEst  = (p_foc->uq - (p_foc->motor_enc.speedEst.speedElec * p_foc->motor_cfg.ke)) / p_foc->iq;

    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(p_foc->iq) > FOC_RE_CURRENT_THRESHOLD)
        UOMODRI_RES_ESTIM_FLT(p_motor->statorResEstLpf, p_motor->statorResEst, resEst);

    return;
}
