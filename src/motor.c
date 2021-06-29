/*
 * motor.c
 *
 *  Created on: 20 mai 2021
 *      Author: jmanhes
 */

#include "motor.h"

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

inline void MOTOR_SetPwmCmd(MOTOR_STRUCT* p_motor, float32_t CmdChA, float32_t CmdChB, float32_t CmdChC)
{
//    uint16_t cmdVal[3]  = {0, 0, 0};
//    uint8_t loop        = 0;
//
//    cmdVal[0] = CmdChA * PWM_CMD_SCALE;
//    cmdVal[1] = CmdChB * PWM_CMD_SCALE;
//    cmdVal[2] = CmdChC * PWM_CMD_SCALE;
//
//    for(i = 0; i < 3; i++)
//    {
//        if(cmdVal[i] > PWM_CMD_SCALE)
//            cmdVal[i] = PWM_CMD_SCALE;
//        else if(cmdVal[i] < 0)
//            cmdVal[i] = 0;
//    }

    *(p_motor->pwmChAReg) = CmdChA;
    *(p_motor->pwmChBReg) = CmdChB;
    *(p_motor->pwmChCReg) = CmdChC;
}



static void ALIGN_reset(AlignRoutine *align)
{
    align->state = ALIGN_COUNT_UP;
    align->counter = 0;
    align->idAlign = 0.0f;
}

// Return true if calibration is finished
static void ALIGN_run(AlignRoutine *align)
{
    switch (align->state)
    {
    case ALIGN_COUNT_UP:
        // Increment alignment current
        align->idAlign += 1e-5;

        if(align->idAlign >= align->idAlignRef)
        {
            align->state = ALIGN_STAY;
        }
        break;

    case ALIGN_STAY:
        // Maintain alignment current
        if(++align->counter >= align->max)
        {
            align->state = ALIGN_COUNT_DOWN;
        }
        break;

    case ALIGN_COUNT_DOWN:
        // Decrement alignment current
        align->idAlign -= 1e-5;

        if(align->idAlign <= 0.0f)
        {
            align->counter = 0;
            align->idAlign = 0.0f;
            align->state = ALIGN_DONE;
        }
        break;
    }
}
