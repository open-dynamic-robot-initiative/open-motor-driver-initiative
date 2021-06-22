/*
 * motor.c
 *
 *  Created on: 20 mai 2021
 *      Author: jmanhes
 */

#include "motor.h"

static void LPF_setBandwidth_simple(FILTER_PARAMS* p_flt, float32_t freq, float32_t bw)
{
    p_flt->a = 1.0 / (1.0 + 2.0 * M_PI * bw / freq);
    p_flt->b = 1.0 - p_flt->a;
}

#define LPF_FILTER(filt, x, y)     (x) = filt.b * (y) + filt.a * (x);



// Read current measures
inline void MOTOR_getCurrent(MOTOR_STRUCT* p_motor, const IT_TypeDef* p_ITsrc)
{
    p_motor->motorMeas->ia = (float32_t)(*p_motor->motorMeas->iaMeasReg) * ADC_CURRENT_SCALE;
    p_motor->motorMeas->ib = (float32_t)(*p_motor->motorMeas->ibMeasReg) * ADC_CURRENT_SCALE;
    p_motor->motorMeas->ic = (float32_t)(*p_motor->motorMeas->icMeasReg) * ADC_CURRENT_SCALE;
    p_motor->motorMeas->MeasNew = true;

    ADC_IntNumber   adcIntNum   = (ADC_IntNumber)(p_ITsrc->intPeriphSrc);
    uint32_t        adcBase     = p_ITsrc->intPeriphBase;

    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(adcBase, adcIntNum);   // HWREGH(adcBase + ADC_O_INTOVFCLR) = 1U << (uint16_t)adcIntNum;
    // Clear the interrupt flag
    ADC_clearInterruptStatus(adcBase, adcIntNum);           // HWREGH(adcBase + ADC_O_INTFLGCLR) = 1U << (uint16_t)adcIntNum;
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
    Interrupt_clearACKGroup(p_ITsrc->intAckGroup);          // HWREGH(PIECTRL_BASE + PIE_O_ACK) = p_ITsrc->intAckGroup;
}

inline void MOTOR_getVoltage(MOTOR_STRUCT* p_motor)
{
    p_motor->motorMeas->va      = (float32_t)(*p_motor->motorMeas->vaMeasReg)   * ADC_VOLTAGE_SCALE;
    p_motor->motorMeas->vb      = (float32_t)(*p_motor->motorMeas->vbMeasReg)   * ADC_VOLTAGE_SCALE;
    p_motor->motorMeas->vc      = (float32_t)(*p_motor->motorMeas->vcMeasReg)   * ADC_VOLTAGE_SCALE;
    p_motor->motorMeas->vbus    = (float32_t)(*p_motor->motorMeas->vBusMeasReg) * ADC_VOLTAGE_SCALE;
}

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

    *(p_motor->pwmChAReg) = CmdChA * PWM_CMD_SCALE;
    *(p_motor->pwmChBReg) = CmdChB * PWM_CMD_SCALE;
    *(p_motor->pwmChCReg) = CmdChC * PWM_CMD_SCALE;
}
