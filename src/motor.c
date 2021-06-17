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
inline void MOTOR_getCurrent(MOTOR_STRUCT* p_motor)
{
    p_motor->motorMeas->ia = (float32_t)(*p_motor->motorMeas->iaMeasReg) * ADC_CURRENT_SCALE;
    p_motor->motorMeas->ib = (float32_t)(*p_motor->motorMeas->ibMeasReg) * ADC_CURRENT_SCALE;
    p_motor->motorMeas->ic = (float32_t)(*p_motor->motorMeas->icMeasReg) * ADC_CURRENT_SCALE;
    p_motor->motorMeas->iMeasNew = true;

    if(p_motor->motorID == MOTOR_1)
    {
        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER3);
        //        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER3);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER3);
        //        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER3);
        //HWREG(PIECTRL_BASE + PIE_O_IFR10) = (1U << ADC_INT_NUMBER3);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10;
        //        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
    }
    else
    {
        // Clear the interrupt overflow flag
        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER4);
        //        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER4);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER4);
        //        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER4);
        //HWREG(PIECTRL_BASE + PIE_O_IFR10) = (1U << ADC_INT_NUMBER4);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10;
        //        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

    }
}

inline void MOTOR_getVoltage(MOTOR_STRUCT* p_motor)
{
    p_motor->motorMeas->va = (float32_t)(*p_motor->motorMeas->vaMeasReg) * ADC_VOLTAGE_SCALE;
    p_motor->motorMeas->vb = (float32_t)(*p_motor->motorMeas->vbMeasReg) * ADC_VOLTAGE_SCALE;
    p_motor->motorMeas->vc = (float32_t)(*p_motor->motorMeas->vcMeasReg) * ADC_VOLTAGE_SCALE;
    p_motor->motorMeas->vMeasNew = true;

//    if( p_motor->motorID == MOTOR_1)
//    {
//        p_motor->motorMeas->vbus    = __fmax((float32_t)(*p_motor->motorMeas->vBusMeasReg) * ADC_VOLTAGE_SCALE, 1.0f);
//
//        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER1);
//        //        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
//        // Clear the interrupt flag
//        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER1);
//        //        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
//        //        HWREG(PIECTRL_BASE + PIE_O_IFR1) = (1U << ADC_INT_NUMBER1);
//        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 1)
//        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP1;
//        //        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
//    }
//    else
//    {
//        // Clear the interrupt overflow flag
//        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER2);
//        //        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2);
//        // Clear the interrupt flag
//        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER2);
//        //        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);
//        //        HWREG(PIECTRL_BASE + PIE_O_IFR10) = (1U << ADC_INT_NUMBER2);
//        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
//        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10;
//        //        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
//    }
}

