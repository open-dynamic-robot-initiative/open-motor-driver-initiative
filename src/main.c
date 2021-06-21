#include "main.h"


// FOC structure declarations
//extern HAL hal;
//extern FOC foc[];

/**
 * main.c
 */
int main(void)
{
    // Control card devices initializations
    Mcu_init();
    // Hardware Abstraction Layer initialization
    HAL_init(&hal);
    // DRV8323 initialization
    DRV_init(motor[MOTOR_1].motorDRV);
    DRV_init(motor[MOTOR_2].motorDRV);
    // ADC offset calib for debug
    HAL_ADC_offsetCalib(motor[MOTOR_1].motorHalCfg);
    HAL_ADC_offsetCalib(motor[MOTOR_2].motorHalCfg);
    // Interrupts initialization
    HAL_IT_init(InterruptList);

    // Reset controllers
//    CL_resetFoc(&foc[MOTOR_1]);
//    CL_resetFoc(&foc[MOTOR_2]);
    // Initialize FOC structures
//    CL_initFoc(&motor_foc[MOTOR_1], MOTOR_1);
//    CL_initFoc(&motor_foc[MOTOR_2], MOTOR_2);

    // PWM test section, replace 0 by an integer between 0x0000 and 0xFFFF
//    EPWM_setCounterCompareValue(MOTOR1_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR1_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR1_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR2_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR2_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR2_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);

    EPWM_setCounterCompareValue(MOTOR1_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 3 * PWM_TBPRD / 4);
    EPWM_setCounterCompareValue(MOTOR1_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 2 * PWM_TBPRD / 4);
    EPWM_setCounterCompareValue(MOTOR1_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 1 * PWM_TBPRD / 4);
    EPWM_setCounterCompareValue(MOTOR2_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 3 * PWM_TBPRD / 4);
    EPWM_setCounterCompareValue(MOTOR2_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 2 * PWM_TBPRD / 4);
    EPWM_setCounterCompareValue(MOTOR2_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 1 * PWM_TBPRD / 4);

    // Infinite loop
    while(1)
    {
        if(motor[MOTOR_1].motorEncoder->posNew == true)
        {
            // Compute sine cosine
            FOC_sineCosine(motor[MOTOR_1].motorFOC);
            motor[MOTOR_1].motorEncoder->posNew = false;
        }

        if(motor[MOTOR_2].motorEncoder->posNew == true)
        {
            // Compute sine cosine
            FOC_sineCosine(motor[MOTOR_2].motorFOC);
            motor[MOTOR_2].motorEncoder->posNew = false;
        }

        if(motor[MOTOR_1].motorMeas->MeasNew == true)
        {
//            // Current transforms
//            FOC_parkClarke(motor[MOTOR_1].motorFOC);
//            // Run PI controllers
//            FOC_runPi(motor[MOTOR_1].motorFOC);
//            // Voltage transforms
//            FOC_invParkClarke(motor[MOTOR_1].motorFOC);
//            // Update PWM compare registers
//            FOC_svm(motor[MOTOR_1].motorFOC);
            motor[MOTOR_1].motorMeas->MeasNew = false;
        }

        if(motor[MOTOR_2].motorMeas->MeasNew == true)
        {
//            // Current transforms
//            FOC_parkClarke(motor[MOTOR_2].motorFOC);
//            // Run PI controllers
//            FOC_runPi(motor[MOTOR_2].motorFOC);
//            // Voltage transforms
//            FOC_invParkClarke(motor[MOTOR_2].motorFOC);
//            // Update PWM compare registers
//            FOC_svm(motor[MOTOR_2].motorFOC);
            motor[MOTOR_2].motorMeas->MeasNew = false;
        }
    }
}

/**
 * @brief   Microcontroller initialization
 *          - Configure master clock to 200MHz
 *          - Enable GPIO clocking
 *          - Configure the interruption vector table
 */
static void Mcu_init(void)
{
    // Initialize clock and peripherals
    Device_init();
    // Initialize GPIO
    Device_initGPIO();
    // Initialize PIE control registers and flags
    Interrupt_initModule();
    // Initialize vector table
    Interrupt_initVectorTable();
}

/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_1
 */
__interrupt void adc_isrMotor1(void)
{
    GPIO_writePin(DEBUG_INT_PIN1, 1);
    MOTOR_getCurrent(&motor[MOTOR_1], &InterruptList[ADC_IT_MOTOR_1]);
    MOTOR_getVoltage(&motor[MOTOR_1]);
    GPIO_writePin(DEBUG_INT_PIN1, 0);
}

/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_2
 */
__interrupt void adc_isrMotor2(void)
{
    GPIO_writePin(DEBUG_INT_PIN2, 1);
    MOTOR_getCurrent(&motor[MOTOR_2], &InterruptList[ADC_IT_MOTOR_2]);
    MOTOR_getVoltage(&motor[MOTOR_2]);
    GPIO_writePin(DEBUG_INT_PIN2, 0);
}


////    if(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER4))
////    {
////        uint8_t loop = 0;
//
//        GPIO_writePin(DEBUG_INT_PIN2, 1);
////        motor_hal_cfg[MOTOR_2].dataValid[1] = true;
//
//        //        FOC_getPhaseVoltage(&foc[MOTOR_2], &motor_hal_cfg[MOTOR_2]);
////        for(loop = 0; loop < 3; loop++)
////            motor_hal_cfg[MOTOR_2].i[loop] = (float32_t)HWSREGH(motor_hal_cfg[MOTOR_2].p_iConv[loop]->adcPPBResultBase) * ADC_CURRENT_SCALE;
////        motor[MOTOR_2].ia = (float32_t)HWSREGH(motor[MOTOR_2].motorHalCfg->p_iConv[0]->adcPPBResultBase) * ADC_CURRENT_SCALE;
////        motor[MOTOR_2].ib = (float32_t)HWSREGH(motor[MOTOR_2].motorHalCfg->p_iConv[1]->adcPPBResultBase) * ADC_CURRENT_SCALE;
////        motor[MOTOR_2].ic = (float32_t)HWSREGH(motor[MOTOR_2].motorHalCfg->p_iConv[2]->adcPPBResultBase) * ADC_CURRENT_SCALE;
//
//        motor[MOTOR_2].motorMeas->ia    = (float32_t)(*motor[MOTOR_2].motorMeas->iaMeasReg) * ADC_CURRENT_SCALE;
//        motor[MOTOR_2].motorMeas->ib    = (float32_t)(*motor[MOTOR_2].motorMeas->ibMeasReg) * ADC_CURRENT_SCALE;
//        motor[MOTOR_2].motorMeas->ic    = (float32_t)(*motor[MOTOR_2].motorMeas->icMeasReg) * ADC_CURRENT_SCALE;
//        // Clear the interrupt overflow flag
//        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER4);
//        //        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER4);
//        // Clear the interrupt flag
//        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER4);
//        //        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER4);
//        //HWREG(PIECTRL_BASE + PIE_O_IFR10) = (1U << ADC_INT_NUMBER4);
//        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
//        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10;
//        //        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
//
//        GPIO_writePin(DEBUG_INT_PIN2, 0);
////}
//}

__interrupt void pwm_isrMotor1(void)
{
    GPIO_writePin(DEBUG_INT_PIN1, 1);
    if (motor[MOTOR_1].motorFOC->enabled)
    {
        /* FOC update */
        // Read encoder position
        encoder_GetPosition(motor[MOTOR_1].motorEncoder);
        // Compute sine cosine
        FOC_sineCosine(motor[MOTOR_1].motorFOC);


        //encoder_GetSpeed(motor[MOTOR_1].motorEncoder);

        //bwhile((motor[MOTOR_1].motorFOC->adcIntFlag->all & (1<<MOTOR_1)) == 0);
    }
    else
    {
        // Don't send current to motors
        //        FOC_noVoltage(0);
        //
        //        // Reset controllers
        //        CL_resetFoc(&foc[0]);
    }

    //-------------------------------------------------------------------//
    //                          FOC update                               //
    //-------------------------------------------------------------------//
    // Read encoder position

    //    foc->encoder.qepHandle = (struct EQEP_REGS *)MOTOR1_QEP_BASE;
    //    foc->encoder.mechScaler = 0.25 / QEP_MOTOR1_ENCODER_LINES_NB;
    //    foc->encoder.polePairs = M1_POLES_PAIRS;
    //
    //
    //    ENC_read(&foc[motorIndex].encoder);


    //    if(EPWM_getEventTriggerInterruptStatus(MOTOR1_PWM1_BASE))
    //    EPWM_clearEventTriggerInterruptFlag(MOTOR1_PWM1_BASE);

    // Clear the interrupt flag
    EPWM_clearEventTriggerInterruptFlag(InterruptList[PWM_IT_MOTOR_1].intPeriphBase);
//    HWREGH(EPWM1_BASE + EPWM_O_ETCLR) |= EPWM_ETCLR_INT; // EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    //HWREG(PIECTRL_BASE + PIE_O_IFR3) = (1U << ADC_INT_NUMBER4);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
    Interrupt_clearACKGroup(InterruptList[PWM_IT_MOTOR_1].intAckGroup);
//    HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP3; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
    GPIO_writePin(DEBUG_INT_PIN1, 0);
}

__interrupt void pwm_isrMotor2(void)
{
    GPIO_writePin(DEBUG_INT_PIN2, 1);
    if (motor[MOTOR_2].motorFOC->enabled)
    {
        /* FOC update */
        // Read encoder position
        encoder_GetPosition(motor[MOTOR_2].motorEncoder);
        // Compute sine cosine
        FOC_sineCosine(motor[MOTOR_2].motorFOC);


        //encoder_GetSpeed(motor[MOTOR_1].motorEncoder);

        //bwhile((motor[MOTOR_1].motorFOC->adcIntFlag->all & (1<<MOTOR_1)) == 0);
    }
    else
    {
        // Don't send current to motors
        //        FOC_noVoltage(0);
        //
        //        // Reset controllers
        //        CL_resetFoc(&foc[0]);
    }

    //-------------------------------------------------------------------//
    //                          FOC update                               //
    //-------------------------------------------------------------------//
    // Read encoder position

    //    foc->encoder.qepHandle = (struct EQEP_REGS *)MOTOR1_QEP_BASE;
    //    foc->encoder.mechScaler = 0.25 / QEP_MOTOR1_ENCODER_LINES_NB;
    //    foc->encoder.polePairs = M1_POLES_PAIRS;
    //
    //
    //    ENC_read(&foc[motorIndex].encoder);


    //    if(EPWM_getEventTriggerInterruptStatus(MOTOR1_PWM1_BASE))
    //    EPWM_clearEventTriggerInterruptFlag(MOTOR1_PWM1_BASE);

    // Clear the interrupt flag
    EPWM_clearEventTriggerInterruptFlag(InterruptList[PWM_IT_MOTOR_2].intPeriphBase);
//    HWREGH(EPWM4_BASE + EPWM_O_ETCLR) |= EPWM_ETCLR_INT; // EPWM_clearEventTriggerInterruptFlag(EPWM4_BASE);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
    Interrupt_clearACKGroup(InterruptList[PWM_IT_MOTOR_2].intAckGroup);
//    HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP3; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
    GPIO_writePin(DEBUG_INT_PIN2, 0);
}
