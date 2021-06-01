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
    DRV_init(drv);
    // ADC offset calib for debug
    HAL_ADC_offsetCalib(motor_hal_cfg);
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
 * @brief   ADC1 interrupt. Read Ia, Ib, Ic \& Vbus of MOTOR_1
 */
__interrupt void adc1ISR(void)
{
    if(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        uint8_t loop = 0;

        motor_hal_cfg[MOTOR_1].dataValid[0] = true;

//        FOC_getPhaseCurrent(&foc[MOTOR_1], &motor_hal_cfg[MOTOR_1]);
//        FOC_getBusVoltage(&foc[MOTOR_1], &motor_hal_cfg[MOTOR_1]);
        for(loop = 0; loop < 3; loop++)
            motor_hal_cfg[MOTOR_1].v[loop] = (float32_t)HWSREGH(motor_hal_cfg[MOTOR_1].p_vConv[loop]->adcPPBResultBase) * ADC_VOLTAGE_SCALE;
        motor_hal_cfg[MOTOR_1].vbus = (float32_t)HWSREGH(motor_hal_cfg[MOTOR_1].p_vbusConv->adcPPBResultBase) * ADC_VOLTAGE_SCALE;

        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER1); // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER1); // ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 1)
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP1; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    }
}

__interrupt void adc2ISR(void)
{
    if(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2))
    {
        uint8_t loop = 0;
        motor_hal_cfg[MOTOR_2].dataValid[0] = true;

//        FOC_getPhaseCurrent(&foc[MOTOR_2], &motor_hal_cfg[MOTOR_2]);
//        FOC_getBusVoltage(&foc[MOTOR_2], &motor_hal_cfg[MOTOR_2]);
        for(loop = 0; loop < 3; loop++)
            motor_hal_cfg[MOTOR_2].v[loop] = (float32_t)HWSREGH(motor_hal_cfg[MOTOR_2].p_vConv[loop]->adcPPBResultBase) * ADC_VOLTAGE_SCALE;
        //motor_hal_cfg[MOTOR_2].vbus = (float32_t)HWSREGH(motor_hal_cfg[MOTOR_2].p_vbusConv->adcPPBResultBase) * ADC_VOLTAGE_SCALE;

        // Clear the interrupt overflow flag
        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER2); // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER2); // ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

//        GPIO_writePin(DEBUG_INT_PIN2, 0);
    }
}

__interrupt void adc3ISR(void)
{
    if(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER3))
    {
        uint8_t loop = 0;

        GPIO_writePin(DEBUG_INT_PIN2, 1);
        motor_hal_cfg[MOTOR_1].dataValid[1] = true;

//        FOC_getPhaseVoltage(&foc[MOTOR_1], &motor_hal_cfg[MOTOR_1]);
        for(loop = 0; loop < 3; loop++)
            motor_hal_cfg[MOTOR_1].i[loop] = (float32_t)HWSREGH(motor_hal_cfg[MOTOR_1].p_iConv[loop]->adcPPBResultBase) * ADC_CURRENT_SCALE;

        // Clear the interrupt overflow flag
        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER3); // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER3);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER3); // ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER3);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

        GPIO_writePin(DEBUG_INT_PIN2, 0);
    }
}

__interrupt void adc4ISR(void)
{
    if(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER4))
    {
        uint8_t loop = 0;

        GPIO_writePin(DEBUG_INT_PIN1, 1);
        motor_hal_cfg[MOTOR_2].dataValid[1] = true;

//        FOC_getPhaseVoltage(&foc[MOTOR_2], &motor_hal_cfg[MOTOR_2]);
        for(loop = 0; loop < 3; loop++)
            motor_hal_cfg[MOTOR_2].i[loop] = (float32_t)HWSREGH(motor_hal_cfg[MOTOR_2].p_iConv[loop]->adcPPBResultBase) * ADC_CURRENT_SCALE;

        motor_hal_cfg[MOTOR_2].dataValid[1] = true;
        // Clear the interrupt overflow flag
        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER4); // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER4);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER4); // ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER4);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

        GPIO_writePin(DEBUG_INT_PIN1, 0);
    }
}

__interrupt void isrMotor1(void)
{
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
    HWREGH(EPWM1_BASE + EPWM_O_ETCLR) = 1U; // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
    HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP3; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);


}
