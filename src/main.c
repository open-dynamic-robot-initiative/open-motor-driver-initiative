#include "main.h"

// Hardware Abstraction Layer struct instance
//HAL hal =
//{
// .gpioHandler = GpioPinsCfgList,
// .adcHandler  = AdcCfgList,
// .epwmHandler = PwmCfgList,
// .eqepHandler = QepCfgList,
// .dmaHandler  = NULL,
// .spiHandler  = spiCfgList
//};
//// DRV8323 Hardware Abstraction Layers
//DRV8353 drv =
//{
// .drvCfgHandler = DrvCfgList,
// .drvRegHandler = DrvRegList
//};
// Motor Hardware Abstraction Layers
//MOTOR_ACQ motor[2];
//=
//{
// {.dataValid = {false, false}, .offsetCal = true},
// {.dataValid = {false, false}, .offsetCal = true},
//};
//MOTOR_ACQ motor[3] =
//{
// {
//  .motorID      = MOTOR_1,
//  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[0], &PwmCounterCompareCfgList[1], &PwmCounterCompareCfgList[2]},
//  .p_iConv      = {&AdcPPBsList[0], &AdcPPBsList[4], &AdcPPBsList[10]},
//  .p_vConv      = {&AdcPPBsList[2], &AdcPPBsList[6], &AdcPPBsList[12]},
//  .p_vbusConv   = &AdcPPBsList[8],
//  //.p_iMesured   = {(*((volatile uint16_t *)(.p_iConv[0]->adcResultBase + ADC_O_PPB1RESULT))), (&((.p_iConv[0]->adcResultBase + ADC_O_PPB1RESULT))), (&((.p_iConv[0]->adcResultBase + ADC_O_PPB1RESULT)))},
////  .p_iMesured   = {(*((volatile uint16_t *)(AdcPPBsList[0].adcResultBase)))},
////  .p_iMesured   = {&(.p_iConv[0]->adcResultBase), (AdcPPBsList[4].adcResultBase), (AdcPPBsList[10].adcResultBase)},
////  .p_iMesured   = {((volatile int16_t *)(AdcPPBsList[0].adcResultBase)),
////                   ((volatile int16_t *)(AdcPPBsList[4].adcResultBase)),
////                   ((volatile int16_t *)(AdcPPBsList[10].adcResultBase))},
////  .p_vMesured   = {((volatile int16_t *)(AdcPPBsList[2].adcResultBase)),
////                   ((volatile int16_t *)(AdcPPBsList[6].adcResultBase)),
////                   ((volatile int16_t *)(AdcPPBsList[12].adcResultBase))},
////  .p_vBusMesured= (volatile int16_t *)(AdcPPBsList[8].adcResultBase),
//  .dataValid    = {false, false},
////  .offsetCal      = true
// },
// {
//  .motorID      = MOTOR_2,
//  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[3], &PwmCounterCompareCfgList[4], &PwmCounterCompareCfgList[5]},
//  .p_iConv      = {&AdcPPBsList[1], &AdcPPBsList[5], &AdcPPBsList[11]},
//  .p_vConv      = {&AdcPPBsList[3], &AdcPPBsList[7], &AdcPPBsList[13]},
//  .p_vbusConv   = &AdcPPBsList[9],
////  .p_ia         = (((volatile int16_t *)((uintptr_t)(M2_IA_ADC_PPB_RESULT_ADDR)))),
//
////  .p_ia         = (&((volatile int16_t *)((AdcPPBsList[1].adcPPBResultBase)))),
//
////  .p_ia         = (((volatile int16_t *)((uintptr_t)&(AdcPPBsList[1]->adcPPBResultBase)))),
//
////  .p_ia         = (&((volatile uint16_t *)((uintptr_t)(motor[MOTOR_2].p_iConv[0]->adcPPBResultBase)))),
//
////  .p_ia         = ((int32_t *)(AdcPPBsList[1].adcPPBResultBase)),
////  .p_iMesured   = {((volatile int32_t *)(AdcPPBsList[1].adcPPBResultBase)), ((volatile int32_t *)(AdcPPBsList[5].adcPPBResultBase)), ((volatile int32_t *)(AdcPPBsList[11].adcPPBResultBase))},
////  .p_iMesured   = {(*((volatile uint16_t *)(AdcPPBsList[1].adcResultBase)))},
////
////  {((volatile int16_t *)(AdcPPBsList[1].adcResultBase)),
////                   ((volatile int16_t *)(AdcPPBsList[5].adcResultBase)),
////                   ((volatile int16_t *)(AdcPPBsList[11].adcResultBase))},
////  .p_vMesured   = {((volatile int16_t *)(AdcPPBsList[3].adcResultBase)),
////                   ((volatile int16_t *)(AdcPPBsList[7].adcResultBase)),
////                   ((volatile int16_t *)(AdcPPBsList[13].adcResultBase))},
////  .p_vBusMesured= (volatile int16_t *)(AdcPPBsList[9].adcResultBase),
//  .dataValid    = {false, false},
////  .offsetCal      = true
// },
// {
//  .motorID      = NO_MOT,
// }
//};


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
    DRV_init(&drv);
    // Interrupts initialization
    HAL_IT_init();
    // ADC offset calib for debug
    HAL_ADC_offsetCalib(motor);
    // PWM test section, replace 0 by an integer between 0x0000 and 0xFFFF
    EPWM_setCounterCompareValue(MOTOR1_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareValue(MOTOR1_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareValue(MOTOR1_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareValue(MOTOR2_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareValue(MOTOR2_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareValue(MOTOR2_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);

//    EPWM_setCounterCompareValue(MOTOR1_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 3 * PWM_TBPRD / 4);
//    EPWM_setCounterCompareValue(MOTOR1_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 2 * PWM_TBPRD / 4);
//    EPWM_setCounterCompareValue(MOTOR1_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 1 * PWM_TBPRD / 4);
//    EPWM_setCounterCompareValue(MOTOR2_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 3 * PWM_TBPRD / 4);
//    EPWM_setCounterCompareValue(MOTOR2_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 2 * PWM_TBPRD / 4);
//    EPWM_setCounterCompareValue(MOTOR2_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 1 * PWM_TBPRD / 4);

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

        GPIO_writePin(DEBUG_INT_PIN2, 1);
        motor[MOTOR_1].dataValid[0] = true;

//        FOC_getPhaseCurrent(&foc[MOTOR_1], &motor[MOTOR_1]);
//        FOC_getBusVoltage(&foc[MOTOR_1], &motor[MOTOR_1]);
        for(loop = 0; loop < 3; loop++)
            motor[MOTOR_1].i[loop] = (float32_t)HWSREGH(motor[MOTOR_1].p_iConv[loop]->adcPPBResultBase) * ADC_CURRENT_SCALE;
        motor[MOTOR_1].vbus = (float32_t)HWSREGH(motor[MOTOR_1].p_vbusConv->adcPPBResultBase) * ADC_VOLTAGE_SCALE;

        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = 1U; // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = 1U; // ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 1)
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP1; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

        GPIO_writePin(DEBUG_INT_PIN2, 0);
    }
}

__interrupt void adc2ISR(void)
{
    if(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2))
    {
        uint8_t loop = 0;
        motor[MOTOR_2].dataValid[0] = true;

//        FOC_getPhaseCurrent(&foc[MOTOR_2], &motor[MOTOR_2]);
//        FOC_getBusVoltage(&foc[MOTOR_2], &motor[MOTOR_2]);
        for(loop = 0; loop < 3; loop++)
            motor[MOTOR_2].i[loop] = (float32_t)HWSREGH(motor[MOTOR_2].p_iConv[loop]->adcPPBResultBase) * ADC_CURRENT_SCALE;
        motor[MOTOR_2].vbus = (float32_t)HWSREGH(motor[MOTOR_2].p_vbusConv->adcPPBResultBase) * ADC_VOLTAGE_SCALE;

        // Clear the interrupt overflow flag
        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = 2U; // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = 2U; // ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);
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

        motor[MOTOR_1].dataValid[1] = true;

//        FOC_getPhaseVoltage(&foc[MOTOR_1], &motor[MOTOR_1]);
        for(loop = 0; loop < 3; loop++)
            motor[MOTOR_1].v[loop] = (float32_t)HWSREGH(motor[MOTOR_1].p_vConv[loop]->adcPPBResultBase) * ADC_VOLTAGE_SCALE;

        // Clear the interrupt overflow flag
        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = 4U; // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER3);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = 4U; // ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER3);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

//        GPIO_writePin(DEBUG_INT_PIN1, 0);
    }
}

__interrupt void adc4ISR(void)
{
    if(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER4))
    {
        uint8_t loop = 0;

//        FOC_getPhaseVoltage(&foc[MOTOR_2], &motor[MOTOR_2]);
        for(loop = 0; loop < 3; loop++)
            motor[MOTOR_2].v[loop] = (float32_t)HWSREGH(motor[MOTOR_2].p_vConv[loop]->adcPPBResultBase) * ADC_VOLTAGE_SCALE;

        motor[MOTOR_2].dataValid[1] = true;
        // Clear the interrupt overflow flag
        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = 8U; // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER4);
        // Clear the interrupt flag
        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = 8U; // ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER4);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

//        GPIO_writePin(DEBUG_INT_PIN2, 0);
    }
}

__interrupt void pwm1ISR(void)
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
    GPIO_writePin(DEBUG_INT_PIN1, 1);
    // Clear the interrupt flag
    HWREGH(EPWM1_BASE + EPWM_O_ETCLR) = 1U; // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
    HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP3; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

    GPIO_writePin(DEBUG_INT_PIN1, 0);
}
