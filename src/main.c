#include "main.h"

// Hardware Abstraction Layer struct instance
HAL hal =
{
 .gpioHandler = GpioPinsCfgList,
 .adcHandler  = AdcCfgList,
 .epwmHandler = PwmCfgList,
 .eqepHandler = QepCfgList,
 .dmaHandler  = NULL,
 .spiHandler  = spiCfgList
};
// DRV8323 Hardware Abstraction Layers
DRV8353 drv =
{
 .drvCfgHandler = DrvCfgList,
 .drvRegHandler = DrvRegList
};
// Motor Hardware Abstraction Layers
//MOTOR_ACQ motors_acq[2];
//=
//{
// {.dataValid = {false, false}, .offsetCal = true},
// {.dataValid = {false, false}, .offsetCal = true},
//};
MOTOR_ACQ motors_acq[3] =
{
 {
  .motorID      = MOTOR_1,
  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[0], &PwmCounterCompareCfgList[1], &PwmCounterCompareCfgList[2]},
  .p_iConv      = {&AdcPPBsList[0], &AdcPPBsList[4], &AdcPPBsList[10]},
  .p_vConv      = {&AdcPPBsList[2], &AdcPPBsList[6], &AdcPPBsList[12]},
  .p_vbusConv   = &AdcPPBsList[8],
  .dataValid    = {false, false},
//  .offsetCal      = true
 },
 {
  .motorID      = MOTOR_2,
  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[3], &PwmCounterCompareCfgList[4], &PwmCounterCompareCfgList[5]},
  .p_iConv      = {&AdcPPBsList[1], &AdcPPBsList[5], &AdcPPBsList[11]},
  .p_vConv      = {&AdcPPBsList[3], &AdcPPBsList[7], &AdcPPBsList[13]},
  .p_vbusConv   = &AdcPPBsList[9],
  .dataValid    = {false, false},
//  .offsetCal      = true
 },
 {
  .motorID      = NO_MOT,
 }
};

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
    HAL_ADC_offsetCalib(motors_acq);
    // PWM test section, replace 0 by an integer between 0x0000 and 0xFFFF
    EPWM_setCounterCompareValue(MOTOR1_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareValue(MOTOR1_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareValue(MOTOR1_PWM3_BASE, EPWM_COUNTER_COMPARE_A, PWM_TBPRD);
    EPWM_setCounterCompareValue(MOTOR2_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareValue(MOTOR2_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareValue(MOTOR2_PWM3_BASE, EPWM_COUNTER_COMPARE_A, PWM_TBPRD);

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
        GPIO_writePin(DEBUG_INT_PIN2, 1);

//        // Get the latest position of the Quadrature Encoder
//        motors_acq[MOTOR_1].encoderPos = HWREG(EQEP1_BASE + EQEP_O_QPOSCNT);
//        // Get the latest ADC results to the buffer
//        motors_acq[MOTOR_1].raw.ia      = HWREGH(ADCARESULT_BASE + ADC_SOC_NUMBER0);
//        motors_acq[MOTOR_1].raw.ib      = HWREGH(ADCBRESULT_BASE + ADC_SOC_NUMBER0);
//        motors_acq[MOTOR_1].raw.ic      = HWREGH(ADCDRESULT_BASE + ADC_SOC_NUMBER0);
//        motors_acq[MOTOR_1].raw.vbus    = HWREGH(ADCCRESULT_BASE + ADC_SOC_NUMBER0);
//
//        if(motors_acq[MOTOR_1].offsetCal == false)
//        {
//            motors_acq[MOTOR_1].raw.ia      -= motors_acq[MOTOR_1].offset.ia;
//            motors_acq[MOTOR_1].raw.ib      -= motors_acq[MOTOR_1].offset.ib;
//            motors_acq[MOTOR_1].raw.ic      -= motors_acq[MOTOR_1].offset.ic;
//            motors_acq[MOTOR_1].raw.vbus    -= motors_acq[MOTOR_1].offset.vbus;
//        }
        motors_acq[MOTOR_1].dataValid[0] = true;
        // Clear the interrupt overflow flag
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
//        GPIO_writePin(DEBUG_INT_PIN2, 1);
        // Get the latest position of the Quadrature Encoder
//        motors_acq[MOTOR_2].encoderPos = HWREG(EQEP2_BASE + EQEP_O_QPOSCNT);
//        // Get the latest ADC results to the buffer
//        motors_acq[MOTOR_2].raw.ia      = HWREGH(ADCARESULT_BASE + ADC_SOC_NUMBER1);
//        motors_acq[MOTOR_2].raw.ib      = HWREGH(ADCBRESULT_BASE + ADC_SOC_NUMBER1);
//        motors_acq[MOTOR_2].raw.ic      = HWREGH(ADCDRESULT_BASE + ADC_SOC_NUMBER1);
//        motors_acq[MOTOR_2].raw.vbus    = HWREGH(ADCCRESULT_BASE + ADC_SOC_NUMBER1);
//
//        if(motors_acq[MOTOR_2].offsetCal == false)
//        {
//            motors_acq[MOTOR_2].raw.ia      -= motors_acq[MOTOR_2].offset.ia;
//            motors_acq[MOTOR_2].raw.ib      -= motors_acq[MOTOR_2].offset.ib;
//            motors_acq[MOTOR_2].raw.ic      -= motors_acq[MOTOR_2].offset.ic;
//            motors_acq[MOTOR_2].raw.vbus    -= motors_acq[MOTOR_2].offset.vbus;
//        }
        motors_acq[MOTOR_2].dataValid[0] = true;
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
//        GPIO_writePin(DEBUG_INT_PIN1, 1);
        // Get the latest results to the buffer
//        motors_acq[MOTOR_1].raw.va  = HWREGH(ADCARESULT_BASE + ADC_SOC_NUMBER2);
//        motors_acq[MOTOR_1].raw.vb  = HWREGH(ADCBRESULT_BASE + ADC_SOC_NUMBER2);
//        motors_acq[MOTOR_1].raw.vc  = HWREGH(ADCDRESULT_BASE + ADC_SOC_NUMBER2);
//
//        if(motors_acq[MOTOR_1].offsetCal == false)
//        {
//            motors_acq[MOTOR_1].raw.va  -= motors_acq[MOTOR_1].offset.va;
//            motors_acq[MOTOR_1].raw.vb  -= motors_acq[MOTOR_1].offset.vb;
//            motors_acq[MOTOR_1].raw.vc  -= motors_acq[MOTOR_1].offset.vc;
//        }
        motors_acq[MOTOR_1].dataValid[1] = true;
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
//        GPIO_writePin(DEBUG_INT_PIN2, 1);        // Get the latest results to the buffer
//        motors_acq[MOTOR_2].raw.va  = HWREGH(ADCARESULT_BASE + ADC_SOC_NUMBER3);
//        motors_acq[MOTOR_2].raw.vb  = HWREGH(ADCBRESULT_BASE + ADC_SOC_NUMBER3);
//        motors_acq[MOTOR_2].raw.vc  = HWREGH(ADCDRESULT_BASE + ADC_SOC_NUMBER3);
//
//        if(motors_acq[MOTOR_2].offsetCal == false)
//        {
//            motors_acq[MOTOR_2].raw.va  -= motors_acq[MOTOR_2].offset.va;
//            motors_acq[MOTOR_2].raw.vb  -= motors_acq[MOTOR_2].offset.vb;
//            motors_acq[MOTOR_2].raw.vc  -= motors_acq[MOTOR_2].offset.vc;
//        }
        motors_acq[MOTOR_2].dataValid[1] = true;
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
//    if(EPWM_getEventTriggerInterruptStatus(MOTOR1_PWM1_BASE))
//    EPWM_clearEventTriggerInterruptFlag(MOTOR1_PWM1_BASE);
    GPIO_writePin(DEBUG_INT_PIN1, 1);
    // Clear the interrupt flag
    HWREGH(EPWM1_BASE + EPWM_O_ETCLR) = 1U; // ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
    HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP3; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

    GPIO_writePin(DEBUG_INT_PIN1, 0);
}
