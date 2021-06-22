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

    // Set PWM to an initial output
    MOTOR_SetPwmCmd(&motor[MOTOR_1], 3 * PWM_TBPRD / 4, 2 * PWM_TBPRD / 4, 1 * PWM_TBPRD / 4);
    MOTOR_SetPwmCmd(&motor[MOTOR_2], 3 * PWM_TBPRD / 4, 2 * PWM_TBPRD / 4, 1 * PWM_TBPRD / 4);

//    MOTOR_SetPwmCmd(&motor[MOTOR_1], Sinus1kHz[i[0]], Sinus1kHz[i[1]], Sinus1kHz[i[2]]);
//    MOTOR_SetPwmCmd(&motor[MOTOR_2], Sinus1kHz[j[0]], Sinus1kHz[j[1]], Sinus1kHz[j[2]]);

//    *(motor[MOTOR_1].pwmChAReg) = 3 * PWM_TBPRD / 4;
//    *(motor[MOTOR_1].pwmChBReg) = 2 * PWM_TBPRD / 4;
//    *(motor[MOTOR_1].pwmChCReg) = 1 * PWM_TBPRD / 4;
//    *(motor[MOTOR_2].pwmChAReg) = 3 * PWM_TBPRD / 4;
//    *(motor[MOTOR_2].pwmChBReg) = 2 * PWM_TBPRD / 4;
//    *(motor[MOTOR_2].pwmChCReg) = 1 * PWM_TBPRD / 4;

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
//            for (ch = 0; ch < 3; ch++)
//                i[ch] = (i[ch] < 199) ? (i[ch] + 1) : (0);
//
//            MOTOR_SetPwmCmd(&motor[MOTOR_1], Sinus1kHz[i[0]], Sinus1kHz[i[1]], Sinus1kHz[i[2]]);

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
//            for (ch = 0; ch < 3; ch++)
//                j[ch] = (j[ch] < 199) ? (j[ch] + 1) : (0);
//
//            MOTOR_SetPwmCmd(&motor[MOTOR_2], Sinus1kHz[j[0]], Sinus1kHz[j[1]], Sinus1kHz[j[2]]);

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
#ifdef DEBUG
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    MOTOR_MEASURES*  measures   = motor[MOTOR_1].motorMeas;
    uint32_t         base       = InterruptList[ADC_IT_MOTOR_1].intPeriphBase;
    uint16_t         group      = InterruptList[ADC_IT_MOTOR_1].intAckGroup;
    ADC_IntNumber    adcIntNum  = (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_1].intPeriphSrc;
    // Read the phases volatge + Vbus
    MOTOR_getVoltage(measures);
    // Read the phases currents
    MOTOR_getCurrent(measures);
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(base, adcIntNum);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(base, adcIntNum);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
    Interrupt_clearACKGroup(group);
#ifdef DEBUG
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_2
 */
__interrupt void adc_isrMotor2(void)
{
#ifdef DEBUG
    GPIO_writePin(DEBUG_INT_PIN2, 1);
#endif
    MOTOR_MEASURES*  measures   = motor[MOTOR_1].motorMeas;
    uint32_t         base       = InterruptList[ADC_IT_MOTOR_2].intPeriphBase;
    uint16_t         group      = InterruptList[ADC_IT_MOTOR_2].intAckGroup;
    ADC_IntNumber    adcIntNum  = (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_2].intPeriphSrc;
    // Read the phases volatge + Vbus
    MOTOR_getVoltage(measures);
    // Read the phases currents
    MOTOR_getCurrent(measures);
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(base, adcIntNum);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(base, adcIntNum);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
    Interrupt_clearACKGroup(group);
#ifdef DEBUG
    GPIO_writePin(DEBUG_INT_PIN2, 0);
#endif
}

__interrupt void pwm_isrMotor1(void)
{
#ifdef DEBUG
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    ENCODER* encoder    = motor[MOTOR_1].motorEncoder;
    uint32_t base       = InterruptList[PWM_IT_MOTOR_1].intPeriphBase;
    uint16_t group      = InterruptList[PWM_IT_MOTOR_1].intAckGroup;
    // Read encoder position
    encoder_GetPosition(encoder);
    // Clear the interrupt flag
    EPWM_clearEventTriggerInterruptFlag(base);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
    Interrupt_clearACKGroup(group);
#ifdef DEBUG
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

__interrupt void pwm_isrMotor2(void)
{
#ifdef DEBUG
    GPIO_writePin(DEBUG_INT_PIN2, 1);
#endif
    ENCODER* encoder    = motor[MOTOR_2].motorEncoder;
    uint32_t base       = InterruptList[PWM_IT_MOTOR_2].intPeriphBase;
    uint16_t group      = InterruptList[PWM_IT_MOTOR_2].intAckGroup;
    // Read encoder position
    encoder_GetPosition(encoder);
    // Clear the interrupt flag
    EPWM_clearEventTriggerInterruptFlag(base);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
    Interrupt_clearACKGroup(group);
#ifdef DEBUG
    GPIO_writePin(DEBUG_INT_PIN2, 0);
#endif
}
