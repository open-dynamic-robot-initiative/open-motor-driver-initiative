#include "main.h"

// FOC structure declarations
//extern HAL hal;
//extern FOC foc[];

// Communication interface
//#pragma DATA_SECTION(comInterface, "ramgs0");
extern ComInterface comInterface;
extern MOTOR_STRUCT motor[];
extern const HAL    hal;

/**
 * main.c
 */
int main(void)
{
    // Control card devices initializations
    MCU_init();
    // Hardware Abstraction Layer initialization
    HAL_init(&hal);
    // DRV8323 initialization
    DRV_init(motor[MOTOR_1].motorDRV);
    DRV_init(motor[MOTOR_2].motorDRV);
    // Communication initialization
    SPI_initCommunication(hal.dmaHandler);
    // ADC offset calib for debug
    HAL_ADC_offsetCalib(motor[MOTOR_1].motorHalCfg);
    HAL_ADC_offsetCalib(motor[MOTOR_2].motorHalCfg);
    // Interrupts initialization
    HAL_IT_init(InterruptList);

#if UOMODRI_SPI_DEBUG_CONFIG
    comInterface.systemEnabled = true;
    motor_foc[MOTOR_1].enabled = true;
#endif
//    // Set PWM to an initial output
//    MOTOR_SetPwmCmd(&motor[MOTOR_1], 3 * PWM_TBPRD / 4, 2 * PWM_TBPRD / 4, 1 * PWM_TBPRD / 4);
//    MOTOR_SetPwmCmd(&motor[MOTOR_2], 3 * PWM_TBPRD / 4, 2 * PWM_TBPRD / 4, 1 * PWM_TBPRD / 4);

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
#if UOMODRI_SPI_DEBUG_CONFIG
        // Continuously write data for logging
        UpdateLogPacket();
#else
        // Continuously read and write data to master board
        UpdateSensorPacket();
        ReadCommandPacket();
#endif
//        if(motor[MOTOR_1].motorFOC->motor_enc.posNew == true)
//        {
//            // Compute sine cosine
//            FOC_sineCosine(motor[MOTOR_1].motorFOC);
//            motor[MOTOR_1].motorFOC->motor_enc.posNew = false;
//        }
//
//        if(motor[MOTOR_2].motorFOC->motor_enc.posNew == true)
//        {
//            // Compute sine cosine
//            FOC_sineCosine(motor[MOTOR_2].motorFOC);
//            motor[MOTOR_2].motorFOC->motor_enc.posNew = false;
//        }
//
//        if(motor[MOTOR_1].motorFOC->motor_acq.measNew == true)
//        {
//            FOC_control(motor[MOTOR_1].motorFOC);
//            // Current transforms
//            FOC_parkClarke(motor[MOTOR_1].motorFOC);
//            // Run PI controllers
//            FOC_runPi(motor[MOTOR_1].motorFOC);
//
//
//            for (ch = 0; ch < 3; ch++)
//                i[ch] = (i[ch] < 199) ? (i[ch] + 1) : (0);
//
//            MOTOR_SetPwmCmd(&motor[MOTOR_1], Sinus1kHz[i[0]], Sinus1kHz[i[1]], Sinus1kHz[i[2]]);
//
//            // Current transforms
//            FOC_parkClarke(motor[MOTOR_1].motorFOC);
//            // Run PI controllers
//            FOC_runPi(motor[MOTOR_1].motorFOC);
//            // Voltage transforms
//            FOC_invParkClarke(motor[MOTOR_1].motorFOC);
//            // Update PWM compare registers
//            FOC_svm(motor[MOTOR_1].motorFOC);
//            motor[MOTOR_1].motorFOC->motor_acq.measNew = false;
//        }
//
//        if(motor[MOTOR_2].motorFOC->motor_acq.measNew == true)
//        {
//            FOC_control(motor[MOTOR_2].motorFOC);
//            // Current transforms
//            FOC_parkClarke(motor[MOTOR_2].motorFOC);
//            // Run PI controllers
//            FOC_runPi(motor[MOTOR_2].motorFOC);
//            for (ch = 0; ch < 3; ch++)
//                j[ch] = (j[ch] < 199) ? (j[ch] + 1) : (0);
//
//            MOTOR_SetPwmCmd(&motor[MOTOR_2], Sinus1kHz[j[0]], Sinus1kHz[j[1]], Sinus1kHz[j[2]]);
//
//            // Current transforms
//            FOC_parkClarke(motor[MOTOR_2].motorFOC);
//            // Run PI controllers
//            FOC_runPi(motor[MOTOR_2].motorFOC);
//            // Voltage transforms
//            FOC_invParkClarke(motor[MOTOR_2].motorFOC);
//            // Update PWM compare registers
//            FOC_svm(motor[MOTOR_2].motorFOC);
//            motor[MOTOR_2].motorFOC->motor_acq.measNew = false;
//        }
    }
}

/**
 * @brief   Microcontroller initialization
 *          - Configure master clock to 200MHz
 *          - Enable GPIO clocking
 *          - Configure the interruption vector table
 */
static void MCU_init(void)
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
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    uint32_t        base        = InterruptList[ADC_IT_MOTOR_1].intPeriphBase;
    uint16_t        group       = InterruptList[ADC_IT_MOTOR_1].intAckGroup;
    ADC_IntNumber   adcIntNum   = (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_1].intPeriphSrc;

    if (comInterface.systemEnabled && motor[MOTOR_1].motorFOC->enabled)
    {
        // Read the phases current, voltage & Vbus
        FOC_getMeasures(&motor[MOTOR_1].motorFOC->motor_acq);
        // Compute FOC algorithm
        FOC_control(motor[MOTOR_1].motorFOC);
    }
    else
    {
        // Don't send current to motors
        MOTOR_command(&motor[MOTOR_1], 0.0, 0.0, 0.0);
        // Reset controllers
        FOC_resetStruct(motor[MOTOR_1].motorFOC);
    }
    //-------------------------------------------------------------------//
    //                         Wrap functions                            //
    //-------------------------------------------------------------------//
    // Functions that can be executed between two control loop executions

    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(base, adcIntNum);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(base, adcIntNum);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
    Interrupt_clearACKGroup(group);

    // Run encoder update
    encoder_GetSpeed(&motor[MOTOR_1].motorFOC->motor_enc);
    //ENC_update(motor[MOTOR_1].motorFOC->motor_enc);
    // Check for roll-over error
    if (comInterface.rolloverErrorFlag && motor[MOTOR_1].motorFOC->motor_enc.rollOverFlag)
    {
        // Set CAN error
        comInterface.canError = SPI_SENSOR_STATUS_ERROR_POS_ROLLOVER;
        // Disable the system
        MOTOR_emergencyStop();
    }
    // Run resistance estimation
    MOTOR_StatorResEst(&motor[MOTOR_1]);
    //-------------------------------------------------------------------//
    //                       Alignment routine                           //
    //-------------------------------------------------------------------//
    if(motor[MOTOR_1].alignRoutine.state != ALIGN_DONE)
    {
        // Run alignment routine
        MOTOR_alignRun(&motor[MOTOR_1].alignRoutine);

        // Reset QEP position counter to set angle to zero
        if (motor[MOTOR_1].alignRoutine.state <= ALIGN_STAY)
            encoder_SetPosition(&motor[MOTOR_1].motorFOC->motor_enc, 0);
    }

    //-------------------------------------------------------------------//
    //                         Reference update                          //
    //-------------------------------------------------------------------//

    // Id reference update
    motor[MOTOR_1].motorFOC->idRef      = motor[MOTOR_1].alignRoutine.state == ALIGN_DONE ? 0.0f : motor[MOTOR_1].alignRoutine.idAlign;

    // Iq reference update

#if SPI_DEBUG_CONFIGURATION

    // Set predefined trajectory reference
    comInterface.positionRef[motorIndex] = __sinpuf32(debugFreq * TIME_S);
    comInterface.velocityRef[motorIndex] = 4.0f * FM_PI * FM_PI * debugFreq * __cospuf32(debugFreq * TIME_S);
    comInterface.iqRef[motorIndex] = 0.0f;

#endif

    // Set position reference
    motor[MOTOR_1].motorFOC->pdPos.err  = 2.0f * M_PI * (comInterface.positionRef[MOTOR_1] - motor[MOTOR_1].motorFOC->motor_enc.absoluteAngle); // Convert to radians
    // Set speed reference
    motor[MOTOR_1].motorFOC->pdPos.vErr = comInterface.velocityRef[MOTOR_1] - motor[MOTOR_1].motorFOC->motor_enc.speedEst.mechSpeed;
    // Set feed-forward current
    motor[MOTOR_1].motorFOC->pdPos.ff   = comInterface.iqRef[MOTOR_1];
    // Run PD controller
    FOC_pd(&motor[MOTOR_1].motorFOC->pdPos);

    // Set current reference
    motor[MOTOR_1].motorFOC->iqRef      =  motor[MOTOR_1].alignRoutine.state == ALIGN_DONE ?  motor[MOTOR_1].motorFOC->pdPos.out : 0.0f;

    //-------------------------------------------------------------------//
    //                           Anti-coagging                           //
    //-------------------------------------------------------------------//
#if M1_ACOG_ENABLE || M2_ACOG_ENABLE

    // Check that index have been detected to be sure indexedMechAngle is valid
    if (foc[motorIndex].encoder.indexDetected && foc[motorIndex].acog.enabled)
    {
        // Add anti-coagging current to reference
        foc[motorIndex].iqRef += ACOG_getAcogCurrent(&foc[motorIndex].acog, foc[motorIndex].encoder.indexedMechAngle);
    }

#endif

#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_2
 */
__interrupt void adc_isrMotor2(void)
{
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN2, 1);
#endif
    uint32_t        base        = InterruptList[ADC_IT_MOTOR_2].intPeriphBase;
    uint16_t        group       = InterruptList[ADC_IT_MOTOR_2].intAckGroup;
    ADC_IntNumber   adcIntNum   = (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_2].intPeriphSrc;
    if (comInterface.systemEnabled && motor[MOTOR_2].motorFOC->enabled)
    {
        // Read the phases current, voltage & Vbus
        FOC_getMeasures(&motor[MOTOR_2].motorFOC->motor_acq);
        // Compute FOC algorithm
        FOC_control(motor[MOTOR_2].motorFOC);
    }
    else
    {
        // Don't send current to motors
        MOTOR_command(&motor[MOTOR_2], 0.0, 0.0, 0.0);
        // Reset controllers
        FOC_resetStruct(motor[MOTOR_2].motorFOC);
    }
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(base, adcIntNum);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(base, adcIntNum);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
    Interrupt_clearACKGroup(group);
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN2, 0);
#endif
}

/**
 * @brief   PWM interrupt. Read encoder position of MOTOR_1
 */
__interrupt void pwm_isrMotor1(void)
{
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    ENCODER* p_enc  = &motor[MOTOR_1].motorFOC->motor_enc;
    uint32_t base   = InterruptList[PWM_IT_MOTOR_1].intPeriphBase;
    uint16_t group  = InterruptList[PWM_IT_MOTOR_1].intAckGroup;
    // Read encoder position
    encoder_GetPosition(p_enc);
    // Clear the interrupt flag
    EPWM_clearEventTriggerInterruptFlag(base);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
    Interrupt_clearACKGroup(group);
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

/**
 * @brief   PWM interrupt. Read encoder position of MOTOR_2
 */
__interrupt void pwm_isrMotor2(void)
{
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN2, 1);
#endif
    ENCODER* p_enc  = &motor[MOTOR_2].motorFOC->motor_enc;
    uint32_t base   = InterruptList[PWM_IT_MOTOR_2].intPeriphBase;
    uint16_t group  = InterruptList[PWM_IT_MOTOR_2].intAckGroup;
    // Read encoder position
    encoder_GetPosition(p_enc);
    // Clear the interrupt flag
    EPWM_clearEventTriggerInterruptFlag(base);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
    Interrupt_clearACKGroup(group);
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN2, 0);
#endif
}

__interrupt void gpio_isrFault1(void)
{
    // Trigger emergency stop
    MOTOR_emergencyStop();

    // Read DRV data to get status registers
    DRV_readStatus(motor[MOTOR_1].motorDRV->drvCfgHandler, motor[MOTOR_1].motorDRV->drvRegHandler);
//    HAL_readDRVData(&hal_motors[0].drvHandle, &drvVars[0]);

    // Set CAN error
    comInterface.canError = SPI_SENSOR_STATUS_ERROR_DRV_NFAULT;

    // Update sensor packet
    UpdateSensorPacket();

    // Infinite loop
    while(1);

}

__interrupt void gpio_isrFault2(void)
{
    // Trigger emergency stop
    MOTOR_emergencyStop();

    // Read DRV data to get status registers
    DRV_readStatus(motor[MOTOR_2].motorDRV->drvCfgHandler, motor[MOTOR_2].motorDRV->drvRegHandler);
//    HAL_readDRVData(&hal_motors[0].drvHandle, &drvVars[0]);

    // Set CAN error
    comInterface.canError = SPI_SENSOR_STATUS_ERROR_DRV_NFAULT;

    // Update sensor packet
    UpdateSensorPacket();

    // Infinite loop
    while(1);
}
