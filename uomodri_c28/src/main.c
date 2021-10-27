#include "main.h"


// Communication interface
//#pragma DATA_SECTION(comInterface, "ramgs0");
//extern ComInterface comInterface;

/**
 * main.c
 */
int main(void)
{
    // uOmodri low level devices initializations
    MCU_init();
    // Hardware Abstraction Layer initialization
    HAL_init(&hal);
    // DRV8323 initialization
    DRV_init(motor[MOTOR_1].p_motorDRV);
    DRV_init(motor[MOTOR_2].p_motorDRV);
    // ADC offset calib for debug
    HAL_ADC_offsetCalib(motor[MOTOR_1].p_motorHalCfg);
    HAL_ADC_offsetCalib(motor[MOTOR_2].p_motorHalCfg);

#if (UOMODRI_SPI_DEBUG_CONFIG == 1)
    comInterface.systemEnabled          = true;
    motor[MOTOR_1].p_motorFOC->enabled  = true;
    motor[MOTOR_2].p_motorFOC->enabled  = false;
#endif

//#if (CM == 1)
//    // Clear any IPC flags if set already
//    IPC_clearFlagLtoR(IPC_CPU1_L_CM_R, IPC_FLAG_ALL);
////    // Initialize message queue
////    IPC_initMessageQueue(IPC_CPU1_L_CM_R, &msgQueue, IPC_INT1, IPC_INT1);
//    // Synchronize both the cores.
//    IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG31);
//#endif
    // Interrupts initialization
    HAL_IT_init(InterruptList);

//#if (CM == 1)
//    // Enable realtime interrupt (DBGM)
//    ERTM;
//
//    // Fill in the data to be sent
//    for(i = 0; i < 10; i++)
//        readData[i] = i;
//
//    // Update the message
//    msgTx.command = IPC_CMD_READ_MEM;
//    msgTx.address = (uint32_t)readData;
//    msgTx.dataw1  = 10;  // Using dataw1 as data length
//    msgTx.dataw2  = 1;   // Message identifier
//
//    /* Send message to the queue
//     * Since C28x and CM does not share the same address space for shared RAM,
//     * ADDRESS_CORRECTION is enabled
//     */
//    IPC_sendMessageToQueue(IPC_CPU1_L_CM_R, &msgQueue, IPC_ADDR_CORRECTION_ENABLE, &msgTx, IPC_BLOCKING_CALL);
//    /* Read message from the queue
//     * Return message from CM does not use the address field, hence
//     * ADDRESS_CORRECTION feature is not used
//     */
//    IPC_readMessageFromQueue(IPC_CPU1_L_CM_R, &msgQueue, IPC_ADDR_CORRECTION_DISABLE, &msgRx, IPC_BLOCKING_CALL);
//
//    pass = ((msgRx.command == IPC_CMD_RESP) && (msgRx.dataw1 == TEST_PASS) && (msgRx.dataw2 == 1)) ? (0) : (1);
//    /* Send a message without message queue
//     * Since C28x and CM does not share the same address space for shared RAM,
//     * ADDRESS_CORRECTION is enabled
//     * Length of the data to be read is passed as data.
//     */
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_READ_MEM, (uint32_t)readData, 10);
//    // Wait for acknowledgment
//    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG0);
//    // Read response
//    pass = (IPC_getResponse(IPC_CPU1_L_CM_R) == TEST_PASS) ? (1) : (0);
//#endif

//#if (UOMODRI == 0)
//    GPIO_writePin(DEBUG_LED_PIN1, 1);
//#endif

    // Infinite loop
    while(1);
//    {
//#if UOMODRI_SPI_DEBUG_CONFIG
//        // Continuously write data for logging
//        UpdateLogPacket();
//#else
//        // Continuously read and write data to master board
//        UpdateSensorPacket();
//        ReadCommandPacket();
//#endif
//    }
}

/**
 * @brief   Microcontroller initialization
 *          - Configure master clock to 200MHz
 *          - Configure auxiliary clock (CM core) to 125MHz (USB à 60MHz)
 *          - Enable GPIO clocking
 *          - Configure the interruption vector table
 *          - Boot CM from flash or RAM
 */
static void MCU_init(void)
{
    // Initialize clock and peripherals
    Device_init();
    /* Set up the auxiliary PLL to 125MHz for CM use. */
    SysCtl_setAuxClock(SYSCTL_AUXPLL_OSCSRC_XTAL |
                       SYSCTL_AUXPLL_IMULT(50) |
                       SYSCTL_REFDIV(2U) |
                       SYSCTL_ODIV(5U) |
                       SYSCTL_AUXPLL_DIV_2 |
                       SYSCTL_AUXPLL_ENABLE |
                       SYSCTL_DCC_BASE_0);
    // Allocate Shared Peripheral USB to the CM Side.
//    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
    /* Set the CM Clock to run at 125MHz.
     * The CM Clock is a fractional multiple of the AUXPLL Clock (125 MHz). */
    SysCtl_setCMClk(SYSCTL_CMCLKOUT_DIV_1, SYSCTL_SOURCE_AUXPLL);
    // Initialize GPIO
    Device_initGPIO();
    // Initialize PIE control registers and flags. Disables CPU interrupts.
    Interrupt_initModule();
    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    Interrupt_initVectorTable();
    // Boot CM core
#if (CM == 1)
#ifdef _FLASH
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif
#endif
}

//-------------------------------------------------------------------------------------------------
__attribute__((interrupt)) void IPC_ISR1(void)
{
//    ComInterface* p_comInterface = NULL;
    uint32_t command, addr, data;
    IPC_readCommand(IPC_CPU1_L_CM_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE, &command, &addr, &data);
//    DmaRegs.CH5.SRC_BEG_ADDR_ACTIVE = addr;
//    DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);//dmaCfgList[0].dmaChBase);
    DMA_forceTrigger(DMA_COM_SPI_TX_BASE_ADDR);
    if(DmaRegs.CH5.CONTROL.bit.RUNSTS)
        GPIO_togglePin(DEBUG_INT_PIN1);

//    p_comInterface = (ComInterface *)addr;
//    comInterface.positionRef[MOTOR_1]   = p_comInterface->positionRef[MOTOR_1];
//    comInterface.velocityRef[MOTOR_1]   = p_comInterface->velocityRef[MOTOR_1];
//    comInterface.iqRef[MOTOR_1]         = p_comInterface->iqRef[MOTOR_1];
    IPC_ackFlagRtoL(IPC_CPU1_L_CM_R, IPC_FLAG1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
}

//-------------------------------------------------------------------------------------------------
//__attribute__((interrupt)) void dma5_isr(void)
//{
//    DMA_stopChannel(DMA_COM_SPI_TX_BASE_ADDR);
//    DmaRegs.CH5.CONTROL.bit.PERINTCLR = 1;
//    //IPC_ackFlagRtoL(IPC_CPU1_L_CM_R, IPC_FLAG1);
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
//
//}

//-------------------------------------------------------------------------------------------------
/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_1
 */
__attribute__((interrupt)) void adc_isrMotor1(void)
{
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    FOC_STRUCT*     p_foc       = motor[MOTOR_1].p_motorFOC;
    ALIGN_STRUCT*   p_align     = &motor[MOTOR_1].motorAlign;
    uint32_t        base        = InterruptList[ADC_IT_MOTOR_1].intPeriphBase; //MOTOR12_IVA_ADC_ADDR;
    uint16_t        group       = InterruptList[ADC_IT_MOTOR_1].intAckGroup;
    ADC_IntNumber   adcIntNum   = (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_1].intPeriphSrc; //MOTOR1_IA_INT_NUM;

    if(comInterface.systemEnabled && p_foc->enabled)
    {
        // Read encoder position
        ENC_getPosition(&p_foc->motor_enc);
        // Read the phases current, voltage & Vbus
        FOC_getMeasures(&p_foc->motor_acq);
        // Compute FOC algorithm
        FOC_runControl(p_foc);
        // Set new PWM command
        MOT_runCommand(&motor[MOTOR_1], p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);

        //-------------------------------------------------------------------//
        //                         Wrap functions                            //
        //-------------------------------------------------------------------//
        // Functions that can be executed between two control loop executions

        // Clear the interrupt overflow flag
        ADC_clearInterruptOverflowStatus(base, adcIntNum);
        // Clear the interrupt flag
        ADC_clearInterruptStatus(base, adcIntNum);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
        //        Interrupt_clearACKGroup(group);
        // Run encoder update
        ENC_getSpeed(&p_foc->motor_enc);
        //ENC_update(motor[MOTOR_1].motorFOC->motor_enc);
        // Check for roll-over error
        if (comInterface.rolloverErrorFlag && p_foc->motor_enc.rollOverFlag)
        {
            // Set CAN error
            comInterface.canError = SPI_SENSOR_STATUS_ERROR_POS_ROLLOVER;
            // Disable the system
            MOT_emergencyStop();
        }
        // Run resistance estimation
        MOT_resistorEstimation(&motor[MOTOR_1]);
        //-------------------------------------------------------------------//
        //                       Alignment routine                           //
        //-------------------------------------------------------------------//
        if(p_align->state != ALIGN_DONE)
        {
            // Run alignment routine
            MOT_runAlign(p_align);

            // Reset QEP position counter to set angle to zero
            if (p_align->state <= ALIGN_STAY)
                ENC_setPosition(&p_foc->motor_enc, 0);
        }

        //-------------------------------------------------------------------//
        //                         Reference update                          //
        //-------------------------------------------------------------------//
        // Iq reference update

#if UOMODRI_SPI_DEBUG_CONFIG
        // Set predefined trajectory reference
        comInterface.positionRef[MOTOR_1]   = __sinpuf32(debugFreq * TIME_M1_S);//debugFreq * TIME_S;//
        comInterface.velocityRef[MOTOR_1]   = 4.0f * M_PI * M_PI * debugFreq * __cospuf32(debugFreq * TIME_M1_S);//debugFreq * TIME_S;//
        comInterface.iqRef[MOTOR_1]         = 0.0f;
#endif
        // Set feed-forward current
        p_foc->pdPos.ff = comInterface.iqRef[MOTOR_1];
        // Run PD controller
        FOC_runPD(&p_foc->pdPos);
        // Set current reference
        p_foc->iqRef    = (p_align->state == ALIGN_DONE) ? (p_foc->pdPos.out) : (0.0f);
        // Id reference update
        p_foc->idRef    = (p_align->state == ALIGN_DONE) ? (0.0f) : (p_align->idAlign);

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
    }
    else
    {
        // Don't send current to motors
        MOT_stopCommand(&motor[MOTOR_1]);
        // Reset controllers
        FOC_resetStruct(p_foc);
    }

    //-------------------------------------------------------------------//
    //                   Interrupt wrap functions                        //
    //-------------------------------------------------------------------//
    if(p_align->state == ALIGN_DONE)
    {
        // Save number of cycles elapsed since control loop startup
        motor[MOTOR_1].clCycleNb = EPWM_getTimeBaseCounterValue(MOTOR1_PWM1_BASE);
        // Increment interrupt counter
        motor[MOTOR_1].interruptCnt++;
        // Compute time
        comInterface.time = TIME_M1_S;//(motor[MOTOR_1].interruptCounter / PWM_FREQ);
    }

    //-------------------------------------------------------------------//
    //                   Transmit structure functions                    //
    //-------------------------------------------------------------------//
    memcpy(&motorCpuToCM[MOTOR_1], motor[MOTOR_1].p_motorFOC, sizeof(FOC_STRUCT));
    //motorCpuToCM[MOTOR_1] = *motor[MOTOR_1].p_motorFOC;
    /* Send a message without message queue.
     * Since C28x and CM does not share the same address space for shared RAM,
     * ADDRESS_CORRECTION is enabled
     * Length of the data to be read is passed as data. */
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_READ_MEM, (uint32_t)(&motorCpuToCM[MOTOR_1]), 1);

    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(group);

#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_2
 */
__attribute__((interrupt)) void adc_isrMotor2(void)
{
//#if (UOMODRI == 0)
//    GPIO_writePin(DEBUG_INT_PIN2, 1);
//#endif
    FOC_STRUCT*     p_foc       = motor[MOTOR_2].p_motorFOC;
    ALIGN_STRUCT*   p_align     = &motor[MOTOR_2].motorAlign;
    uint32_t        base        = InterruptList[ADC_IT_MOTOR_2].intPeriphBase;
    uint16_t        group       = InterruptList[ADC_IT_MOTOR_2].intAckGroup;
    ADC_IntNumber   adcIntNum   = (ADC_IntNumber)InterruptList[ADC_IT_MOTOR_2].intPeriphSrc;

    if (comInterface.systemEnabled && p_foc->enabled)
    {
        // Read encoder position
        ENC_getPosition(&p_foc->motor_enc);
        // Read the phases current, voltage & Vbus
        FOC_getMeasures(&p_foc->motor_acq);
        // Compute FOC algorithm
        FOC_runControl(p_foc);
        // Set new PWM command
        MOT_runCommand(&motor[MOTOR_2], p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);

        //-------------------------------------------------------------------//
        //                         Wrap functions                            //
        //-------------------------------------------------------------------//
        // Functions that can be executed between two control loop executions

        // Clear the interrupt overflow flag
        ADC_clearInterruptOverflowStatus(base, adcIntNum);
        // Clear the interrupt flag
        ADC_clearInterruptStatus(base, adcIntNum);
        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
        //        Interrupt_clearACKGroup(group);
        // Run encoder update
        ENC_getSpeed(&p_foc->motor_enc);
        //ENC_update(motor[MOTOR_1].motorFOC->motor_enc);
        // Check for roll-over error
        if (comInterface.rolloverErrorFlag && p_foc->motor_enc.rollOverFlag)
        {
            // Set CAN error
            comInterface.canError = SPI_SENSOR_STATUS_ERROR_POS_ROLLOVER;
            // Disable the system
            MOT_emergencyStop();
        }
        // Run resistance estimation
        MOT_resistorEstimation(&motor[MOTOR_2]);
        //-------------------------------------------------------------------//
        //                       Alignment routine                           //
        //-------------------------------------------------------------------//
        if(p_align->state != ALIGN_DONE)
        {
            // Run alignment routine
            MOT_runAlign(p_align);

            // Reset QEP position counter to set angle to zero
            if (p_align->state <= ALIGN_STAY)
                ENC_setPosition(&p_foc->motor_enc, 0);
        }

        //-------------------------------------------------------------------//
        //                         Reference update                          //
        //-------------------------------------------------------------------//
        // Iq reference update

#if UOMODRI_SPI_DEBUG_CONFIG
        // Set predefined trajectory reference
        comInterface.positionRef[MOTOR_2]   = __sinpuf32(debugFreq * TIME_M2_S);//debugFreq * TIME_S;//
        comInterface.velocityRef[MOTOR_2]   = 4.0f * M_PI * M_PI * debugFreq * __cospuf32(debugFreq * TIME_M2_S);//debugFreq * TIME_S;//
        comInterface.iqRef[MOTOR_2]         = 0.0f;
#endif
        // Set feed-forward current
        p_foc->pdPos.ff = comInterface.iqRef[MOTOR_2];
        // Run PD controller
        FOC_runPD(&p_foc->pdPos);
        // Set current reference
        p_foc->iqRef    = (p_align->state == ALIGN_DONE) ? (p_foc->pdPos.out) : (0.0f);
        // Id reference update
        p_foc->idRef    = (p_align->state == ALIGN_DONE) ? (0.0f) : (p_align->idAlign);

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
    }
    else
    {
        // Don't send current to motors
        MOT_stopCommand(&motor[MOTOR_2]);
        // Reset controllers
        FOC_resetStruct(p_foc);
    }

    //-------------------------------------------------------------------//
    //                   Interrupt wrap functions                        //
    //-------------------------------------------------------------------//
    if(p_align->state == ALIGN_DONE)
    {
        // Save number of cycles elapsed since control loop startup
        motor[MOTOR_2].clCycleNb = EPWM_getTimeBaseCounterValue(MOTOR2_PWM1_BASE);
        // Increment interrupt counter
        motor[MOTOR_2].interruptCnt++;
        // Compute time
        comInterface.time = TIME_M2_S;//(motor[MOTOR_1].interruptCounter / PWM_FREQ);
    }

    //-------------------------------------------------------------------//
    //                   Transmit structure functions                    //
    //-------------------------------------------------------------------//
    //memcpy((FOC_STRUCT *)motor[MOTOR_2].p_motorFOC, (FOC_STRUCT *)&motorMsg[MOTOR_2], sizeof(FOC_STRUCT)); //motorFOCMsg[MOTOR_1] = motor_foc[MOTOR_1];//*(motor[MOTOR_1].motorFOC);//&(FOC_STRUCT *)(motor[MOTOR_1].motorFOC);
    //memcpy(&motorCpuToCM[MOTOR_2], motor[MOTOR_2].p_motorFOC, sizeof(FOC_STRUCT));
//    motorCpuToCM[MOTOR_2] = *motor[MOTOR_2].p_motorFOC;
    /* Send a message without message queue.
     * Since C28x and CM does not share the same address space for shared RAM,
     * ADDRESS_CORRECTION is enabled
     * Length of the data to be read is passed as data. */
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_READ_MEM, (uint32_t)(&motorCpuToCM[MOTOR_2]), 1);

    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(group);

//#if (UOMODRI == 0)
//    GPIO_writePin(DEBUG_INT_PIN2, 0);
//#endif
}

/**
 * @brief   PWM interrupt. Read encoder position of MOTOR_1
 */
//__interrupt void pwm_isrMotor1(void)
//{
//#if (UOMODRI == 0)
//    GPIO_writePin(DEBUG_INT_PIN1, 1);
//#endif
//    ENCODER* p_enc  = &motor[MOTOR_1].motorFOC->motor_enc;
//    uint32_t base   = InterruptList[PWM_IT_MOTOR_1].intPeriphBase;
//    uint16_t group  = InterruptList[PWM_IT_MOTOR_1].intAckGroup;
//    // Read encoder position
//    encoder_GetPosition(p_enc);
//    // Clear the interrupt flag
//    EPWM_clearEventTriggerInterruptFlag(base);
//    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
//    Interrupt_clearACKGroup(group);
//#if (UOMODRI == 0)
//    GPIO_writePin(DEBUG_INT_PIN1, 0);
//#endif
//}

/**
 * @brief   PWM interrupt. Read encoder position of MOTOR_2
 */
//__interrupt void pwm_isrMotor2(void)
//{
//#if (UOMODRI == 0)
//    GPIO_writePin(DEBUG_INT_PIN2, 1);
//#endif
//    ENCODER* p_enc  = &motor[MOTOR_2].motorFOC->motor_enc;
//    uint32_t base   = InterruptList[PWM_IT_MOTOR_2].intPeriphBase;
//    uint16_t group  = InterruptList[PWM_IT_MOTOR_2].intAckGroup;
//    // Read encoder position
//    encoder_GetPosition(p_enc);
//    // Clear the interrupt flag
//    EPWM_clearEventTriggerInterruptFlag(base);
//    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
//    Interrupt_clearACKGroup(group);
//#if (UOMODRI == 0)
//    GPIO_writePin(DEBUG_INT_PIN2, 0);
//#endif
//}

__attribute__((interrupt)) void gpio_isrFault1(void)
{
    // Trigger emergency stop
    MOT_emergencyStop();

    // Read DRV data to get status registers
    DRV_readStatus(motor[MOTOR_1].p_motorDRV->drvCfgHandler, motor[MOTOR_1].p_motorDRV->drvRegHandler);
//    HAL_readDRVData(&hal_motors[0].drvHandle, &drvVars[0]);

    // Set CAN error
    comInterface.canError = SPI_SENSOR_STATUS_ERROR_DRV_NFAULT;

#if !UOMODRI_SPI_DEBUG_CONFIG
    // Update sensor packet
    UpdateSensorPacket();
#endif
    // Infinite loop
    while(1);

}

__attribute__((interrupt)) void gpio_isrFault2(void)
{
    // Trigger emergency stop
    MOT_emergencyStop();

    // Read DRV data to get status registers
    DRV_readStatus(motor[MOTOR_2].p_motorDRV->drvCfgHandler, motor[MOTOR_2].p_motorDRV->drvRegHandler);
//    HAL_readDRVData(&hal_motors[0].drvHandle, &drvVars[0]);

    // Set CAN error
    comInterface.canError = SPI_SENSOR_STATUS_ERROR_DRV_NFAULT;

#if !UOMODRI_SPI_DEBUG_CONFIG
    // Update sensor packet
    UpdateSensorPacket();
#endif
    // Infinite loop
    while(1);
}

#if (CLA == 1)
__attribute__((interrupt)) void cla1_isr1(void)
{
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN2, 1);
#endif
    // Clear the ePWM  interrupt flag so the next INT can occur
    EPwm1Regs.ETCLR.bit.INT = 1;

    motor[MOTOR_1].p_motorFOC->motor_enc.thetaDir       = (bool_t)thetaDir;
    motor[MOTOR_1].p_motorFOC->motor_enc.thetaElec      = (float32_t)thetaElec;
    motor[MOTOR_1].p_motorFOC->motor_enc.thetaMech[OLD] = (float32_t)thetaMech[OLD];
    motor[MOTOR_1].p_motorFOC->motor_enc.thetaMech[NEW] = (float32_t)thetaMech[NEW];
//    motor[MOTOR_1].p_motorFOC->motor_enc.thetaMech[0]   = thetaMechCal[0]; // [pu]
//    motor[MOTOR_1].p_motorFOC->motor_enc.thetaMech[1]   = thetaMechCal[1];
//    EPWM_clearEventTriggerInterruptFlag(MOTOR1_PWM1_BASE);
    /*    EALLOW;
     *    EPwm1Regs.ETCLR.bit.INT = 1;
     *    EDIS; */
    // Acknowledge the end-of-task interrupt for task 1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11); //    PieCtrlRegs.PIEACK.all = M_INT11;

#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN2, 0);
#endif
}
#endif
