/*
 * File name: main.c
 * Description: Source file containing main function and IT running on CPU1 - C28 processor.
 */

#include "main.h"

//cmd_msg_t* p_CmdMsgAddr = &cmd_msg[0];
bool newFocCtrl_M1      = false;
bool newFocCtrl_M2      = false;
bool newCmdMsg          = false;
cmd_msg_t*      p_crCmdMsgAddr      = &cmd_msg[0];
cmd_msg_t*      p_nxCmdMsgAddr      = &cmd_msg[1];
status_msg_t*   p_crStatusMsgAddr   = &status_msg[0];
status_msg_t*   p_nxStatusMsgAddr   = &status_msg[1];

/**
 * @brief   Main function running on CPU1/C28. MCU init, DRV init, ADC init \& calibration, IT init.
 */
void main(void)
{
    // uOmodri low level device initialization
    MCU_init();
    // Hardware Abstraction Layer initialization
    HAL_init(&hal);
//    uint8_t index = 0;
//    while(dmaCfgList[(dmaCfgList[index].p_destAddr != (uint16_t *)COM_SPI_TX_BASE_ADDR) ? (index++) : (index)].p_destAddr != (uint16_t *)COM_SPI_TX_BASE_ADDR);

//    p_crStatusMsgAddr   = (status_msg_t *)HWREG(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_SRC_BEG_ADDR_SHADOW);
//    p_nxStatusMsgAddr   = (p_crStatusMsgAddr == &status_msg[0]) ? (&status_msg[1]) : (&status_msg[0]);
//    p_crCmdMsgAddr      = (cmd_msg_t *) HWREG(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_DST_BEG_ADDR_SHADOW);
//    p_nxCmdMsgAddr      = (p_crCmdMsgAddr == &cmd_msg[0]) ? (&cmd_msg[1]) : (&cmd_msg[0]);

//    status_msg_t* p_crAddr = (status_msg_t *)HWREG(dmaCfgList[index].dmaChBase + DMA_O_SRC_BEG_ADDR_SHADOW);// = *((volatile status_msg_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_SRC_BEG_ADDR_SHADOW));
//    status_msg_t*   p_crStatusMsgAddr   = (status_msg_t *)HWREG(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_SRC_BEG_ADDR_SHADOW);
//    status_msg_t*   p_nxStatusMsgAddr   = (p_crStatusMsgAddr == &status_msg[0]) ? (&status_msg[1]) : (&status_msg[0]);
//    cmd_msg_t*      p_crCmdMsgAddr      = (cmd_msg_t *) HWREG(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_DST_BEG_ADDR_SHADOW);
//    cmd_msg_t*      p_nxCmdMsgAddr      = (p_crCmdMsgAddr == &cmd_msg[0]) ? (&cmd_msg[1]) : (&cmd_msg[0]);
    // DRV8323 initialization
    DRV_init(motor[MOTOR_1].p_motorDRV);
    DRV_init(motor[MOTOR_2].p_motorDRV);
    // ADC offset calib for debug
    HAL_ADC_offsetCalib(motor[MOTOR_1].p_motorHalCfg);
    HAL_ADC_offsetCalib(motor[MOTOR_2].p_motorHalCfg);

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
    // enable DMA channels
    HAL_DMA_start(hal.p_dmaHandler);

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
    while(1)
    {
//        if(newCmdMsg)
//        {
//            newCmdMsg = false;
//            com_cmdMsgExtract(p_crCmdMsgAddr, &cmd_uOmodri);
//            p_crCmdMsgAddr  = p_nxCmdMsgAddr;
//            p_nxCmdMsgAddr  = (p_crCmdMsgAddr == &cmd_msg[0]) ? (&cmd_msg[1]) : (&cmd_msg[0]);
//
//        }
        if(newFocCtrl_M1)
        {
            com_statusMsgCreate(&motor_foc[MOTOR_1], &cmd_uOmodri, p_nxStatusMsgAddr);
            newFocCtrl_M1 = false;
        }
        if(newFocCtrl_M2)
        {
            com_statusMsgCreate(&motor_foc[MOTOR_2], &cmd_uOmodri, p_nxStatusMsgAddr);
            //while(GPIO_ReadPin(COM_SPI_CS) == 0);
            //while(HWREG(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL) & DMA_CONTROL_TRANSFERSTS);
            DMA_configSourceAddress(DMA_COM_SPI_TX_BASE_ADDR, (status_msg_t *)p_nxStatusMsgAddr);
            //SPI_resetTxFIFO(COM_SPI_BASE);
//            DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);

            status_msg_t* p_tmpStatusMsgAddr = p_crStatusMsgAddr;
            p_crStatusMsgAddr   = p_nxStatusMsgAddr;
            p_nxStatusMsgAddr   = p_tmpStatusMsgAddr;
            newFocCtrl_M2       = false;
        }
#if (UOMODRI == 0)
        if(cpt >= 40)
        {
//            uint32_t* p_pos_m1                          = (uint32_t *)&cmd_Master_msg.positionRef[MOTOR_1];
//            uint32_t* p_pos_m2                          = (uint32_t *)&cmd_Master_msg.positionRef[MOTOR_2];
//            uint32_t* p_pos_m1                          = (uint32_t *)&cmd_Master_msg.positionRef[MOTOR_1].u16_lsb;
//            uint32_t* p_pos_m2                          = (uint32_t *)&cmd_Master_msg.positionRef[MOTOR_2].u16_lsb;
//            *p_pos_m1   = 0x01234567;
//            *p_pos_m2   = 0x89ABCDEF;
            //uint64_t interruptCnt                       = motor[MOTOR_1].interruptCnt;
            cmd_Master_msg.cmdReg.bit.ES                = true;
            cmd_Master_msg.cmdReg.bit.EM1               = true;
            cmd_Master_msg.cmdReg.bit.EM2               = true;
            cmd_Master_msg.cmdReg.bit.EPRE              = false;
            cmd_Master_msg.cmdReg.bit.EI1OC             = false;
            cmd_Master_msg.cmdReg.bit.EI2OC             = false;
            cmd_Master_msg.cmdReg.bit.TIMEOUT           = 0xFF;
            float32_t   intcnt_m1                       = (float32_t)motor[MOTOR_1].interruptCnt / PWM_FREQ;
            int32_t     pos_m1                          = (int32_t)(__sinpuf32(intcnt_m1) / POSITION_LSB);
            cmd_Master_msg.positionRef[MOTOR_1].u16_msb = MSB_32(pos_m1);//(int32_t)(__sinpuf32((float32_t)interruptCnt / PWM_FREQ) / POSITION_LSB);
            cmd_Master_msg.positionRef[MOTOR_1].u16_lsb = LSB_32(pos_m1);
            float32_t   intcnt_m2                       = (float32_t)motor[MOTOR_2].interruptCnt / PWM_FREQ;
            int32_t     pos_m2                          = (int32_t)(__sinpuf32(intcnt_m2) / POSITION_LSB);
            cmd_Master_msg.positionRef[MOTOR_2].u16_msb = MSB_32(pos_m2);
            cmd_Master_msg.positionRef[MOTOR_2].u16_lsb = LSB_32(pos_m2);
            cmd_Master_msg.velocityRef[MOTOR_1]         = (int16_t)(4.0f * M_PI * M_PI * __cospuf32(intcnt_m1) / VELOCITY_LSB);
            //(int16_t)(4.0f * M_PI * M_PI * __cospuf32((float32_t)interruptCnt / PWM_FREQ) / VELOCITY_LSB);
            cmd_Master_msg.velocityRef[MOTOR_2]         = (int16_t)(4.0f * M_PI * M_PI * __cospuf32(intcnt_m2) / VELOCITY_LSB);
//            cmd_Master_msg.velocityRef[MOTOR_2]         = vel_m2;
            cmd_Master_msg.iqRef[MOTOR_1]               = 0;
            cmd_Master_msg.iqRef[MOTOR_2]               = 0;
            cmd_Master_msg.kpCoeff[MOTOR_1]             = (uint16_t)(MOTOR1_SPEED_REGULATOR_KP_COEF / KP_LSB);
            cmd_Master_msg.kpCoeff[MOTOR_2]             = (uint16_t)(MOTOR2_SPEED_REGULATOR_KP_COEF / KP_LSB);
            cmd_Master_msg.kdCoeff[MOTOR_1]             = (uint16_t)(MOTOR1_SPEED_REGULATOR_KD_COEF / KD_LSB);
            cmd_Master_msg.kdCoeff[MOTOR_2]             = (uint16_t)(MOTOR2_SPEED_REGULATOR_KD_COEF / KD_LSB);
            cmd_Master_msg.iSat                         = 0;
            cmd_Master_msg.index                        = 0;
            uint32_t    tmp                             = com_crc32((uint16_t *) &cmd_Master_msg, DMA_MST_SPI_TX_RX_16BIT_PAYLOAD_LENGTH);
            cmd_Master_msg.crc.u16_msb                  = MSB_32(tmp);
            cmd_Master_msg.crc.u16_lsb                  = LSB_32(tmp);
            cpt                                         = 0;

            while(HWREG(DMA_MST_SPI_TX_BASE_ADDR + DMA_O_CONTROL) & DMA_CONTROL_TRANSFERSTS);
            DMA_startChannel(DMA_MST_SPI_TX_BASE_ADDR);
//            DMA_forceTrigger(DMA_CH3_BASE);//DMA_MST_SPI_TX_BASE_ADDR);
//            DMA_startChannel(DMA_MST_SPI_TX_BASE_ADDR);
//            DMA_startChannel(DMA_MST_SPI_RX_BASE_ADDR);
//            DMA_forceTrigger(DMA_MST_SPI_TX_BASE_ADDR);
        }
#endif
    }
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
//__attribute__((interrupt)) void ipc_isrStatusMsg(void)
//{
//    uint32_t    cmd, data, addr;
//    uint8_t     ipcCfgIdx = 0, i = 0;
//    // Get index associated to IPC structure configuration
//    while(ipcCfgList[(ipcCfgList[ipcCfgIdx].p_ipcIntHandler != &ipc_isrStatusMsg) ? (ipcCfgIdx++) : (ipcCfgIdx)].p_ipcIntHandler != &ipc_isrStatusMsg);
//    // Get structure address
//    IPC_readCommand(ipcCfgList[ipcCfgIdx].ipcType, ipcCfgList[ipcCfgIdx].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, &cmd, &addr, &data);
//    i = motor[MOTOR_1].interruptCnt % 2;
//    memcpy(&status_msg[i], (status_msg_t *)addr, sizeof(status_msg_t));
//
//    if(!(*((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL)) &
//            (DMA_CONTROL_PERINTFLG | DMA_CONTROL_TRANSFERSTS | DMA_CONTROL_BURSTSTS)))
//    {
//        *((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL))             |= DMA_CONTROL_HALT;
//        *((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_SRC_BEG_ADDR_SHADOW))  = (uint32_t)&status_msg[i];
//        *((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_SRC_ADDR_SHADOW))      = (uint32_t)&status_msg[i];
//        *((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL))             |= DMA_CONTROL_RUN;
//    }
//
//    IPC_ackFlagRtoL(ipcCfgList[ipcCfgIdx].ipcType, ipcCfgList[ipcCfgIdx].ipcFlag);
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
//}

//-------------------------------------------------------------------------------------------------
//__attribute__((interrupt)) void ipc_isrMsgCmd(void)
//{
//    uint32_t    cmd, data, addr;
//    uint8_t     ipcCfgIdx = 0;
//    // Get index associated to IPC structure configuration
//    while(ipcCfgList[(ipcCfgList[ipcCfgIdx].p_ipcIntHandler != &ipc_isrMsgCmd) ? (ipcCfgIdx++) : (ipcCfgIdx)].p_ipcIntHandler != &ipc_isrMsgCmd);
//    // Get structure address
//    IPC_readCommand(ipcCfgList[ipcCfgIdx].ipcType, ipcCfgList[ipcCfgIdx].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, &cmd, &addr, &data);
//    memcpy(&cmd_msg, (cmd_uomodri_t *)addr, sizeof(cmd_uomodri_t));
//    IPC_ackFlagRtoL(ipcCfgList[ipcCfgIdx].ipcType, ipcCfgList[ipcCfgIdx].ipcFlag);
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
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
    FOC_STRUCT* p_foc = motor[MOTOR_1].p_motorFOC;

    if(cmd_uOmodri.cmdReg.bit.ES && cmd_uOmodri.cmdReg.bit.EM1)
    {
        // Read encoder position
        ENC_getPosition(&p_foc->motor_enc);
        // Read the phases current, voltage & Vbus
        FOC_getMeasures(&p_foc->motor_acq);
        // Compute FOC algorithm
        FOC_runControl(p_foc);
        // Set new PWM command
        MOT_runCommand(&motor[MOTOR_1], p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        // Clear the interrupt overflow flag
        ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
        // Clear the interrupt flag
        ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);

        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
        //        Interrupt_clearACKGroup(group);
        // Run encoder update
        ENC_getSpeed(&p_foc->motor_enc);
        // Check for roll-over error
        if (p_foc->motor_enc.rollOverErrorFlag && cmd_uOmodri.cmdReg.bit.EPRE)
        {
            p_foc->errorMsg = SPI_SENSOR_STATUS_ERROR_POS_ROLLOVER;
            // Tie phases to ground
            MOT_stopCommand(&motor[MOTOR_1]);
            // Reset controllers
            FOC_resetStruct(p_foc);
        }
        // Run resistance estimation
//        MOT_resistorEstimation(&motor[MOTOR_1]);
        //-------------------------------------------------------------------//
        //                       Alignment routine                           //
        //-------------------------------------------------------------------//
        if(p_foc->motor_align.state != ALIGN_DONE)
        {
            // Run alignment routine
            MOT_runAlign(&p_foc->motor_align);
            // Reset QEP position counter to set angle to zero
            if(p_foc->motor_align.state <= ALIGN_STAY)
                ENC_setPosition(&p_foc->motor_enc, 0);
        }

        //-------------------------------------------------------------------//
        //                         Reference update                          //
        //-------------------------------------------------------------------//
        // Iq reference update

#if UOMODRI_SPI_DEBUG_CONFIG
        // Set predefined trajectory reference
//        cmd_uOmodri.positionRef[MOTOR_1]   = __sinpuf32(debugFreq * TIME_M1_S);
//        cmd_uOmodri.velocityRef[MOTOR_1]   = 4.0f * M_PI * M_PI * debugFreq * __cospuf32(debugFreq * TIME_M1_S);//debugFreq * TIME_S;//
//        cmd_uOmodri.iqRef[MOTOR_1]         = 0.0f;
        cmd_uOmodri.positionRef[MOTOR_1] = __sinpuf32((float32_t)motor[MOTOR_1].interruptCnt / PWM_FREQ);
        cmd_uOmodri.velocityRef[MOTOR_1] = 4.0f * M_PI * M_PI * __cospuf32((float32_t)motor[MOTOR_1].interruptCnt / PWM_FREQ);
        cmd_uOmodri.iqRef[MOTOR_1]       = 0.0f;
#endif
        p_foc->pdPos.inTheta    = p_foc->motor_enc.thetaAbsolute;
        p_foc->pdPos.setTheta   = cmd_uOmodri.positionRef[MOTOR_1];
        p_foc->pdPos.inSpeed    = p_foc->motor_enc.speedEst.speedMech;
        p_foc->pdPos.setSpeed   = cmd_uOmodri.velocityRef[MOTOR_1];
        p_foc->pdPos.ff         = cmd_uOmodri.iqRef[MOTOR_1];// Set feed-forward current
        // Run PD controller
        FOC_runPD(&p_foc->pdPos);
        // Set current reference
        p_foc->iqRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (p_foc->pdPos.out) : (0.0f);
        // Id reference update
        p_foc->idRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (0.0f) : (p_foc->motor_align.idAlign);

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
    if(p_foc->motor_align.state == ALIGN_DONE)
    {
        // Save number of cycles elapsed since control loop startup
        motor[MOTOR_1].clCycleNb = EPWM_getTimeBaseCounterValue(MOTOR1_PWM1_BASE);
        // Increment interrupt counter
        motor[MOTOR_1].interruptCnt++;
        // Compute time
//        cmd_uOmodri.time[MOTOR_1] = TIME_M1_S;//(motor[MOTOR_1].interruptCounter / PWM_FREQ);
    }

    //-------------------------------------------------------------------//
    //                   Transmit structure functions                    //
    //-------------------------------------------------------------------//
//    memcpy(&motorCpuToCM[MOTOR_1], motor[MOTOR_1].p_motorFOC, sizeof(FOC_STRUCT));
    //motorCpuToCM[MOTOR_1] = *motor[MOTOR_1].p_motorFOC;
    /* Send a message without message queue.
     * Since C28x and CM does not share the same address space for shared RAM,
     * ADDRESS_CORRECTION is enabled
     * Length of the data to be read is passed as data. */
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_READ_MEM, (uint32_t)(&motorCpuToCM[MOTOR_1]), 1);
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_READ_MEM, (uint32_t)(p_foc), 1);
    newFocCtrl_M1 = true;
    cpt++;
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR1_IA_INT_ACK_GROUP);
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_2
 */
__attribute__((interrupt)) void adc_isrMotor2(void)
{
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    FOC_STRUCT* p_foc = motor[MOTOR_2].p_motorFOC;

    if(cmd_uOmodri.cmdReg.bit.ES && cmd_uOmodri.cmdReg.bit.EM2)
    {
        // Read encoder position
        ENC_getPosition(&p_foc->motor_enc);
        // Read the phases current, voltage & Vbus
        FOC_getMeasures(&p_foc->motor_acq);
        // Compute FOC algorithm
        FOC_runControl(p_foc);
        // Set new PWM command
        MOT_runCommand(&motor[MOTOR_2], p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        // Clear the interrupt overflow flag
        ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);
        // Clear the interrupt flag
        ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);

        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group)
        //        Interrupt_clearACKGroup(group);
        // Run encoder update
        ENC_getSpeed(&p_foc->motor_enc);

        //ENC_update(motor[MOTOR_1].motorFOC->motor_enc);
        // Check for roll-over error
        if (p_foc->motor_enc.rollOverErrorFlag && cmd_uOmodri.cmdReg.bit.EPRE)
        {
            p_foc->errorMsg = SPI_SENSOR_STATUS_ERROR_POS_ROLLOVER;
            // Turn off control loop
//            motor[MOTOR_1].p_motorFOC->enabled  = false;
            // Disable ePWM interrupts
//            Interrupt_disableInCPU(INTERRUPT_CPU_INT1);
            // Tie phases to ground
            MOT_stopCommand(&motor[MOTOR_2]);
            // Reset controllers
            FOC_resetStruct(p_foc);
//            MOT_emergencyStop(&motor[MOTOR_2]);
        }
        // Run resistance estimation
//        MOT_resistorEstimation(&motor[MOTOR_2]);
        //-------------------------------------------------------------------//
        //                       Alignment routine                           //
        //-------------------------------------------------------------------//
        if(p_foc->motor_align.state != ALIGN_DONE)
        {
            // Run alignment routine
            MOT_runAlign(&p_foc->motor_align);

            // Reset QEP position counter to set angle to zero
            if (p_foc->motor_align.state <= ALIGN_STAY)
                ENC_setPosition(&p_foc->motor_enc, 0);
        }

        //-------------------------------------------------------------------//
        //                         Reference update                          //
        //-------------------------------------------------------------------//
        // Iq reference update

#if UOMODRI_SPI_DEBUG_CONFIG
        // Set predefined trajectory reference
//        comInterface.positionRef[MOTOR_2]   = __sinpuf32(debugFreq * TIME_M2_S);//debugFreq * TIME_S;//
//        comInterface.velocityRef[MOTOR_2]   = 4.0f * M_PI * M_PI * debugFreq * __cospuf32(debugFreq * TIME_M2_S);//debugFreq * TIME_S;//
//        comInterface.iqRef[MOTOR_2]         = 0.0f;
        cmd_uOmodri.positionRef[MOTOR_2] = __sinpuf32((float32_t)motor[MOTOR_2].interruptCnt / PWM_FREQ);
        cmd_uOmodri.velocityRef[MOTOR_2] = 4.0f * M_PI * M_PI * __cospuf32((float32_t)motor[MOTOR_2].interruptCnt / PWM_FREQ);
        cmd_uOmodri.iqRef[MOTOR_2]       = 0.0f;
#endif
        p_foc->pdPos.inTheta    = p_foc->motor_enc.thetaAbsolute;
        p_foc->pdPos.setTheta   = cmd_uOmodri.positionRef[MOTOR_2];
        p_foc->pdPos.inSpeed    = p_foc->motor_enc.speedEst.speedMech;
        p_foc->pdPos.setSpeed   = cmd_uOmodri.velocityRef[MOTOR_2];
        p_foc->pdPos.ff         = cmd_uOmodri.iqRef[MOTOR_2];// Set feed-forward current
        // Run PD controller
        FOC_runPD(&p_foc->pdPos);
        // Set current reference
        p_foc->iqRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (p_foc->pdPos.out) : (0.0f);
        // Id reference update
        p_foc->idRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (0.0f) : (p_foc->motor_align.idAlign);

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
    if(p_foc->motor_align.state == ALIGN_DONE)
    {
        // Save number of cycles elapsed since control loop startup
        motor[MOTOR_2].clCycleNb = EPWM_getTimeBaseCounterValue(MOTOR2_PWM1_BASE);
        // Increment interrupt counter
        motor[MOTOR_2].interruptCnt++;

        // Compute time
//        cmd_uOmodri.time[MOTOR_2] = TIME_M2_S;//(motor[MOTOR_1].interruptCounter / PWM_FREQ);
    }

    //-------------------------------------------------------------------//
    //                   Transmit structure functions                    //
    //-------------------------------------------------------------------//
//    memcpy(&motorCpuToCM[MOTOR_1], motor[MOTOR_1].p_motorFOC, sizeof(FOC_STRUCT));
    //motorCpuToCM[MOTOR_1] = *motor[MOTOR_1].p_motorFOC;
    /* Send a message without message queue.
     * Since C28x and CM does not share the same address space for shared RAM,
     * ADDRESS_CORRECTION is enabled
     * Length of the data to be read is passed as data. */
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_READ_MEM, (uint32_t)(&motorCpuToCM[MOTOR_1]), 1);
    //IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_READ_MEM, (uint32_t)(motor[MOTOR_1].p_motorFOC), 1);
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_READ_MEM, (uint32_t)(p_foc), 1);
    // Acknowledge PWM and CLA interrupt flag
    newFocCtrl_M2 = true;
    Interrupt_clearACKGroup(MOTOR2_IA_INT_ACK_GROUP);
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
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
__attribute__((interrupt)) void dma_isrCmdMsg(void)
{
#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN2, 1);
#endif
//    DMA_stopChannel(DMA_COM_SPI_RX_BASE_ADDR);
    //while(HWREG(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_CONTROL) & DMA_CONTROL_TRANSFERSTS);
    DMA_configDestAddress(DMA_COM_SPI_RX_BASE_ADDR, p_nxCmdMsgAddr);
//    DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
//    DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
//    EALLOW;
//    HWREG(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_DST_BEG_ADDR_SHADOW) = (uint32_t)p_nxCmdMsgAddr;
//    EDIS;
//    DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
//    newCmdMsg = true;
//    p_nxCmdMsgAddr  = (p_crCmdMsgAddr == &cmd_msg[0]) ? (&cmd_msg[1]) : (&cmd_msg[0]);
    com_cmdMsgExtract(p_crCmdMsgAddr, &cmd_uOmodri);
    cmd_msg_t* p_tmpCmdMsgAddr  = p_crCmdMsgAddr;
    p_crCmdMsgAddr              = p_nxCmdMsgAddr;
    p_nxCmdMsgAddr              = p_tmpCmdMsgAddr;
    // Clear the interrupt flag
    Interrupt_clearACKGroup(DMA_COM_SPI_RX_INT_ACK_GROUP);

#if (UOMODRI == 0)
    GPIO_writePin(DEBUG_INT_PIN2, 0);
#endif
}

//__attribute__((interrupt)) void spi_isrRx(void)
//{
////    uint8_t index               = 0;
////    while(InterruptList[index].p_intHandler != &spi_isrRx) index++;
////    uint16_t group              = InterruptList[index].intAckGroup;
//    static uint16_t tmp[17];
//    static uint8_t i = 0;
//
//    while((SPI_getRxFIFOStatus(COM_SPI_BASE) != SPI_FIFO_RXEMPTY) && (i < 17))
//        tmp[i++] = SPI_readDataNonBlocking(COM_SPI_BASE);
//
//    if(i == 0)
//        p_CmdMsgAddr = (p_CmdMsgAddr == &cmd_msg[0]) ? (&cmd_msg[1]) : (&cmd_msg[0]);
//    else if(i >= 17)
//    {
//        i = 0;
//        memcpy((cmd_msg_t*)p_CmdMsgAddr, &tmp[0], sizeof(cmd_msg_t));
//        newCmdMsg = true;
//    }
//
//    SPI_clearInterruptStatus(COM_SPI_BASE, SPI_INT_RXFF);
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
//}

__attribute__((interrupt)) void gpio_isrFaultMotor1(void)
{
    cmd_uOmodri.cmdReg.bit.ES = false;
    MOT_stopCommand(&motor[MOTOR_1]);
    // Reset controllers
    FOC_resetStruct(motor[MOTOR_1].p_motorFOC);
    // Trigger emergency stop
//    MOT_emergencyStop(&motor[MOTOR_1]);

    // Read DRV data to get status registers
    DRV_readStatus(motor[MOTOR_1].p_motorDRV->drvCfgHandler, motor[MOTOR_1].p_motorDRV->drvRegHandler);
//    HAL_readDRVData(&hal_motors[0].drvHandle, &drvVars[0]);

    // Set CAN error
    motor[MOTOR_1].p_motorFOC->errorMsg = SPI_SENSOR_STATUS_ERROR_DRV_NFAULT;

#if !UOMODRI_SPI_DEBUG_CONFIG
    // Update sensor packet
//    UpdateSensorPacket();
#endif
    // Infinite loop
    while(1);

}

__attribute__((interrupt)) void gpio_isrFaultMotor2(void)
{
    cmd_uOmodri.cmdReg.bit.ES = false;
    MOT_stopCommand(&motor[MOTOR_2]);
    // Reset controllers
    FOC_resetStruct(motor[MOTOR_2].p_motorFOC);
    // Trigger emergency stop
//    MOT_emergencyStop(&motor[MOTOR_2]);

    // Read DRV data to get status registers
    DRV_readStatus(motor[MOTOR_2].p_motorDRV->drvCfgHandler, motor[MOTOR_2].p_motorDRV->drvRegHandler);
//    HAL_readDRVData(&hal_motors[0].drvHandle, &drvVars[0]);

    // Set CAN error
    motor[MOTOR_1].p_motorFOC->errorMsg = SPI_SENSOR_STATUS_ERROR_DRV_NFAULT;

#if !UOMODRI_SPI_DEBUG_CONFIG
    // Update sensor packet
//    UpdateSensorPacket();
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
