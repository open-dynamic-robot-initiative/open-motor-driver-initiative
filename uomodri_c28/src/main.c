/*
 * File name: main.c
 * Description: Source file containing main function and IT running on CPU1 - C28 processor.
 */

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "main.h"

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
bool_t          newFocCtrl_M1       = false;
bool_t          newFocCtrl_M2       = false;
mst2slv_msg_t*  p_crCmdMsgAddr      = &cmd_msg[0];
mst2slv_msg_t*  p_nxCmdMsgAddr      = &cmd_msg[1];
slv2mst_msg_t*  p_crStatusMsgAddr   = &status_msg[0];
slv2mst_msg_t*  p_nxStatusMsgAddr   = &status_msg[1];

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
static void MCU_ini(void);

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief   Main function running on CPU1/C28. MCU init, DRV init, ADC init \& calibration, IT init.
 */
void main(void)
{
    // uOmodri low level device initialization
    MCU_ini();
    // Hardware Abstraction Layer initialization
    HAL_ini(&hal);
    // DRV8323 initialization
    DRV_ini(motor[MOTOR_1].p_motorDRV);
    DRV_ini(motor[MOTOR_2].p_motorDRV);
    // ADC offset calibration for debug
    HAL_ADC_offsetCalib(motor[MOTOR_1].p_motorHalCfg);
    HAL_ADC_offsetCalib(motor[MOTOR_2].p_motorHalCfg);
    // Interrupts initialization
    HAL_INT_ini(InterruptList);

    // Infinite loop
    while(1)
    {
        if(newFocCtrl_M1)
        {
            com_MsgCreate(&motor_foc[MOTOR_1], &cmd_uOmodri, p_nxStatusMsgAddr);
            newFocCtrl_M1 = false;
        }
        if(newFocCtrl_M2)
        {
            com_MsgCreate(&motor_foc[MOTOR_2], &cmd_uOmodri, p_nxStatusMsgAddr);
            DMA_configSourceAddress(DMA_COM_SPI_TX_BASE_ADDR, (slv2mst_msg_t *)p_nxStatusMsgAddr);
            slv2mst_msg_t* p_tmpStatusMsgAddr   = p_crStatusMsgAddr;
            p_crStatusMsgAddr                   = p_nxStatusMsgAddr;
            p_nxStatusMsgAddr                   = p_tmpStatusMsgAddr;
            newFocCtrl_M2                       = false;
        }
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
        if(cpt >= 40)
        {
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
}

/**
 * @brief   Microcontroller initialization function
 *          - Configure master clock to 200MHz
 *          - Configure auxiliary clock (CM core) to 125MHz (USB à 60MHz)
 *          - Enable GPIO clocking
 *          - Configure the interruption vector table
 *          - Boot CM from flash or RAM
 */
static void MCU_ini(void)
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
    /* Set the CM Clock to run at 125MHz.
     * The CM Clock is a fractional multiple of the AUXPLL Clock (125 MHz). */
#if (defined CM_ENABLE) && (CM_ENABLE)
    SysCtl_setCMClk(SYSCTL_CMCLKOUT_DIV_1, SYSCTL_SOURCE_AUXPLL);
    /* Allocate Shared Peripheral USB to the CM Side. */
//    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
#endif
    // Initialize GPIO
    Device_initGPIO();
    // Initialize PIE control registers and flags. Disables CPU interrupts.
    Interrupt_initModule();
    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    Interrupt_initVectorTable();
    // Boot CM core
#if (defined CM_ENABLE) && (CM_ENABLE)
#ifdef _FLASH
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif
#endif
}

//-------------------------------------------------------------------------------------------------
#if (defined CM_ENABLE) && (CM_ENABLE)
__attribute__((interrupt)) void ipc_isrStatusMsg(void)
{
    uint32_t    cmd, data, addr;
    uint8_t     ipcCfgIdx = 0, i = 0;
    // Get index associated to IPC structure configuration
    while(ipcCfgList[(ipcCfgList[ipcCfgIdx].p_ipcIntHandler != &ipc_isrStatusMsg) ? (ipcCfgIdx++) : (ipcCfgIdx)].p_ipcIntHandler != &ipc_isrStatusMsg);
    // Get structure address
    IPC_readCommand(ipcCfgList[ipcCfgIdx].ipcType, ipcCfgList[ipcCfgIdx].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, &cmd, &addr, &data);
    i = motor[MOTOR_1].interruptCnt % 2;
    memcpy(&status_msg[i], (status_msg_t *)addr, sizeof(status_msg_t));

    if(!(*((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL)) &
            (DMA_CONTROL_PERINTFLG | DMA_CONTROL_TRANSFERSTS | DMA_CONTROL_BURSTSTS)))
    {
        *((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL))             |= DMA_CONTROL_HALT;
        *((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_SRC_BEG_ADDR_SHADOW))  = (uint32_t)&status_msg[i];
        *((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_SRC_ADDR_SHADOW))      = (uint32_t)&status_msg[i];
        *((volatile uint32_t *)(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL))             |= DMA_CONTROL_RUN;
    }

    IPC_ackFlagRtoL(ipcCfgList[ipcCfgIdx].ipcType, ipcCfgList[ipcCfgIdx].ipcFlag);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
}
#endif
//-------------------------------------------------------------------------------------------------
#if (defined CM_ENABLE) && (CM_ENABLE)
__attribute__((interrupt)) void ipc_isrMsgCmd(void)
{
    uint32_t    cmd, data, addr;
    uint8_t     ipcCfgIdx = 0;
    // Get index associated to IPC structure configuration
    while(ipcCfgList[(ipcCfgList[ipcCfgIdx].p_ipcIntHandler != &ipc_isrMsgCmd) ? (ipcCfgIdx++) : (ipcCfgIdx)].p_ipcIntHandler != &ipc_isrMsgCmd);
    // Get structure address
    IPC_readCommand(ipcCfgList[ipcCfgIdx].ipcType, ipcCfgList[ipcCfgIdx].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, &cmd, &addr, &data);
    memcpy(&cmd_msg, (cmd_uomodri_t *)addr, sizeof(cmd_uomodri_t));
    IPC_ackFlagRtoL(ipcCfgList[ipcCfgIdx].ipcType, ipcCfgList[ipcCfgIdx].ipcFlag);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
}
#endif
//-------------------------------------------------------------------------------------------------
/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_1
 */
__attribute__((interrupt)) void adc_isrMotor1(void)
{
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    foc_t* p_foc = motor[MOTOR_1].p_motorFOC;
//    align_t align = p_foc->motor_align;
//    static motor_state crState = MOTOR_ALIGN_COUNT_UP, nxState = MOTOR_ALIGN_COUNT_UP;

    if(cmd_uOmodri.mode.bit.ES && cmd_uOmodri.mode.bit.EM1)
    {
        /*----------------------------------------------------------------------------*/
//        switch(crState)
//        case MOTOR_ALIGN_COUNT_UP:  // Increment alignment current
//            align.idAlign  += 1e-5;
//            p_foc->iqRef    = 0.0f;
//            p_foc->idRef    = align.idAlign;
//            nxState         = (align.idAlign >= align.idAlignRef) ? (MOTOR_ALIGN_STAY) : (crState);
//            ENC_setPosition(&p_foc->motor_enc, 0);
//            break;
//
//        case MOTOR_ALIGN_STAY : // Maintain alignment current
//            align.idAlign   = align.idAlign;
//            p_foc->iqRef    = 0.0f;
//            p_foc->idRef    = align.idAlign;
//            nxState         = (++align.cnt >= align.cntMax) ? (MOTOR_ALIGN_COUNT_DOWN) : (crState);
//            ENC_setPosition(&p_foc->motor_enc, 0);
//            break;
//
//        case MOTOR_ALIGN_COUNT_DOWN :   // Decrement alignment current
//            align.idAlign  -= 1e-5;
//            p_foc->iqRef    = 0.0f;
//            p_foc->idRef    = align.idAlign;
//            align.cnt       = 0;
////            align.idAlign   = (align.idAlign > 0.0f) ? (align.idAlign - 1e-5) : (0.0f);
//            nxState         = (align.idAlign <= 0.0f) ? (MOTOR_ALIGN_DONE) : (crState);
//            break;
//
//        case MOTOR_ALIGN_DONE :
//            align.idAlign   = 0.0f;
//            p_foc->iqRef    = p_foc->pdPos.out;
//            p_foc->idRef    = 0.0f;
//            align.cnt       = 0;
//            nxState         = MOTOR_RUN;
//            break;
//
//        case MOTOR_RUN :
//            p_foc->iqRef    = p_foc->pdPos.out;
//            p_foc->idRef    = 0.0f;
//            align.cnt       = 0;
//            nxState         = MOTOR_RUN;
        /*----------------------------------------------------------------------------*/
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
        // Run encoder update
        ENC_getSpeed(&p_foc->motor_enc);
        ENC_getTheta(&p_foc->motor_enc);
        // Check for roll-over error
        if (p_foc->motor_enc.rollOverFlag && cmd_uOmodri.mode.bit.EPRE)
        {
            p_foc->errorMsg = FOC_ERROR_POS_ROLLOVER;
            MOT_stopCommand(&motor[MOTOR_1]);
            FOC_resetStruct(p_foc);
        }
        // Run resistance estimation
//        MOT_resistorEstimation(&motor[MOTOR_1]);
        //-------------------------------------------------------------------
        //     ALIGNMENT ROUTINE
        //-------------------------------------------------------------------
        if(p_foc->motor_align.state != ALIGN_DONE)
        {
            // Run alignment routine
            MOT_runAlign(&p_foc->motor_align);
            // Reset QEP position counter to set angle to zero
            if(p_foc->motor_align.state <= ALIGN_STAY)
                ENC_setPosition(&p_foc->motor_enc, 0);
        }
        //-------------------------------------------------------------------
        //     SPEED & VELOCITY CONTROL
        //-------------------------------------------------------------------
#ifdef UOMODRI_SPI_DEBUG_CONFIG
        // Set predefined trajectory reference
        cmd_uOmodri.position[MOTOR_1]   = __sinpuf32((float32_t)motor[MOTOR_1].interruptCnt / PWM_FREQ);
        cmd_uOmodri.velocity[MOTOR_1]   = 4.0f * M_PI * M_PI * __cospuf32((float32_t)motor[MOTOR_1].interruptCnt / PWM_FREQ);
        cmd_uOmodri.current[MOTOR_1]    = 0.0f;
#endif
//        p_foc->pdPos.inTheta    = p_foc->motor_enc.thetaAbsolute;
//        p_foc->pdPos.setTheta   = cmd_uOmodri.position[MOTOR_1];
//        p_foc->pdPos.inSpeed    = p_foc->motor_enc.speed.speedMech;
//        p_foc->pdPos.setSpeed   = cmd_uOmodri.velocity[MOTOR_1];
//        p_foc->pdPos.ff         = cmd_uOmodri.current[MOTOR_1];// Set feed-forward current
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
//    cpt++;
    motor[MOTOR_1].interruptCnt++;
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR1_IA_INT_ACK_GROUP);
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_2
 */
__attribute__((interrupt)) void adc_isrMotor2(void)
{
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    foc_t* p_foc = motor[MOTOR_2].p_motorFOC;

    if(cmd_uOmodri.mode.bit.ES && cmd_uOmodri.mode.bit.EM2)
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
        // Run encoder update
        ENC_getSpeed(&p_foc->motor_enc);
        ENC_getTheta(&p_foc->motor_enc);
        // Check for roll-over error
        if (p_foc->motor_enc.rollOverFlag && cmd_uOmodri.mode.bit.EPRE)
        {
            p_foc->errorMsg = FOC_ERROR_POS_ROLLOVER;
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
            if(p_foc->motor_align.state <= ALIGN_STAY)
                ENC_setPosition(&p_foc->motor_enc, 0);
        }

        //-------------------------------------------------------------------//
        //                         Reference update                          //
        //-------------------------------------------------------------------//
        // Iq reference update

#ifdef UOMODRI_SPI_DEBUG_CONFIG
        // Set predefined trajectory reference
        cmd_uOmodri.position[MOTOR_2]   = __sinpuf32((float32_t)motor[MOTOR_2].interruptCnt / PWM_FREQ);
        cmd_uOmodri.velocity[MOTOR_2]   = 4.0f * M_PI * M_PI * __cospuf32((float32_t)motor[MOTOR_2].interruptCnt / PWM_FREQ);
        cmd_uOmodri.current[MOTOR_2]    = 0.0f;
#endif
//        p_foc->pdPos.inTheta    = p_foc->motor_enc.thetaAbsolute;
//        p_foc->pdPos.setTheta   = cmd_uOmodri.position[MOTOR_2];
//        p_foc->pdPos.inSpeed    = p_foc->motor_enc.speed.speedMech;
//        p_foc->pdPos.setSpeed   = cmd_uOmodri.velocity[MOTOR_2];
//        p_foc->pdPos.ff         = cmd_uOmodri.current[MOTOR_2];// Set feed-forward current
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
    motor[MOTOR_2].interruptCnt++;
    Interrupt_clearACKGroup(MOTOR2_IA_INT_ACK_GROUP);
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

/**
 * @brief   DMA interrupt. New command message has been received from master
 */
__attribute__((interrupt)) void dma_isrCmdMsg(void)
{
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
    GPIO_writePin(DEBUG_INT_PIN2, 1);
#endif
//    DMA_stopChannel(DMA_COM_SPI_RX_BASE_ADDR);

//    while(HWREG(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_CONTROL) & DMA_CONTROL_TRANSFERSTS);
//    SPI_isBusy(COM_SPI_BASE);

    DMA_configDestAddress(DMA_COM_SPI_RX_BASE_ADDR, p_nxCmdMsgAddr);
    //while(SPI_isBusy(COM_SPI_BASE) == true);
//    DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
//    DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
//    DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
//    EALLOW;
//    HWREG(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_DST_BEG_ADDR_SHADOW) = (uint32_t)p_nxCmdMsgAddr;
//    EDIS;
//    DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
//    newCmdMsg = true;
//    p_nxCmdMsgAddr  = (p_crCmdMsgAddr == &cmd_msg[0]) ? (&cmd_msg[1]) : (&cmd_msg[0]);
    com_MsgExtract(p_crCmdMsgAddr, &cmd_uOmodri);
    cmd_uOmodri.mode.bit.EI2OC      = false;
    cmd_uOmodri.mode.bit.EI1OC      = false;
    cmd_uOmodri.mode.bit.EPRE       = false;
    cmd_uOmodri.mode.bit.EM2        = false;
    cmd_uOmodri.mode.bit.EM1        = true;
    cmd_uOmodri.position[MOTOR_1]   = __sinpuf32((float32_t)motor[MOTOR_1].interruptCnt / 2.0 / PWM_FREQ);
    cmd_uOmodri.position[MOTOR_2]   = cmd_uOmodri.position[MOTOR_1];//__sinpuf32((float32_t)motor[MOTOR_2].interruptCnt / PWM_FREQ);
    cmd_uOmodri.velocity[MOTOR_1]   = 4.0f * M_PI * M_PI * __cospuf32((float32_t)motor[MOTOR_1].interruptCnt / 1.0 / PWM_FREQ);
    cmd_uOmodri.velocity[MOTOR_2]   = cmd_uOmodri.velocity[MOTOR_1];//4.0f * M_PI * M_PI * __cospuf32((float32_t)motor[MOTOR_2].interruptCnt / PWM_FREQ);
    cmd_uOmodri.current[MOTOR_1]    = 0.0f;
    cmd_uOmodri.current[MOTOR_2]    = 0.0f;
    cmd_uOmodri.kpCoeff[MOTOR_1]    = (float32_t)MOTOR1_SPEED_REGULATOR_KP_COEF;
    cmd_uOmodri.kpCoeff[MOTOR_2]    = (float32_t)MOTOR2_SPEED_REGULATOR_KP_COEF;
    cmd_uOmodri.kdCoeff[MOTOR_1]    = (float32_t)MOTOR1_SPEED_REGULATOR_KD_COEF;
    cmd_uOmodri.kdCoeff[MOTOR_2]    = (float32_t)MOTOR2_SPEED_REGULATOR_KD_COEF;
    cmd_uOmodri.iSat[MOTOR_1]       = (float32_t)MOTOR1_SPEED_REGULATOR_SATURATION;
    cmd_uOmodri.iSat[MOTOR_2]       = (float32_t)MOTOR2_SPEED_REGULATOR_SATURATION;
    mst2slv_msg_t* p_tmpCmdMsgAddr  = p_crCmdMsgAddr;
    p_crCmdMsgAddr                  = p_nxCmdMsgAddr;
    p_nxCmdMsgAddr                  = p_tmpCmdMsgAddr;
    static uint8_t i = 0;
//    for(i = 0; i < 6; i++)
//        SPI_writeDataNonBlocking(LED_SPI_BASE, led_msg[i]);
    ws2812b_writeLed(0, ((i < 256) ? (i++) : (i = 0)), 0, led_msg);
    DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
    DMA_forceTrigger(DMA_LED_SPI_TX_BASE_ADDR);
    // Clear the interrupt flag
    Interrupt_clearACKGroup(DMA_COM_SPI_RX_INT_ACK_GROUP);

#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
    GPIO_writePin(DEBUG_INT_PIN2, 0);
#endif
}

__attribute__((interrupt)) void gpio_isrFaultMotor1(void)
{
    cmd_uOmodri.mode.bit.ES = false;
    MOT_stopCommand(&motor[MOTOR_1]);
    // Reset controllers
    FOC_resetStruct(motor[MOTOR_1].p_motorFOC);
    // Trigger emergency stop
//    MOT_emergencyStop(&motor[MOTOR_1]);

    // Read DRV data to get status registers
    DRV_readStatus(motor[MOTOR_1].p_motorDRV);
//    HAL_readDRVData(&hal_motors[0].drvHandle, &drvVars[0]);

    // Set CAN error
    motor[MOTOR_1].p_motorFOC->errorMsg = FOC_ERROR_DRV_NFAULT;

//#if !UOMODRI_SPI_DEBUG_CONFIG
//    // Update sensor packet
////    UpdateSensorPacket();
//#endif
    // Infinite loop
    while(1);

}

__attribute__((interrupt)) void gpio_isrFaultMotor2(void)
{
    cmd_uOmodri.mode.bit.ES = false;
    MOT_stopCommand(&motor[MOTOR_2]);
    // Reset controllers
    FOC_resetStruct(motor[MOTOR_2].p_motorFOC);
    // Trigger emergency stop
//    MOT_emergencyStop(&motor[MOTOR_2]);

    // Read DRV data to get status registers
    DRV_readStatus(motor[MOTOR_2].p_motorDRV);
//    HAL_readDRVData(&hal_motors[0].drvHandle, &drvVars[0]);

    // Set CAN error
    motor[MOTOR_1].p_motorFOC->errorMsg = FOC_ERROR_DRV_NFAULT;

//#if !UOMODRI_SPI_DEBUG_CONFIG
//    // Update sensor packet
////    UpdateSensorPacket();
//#endif
    // Infinite loop
    while(1);
}

#if (defined CLA_ENABLE) && (CLA_ENABLE)
__attribute__((interrupt)) void cla1_isr1(void)
{
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
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

#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
    GPIO_writePin(DEBUG_INT_PIN2, 0);
#endif
}
#endif
