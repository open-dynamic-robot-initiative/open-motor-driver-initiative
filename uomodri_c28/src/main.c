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

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
static void MCU_ini(void);
inline void ptr_swap(int32_t**, int32_t**);

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief   Main function running on CPU1/C28. MCU init, DRV init, ADC init \& calibration, IT init.
 */
void main(void)
{
    uint16_t* p_cmdMsg          = (uint16_t *)&cmd_msg;
    uint16_t* p_statusMsg       = (uint16_t *)&status_msg;
    uint16_t* p_ledMsg          = (uint16_t *)&led_msg;
    uint16_t spi_com_rx_loop    = 0;
    uint16_t spi_com_tx_loop    = 0;
    uint16_t spi_led_tx_loop    = 0;
    uint16_t timer_cpt_ms       = 0;

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
        // SPI COM RX management - Receive & extract message
        if(SPI_getRxFIFOStatus(COM_SPI_BASE))
        {
            while((SPI_getRxFIFOStatus(COM_SPI_BASE) != SPI_FIFO_RXEMPTY) && (spi_com_rx_loop++ < COM_MSG_TX_RX_16BIT_FULL_LENGTH))
                *p_cmdMsg++     = SPI_readDataNonBlocking(COM_SPI_BASE);
            if(spi_com_rx_loop >= COM_MSG_TX_RX_16BIT_FULL_LENGTH)
                com_MsgExtract(&cmd_msg, &cmd_uOmodri);
            if(GPIO_ReadPin(COM_SPI_CS))
            {
                spi_com_rx_loop = 0;
                p_cmdMsg        = (uint16_t *)&cmd_msg;
                SPI_resetRxFIFO(COM_SPI_BASE);
            }
        }
        // SPI COM TX management - Create message & send it
        if(motor[MOTOR_2].interruptDone)
        {
            com_MsgCreate(&motor_foc[MOTOR_1], &cmd_uOmodri, &status_msg);
            com_MsgCreate(&motor_foc[MOTOR_2], &cmd_uOmodri, &status_msg);
            spi_com_tx_loop = 0;
            p_statusMsg     = (uint16_t *)&status_msg;
            if(GPIO_ReadPin(COM_SPI_CS))
            {
                // Force first write in SPI_O_DAT
                SPI_writeDataNonBlocking(COM_SPI_BASE, (uint16_t ) *p_statusMsg++);
                spi_com_tx_loop++;
                SPI_resetTxFIFO(COM_SPI_BASE);
                // Force sixteen writes in SPI_O_TXBUF
                while((SPI_getTxFIFOStatus(COM_SPI_BASE) != SPI_FIFO_TXFULL) && (spi_com_tx_loop++ < COM_MSG_TX_RX_16BIT_FULL_LENGTH))
                    SPI_writeDataNonBlocking(COM_SPI_BASE, (uint16_t) *p_statusMsg++);
            }
            motor[MOTOR_2].interruptDone = false;
        }
        // SPI LED TX management - Create message & send it @ 1ms
        if(CPUTimer_getTimerOverflowStatus(CPU_TIMER_0_BASE))
        {
            float32_t timer_fade_blink  = (float32_t)((timer_cpt_ms < 500) ? (timer_cpt_ms) : (1000.0 - timer_cpt_ms)) / 500.0f;
            timer_cpt_ms                = (timer_cpt_ms < 1000) ? (timer_cpt_ms + 1) : (0);
            spi_led_tx_loop             = 0;
            p_ledMsg                    = (uint16_t *)&led_msg;
            ws2812b_writeLed(0, (uint16_t)(0xff * timer_fade_blink), 0, p_ledMsg);
            while((SPI_getTxFIFOStatus(LED_SPI_BASE) != SPI_FIFO_TXFULL) && (spi_led_tx_loop++ < 6))
                SPI_writeDataNonBlocking(LED_SPI_BASE, (uint16_t) *p_ledMsg++);
            CPUTimer_clearOverflowFlag(CPU_TIMER_0_BASE);
        }
    }
}

/**
 * @brief           Microcontroller initialization function
 *                  - Configure master clock to 200MHz
 *                  - Configure auxiliary clock (CM core) to 125MHz (USB à 60MHz)
 *                  - Enable GPIO clocking
 *                  - Configure the interruption vector table
 *                  - Boot CM from flash or RAM
 */
static void MCU_ini(void)
{
    // Initialize clock and peripherals
    Device_init();
    /* Set up the auxiliary PLL to 125MHz for CM use. */
    SysCtl_setAuxClock(SYSCTL_AUXPLL_OSCSRC_XTAL | SYSCTL_AUXPLL_IMULT(50) | SYSCTL_REFDIV(2U) |
                       SYSCTL_ODIV(5U) | SYSCTL_AUXPLL_DIV_2 | SYSCTL_AUXPLL_ENABLE | SYSCTL_DCC_BASE_0);
    /* Set the CM Clock to run at 125MHz.
     * The CM Clock is a fractional multiple of the AUXPLL Clock (125 MHz). */
#if (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE)
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
#if (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE) && (defined _FLASH)
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#elif (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE) && !(defined _FLASH)
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif
}

/**
 * @brief           Swap the pointing addresses of p_a \& p_b
 * @param[inout]    p_a Pointer on the first pointer to be swapped
 * @param[inout]    p_b Pointer on the second pointer to be swapped
 */
inline void ptr_swap(int32_t** p_a, int32_t** p_b)
{
    int32_t* p_tmp  = *p_a;
    *p_a            = *p_b;
    *p_b            = p_tmp;
}

/**
 * @brief           ADC interrupt for MOTOR_1. Read currents \& voltages and compute control.
 */
__attribute__((interrupt)) void adc_isrMotor1(void)
{
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    foc_t* p_foc = motor[MOTOR_1].p_motorFOC;

    // Read encoder position
    ENC_getPosition(&p_foc->motor_enc);
    // Read the phases current, voltage & Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Estimate speed
    ENC_getSpeed(&p_foc->motor_enc);
    // Rollover test & index
    ENC_getTheta(&p_foc->motor_enc);

    if(cmd_uOmodri.mode.bit.ES && cmd_uOmodri.mode.bit.EM1)
    {
        if(p_foc->motor_enc.rollOverFlag && cmd_uOmodri.mode.bit.EPRE)
        {
            // rollover reached. Force a hard stop & reset structure.
            p_foc->errorMsg = FOC_ERROR_POS_ROLLOVER;
            MOT_stopCommand(&motor[MOTOR_1]);
            FOC_resetStruct(p_foc);
        }
        else
        {
            // nominal state. Compute FOC algorithm & set new PWM command
            p_foc->errorMsg = FOC_ERROR_NO_ERROR;
            FOC_runControl(p_foc);
            MOT_runCommand(&motor[MOTOR_1], p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        }
        //---------------------------------------------------------------
        //     ALIGNMENT ROUTINE
        //---------------------------------------------------------------
        if(p_foc->motor_align.state != ALIGN_DONE)
        {
            // Run alignment routine
            MOT_runAlign(&p_foc->motor_align);
            // Reset QEP position counter to set angle to zero
            if(p_foc->motor_align.state <= ALIGN_STAY)
                ENC_setPosition(&p_foc->motor_enc, 0);
            motor[MOTOR_1].interruptCnt = 0;
        }
        else
        {
            // Save number of cycles elapsed since control loop startup
            motor[MOTOR_1].clCycleNb = EPWM_getTimeBaseCounterValue(MOTOR1_PWM1_BASE);
            // Increment interrupt counter
            motor[MOTOR_1].interruptCnt++;
        }
        //-------------------------------------------------------------------
        //     SPEED & VELOCITY CONTROL
        //-------------------------------------------------------------------
        // Run PD controller
        FOC_runPD(&p_foc->pdPos);
        // Set current reference
        p_foc->iqRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (p_foc->pdPos.out) : (0.0f);
        // Id reference update
        p_foc->idRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (0.0f) : (p_foc->motor_align.idAlign);
    }
    else
    {
        p_foc->errorMsg = FOC_ERROR_NO_ERROR;
        // Don't send current to motors
        MOT_stopCommand(&motor[MOTOR_1]);
        // Reset controllers
        FOC_resetStruct(p_foc);
    }
    //-------------------------------------------------------------------//
    //                   Transmit structure functions                    //
    //-------------------------------------------------------------------//
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_M1, (uint32_t)(p_foc), 1);
    motor[MOTOR_1].interruptDone = true;
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR1_IA_INT_ACK_GROUP);
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

/**
 * @brief           ADC interrupt for MOTOR_2. Read currents \& voltages and compute control.
 */
__attribute__((interrupt)) void adc_isrMotor2(void)
{
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
    GPIO_writePin(DEBUG_INT_PIN1, 1);
#endif
    foc_t* p_foc = motor[MOTOR_2].p_motorFOC;

    // Read encoder position
    ENC_getPosition(&p_foc->motor_enc);
    // Read the phases current, voltage & Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Estimate speed
    ENC_getSpeed(&p_foc->motor_enc);
    // Rollover test & index
    ENC_getTheta(&p_foc->motor_enc);

    if(cmd_uOmodri.mode.bit.ES && cmd_uOmodri.mode.bit.EM2)
    {
        // Check for roll-over error
        if(p_foc->motor_enc.rollOverFlag && cmd_uOmodri.mode.bit.EPRE)
        {
            p_foc->errorMsg = FOC_ERROR_POS_ROLLOVER;
            MOT_stopCommand(&motor[MOTOR_2]);
            FOC_resetStruct(p_foc);
        }
        else
        {
            p_foc->errorMsg = FOC_ERROR_NO_ERROR;
            // Compute FOC algorithm
            FOC_runControl(p_foc);
            // Set new PWM command
            MOT_runCommand(&motor[MOTOR_2], p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        }
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
            motor[MOTOR_2].interruptCnt = 0;
        }
        else
        {
            // Save number of cycles elapsed since control loop startup
            motor[MOTOR_2].clCycleNb = EPWM_getTimeBaseCounterValue(MOTOR2_PWM1_BASE);
            // Increment interrupt counter
            motor[MOTOR_2].interruptCnt++;
        }
        //-------------------------------------------------------------------
        //     SPEED & VELOCITY CONTROL
        //-------------------------------------------------------------------
        // Run PD controller
        FOC_runPD(&p_foc->pdPos);
        // Set current reference
        p_foc->iqRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (p_foc->pdPos.out) : (0.0f);
        // Id reference update
        p_foc->idRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (0.0f) : (p_foc->motor_align.idAlign);
    }
    else
    {
        p_foc->errorMsg = FOC_ERROR_NO_ERROR;
        // Don't send current to motors
        MOT_stopCommand(&motor[MOTOR_2]);
        // Reset controllers
        FOC_resetStruct(p_foc);
    }
    //-------------------------------------------------------------------//
    //                   Transmit structure functions                    //
    //-------------------------------------------------------------------//
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_M2, (uint32_t)(p_foc), 1);
    motor[MOTOR_2].interruptDone = true;
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR2_IA_INT_ACK_GROUP);
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

__attribute__((interrupt)) void gpio_isrFaultMotor1(void)
{
    foc_t* p_foc = motor[MOTOR_1].p_motorFOC;

    cmd_uOmodri.mode.bit.ES = false;
    MOT_stopCommand(&motor[MOTOR_1]);
    // Reset controllers
    FOC_resetStruct(p_foc);
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

#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
__attribute__((interrupt)) void cla1_isr1(void)
{
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
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

#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
    GPIO_writePin(DEBUG_INT_PIN2, 0);
#endif
}
#endif


//while(1)
//{
//    if(HWREGH(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_TRANSFER_COUNT) == 0)
//    {
//        com_MsgExtract(p_cmdMsg[CURRENT], &cmd_uOmodri);
//        while(GPIO_ReadPin(COM_SPI_CS) == 0);
//        HWREGH(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_SOFTRESET; /* Reset DMA associated with COM SPI Tx */
//        DMA_configDestAddress(DMA_COM_SPI_RX_BASE_ADDR, (mst2slv_msg_t *)p_cmdMsg[NEXT]);
//        HWREGH(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
//        ptr_swap((int32_t**)&p_cmdMsg[CURRENT], (int32_t**)&p_cmdMsg[NEXT]);
//    }
//
//    if(motor[MOTOR_2].interruptDone)
//    {
//        com_MsgCreate(&motor_foc[MOTOR_1], &cmd_uOmodri, p_statusMsg[NEXT]);
//        com_MsgCreate(&motor_foc[MOTOR_2], &cmd_uOmodri, p_statusMsg[NEXT]);
//        while(GPIO_ReadPin(COM_SPI_CS) == 0);
//        HWREGH(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_SOFTRESET; /* Reset DMA associated with COM SPI Tx */
//        DMA_configSourceAddress(DMA_COM_SPI_TX_BASE_ADDR, (slv2mst_msg_t *)p_statusMsg[NEXT]);
//        HWREGH(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
//        ptr_swap((int32_t**)&p_statusMsg[CURRENT], (int32_t**)&p_statusMsg[NEXT]);
//        motor[MOTOR_2].interruptDone = false;
//    }
//}


//while(1)
//{
////        com_MsgExtract(p_cmdMsg[CURRENT], &cmd_uOmodri);
////        com_MsgCreate(&motor_foc[MOTOR_1], &cmd_uOmodri, p_statusMsg[NEXT]);
////        com_MsgCreate(&motor_foc[MOTOR_2], &cmd_uOmodri, p_statusMsg[NEXT]);
////
////        while(GPIO_ReadPin(COM_SPI_CS) == 0);
////
////        HWREGH(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_SOFTRESET; /* Reset DMA associated with COM SPI Tx */
////        DMA_configSourceAddress(DMA_COM_SPI_TX_BASE_ADDR, (slv2mst_msg_t *)p_statusMsg[NEXT]);
////        HWREGH(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
////        ptr_swap((int32_t**)&p_statusMsg[CURRENT], (int32_t**)&p_statusMsg[NEXT]);
//
////        if(HWREGH(DMA_COM_SPI_RX_BASE_ADDR1 + DMA_O_TRANSFER_COUNT) == 0)
//    if((!(HWREGH(DMA_COM_SPI_RX_BASE_ADDR1 + DMA_O_CONTROL) & DMA_CONTROL_RUNSTS)) && (p_cmdMsg[CURRENT] == &cmd_msg[0]))
//    {
////            com_MsgExtract(p_cmdMsg[CURRENT], &cmd_uOmodri);
//        HWREGH(DMA_COM_SPI_RX_BASE_ADDR2 + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
//        com_MsgExtract(&cmd_msg[0], &cmd_uOmodri);
////            while(GPIO_ReadPin(COM_SPI_CS) == 0);
////            HWREGH(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_SOFTRESET; /* Reset DMA associated with COM SPI Tx */
////            DMA_configDestAddress(DMA_COM_SPI_RX_BASE_ADDR, (mst2slv_msg_t *)p_cmdMsg[NEXT]);
////            HWREGH(DMA_COM_SPI_RX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
////            ptr_swap((int32_t**)&p_cmdMsg[CURRENT], (int32_t**)&p_cmdMsg[NEXT]);
//        p_cmdMsg[CURRENT]   = &cmd_msg[1];
//        p_cmdMsg[NEXT]      = &cmd_msg[0];
//    }
//    if((!(HWREGH(DMA_COM_SPI_RX_BASE_ADDR2 + DMA_O_CONTROL) & DMA_CONTROL_RUNSTS)) && (p_cmdMsg[CURRENT] == &cmd_msg[1]))
//    {
//        HWREGH(DMA_COM_SPI_RX_BASE_ADDR1 + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
//        com_MsgExtract(&cmd_msg[1], &cmd_uOmodri);
////            ptr_swap((int32_t**)&p_cmdMsg[CURRENT], (int32_t**)&p_cmdMsg[NEXT]);
//        p_cmdMsg[CURRENT]   = &cmd_msg[0];
//        p_cmdMsg[NEXT]      = &cmd_msg[1];
//    }
//
////        if(motor[MOTOR_1].interruptDone)
////        {
////            com_MsgCreate(&motor_foc[MOTOR_1], &cmd_uOmodri, p_statusMsg[NEXT]);
////            motor[MOTOR_1].interruptDone = false;
////        }
//
//    if(motor[MOTOR_2].interruptDone)
//    {
////            uint16_t i = 0;
////            uint16_t* p_status = (uint16_t *)p_statusMsg[NEXT];
////            int32_t pos = 0;//(int32_t)(cpt / POSITION_LSB);
//        com_MsgCreate(&motor_foc[MOTOR_1], &cmd_uOmodri, p_statusMsg[NEXT]);
//        com_MsgCreate(&motor_foc[MOTOR_2], &cmd_uOmodri, p_statusMsg[NEXT]);
//
//        if((HWREGH(DMA_COM_SPI_TX_BASE_ADDR1 + DMA_O_CONTROL) & DMA_CONTROL_RUNSTS) && (p_statusMsg[CURRENT] == &status_msg[0]))
//        {
//            HWREGH(DMA_COM_SPI_TX_BASE_ADDR1 + DMA_O_CONTROL) |= DMA_CONTROL_HALT;
//            HWREGH(DMA_COM_SPI_TX_BASE_ADDR2 + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
//
////                ptr_swap((int32_t**)&p_statusMsg[CURRENT], (int32_t**)&p_statusMsg[NEXT]);
////                motor[MOTOR_2].interruptDone = false;
//        }
//
//        if((HWREGH(DMA_COM_SPI_TX_BASE_ADDR2 + DMA_O_CONTROL) & DMA_CONTROL_RUNSTS) && (p_statusMsg[CURRENT] == &status_msg[1]))
//        {
//            HWREGH(DMA_COM_SPI_TX_BASE_ADDR2 + DMA_O_CONTROL) |= DMA_CONTROL_HALT;
//            HWREGH(DMA_COM_SPI_TX_BASE_ADDR1 + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
//
////                ptr_swap((int32_t**)&p_statusMsg[CURRENT], (int32_t**)&p_statusMsg[NEXT]);
////                motor[MOTOR_2].interruptDone = false;
//        }
//
//
//
////            p_statusMsg[NEXT]->status.bit.SE                = cmd_uOmodri.mode.bit.ES;
////            p_statusMsg[NEXT]->status.bit.M1E               = cmd_uOmodri.mode.bit.EM1;                                          // MOTOR_1 enabled
////            p_statusMsg[NEXT]->status.bit.M1R               = (motor[MOTOR_1].p_motorFOC->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_1 ready
////            p_statusMsg[NEXT]->status.bit.M2E               = cmd_uOmodri.mode.bit.EM2;                                          // MOTOR_2 enabled
////            p_statusMsg[NEXT]->status.bit.M2R               = (motor[MOTOR_2].p_motorFOC->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_2 ready
////            p_statusMsg[NEXT]->status.bit.IDX1T             = motor[MOTOR_1].p_motorFOC->motor_enc.indexToggle;                                 // MOTOR_1 index toggle
////            p_statusMsg[NEXT]->status.bit.IDX1D             = motor[MOTOR_1].p_motorFOC->motor_enc.indexDetected;
////            p_statusMsg[NEXT]->status.bit.IDX2T             = motor[MOTOR_2].p_motorFOC->motor_enc.indexToggle;                                 // MOTOR_2 index toggle
////            p_statusMsg[NEXT]->status.bit.IDX2D             = motor[MOTOR_2].p_motorFOC->motor_enc.indexDetected;   // MOTOR_1 index detected
////            p_statusMsg[NEXT]->status.bit.ERROR_CODE        = (motor[MOTOR_1].p_motorFOC->motor_enc.rollOverFlag) ? (STATUS_ERROR_POS_ROLLOVER) : (STATUS_ERROR_NO_ERROR);
////
////            p_statusMsg[NEXT]->position[MOTOR_1].u16_msb    = MSB_32(pos);
////            p_statusMsg[NEXT]->position[MOTOR_1].u16_lsb    = LSB_32(pos);
////            p_statusMsg[NEXT]->position[MOTOR_2].u16_msb    = MSB_32(pos);
////            p_statusMsg[NEXT]->position[MOTOR_2].u16_lsb    = LSB_32(pos);
////            p_statusMsg[NEXT]->velocity[MOTOR_1]            = (int16_t)(cpt * 10.0f / VELOCITY_LSB);
////            p_statusMsg[NEXT]->velocity[MOTOR_2]            = (int16_t)(cpt * 10.0f / VELOCITY_LSB);
////            p_statusMsg[NEXT]->current[MOTOR_1]             = (int16_t)(cpt / IQ_LSB);
////            p_statusMsg[NEXT]->current[MOTOR_2]             = (int16_t)(cpt / IQ_LSB);
////            p_statusMsg[NEXT]->coilRes[MOTOR_1]             = (uint16_t)(cpt / RESISTANCE_LSB);
////            p_statusMsg[NEXT]->coilRes[MOTOR_2]             = (uint16_t)(cpt / RESISTANCE_LSB);
////            p_statusMsg[NEXT]->adcSamples[MOTOR_1]          = (uint16_t)(cpt / VOLTAGE_LSB);
////            p_statusMsg[NEXT]->adcSamples[MOTOR_2]          = (uint16_t)(cpt / VOLTAGE_LSB);
////            p_statusMsg[NEXT]->lastCmdIndex                 = cmd_uOmodri.index;
////                // Compute CRC
////            uint32_t crc = com_crc32((uint16_t *)p_statusMsg[NEXT], DMA_COM_SPI_TX_RX_16BIT_PAYLOAD_LENGTH);
////            p_statusMsg[NEXT]->crc.u16_msb                  = LSB_32(crc);// required for Solo
////            p_statusMsg[NEXT]->crc.u16_lsb                  = MSB_32(crc);
////            cpt = (motor[MOTOR_1].p_motorFOC->motor_align.state == ALIGN_DONE) ? (cpt + 1.0f) : (0.0f);
////            cpt = (cpt < 65535.0f) ? (cpt) : (0.0f);
////            pos = (int32_t)(cpt * 1000.0f / POSITION_LSB);
//
////            while (HWREG(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL) & DMA_CONTROL_TRANSFERSTS);
//        while(GPIO_ReadPin(COM_SPI_CS) == 0);
//
////            HWREGH(COM_SPI_BASE + SPI_O_FFTX) &= ~SPI_FFTX_TXFIFO;         /* Force reset on SPI FIFO Tx pointer */
////            HWREGH(COM_SPI_BASE + SPI_O_FFTX) |= SPI_FFTX_TXFIFO;          /* Enable SPI FIFO Tx pointer */
////            HWREGH(COM_SPI_BASE + SPI_O_FFTX) &= ~SPI_FFTX_SPIRST;
////            HWREGH(COM_SPI_BASE + SPI_O_FFTX) |= SPI_FFTX_SPIRST;
//
//        //HWREGH(COM_SPI_BASE + SPI_O_DAT) = (uint16_t ) *p_status++;
//
////            for(i = 0; i < 17; i++)
////                HWREGH(COM_SPI_BASE + SPI_O_TXBUF) = (uint16_t) *p_status++;
//
////            HWREGH(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_SOFTRESET; /* Reset DMA associated with COM SPI Tx */
////            DMA_configSourceAddress(DMA_COM_SPI_TX_BASE_ADDR, (slv2mst_msg_t *)p_statusMsg[NEXT]);
////            DMA_configSourceAddress(DMA_COM_SPI_TX_BASE_ADDR, (uint16_t *)p_statusMsg[CURRENT]);
////            HWREGH(DMA_COM_SPI_TX_BASE_ADDR + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
//        ptr_swap((int32_t**)&p_statusMsg[CURRENT], (int32_t**)&p_statusMsg[NEXT]);
//        motor[MOTOR_2].interruptDone = false;
//    }
//}
