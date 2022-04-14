/*
 * File name: main.c
 * Description: Source file containing main function and IT running on CPU1 - C28 processor.
 */

/***********************************************************************
 * INCLUDES
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
 * FUNCTIONS DEFINITION
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
    // Force a reset of FOC structures
    FOC_resetStruct(motor[MOTOR_1].p_motorFOC);
    FOC_resetStruct(motor[MOTOR_2].p_motorFOC);
    COM_resetCmdStruct(&cmd_uOmodri);
    // Interrupts initialization
    HAL_INT_ini(InterruptList);
//    // Send translation address for status message structure
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, STATUS_MSG, (uint32_t)&status_msg.status, 1);
//    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send translation address for status message structure
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, POS_M1_LSB, (uint32_t)&status_msg.position[MOTOR_1].u16_lsb, 1);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send translation address for status message structure
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, POS_M1_MSB, (uint32_t)&status_msg.position[MOTOR_1].u16_msb, 1);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send translation address for uOmodri command structure
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, CMD_IQREF, (uint32_t)&cmd_uOmodri.iqRef[MOTOR_1], 1);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send translation address for M1 FOC motor structure
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, FOC_IQ_SENS, (uint32_t)&motor_foc[MOTOR_1].iq, 1);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send translation address for M1 FOC motor structure
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, FOC_ID_SENS, (uint32_t)&motor_foc[MOTOR_1].id, 1);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send translation address for M1 FOC motor structure
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, FOC_DTC_U, (uint32_t)&motor_foc[MOTOR_1].dtc_u, 1);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send translation address for M1 FOC motor structure
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, FOC_DTC_V, (uint32_t)&motor_foc[MOTOR_1].dtc_v, 1);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send translation address for M1 FOC motor structure
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, FOC_DTC_W, (uint32_t)&motor_foc[MOTOR_1].dtc_w, 1);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
//    // Send translation address for M1 FOC motor structure
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, FOC_FSM_M2, (uint32_t)&motor_foc[MOTOR_2].motor_state, 1);
//    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    // Infinite loop
    while(1)
    {
        // SPI COM RX management - Receive & extract message
        if(!DMA_getRunStatusFlag(DMA_COM_SPI_RX_BASE_ADDR) && GPIO_readPin(COM_SPI_CS))
        {
            COM_msgExtract(&cmd_msg, &cmd_uOmodri);
            DMA_clearTriggerFlag(DMA_COM_SPI_RX_BASE_ADDR);// DMA_triggerSoftReset(DMA_COM_SPI_RX_BASE_ADDR);
            SPI_resetRxFIFO(COM_SPI_BASE);
            DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
        }
        // SPI COM TX management - Create message & send it
        if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && motor[MOTOR_2].itDone)// && motor[MOTOR_1].itDone)
        {
            COM_msgCreate(&motor_foc[MOTOR_1], &cmd_uOmodri, &status_msg);
            COM_msgCreate(&motor_foc[MOTOR_2], &cmd_uOmodri, &status_msg);
            DMA_clearTriggerFlag(DMA_COM_SPI_TX_BASE_ADDR);
            if(GPIO_readPin(COM_SPI_CS))
            {
                // Force first write in SPI_O_DAT
                HWREGH(COM_SPI_BASE + SPI_O_DAT) = (uint16_t) status_msg.status.all;// DMA_triggerSoftReset(DMA_COM_SPI_TX_BASE_ADDR);
                SPI_resetTxFIFO(COM_SPI_BASE);
                DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
            }
            motor[MOTOR_1].itDone   = false;
            motor[MOTOR_2].itDone   = false;
            if(!IPC_isFlagBusyLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG))
                IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG);
//            IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_DISABLE, foc_m1_addr, foc_m2_addr, status_msg_addr);
        }
        /*
         * SPI LED TX management - Create message & send it @ 1ms
         * SPI DRV TX/RX management - Read/Write to get DRV status in case of fault @ 1ms
         */
        if(CPUTimer_getTimerOverflowStatus(CPU_TIMER_0_BASE))
        {
            // LED dimming control
            if(!DMA_getRunStatusFlag(DMA_LED_SPI_TX_BASE_ADDR))
            {
                static uint16_t timer_cpt_ms    = 0;
                float32_t       timer_fade_blink= (float32_t)((timer_cpt_ms < 500)  ? (timer_cpt_ms)    : (1000 - timer_cpt_ms)) / 500.0f;
                float32_t       timer_blink     = (timer_cpt_ms < 500)              ? (0.0f)            : (1.0f);
                timer_cpt_ms                    = (timer_cpt_ms < 1000)             ? (timer_cpt_ms + 1): (0);
                switch(motor_foc[MOTOR_1].motor_state)
                {
                case MOTOR_STATE_ALIGN_UP:
                case MOTOR_STATE_ALIGN_FIX:
                    ws2812b_writeLed((uint16_t)(0xff * timer_fade_blink), 0, (uint16_t)(0xff * timer_fade_blink), &led_msg[0]); //Magenta fade,
                    break;
                case MOTOR_STATE_READY:
                case MOTOR_STATE_STOP:
                    ws2812b_writeLed(0, (uint16_t)(0xff * timer_fade_blink), 0, &led_msg[0]);//Green fade, Active control
                    break;
                case MOTOR_STATE_ERROR:
                    ws2812b_writeLed((uint16_t)(0xff * timer_blink), 0, 0, &led_msg[0]);//red fade, error control
                    break;
                case MOTOR_STATE_INIT:
                default:
                    ws2812b_writeLed(0,  0, (uint16_t)(0xff * timer_fade_blink), &led_msg[0]);//blue fade
                }
                DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
            }
            // Read DRV data to get status registers (MOTOR_1)
            if(motor_foc[MOTOR_1].motor_error & MOTOR_ERROR_DRV_NFAULT)
                DRV_readStatus(motor[MOTOR_1].p_motorDRV);
            // Read DRV data to get status registers (MOTOR_2)
            if(motor_foc[MOTOR_2].motor_error & MOTOR_ERROR_DRV_NFAULT)
                DRV_readStatus(motor[MOTOR_2].p_motorDRV);
            // Timeout management
            cmd_uOmodri.cptTimeout++;
            // Clear flag
            CPUTimer_clearOverflowFlag(CPU_TIMER_0_BASE);
        }
        // Force a general microcontroller RESET if requested
        if((motor_foc[MOTOR_1].motor_state == MOTOR_STATE_ERROR) &&
           (motor_foc[MOTOR_2].motor_state == MOTOR_STATE_ERROR) && (cmd_uOmodri.mode.bit.RST))// (cmd_uOmodri.mode.bit.RST == 0b11))
        {
            SysCtl_simulateReset(SYSCTL_CAUSE_CPU1RSN); //SysCtl_resetDevice();
            while(1);
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
    /* Initialize clock and peripherals */
    Device_init();
    /* Set up the auxiliary PLL to 125MHz for CM use. */
//    SysCtl_setAuxClock(SYSCTL_AUXPLL_OSCSRC_XTAL | SYSCTL_AUXPLL_IMULT(50) | SYSCTL_REFDIV(2U) |
//                       SYSCTL_ODIV(5U) | SYSCTL_AUXPLL_DIV_2 | SYSCTL_AUXPLL_ENABLE | SYSCTL_DCC_BASE_0);
    /* Set the CM Clock to run at 125MHz.
     * The CM Clock is a fractional multiple of the AUXPLL Clock (125 MHz). */
#if (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE)
//    SysCtl_setCMClk(SYSCTL_CMCLKOUT_DIV_1, SYSCTL_SOURCE_AUXPLL);
    /* Allocate Shared Peripheral USB to the CM Side. */
//    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
    /* Boot CM core */
#if (defined _FLASH)
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif
#endif
    // Initialize GPIO
    Device_initGPIO();
    // Initialize PIE control registers and flags. Disables CPU interrupts.
    Interrupt_initModule();
    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    Interrupt_initVectorTable();
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
#ifdef DEBUG
    GPIO_writePin(DBG_PIN1, 1);
#endif
    motor_t*    p_motor         = &motor[MOTOR_1];
    foc_t*      p_foc           = p_motor->p_motorFOC;
    encoder_t*  p_enc           = &p_foc->motor_enc;
    uint16_t*   p_state         = &p_foc->motor_state;
    uint8_t     timeout         = (uint8_t)cmd_uOmodri.mode.bit.TIMEOUT;
    bool_t system_disable_flag  = !cmd_uOmodri.mode.bit.ES;
    bool_t motor_disable_flag   = !cmd_uOmodri.mode.bit.EM1;
    p_enc->indexCompensated     = cmd_uOmodri.mode.bit.EI1OC;

    // Read encoder position
    ENC_getPosition(&p_foc->motor_enc);
    // Read the phases current, voltage & Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Estimate speed
    ENC_getSpeed(&p_foc->motor_enc);
    // Rollover test & index
    ENC_getTheta(&p_foc->motor_enc);

    bool_t motor_rollover_flag  = p_foc->motor_enc.rollOverFlag && cmd_uOmodri.mode.bit.EPRE;
    bool_t motor_drv_fault_flag = (!GPIO_readPin(MOTOR1_DRV_GPIO_NFAULT))           ? (true)    : (false);
    bool_t com_timeout_flag     = ((cmd_uOmodri.cptTimeout > timeout) && (timeout)) ? (true)    : (false);
    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->p_motorHalCfg->p_pwmCntCmp[0]->epwmBase);
    p_foc->motor_error         |= (motor_rollover_flag)                             ? (MOTOR_ERROR_POS_ROLLOVER): (p_foc->motor_error);
    p_foc->motor_error         |= (motor_drv_fault_flag)                            ? (MOTOR_ERROR_DRV_NFAULT)  : (p_foc->motor_error);
    p_foc->motor_error         |= (com_timeout_flag)                                ? (MOTOR_ERROR_COM_TIMEOUT) : (p_foc->motor_error);
    *p_state                    = (p_foc->motor_error || cmd_uOmodri.mode.bit.RST)  ? (MOTOR_STATE_ERROR)       : (*p_state);

    switch(*p_state)
    {
    case MOTOR_STATE_INIT:
    default:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
//        p_foc->errorStatus     &= ~(FOC_ERROR_ENCODER | FOC_ERROR_POS_ROLLOVER);
        *p_state                = (motor_disable_flag || system_disable_flag)   ? (MOTOR_STATE_INIT)        : (MOTOR_STATE_ALIGN_UP);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ALIGN_UP:
        p_motor->itCnt          = 0U;
        p_foc->idRef           += 1e-5f;
        p_foc->iqRef            = 0.0f;
        *p_state                = (p_foc->idRef < MOTOR1_CURRENT_ALIGN_MAX)     ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_ALIGN_FIX);
        *p_state                = (motor_disable_flag || system_disable_flag)   ? (MOTOR_STATE_INIT)        : (*p_state);
        ENC_resetStruct(p_enc);
        ENC_resetPeriph(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ALIGN_FIX:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = MOTOR1_CURRENT_ALIGN_MAX; // Maintain alignment current
        p_foc->iqRef            = 0.0f;
        *p_state                = (p_motor->itCnt < (2 * PWM_FREQ))             ? (MOTOR_STATE_ALIGN_FIX)   : (MOTOR_STATE_READY);//(MOTOR_ALIGN_DONE); // wait 1s
        *p_state                = (motor_disable_flag || system_disable_flag)   ? (MOTOR_STATE_INIT)        : (*p_state);
        ENC_resetStruct(p_enc);
        ENC_resetPeriph(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPos);
        *p_state                = (motor_disable_flag)  ? (MOTOR_STATE_STOP)    : (MOTOR_STATE_READY);
        *p_state                = (system_disable_flag) ? (MOTOR_STATE_INIT)    : (*p_state);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        *p_state                = (motor_disable_flag)  ? (MOTOR_STATE_STOP)    : (MOTOR_STATE_READY);
        *p_state                = (system_disable_flag) ? (MOTOR_STATE_INIT)    : (*p_state);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ERROR:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        *p_state                = MOTOR_STATE_ERROR;
        COM_resetCmdMotorStruct(p_foc, &cmd_uOmodri);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;
    }
//#if (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE)
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_M1, (uint32_t)(p_foc), 1);
//#endif
    p_motor->itDone = true;
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR1_IA_INT_ACK_GROUP);
#ifdef DEBUG
    GPIO_writePin(DBG_PIN1, 0);
#endif
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
#ifdef DEBUG
    GPIO_writePin(DBG_PIN1, 1);
#endif
    motor_t*    p_motor         = &motor[MOTOR_2];
    foc_t*      p_foc           = p_motor->p_motorFOC;
    encoder_t*  p_enc           = &p_foc->motor_enc;
    uint16_t*   p_state         = &p_foc->motor_state;
    uint8_t     timeout         = (uint8_t)cmd_uOmodri.mode.bit.TIMEOUT;
    bool_t system_disable_flag  = !cmd_uOmodri.mode.bit.ES;
    bool_t motor_disable_flag   = !cmd_uOmodri.mode.bit.EM2;
    p_enc->indexCompensated     = cmd_uOmodri.mode.bit.EI2OC;

    // Read encoder position
    ENC_getPosition(&p_foc->motor_enc);
    // Read the phases current, voltage & Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Estimate speed
    ENC_getSpeed(&p_foc->motor_enc);
    // Rollover test & index
    ENC_getTheta(&p_foc->motor_enc);

    bool_t motor_rollover_flag  = p_foc->motor_enc.rollOverFlag && cmd_uOmodri.mode.bit.EPRE;
    bool_t motor_drv_fault_flag = (!GPIO_readPin(MOTOR2_DRV_GPIO_NFAULT))           ? (true)    : (false);
    bool_t com_timeout_flag     = ((cmd_uOmodri.cptTimeout > timeout) && (timeout)) ? (true)    : (false);
    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->p_motorHalCfg->p_pwmCntCmp[0]->epwmBase);
    p_foc->motor_error         |= (motor_rollover_flag)                             ? (MOTOR_ERROR_POS_ROLLOVER): (p_foc->motor_error);
    p_foc->motor_error         |= (motor_drv_fault_flag)                            ? (MOTOR_ERROR_DRV_NFAULT)  : (p_foc->motor_error);
    p_foc->motor_error         |= (com_timeout_flag)                                ? (MOTOR_ERROR_COM_TIMEOUT) : (p_foc->motor_error);
    *p_state                    = (p_foc->motor_error || cmd_uOmodri.mode.bit.RST)  ? (MOTOR_STATE_ERROR)       : (*p_state);

    switch(*p_state)
    {
    case MOTOR_STATE_INIT:
    default:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
//        p_foc->errorStatus     &= ~(FOC_ERROR_ENCODER | FOC_ERROR_POS_ROLLOVER);
        *p_state                = (motor_disable_flag || system_disable_flag)   ? (MOTOR_STATE_INIT)        : (MOTOR_STATE_ALIGN_UP);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ALIGN_UP:
        p_motor->itCnt          = 0U;
        p_foc->idRef           += 1e-5f;
        p_foc->iqRef            = 0.0f;
        *p_state                = (p_foc->idRef < MOTOR2_CURRENT_ALIGN_MAX)     ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_ALIGN_FIX);
        *p_state                = (motor_disable_flag || system_disable_flag)   ? (MOTOR_STATE_INIT)        : (*p_state);
        ENC_resetStruct(p_enc);
        ENC_resetPeriph(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ALIGN_FIX:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = MOTOR2_CURRENT_ALIGN_MAX; // Maintain alignment current
        p_foc->iqRef            = 0.0f;
        *p_state                = (p_motor->itCnt < (2 * PWM_FREQ))             ? (MOTOR_STATE_ALIGN_FIX)   : (MOTOR_STATE_READY);//(MOTOR_ALIGN_DONE); // wait 1s
        *p_state                = (motor_disable_flag || system_disable_flag)   ? (MOTOR_STATE_INIT)        : (*p_state);
        ENC_resetStruct(p_enc);
        ENC_resetPeriph(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPos);
        *p_state                = (motor_disable_flag)  ? (MOTOR_STATE_STOP)    : (MOTOR_STATE_READY);
        *p_state                = (system_disable_flag) ? (MOTOR_STATE_INIT)    : (*p_state);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        *p_state                = (motor_disable_flag)  ? (MOTOR_STATE_STOP)    : (MOTOR_STATE_READY);
        *p_state                = (system_disable_flag) ? (MOTOR_STATE_INIT)    : (*p_state);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ERROR:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        *p_state                = MOTOR_STATE_ERROR;
        COM_resetCmdMotorStruct(p_foc, &cmd_uOmodri);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;
    }
//#if (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE)
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_M2, (uint32_t)(p_foc), 1);
//#endif
    p_motor->itDone = true;
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR2_IA_INT_ACK_GROUP);
#ifdef DEBUG
    GPIO_writePin(DBG_PIN1, 0);
#endif
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}
