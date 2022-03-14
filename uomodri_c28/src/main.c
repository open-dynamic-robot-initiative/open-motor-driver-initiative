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
    uint16_t* p_cmdMsg          = (uint16_t *)&cmd_msg;
    uint16_t* p_statusMsg       = (uint16_t *)&status_msg;
    uint16_t* p_ledMsg          = (uint16_t *)&led_msg[0];
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
    // Force a reset of FOC structures
    FOC_resetStruct(motor[MOTOR_1].p_motorFOC);
    FOC_resetStruct(motor[MOTOR_2].p_motorFOC);
    COM_resetCmdStruct(&cmd_uOmodri);
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
                COM_msgExtract(&cmd_msg, &cmd_uOmodri);
            if(GPIO_ReadPin(COM_SPI_CS))
            {
                spi_com_rx_loop = 0;
                p_cmdMsg        = (uint16_t *)&cmd_msg;
                SPI_resetRxFIFO(COM_SPI_BASE);
            }
        }
        // SPI COM TX management - Create message & send it
        if(motor[MOTOR_1].itDone && motor[MOTOR_2].itDone)
        {
            COM_msgCreate(&motor_foc[MOTOR_1], &cmd_uOmodri, &status_msg);
            COM_msgCreate(&motor_foc[MOTOR_2], &cmd_uOmodri, &status_msg);
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
            motor[MOTOR_1].itDone   = false;
            motor[MOTOR_2].itDone   = false;
        }
        // SPI LED TX management - Create message & send it @ 1ms
        if(CPUTimer_getTimerOverflowStatus(CPU_TIMER_0_BASE))
        {
            float32_t timer_fade_blink  = (float32_t)((timer_cpt_ms < 500)  ? (timer_cpt_ms)    : (1000.0 - timer_cpt_ms)) / 500.0f;
            float32_t timer_blink       = (timer_cpt_ms < 500)              ? (0.0f)            : (1.0f);
            timer_cpt_ms                = (timer_cpt_ms < 1000)             ? (timer_cpt_ms + 1): (0);
            spi_led_tx_loop             = 0;
            p_ledMsg                    = (uint16_t *)&led_msg[0];
            switch(motor_foc[MOTOR_1].motor_state[CURRENT])
            {
            case MOTOR_ALIGN_UP:
            case MOTOR_ALIGN_FIX:
            case MOTOR_ALIGN_DONE:
                ws2812b_writeLed((uint16_t)(0xff * timer_fade_blink), 0, (uint16_t)(0xff * timer_fade_blink), p_ledMsg); //Magenta fade,
                break;
            case MOTOR_READY:
            case MOTOR_STOP:
                ws2812b_writeLed(0, (uint16_t)(0xff * timer_fade_blink), 0, p_ledMsg);//Green fade, Active control
                break;
            case MOTOR_ERROR:
                ws2812b_writeLed((uint16_t)(0xff * timer_blink), 0, 0, p_ledMsg);//red fade, error control
                break;
            case MOTOR_INIT:
            default:
                ws2812b_writeLed(0,  0, (uint16_t)(0xff * timer_fade_blink),p_ledMsg);//blue fade
            }

            while((SPI_getTxFIFOStatus(LED_SPI_BASE) != SPI_FIFO_TXFULL) && (spi_led_tx_loop++ < LED_MSG_TX_16BIT_LENGTH))
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
    GPIO_writePin(DBG_PIN1, 1);
    motor_t*    p_motor         = &motor[MOTOR_1];
    foc_t*      p_foc           = p_motor->p_motorFOC;

    // Read encoder position
    ENC_getPosition(&p_foc->motor_enc);
    // Read the phases current, voltage & Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Estimate speed
    ENC_getSpeed(&p_foc->motor_enc);
    // Rollover test & index
    ENC_getTheta(&p_foc->motor_enc);

    bool_t system_disable_flag  = !cmd_uOmodri.mode.bit.ES;
    bool_t motor_disable_flag   = !cmd_uOmodri.mode.bit.EM1;
    bool_t motor_rollover_flag  = p_foc->motor_enc.rollOverFlag && cmd_uOmodri.mode.bit.EPRE;
    bool_t motor_error_flag     = (p_foc->errorStatus & FOC_ERROR_ALL_ERROR_FLAGS) ? (true) : (false);
    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->p_motorHalCfg->p_pwmCntCmp[0]->epwmBase);//MOTOR1_PWM1_REGS->TBCTR;
    p_foc->motor_state[CURRENT] = p_foc->motor_state[NEXT];

    switch(p_foc->motor_state[CURRENT])
    {
    case MOTOR_INIT:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->errorStatus     &= ~(FOC_ERROR_ENCODER | FOC_ERROR_POS_ROLLOVER);
        p_foc->motor_state[NEXT]= (motor_disable_flag || system_disable_flag)   ? (MOTOR_INIT)  : (MOTOR_ALIGN_UP);
        p_foc->motor_state[NEXT]= (motor_error_flag)                            ? (MOTOR_ERROR) : (p_foc->motor_state[NEXT]);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_ALIGN_UP:
        p_motor->itCnt          = 0U;
        p_foc->idRef           += 1e-5f;
        p_foc->iqRef            = 0.0f;
        p_foc->motor_state[NEXT]= (p_foc->idRef < MOTOR1_CURRENT_ALIGN_MAX)     ? (MOTOR_ALIGN_UP)  : (MOTOR_ALIGN_FIX);
        p_foc->motor_state[NEXT]= (motor_disable_flag || system_disable_flag)   ? (MOTOR_INIT)      : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)                            ? (MOTOR_ERROR)     : (p_foc->motor_state[NEXT]);
        ENC_resetStruct(&p_foc->motor_enc);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_ALIGN_FIX:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = MOTOR1_CURRENT_ALIGN_MAX; // Maintain alignment current
        p_foc->iqRef            = 0.0f;
        p_foc->motor_state[NEXT]= (p_motor->itCnt < PWM_FREQ)                   ? (MOTOR_ALIGN_FIX) : (MOTOR_ALIGN_DONE); // wait 1s
        p_foc->motor_state[NEXT]= (motor_disable_flag || system_disable_flag)   ? (MOTOR_INIT)      : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)                            ? (MOTOR_ERROR)     : (p_foc->motor_state[NEXT]);
        ENC_resetStruct(&p_foc->motor_enc);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_ALIGN_DONE:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->motor_state[NEXT]= (motor_disable_flag)  ? (MOTOR_STOP)  : (MOTOR_READY);
        p_foc->motor_state[NEXT]= (system_disable_flag) ? (MOTOR_INIT)  : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)    ? (MOTOR_ERROR) : (p_foc->motor_state[NEXT]);
        ENC_resetStruct(&p_foc->motor_enc);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPos);
        p_foc->errorStatus     |= (motor_rollover_flag) ? (FOC_ERROR_POS_ROLLOVER)  : (p_foc->errorStatus);
        p_foc->motor_state[NEXT]= (motor_disable_flag)  ? (MOTOR_STOP)              : (MOTOR_READY);
        p_foc->motor_state[NEXT]= (system_disable_flag) ? (MOTOR_INIT)              : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)    ? (MOTOR_ERROR)             : (p_foc->motor_state[NEXT]);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->errorStatus     |= (motor_rollover_flag) ? (FOC_ERROR_POS_ROLLOVER)  : (p_foc->errorStatus);
        p_foc->motor_state[NEXT]= (motor_disable_flag)  ? (MOTOR_STOP)              : (MOTOR_READY);
        p_foc->motor_state[NEXT]= (system_disable_flag) ? (MOTOR_INIT)              : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)    ? (MOTOR_ERROR)             : (p_foc->motor_state[NEXT]);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_ERROR:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->errorStatus     |= (motor_rollover_flag) ? (FOC_ERROR_POS_ROLLOVER) : (p_foc->errorStatus);
        p_foc->motor_state[NEXT]= MOTOR_ERROR;
        COM_resetCmdMotorStruct(&cmd_uOmodri, p_foc->motor_id);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

//    default:
//        p_motor->itCnt          = 0U;
//        p_foc->idRef            = 0.0f;
//        p_foc->iqRef            = 0.0f;
//        p_foc->errorStatus     &= ~(FOC_ERROR_ENCODER | FOC_ERROR_POS_ROLLOVER);
//        p_foc->motor_state[NEXT]= (motor_disable_flag || system_disable_flag)   ? (MOTOR_INIT)  : (MOTOR_ALIGN_UP);
//        p_foc->motor_state[NEXT]= (motor_error_flag)                            ? (MOTOR_ERROR) : (p_foc->motor_state[NEXT]);
//        ENC_resetPeriph(&p_foc->motor_enc);
//        FOC_resetStruct(p_foc);
//        MOT_stopCommand(p_motor);
    }

//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_M1, (uint32_t)(p_foc), 1);
    p_motor->itDone = true;
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR1_IA_INT_ACK_GROUP);
    GPIO_writePin(DBG_PIN1, 0);
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
    GPIO_writePin(DBG_PIN1, 1);
    motor_t*    p_motor         = &motor[MOTOR_2];
    foc_t*      p_foc           = p_motor->p_motorFOC;

    // Read encoder position
    ENC_getPosition(&p_foc->motor_enc);
    // Read the phases current, voltage & Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Estimate speed
    ENC_getSpeed(&p_foc->motor_enc);
    // Rollover test & index
    ENC_getTheta(&p_foc->motor_enc);

    bool_t system_disable_flag  = !cmd_uOmodri.mode.bit.ES;
    bool_t motor_disable_flag   = !cmd_uOmodri.mode.bit.EM2;
    bool_t motor_rollover_flag  = p_foc->motor_enc.rollOverFlag && cmd_uOmodri.mode.bit.EPRE;
    bool_t motor_error_flag     = (p_foc->errorStatus & FOC_ERROR_ALL_ERROR_FLAGS) ? (true) : (false);
    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->p_motorHalCfg->p_pwmCntCmp[0]->epwmBase);
    p_foc->motor_state[CURRENT] = p_foc->motor_state[NEXT];

    switch(p_foc->motor_state[CURRENT])
    {
    case MOTOR_INIT:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->errorStatus     &= ~(FOC_ERROR_ENCODER | FOC_ERROR_POS_ROLLOVER);
        p_foc->motor_state[NEXT]= (motor_disable_flag || system_disable_flag)   ? (MOTOR_INIT)  : (MOTOR_ALIGN_UP);
        p_foc->motor_state[NEXT]= (motor_error_flag)                            ? (MOTOR_ERROR) : (p_foc->motor_state[NEXT]);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_ALIGN_UP:
        p_motor->itCnt          = 0U;
        p_foc->idRef           += 1e-5f;
        p_foc->iqRef            = 0.0f;
        p_foc->motor_state[NEXT]= (p_foc->idRef < MOTOR2_CURRENT_ALIGN_MAX)     ? (MOTOR_ALIGN_UP)  : (MOTOR_ALIGN_FIX);
        p_foc->motor_state[NEXT]= (motor_disable_flag || system_disable_flag)   ? (MOTOR_INIT)      : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)                            ? (MOTOR_ERROR)     : (p_foc->motor_state[NEXT]);
        ENC_resetStruct(&p_foc->motor_enc);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_ALIGN_FIX:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = MOTOR2_CURRENT_ALIGN_MAX; // Maintain alignment current
        p_foc->iqRef            = 0.0f;
        p_foc->motor_state[NEXT]= (p_motor->itCnt < PWM_FREQ)                   ? (MOTOR_ALIGN_FIX) : (MOTOR_ALIGN_DONE); // wait 1s
        p_foc->motor_state[NEXT]= (motor_disable_flag || system_disable_flag)   ? (MOTOR_INIT)      : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)                            ? (MOTOR_ERROR)     : (p_foc->motor_state[NEXT]);
        ENC_resetStruct(&p_foc->motor_enc);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_ALIGN_DONE:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->motor_state[NEXT]= (motor_disable_flag)  ? (MOTOR_STOP)  : (MOTOR_READY);
        p_foc->motor_state[NEXT]= (system_disable_flag) ? (MOTOR_INIT)  : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)    ? (MOTOR_ERROR) : (p_foc->motor_state[NEXT]);
        ENC_resetStruct(&p_foc->motor_enc);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPos);
        p_foc->errorStatus     |= (motor_rollover_flag) ? (FOC_ERROR_POS_ROLLOVER)  : (p_foc->errorStatus);
        p_foc->motor_state[NEXT]= (motor_disable_flag)  ? (MOTOR_STOP)              : (MOTOR_READY);
        p_foc->motor_state[NEXT]= (system_disable_flag) ? (MOTOR_INIT)              : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)    ? (MOTOR_ERROR)             : (p_foc->motor_state[NEXT]);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->errorStatus     |= (motor_rollover_flag) ? (FOC_ERROR_POS_ROLLOVER)  : (p_foc->errorStatus);
        p_foc->motor_state[NEXT]= (motor_disable_flag)  ? (MOTOR_STOP)              : (MOTOR_READY);
        p_foc->motor_state[NEXT]= (system_disable_flag) ? (MOTOR_INIT)              : (p_foc->motor_state[NEXT]);
        p_foc->motor_state[NEXT]= (motor_error_flag)    ? (MOTOR_ERROR)             : (p_foc->motor_state[NEXT]);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_ERROR:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->errorStatus     |= (motor_rollover_flag) ? (FOC_ERROR_POS_ROLLOVER) : (p_foc->errorStatus);
        p_foc->motor_state[NEXT]= MOTOR_ERROR;
        COM_resetCmdMotorStruct(&cmd_uOmodri, p_foc->motor_id);
        ENC_resetPeriph(&p_foc->motor_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

//    default:
//        p_motor->itCnt          = 0U;
//        p_foc->idRef            = 0.0f;
//        p_foc->iqRef            = 0.0f;
//        p_foc->errorStatus     &= ~(FOC_ERROR_ENCODER | FOC_ERROR_POS_ROLLOVER);
//        p_foc->motor_state[NEXT]= (motor_disable_flag || system_disable_flag)   ? (MOTOR_INIT)  : (MOTOR_ALIGN_UP);
//        p_foc->motor_state[NEXT]= (motor_error_flag)                            ? (MOTOR_ERROR) : (p_foc->motor_state[NEXT]);
//        ENC_resetPeriph(&p_foc->motor_enc);
//        FOC_resetStruct(p_foc);
//        MOT_stopCommand(p_motor);
    }

//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_M1, (uint32_t)(p_foc), 1);
    p_motor->itDone = true;
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR2_IA_INT_ACK_GROUP);
    GPIO_writePin(DBG_PIN1, 0);

//    foc_t* p_foc = motor[MOTOR_2].p_motorFOC;
//
//    // Read encoder position
//    ENC_getPosition(&p_foc->motor_enc);
//    // Read the phases current, voltage & Vbus
//    FOC_getMeasures(&p_foc->motor_acq);
//    // Estimate speed
//    ENC_getSpeed(&p_foc->motor_enc);
//    // Rollover test & index
//    ENC_getTheta(&p_foc->motor_enc);
//
//    if(cmd_uOmodri.mode.bit.ES && cmd_uOmodri.mode.bit.EM2)
//    {
//        // Check for roll-over error
//        if(p_foc->motor_enc.rollOverFlag && cmd_uOmodri.mode.bit.EPRE)
//        {
//            p_foc->errorStatus = FOC_ERROR_POS_ROLLOVER;
//            MOT_stopCommand(&motor[MOTOR_2]);
//            FOC_resetStruct(p_foc);
//        }
//        else
//        {
//            p_foc->errorStatus = FOC_ERROR_NO_ERROR;
//            // Compute FOC algorithm
//            FOC_runControl(p_foc);
//            // Set new PWM command
//            MOT_runCommand(&motor[MOTOR_2], p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
//        }
//        //-------------------------------------------------------------------
//        //     ALIGNMENT ROUTINE
//        //-------------------------------------------------------------------
//        if(p_foc->motor_align.state != ALIGN_DONE)
//        {
//            // Run alignment routine
//            MOT_runAlign(&p_foc->motor_align);
//            // Reset QEP position counter to set angle to zero
//            if(p_foc->motor_align.state <= ALIGN_STAY)
//                ENC_setPosition(&p_foc->motor_enc, 0);
//            motor[MOTOR_2].itCnt = 0;
//        }
//        else
//        {
//            // Save number of cycles elapsed since control loop startup
//            motor[MOTOR_2].clCycleNb = EPWM_getTimeBaseCounterValue(MOTOR2_PWM1_BASE);
//            // Increment interrupt counter
//            motor[MOTOR_2].itCnt++;
//        }
//        //-------------------------------------------------------------------
//        //     SPEED & VELOCITY CONTROL
//        //-------------------------------------------------------------------
//        // Run PD controller
//        FOC_runPD(&p_foc->pdPos);
//        // Set current reference
//        p_foc->iqRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (p_foc->pdPos.out) : (0.0f);
//        // Id reference update
//        p_foc->idRef    = (p_foc->motor_align.state == ALIGN_DONE) ? (0.0f) : (p_foc->motor_align.idAlign);
//    }
//    else
//    {
//        p_foc->errorStatus = FOC_ERROR_NO_ERROR;
//        // Don't send current to motors
//        MOT_stopCommand(&motor[MOTOR_2]);
//        // Reset controllers
//        FOC_resetStruct(p_foc);
//    }
//    //-------------------------------------------------------------------//
//    //                   Transmit structure functions                    //
//    //-------------------------------------------------------------------//
////    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_M2, (uint32_t)(p_foc), 1);
//    motor[MOTOR_2].itDone = true;
//    // Clear the interrupt overflow flag
//    ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);
//    // Clear the interrupt flag
//    ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);
//    // Acknowledge PWM and CLA interrupt flag
//    Interrupt_clearACKGroup(MOTOR2_IA_INT_ACK_GROUP);
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
    GPIO_writePin(DEBUG_INT_PIN1, 0);
#endif
}

/**
 * @brief           DRV fault detection interrupt for MOTOR_1. Force a hard stop.
 */
__attribute__((interrupt)) void gpio_isrFaultMotor1(void)
{
    motor_t*    p_motor         = &motor[MOTOR_1];
    foc_t*      p_foc           = p_motor->p_motorFOC;

    cmd_uOmodri.mode.bit.EM1    = false;
    p_foc->errorStatus          = FOC_ERROR_DRV_NFAULT;
    p_foc->motor_state[NEXT]    = MOTOR_ERROR;

    COM_resetCmdMotorStruct(&cmd_uOmodri, p_foc->motor_id);
    FOC_resetStruct(p_foc);
    MOT_stopCommand(p_motor);
    // Read DRV data to get status registers
    DRV_readStatus(p_motor->p_motorDRV);

    // Infinite loop
    while(1);

}

/**
 * @brief           DRV fault detection interrupt for MOTOR_2. Force a hard stop.
 */
__attribute__((interrupt)) void gpio_isrFaultMotor2(void)
{
    motor_t*    p_motor         = &motor[MOTOR_2];
    foc_t*      p_foc           = p_motor->p_motorFOC;

    cmd_uOmodri.mode.bit.EM2    = false;
    p_foc->errorStatus          = FOC_ERROR_DRV_NFAULT;
    p_foc->motor_state[NEXT]    = MOTOR_ERROR;

    COM_resetCmdMotorStruct(&cmd_uOmodri, p_foc->motor_id);
    FOC_resetStruct(p_foc);
    MOT_stopCommand(p_motor);
    // Read DRV data to get status registers
    DRV_readStatus(p_motor->p_motorDRV);


//    cmd_uOmodri.mode.bit.ES = false;
//    MOT_stopCommand(&motor[MOTOR_2]);
//    // Reset controllers
//    FOC_resetStruct(motor[MOTOR_2].p_motorFOC);
//    // Trigger emergency stop
////    MOT_emergencyStop(&motor[MOTOR_2]);
//
//    // Read DRV data to get status registers
//    DRV_readStatus(motor[MOTOR_2].p_motorDRV);
////    HAL_readDRVData(&hal_motors[0].drvHandle, &drvVars[0]);



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
