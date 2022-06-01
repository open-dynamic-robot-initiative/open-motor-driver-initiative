/*
 * File name: main.c
 * Description: Source file containing main function and IT running on CPU1 - C28 processor.
 */

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "main.h"
#include "hal.h"
#include "drv8353.h"
#include "ws2812b.h"
#include "encoder.h"
#include "foc.h"
#include "motor.h"
#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"
#include "uomodri_prj_config_handlers.h"

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
extern motor_t          motor[];
extern const hal_cfg_t  hal;
extern cmd_uomodri_t    cmd_uOmodri;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
static void MCU_ini(void);
inline void PTR_swap(int32_t**, int32_t**);
inline void LED_dimming(motor_t*, uint16_t*);

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

    // Compute translation addresses for each element we want to share.
    uint16_t    cpu1_to_cm_loop = 0;
    for(cpu1_to_cm_loop = 0; cpu1_to_cm_loop < (sizeof(cpu1_to_cm_addr) / sizeof(cpu1_to_cm_addr[0])); cpu1_to_cm_loop++)
        cpu1_to_cm_addr[cpu1_to_cm_loop].data_addr =  IPC_CPU1_TO_CM_ADDR_TRANSLATE(cpu1_to_cm_addr[cpu1_to_cm_loop].data_addr);
    // Send translation address for status message structure
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&cpu1_to_cm_addr[0], sizeof(cpu1_to_cm_addr));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

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
        if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && motor[MOTOR_2].itDone && motor[MOTOR_1].itDone)
        {
            COM_msgCreate(&motor[MOTOR_1], &cmd_uOmodri, &status_msg);
            COM_msgCreate(&motor[MOTOR_2], &cmd_uOmodri, &status_msg);
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
                LED_dimming(&motor[MOTOR_1], &led_msg[0]);
                DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
            }
            // Read DRV data to get status registers (MOTOR_1)
            if(motor[MOTOR_1].motor_error & MOTOR_ERROR_DRV_NFAULT)
                DRV_readStatus(motor[MOTOR_1].p_motorDRV);
            // Read DRV data to get status registers (MOTOR_2)
            if(motor[MOTOR_2].motor_error & MOTOR_ERROR_DRV_NFAULT)
                DRV_readStatus(motor[MOTOR_2].p_motorDRV);
            // Timeout management
            cmd_uOmodri.cptTimeout++;
            // Clear flag
            CPUTimer_clearOverflowFlag(CPU_TIMER_0_BASE);
        }
        // Force a general microcontroller RESET if requested
//        if((motor_foc[MOTOR_1].motor_state == MOTOR_STATE_ERROR) &&
//           (motor_foc[MOTOR_2].motor_state == MOTOR_STATE_ERROR) && (cmd_uOmodri.mode.bit.RST))// (cmd_uOmodri.mode.bit.RST == 0b11))
//        {
//            SysCtl_simulateReset(SYSCTL_CAUSE_CPU1RSN); //SysCtl_resetDevice();
//            while(1);
//        }
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

    return;
}

/**
 * @brief           Swap the pointing addresses of p_a \& p_b
 * @param[inout]    p_a Pointer on the first pointer to be swapped
 * @param[inout]    p_b Pointer on the second pointer to be swapped
 */
inline void PTR_swap(int32_t** p_a, int32_t** p_b)
{
    int32_t* p_tmp  = *p_a;
    *p_a            = *p_b;
    *p_b            = p_tmp;

    return;
}

/**
 * @brief           Control the led dimming
 * @param[in]       p_foc   Pointer on data structure array used for motor control.
 * @param[out]      p_msg   Pointer on the message to be sent to led via SPI.
 */
inline void LED_dimming(motor_t* p_motor, uint16_t* p_msg)
{
    static uint16_t timer_cpt_ms    = 0;
    float32_t       timer_fade_blink= (float32_t)((timer_cpt_ms < 500)  ? (timer_cpt_ms)    : (1000 - timer_cpt_ms)) / 500.0f;
    float32_t       timer_blink     = (timer_cpt_ms < 500)              ? (0.0f)            : (1.0f);
    timer_cpt_ms                    = (timer_cpt_ms < 1000)             ? (timer_cpt_ms + 1): (0);
    switch(p_motor->motor_state)
    {
    case MOTOR_STATE_ALIGN_UP:
    case MOTOR_STATE_ALIGN_FIX:
        ws2812b_writeLed((uint16_t)(0xff * timer_fade_blink), 0, (uint16_t)(0xff * timer_fade_blink), p_msg);//&led_msg[0]); //Magenta fade,
        break;
    case MOTOR_STATE_READY:
    case MOTOR_STATE_STOP:
        ws2812b_writeLed(0, (uint16_t)(0xff * timer_fade_blink), 0, p_msg);//, &led_msg[0]);//Green fade, Active control
        break;
    case MOTOR_STATE_ERROR:
        ws2812b_writeLed((uint16_t)(0xff * timer_blink), 0, 0, p_msg);//, &led_msg[0]);//red fade, error control
        break;
    case MOTOR_STATE_INIT:
    default:
        ws2812b_writeLed(0,  0, (uint16_t)(0xff * timer_fade_blink), p_msg);//, &led_msg[0]);//blue fade
    }

    return;
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
    // copy data if access is granted else use previous ones
    if(cmd_uOmodri.cmdAccesValid)
    {
        cmd_t*          p_cmd       = &motor[MOTOR_1].p_motorFOC->motor_cmd;
        cmd_bit_reg_t*  p_bit       = &p_cmd->enableBits.bit;
        p_cmd->posRef               = cmd_uOmodri.posRef[MOTOR_1];
        p_cmd->velRef               = cmd_uOmodri.velRef[MOTOR_1];
        p_cmd->iqRef                = cmd_uOmodri.iqRef[MOTOR_1];
        p_cmd->kpCoeff              = cmd_uOmodri.kpCoeff[MOTOR_1];
        p_cmd->kdCoeff              = cmd_uOmodri.kdCoeff[MOTOR_1];
        p_cmd->iSat                 = cmd_uOmodri.iSat[MOTOR_1];
        p_cmd->cptTimeout           = cmd_uOmodri.cptTimeout;
        p_cmd->timeoutRef           = (uint16_t)cmd_uOmodri.mode.bit.TIMEOUT;
        p_bit->encOffsetCompEnable  = cmd_uOmodri.mode.bit.EI1OC;
        p_bit->rollOverEnable       = cmd_uOmodri.mode.bit.EPRE;
        p_bit->motorEnable          = cmd_uOmodri.mode.bit.EM1;
        p_bit->systemEnable         = cmd_uOmodri.mode.bit.ES;
    }
    // Motor control
    motor[MOTOR_1].itDone = MOT_runControl(&motor[MOTOR_1]);
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
    // copy data if access is granted else use previous ones
    if(cmd_uOmodri.cmdAccesValid)
    {
        cmd_t*          p_cmd       = &motor[MOTOR_2].p_motorFOC->motor_cmd;
        cmd_bit_reg_t*  p_bit       = &p_cmd->enableBits.bit;
        p_cmd->posRef               = cmd_uOmodri.posRef[MOTOR_2];
        p_cmd->velRef               = cmd_uOmodri.velRef[MOTOR_2];
        p_cmd->iqRef                = cmd_uOmodri.iqRef[MOTOR_2];
        p_cmd->kpCoeff              = cmd_uOmodri.kpCoeff[MOTOR_2];
        p_cmd->kdCoeff              = cmd_uOmodri.kdCoeff[MOTOR_2];
        p_cmd->iSat                 = cmd_uOmodri.iSat[MOTOR_2];
        p_cmd->cptTimeout           = cmd_uOmodri.cptTimeout;
        p_cmd->timeoutRef           = (uint16_t)cmd_uOmodri.mode.bit.TIMEOUT;
        p_bit->encOffsetCompEnable  = cmd_uOmodri.mode.bit.EI2OC;
        p_bit->rollOverEnable       = cmd_uOmodri.mode.bit.EPRE;
        p_bit->motorEnable          = cmd_uOmodri.mode.bit.EM2;
        p_bit->systemEnable         = cmd_uOmodri.mode.bit.ES;
    }
    // Motor control
    motor[MOTOR_2].itDone = MOT_runControl(&motor[MOTOR_2]);
#if (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE)
    // Send info to CM
    (!IPC_isFlagBusyLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG)) ? (IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG)) : (0);
#endif
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

