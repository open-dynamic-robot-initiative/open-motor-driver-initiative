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
extern motor_t          motor[2];
extern const hal_cfg_t  hal;
extern cmd_t            cmd_uOmodri[2];

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
    // DRV8353 initialization
    DRV_ini(motor[MOTOR_1].p_motorDRV);
    DRV_ini(motor[MOTOR_2].p_motorDRV);
    // ADC offset calibration for debug
    HAL_ADC_offsetCalib(motor[MOTOR_1].p_motorHalCfg);
    HAL_ADC_offsetCalib(motor[MOTOR_2].p_motorHalCfg);
    // Force a reset of FOC structures
    FOC_resetStruct(motor[MOTOR_1].p_motorFOC);
    FOC_resetStruct(motor[MOTOR_2].p_motorFOC);
    // Force a reset of the intermediate command structures. Working structures are embedded in FOC structures.
    COM_resetCmdStruct(&cmd_uOmodri[MOTOR_1]);
    COM_resetCmdStruct(&cmd_uOmodri[MOTOR_2]);
    // DMA initialization for memory to memory transfer
    HAL_DMA_ini(dmaMem2MemCfList);
    // Interrupts initialization
    HAL_INT_ini(InterruptList);

    bool_t cmd_msg_valid    = false;
    bool_t timeout_flag     = false;

    // Infinite loop
    while(1)
    {
        /*
         * SPI COM RX management - Receive & extract message if valid
         * DMA CMD TO FOC struct - Propagate new commands (message extract or timeout inc)
         */
        if(!DMA_getTransferStatusFlag(DMA_CMD_2_FOC_BASE_ADDR))
        {
            if(timeout_flag)
            {
                cmd_uOmodri[MOTOR_1].cptTimeout++;
                cmd_uOmodri[MOTOR_2].cptTimeout++;
            }
            if(!DMA_getRunStatusFlag(DMA_COM_SPI_RX_BASE_ADDR) && GPIO_readPin(COM_SPI_CS))
            {
                // Prepare SPI and associated DMA for a new receive
                SPI_resetRxFIFO(COM_SPI_BASE);
                DMA_clearTriggerFlag(DMA_COM_SPI_RX_BASE_ADDR);
                // Extract received message & send new commands to FOC structures if valid
                cmd_msg_valid       = COM_msgExtract(&cmd_msg, &cmd_uOmodri[MOTOR_1], &cmd_uOmodri[MOTOR_2]);
                // Restart DMA reception
                DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
            }
            // Send new commands to FOC structures if extract valid
            if(cmd_msg_valid || timeout_flag)
            {
                if(cmd_msg_valid && ((cmd_uOmodri[MOTOR_1].enableReg.all & 0xB000) == 0xB000))
                    cmd_uOmodri[MOTOR_1].posRef += motor[MOTOR_1].p_motorFOC->motor_enc.thetaIndex;
                if(cmd_msg_valid && ((cmd_uOmodri[MOTOR_2].enableReg.all & 0x000B) == 0x000B))
                    cmd_uOmodri[MOTOR_2].posRef += motor[MOTOR_2].p_motorFOC->motor_enc.thetaIndex;
                DMA_forceTrigger(DMA_CMD_2_FOC_BASE_ADDR);
            }
            cmd_msg_valid           = false;
            timeout_flag            = false;
        }
        /*
         * SPI COM TX management - Create message & send it
         */
        if(motor[MOTOR_2].itDone && motor[MOTOR_1].itDone)
        {
            if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR))
            {
                COM_msgCreate(&motor[MOTOR_1], &motor[MOTOR_2], &status_msg);
                if(COM_SPI_CS_READ)//GPIO_readPin(COM_SPI_CS))
                {
                    // Force first write in SPI_O_DAT
                    COM_SPI_DAT(status_msg.status.all);
                    SPI_resetTxFIFO(COM_SPI_BASE);
                    DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
                }
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
            if(motor[MOTOR_1].motor_error.bit.drv_fault)
                DRV_readStatus(motor[MOTOR_1].p_motorDRV);
            // Read DRV data to get status registers (MOTOR_2)
            if(motor[MOTOR_2].motor_error.bit.drv_fault)
                DRV_readStatus(motor[MOTOR_2].p_motorDRV);
            // Timeout management
            timeout_flag            = true;
            // Clear CPU timer flag
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
    /* Initialize clock and peripherals */
    Device_init();
    /* Set up the auxiliary PLL to 125MHz for CM use. */
//    SysCtl_setAuxClock(SYSCTL_AUXPLL_OSCSRC_XTAL | SYSCTL_AUXPLL_IMULT(50) | SYSCTL_REFDIV(2U) |
//                       SYSCTL_ODIV(5U) | SYSCTL_AUXPLL_DIV_2 | SYSCTL_AUXPLL_ENABLE | SYSCTL_DCC_BASE_0);
    /* Set the CM Clock to run at 125MHz.
     * The CM Clock is a fractional multiple of the AUXPLL Clock (125 MHz). */
//    SysCtl_setCMClk(SYSCTL_CMCLKOUT_DIV_1, SYSCTL_SOURCE_AUXPLL);
    /* Allocate Shared Peripheral USB to the CM Side. */
//    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
    /* Boot CM core */
#if (defined _FLASH) && (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE)
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#elif (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE)
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
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
 * @param[in]       p_motor Pointer on data structure array used for motor control.
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
#ifdef DEBUG
    DBG_PIN4_SET;
#endif
    while(DMA_getTransferStatusFlag(DMA_CMD_2_FOC_BASE_ADDR));
    // Motor control
    motor[MOTOR_1].itDone = MOT_runControl(&motor[MOTOR_1]);
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR1_IA_INT_ACK_GROUP);
#ifdef DEBUG
    DBG_PIN4_CLEAR;
#endif
}

/**
 * @brief           ADC interrupt for MOTOR_2. Read currents \& voltages and compute control.
 */
__attribute__((interrupt)) void adc_isrMotor2(void)
{
#ifdef DEBUG
    DBG_PIN4_SET;
#endif
    while(DMA_getTransferStatusFlag(DMA_CMD_2_FOC_BASE_ADDR));
    // Motor control
    motor[MOTOR_2].itDone = MOT_runControl(&motor[MOTOR_2]);
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IVA_ADC_ADDR, MOTOR2_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR2_IA_INT_ACK_GROUP);
#ifdef DEBUG
    DBG_PIN4_CLEAR;
#endif
}

