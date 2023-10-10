/*
 * File name: main.c
 * Description: Source file containing main function and IT running on CPU1 - C28 processor.
 */

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "driverlib.h"
#include "device.h"
#include "board.h"

#include "main.h"
#include "hal.h"
#include "drv8353.h"
#include "ws2812b.h"
#include "motor.h"
#include "communication.h"
#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"
#include "uomodri_prj_config_handlers.h"

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
// Variable in GS0
#pragma DATA_ALIGN(rx_msg, 4)
#pragma DATA_SECTION(rx_msg, "ramgs0")
mst2slv_msg_t           rx_msg;
#pragma DATA_ALIGN(rx_swap, 4)
#pragma DATA_SECTION(rx_swap, "ramgs0")
mst2slv_msg_t           rx_swap;
#pragma DATA_ALIGN(tx_msg, 4)
#pragma DATA_SECTION(tx_msg, "ramgs0")
slv2mst_msg_t           tx_msg[2];
#pragma DATA_ALIGN(tx_swap, 4)
#pragma DATA_SECTION(tx_swap, "ramgs0")
slv2mst_msg_t           tx_swap;
#pragma DATA_ALIGN(led_msg, 4)
#pragma DATA_SECTION(led_msg, "ramgs0")
uint16_t                led_msg[LED_MSG_TX_16BIT_LENGTH];
#pragma DATA_ALIGN(rs485_rx_msg, 4)
#pragma DATA_SECTION(rs485_rx_msg, "ramgs0")
rs485_rx_t              rs485_rx_msg = {.index = 0, .addr_valid = false};
#pragma DATA_ALIGN(rs485_tx_msg, 4)
#pragma DATA_SECTION(rs485_tx_msg, "ramgs0")
rs485_tx_t              rs485_tx_msg = {.index = 0, .remaining = 34};

// Local variables
uint16_t                tx_msg_cr           = 0U;
uint16_t                tx_msg_nx           = 1U;
//uint16_t                rs485_rx_msg_index  = 0;
//uint16_t                rs485_tx_msg_index  = 0;
//uint16_t                rs485_tx_msg_remaining = 34;
// External variables
extern motor_t          motor_m1;
extern motor_t          motor_m2;
extern const hal_cfg_t  hal;
extern cmd_t            cmd_uomodri[2];

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
static void MCU_ini(void);
inline void LED_dimming(motor_t*, motor_t*, uint16_t*);

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
    // Initialization of the shared structure used by CLA & C28 for Motor_1 & Motor_2
    MOT_initStruct(&motor_m1, MOTOR_1);
    MOT_initStruct(&motor_m2, MOTOR_2);
    // Reset the command structure for CLA.
    COM_resetCmdStruct(&cmd_uomodri[MOTOR_1]);
    COM_resetCmdStruct(&cmd_uomodri[MOTOR_2]);
    // Hardware Abstraction Layer initialization
    HAL_ini(&hal);
    // DRV8353 initialization
    cmd_uomodri[MOTOR_1].statusField.drv_initErr = !DRV_ini(&drv_cfg[MOTOR_1], &drv_reg[MOTOR_1]);
    cmd_uomodri[MOTOR_2].statusField.drv_initErr = !DRV_ini(&drv_cfg[MOTOR_2], &drv_reg[MOTOR_2]);
    // Start ADC offset calibration on CLA
    CLA_forceTasks(CLA1_BASE, CLA_ADC_CALIB_TASK_FLAG);
    // Interrupts initialization
    HAL_INT_ini(InterruptList);
    // Infinite loop
    while(1);
}

/**
 * @brief           Microcontroller initialization function
 *                  - Configure master clock to 200MHz
 *                  - Configure auxiliary clock (CM core) to 125MHz (USB � 60MHz)
 *                  - Enable GPIO clocking
 *                  - Configure the interruption vector table
 *                  - Boot CM from flash or RAM
 */
static void MCU_ini(void)
{
//#if (defined USB_BUS_ENABLE_ON_CPU1) || ((defined USB_BUS_ENABLE_ON_CM) && (defined CM_CORE_ENABLE))
    /* Initialize clock and peripherals */
    Device_init();
    /* Modify LSPCLK divider previously defined @ DEVICE_SYSCLK_FREQ / 4 */
#if (UOMODRI_LSPCLK_FREQ == DEVICE_SYSCLK_FREQ)
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_1);
    ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == UOMODRI_LSPCLK_FREQ);
#endif
    /* Set the CLB/tile clock */
#if (CLB_TILE_ENABLE)
    /* Redefined to pass to SysCtl_setAuxClock(). Will configure the clock as follows:
     * AUXPLLCLK = 25MHz (XTAL_OSC) * 48 (IMULT) / (2 (REFDIV) * 5 (ODIV) * 2(AUXPLLDIV) )
     * AUXPLLCLK require a 60MHz clock for USB. AUPLLRAWCLK require a 120MHz clock for CM. */
//#undef DEVICE_AUXSETCLOCK_CFG
//#define DEVICE_AUXSETCLOCK_CFG      (SYSCTL_AUXPLL_OSCSRC_XTAL  | \
//                                     SYSCTL_AUXPLL_IMULT(48)    | \
//                                     SYSCTL_AUXPLL_REFDIV(2U)   | \
//                                     SYSCTL_AUXPLL_ODIV(5U)     | \
//                                     SYSCTL_AUXPLL_DIV_2        | \
//                                     SYSCTL_AUXPLL_ENABLE       | \
//                                     SYSCTL_DCC_BASE_0)
//#undef DEVICE_AUXCLK_FREQ
//#define DEVICE_AUXCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 48) / (2 * 5 * 2))
#endif


#if ((defined USB_BUS_ENABLE_ON_CM) && (defined CM_CORE_ENABLE))
    /* Allocate Shared Peripheral USB to the CM Side. */
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
#endif
    /* Boot CM core */
#if (defined _FLASH) && (CM_CORE_ENABLE)
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#elif (CM_CORE_ENABLE)
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
 * @brief           Control the led dimming
 * @param[in]       p_motor Pointer on data structure array used for motor control.
 * @param[out]      p_msg   Pointer on the message to be sent to led via SPI.
 */
inline void LED_dimming(motor_t* p_motor1, motor_t* p_motor2, uint16_t* p_msg)
{
    static uint16_t timer_cpt_ms        = 0;
    float32_t       timer_fade_blink    = (float32_t)((timer_cpt_ms < 500)  ? (timer_cpt_ms)    : (1000 - timer_cpt_ms)) / 500.0f;
    float32_t       timer_blink         = (timer_cpt_ms < 500)              ? (0.0f)            : (1.0f);
    timer_cpt_ms                        = (timer_cpt_ms < 1000)             ? (timer_cpt_ms + 1): (0);
    uint16_t        motor_state         = MOTOR_STATE_IDLE;
    uint16_t        u16_timer_fade_blink= (uint16_t)(0xff * timer_fade_blink);

    if((p_motor1->state & MOTOR_STATE_BOOT)     || (p_motor2->state & MOTOR_STATE_BOOT))
        motor_state = MOTOR_STATE_IDLE;
    else if((p_motor1->state & MOTOR_STATE_INIT)|| (p_motor2->state & MOTOR_STATE_INIT))
        motor_state = MOTOR_STATE_INIT_FIX;
    else if((p_motor1->state & MOTOR_STATE_OK)  || (p_motor2->state & MOTOR_STATE_OK))
        motor_state = MOTOR_STATE_READY;
    if((p_motor1->state == MOTOR_STATE_ERROR)   || (p_motor2->state == MOTOR_STATE_ERROR))
        motor_state = MOTOR_STATE_ERROR;

    switch(motor_state)
    {
    case MOTOR_STATE_INIT_FW:
    case MOTOR_STATE_INIT_REV:
    case MOTOR_STATE_INIT_TEST:
    case MOTOR_STATE_INIT_FIX:
        ws2812b_writeLed(u16_timer_fade_blink, 0, u16_timer_fade_blink, p_msg); //Magenta fade,
        break;
    case MOTOR_STATE_READY:
    case MOTOR_STATE_STOP:
        ws2812b_writeLed(0, u16_timer_fade_blink, 0, p_msg);//Green fade, Active control
        break;
    case MOTOR_STATE_ERROR:
        ws2812b_writeLed((uint16_t)(0xff * timer_blink), 0, 0, p_msg);//red fade, error control
        break;
//    case MOTOR_STATE_WAIT_SYNC:
//    case MOTOR_STATE_ADC_CALIB:
    case MOTOR_STATE_IDLE:
    default:
        ws2812b_writeLed(0,  0, u16_timer_fade_blink, p_msg);//blue fade
    }

    return;
}

/***********************************************************************
 * INTERRUPT HANDLING FUNCTIONS DEFINITIONS
 ***********************************************************************/
__attribute__((interrupt)) void cla_ClearTask(void)
{
#ifdef DEBUG
//    DBG_PIN2_SET;
#endif
    // Purge pending tasks after ADC calibration.
    CLA_clearTaskFlags(CLA1_BASE, CLA_TASKFLAG_ALL);
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_ADC_CALIB_GROUP);
#ifdef DEBUG
//    DBG_PIN2_CLEAR;
#endif
}

__attribute__((interrupt)) void cla_isrMotor1(void)
{
#ifdef DEBUG
//    DBG_PIN2_SET;
#endif
    // Create partially the Tx message
    COM_msgCreate(&motor_m1, &tx_msg[tx_msg_nx], NULL);
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR1_GROUP);
#ifdef DEBUG
//    DBG_PIN2_CLEAR;
#endif
}

__attribute__((interrupt)) void cla_isrMotor2(void)
{
#ifdef DEBUG
//    DBG_PIN2_SET;
#endif
    // Finalize the Tx message
    COM_msgCreate(&motor_m2, &tx_msg[tx_msg_nx], &tx_swap);
    // Check that SPI is free for transmit
    if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && COM_SPI_CS_READ)
    {
        tx_msg_cr   = (tx_msg_cr) ? (0U) : (1U);
        tx_msg_nx   = (tx_msg_cr) ? (0U) : (1U);
        SPI_resetTxFIFO(COM_SPI_BASE);
        // Force first write in SPI_O_DAT register
        COM_SPI_DAT(tx_msg[tx_msg_cr].status.all);
        DMA_configAddresses(DMA_COM_SPI_TX_BASE_ADDR, (uint16_t *) COM_SPI_TX_BASE_REG_ADDR, (uint16_t *) &tx_msg[tx_msg_cr].timeStamp);
        DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
    }
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR2_GROUP);
#ifdef DEBUG
//    DBG_PIN2_CLEAR;
#endif
}

__attribute__((interrupt)) void tmr_isrLed(void)
{
#ifdef DEBUG
//    DBG_PIN3_SET;
#endif
    // LED dimming control
    if(!DMA_getRunStatusFlag(DMA_LED_SPI_TX_BASE_ADDR))
    {
        LED_dimming(&motor_m1, &motor_m2, &led_msg[0]);
        DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
    }
    // Timeout management
    cmd_uomodri[MOTOR_1].cptTimeout++;
    cmd_uomodri[MOTOR_2].cptTimeout++;
    CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
    Interrupt_clearACKGroup(INT_TIMER_LED_GROUP);
#ifdef DEBUG
//    DBG_PIN3_CLEAR;
#endif
}

__attribute__((interrupt)) void dma_isrComRx(void)
{
#ifdef DEBUG
//    DBG_PIN3_SET;
#endif
    // Extract received message & send new commands to FOC structures if valid
    if(COM_msgExtract(&rx_msg, &rx_swap, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2]))
    {
        cmd_uomodri[MOTOR_1].cptTimeout = 0U;
        cmd_uomodri[MOTOR_2].cptTimeout = 0U;
        CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
    }
    // Restart DMA reception
    DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
    // Reset the SPI Rx fifo
    SPI_resetRxFIFO(COM_SPI_BASE);
    // Clear IT flag
    Interrupt_clearACKGroup(INT_DMA_RX_MSG_GROUP);
#ifdef DEBUG
//    DBG_PIN3_CLEAR;
#endif
}

__attribute__((interrupt)) void sci_isrRx_RS485(void)
{
    // FIFO IT source test.
    uint32_t sci_it_src_test = SCI_getInterruptStatus(COM_SCI_BASE);

    DBG_PIN2_SET;
    // FIFO RX IT?
    if(sci_it_src_test & SCI_INT_RXFF)
    {
        // Is first element a valid address ?
//        if(SCI_getRxStatus(COM_SCI_BASE) & SCI_RXSTATUS_WAKE)
//        {
//            uint32_t clb_gpreg  = CLB_getGPREG(RS485_RX_TX_CLB_EOT_BASE);
//            uint16_t addr_test  = SCI_readCharNonBlocking(COM_SCI_BASE);
//            if((addr_test == 0x61) || (addr_test == 0x6E)) // For test
//            {
//                CLB_setGPREG(RS485_RX_TX_CLB_EOT_BASE, (clb_gpreg | CLB_GPREG_ADDR_VALID));
//                rs485_rx_msg.msg[0] = addr_test;
//                rs485_rx_msg.index  = 1;
//            }
//            else
//                CLB_setGPREG(RS485_RX_TX_CLB_EOT_BASE, (clb_gpreg & (~CLB_GPREG_ADDR_VALID)));
//        }
//        // Valid address ?
//        if(CLB_getGPREG(RS485_RX_TX_CLB_EOT_BASE) & CLB_GPREG_ADDR_VALID)
//            RS485_msgReceive(COM_SCI_BASE, &rs485_rx_msg);
//        else
//        {
//            SCI_resetRxFIFO(COM_SCI_BASE);
//            SCI_disableInterrupt(COM_SCI_BASE, SCI_INT_RXFF);
//        }
        RS485_msgReceive(COM_SCI_BASE, &rs485_rx_msg);
        // Clear IT source
        SCI_clearOverflowStatus(COM_SCI_BASE);
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
    }
    DBG_PIN2_CLEAR;

    // Clear global IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_MSG_GROUP);
}

__attribute__((interrupt)) void clb_isrRxTx_RS485(void)
{
    uint16_t clb_tag_test   = CLB_getInterruptTag(RS485_RX_TX_CLB_EOT_BASE);

    if(clb_tag_test == 0x01)
    {
        DBG_PIN3_SET;
        if(rs485_rx_msg.addr_valid)
        // Valid address ?
//        if(CLB_getGPREG(RS485_RX_TX_CLB_EOT_BASE) & 8UL)
//        {
//            RS485_msgReceive(COM_SCI_BASE, &rs485_rx_msg);
//            //-----------------------------------
////            rs485_rx_msg.index = 0;
////            rs485_tx_msg.index = 0;
////            rs485_tx_msg.remaining = 34;
////            while(rs485_tx_msg.remaining--)
////                rs485_tx_msg.msg[rs485_tx_msg.index++] = rs485_rx_msg.msg[rs485_rx_msg.index++];
////            rs485_tx_msg.index = 0;
////            rs485_tx_msg.remaining = 34;
////            SCI_resetTxFIFO(COM_SCI_BASE);
////            SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_TXFF);
////            SCI_enableTxModule(COM_SCI_BASE);
////            RS485_msgTransmit(COM_SCI_BASE, &rs485_tx_msg);
//            //-----------------------------------
//        }
//        else
//        {
//            SCI_resetRxFIFO(COM_SCI_BASE);
//            SCI_enableInterrupt(COM_SCI_BASE, SCI_INT_RXFF);
//        }
            RS485_msgReceive(COM_SCI_BASE, &rs485_rx_msg);
        SCI_resetRxFIFO(COM_SCI_BASE);
        SCI_disableInterrupt(COM_SCI_BASE, SCI_INT_RXFF | SCI_INT_ALL_ERR);//        SCI_enableInterrupt(COM_SCI_BASE, SCI_INT_RXFF);

        rs485_rx_msg.index      = 0;
        rs485_rx_msg.addr_valid = false;
//        CLB_setGPREG(RS485_RX_TX_CLB_EOT_BASE, 0UL);
        // Clear IT source
        SCI_clearOverflowStatus(COM_SCI_BASE);
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);

        DBG_PIN3_CLEAR;
    }
    if(clb_tag_test == 0x02)
    {
        SCI_resetTxFIFO(COM_SCI_BASE);
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_TXFF);
        SCI_disableTxModule(COM_SCI_BASE);
//        SCI_txDisable(COM_SCI_BASE);
//        SCI_rxEnable(COM_SCI_BASE);
//        DBG_PIN3_CLEAR;
    }

    if((clb_tag_test == 0x10) || (clb_tag_test == 0x20))
    {
        DBG_PIN3_SET;
        rs485_rx_msg.index      = 0;
        rs485_rx_msg.addr_valid = true;
        RS485_msgReceive(COM_SCI_BASE, &rs485_rx_msg);
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
        SCI_enableInterrupt(COM_SCI_BASE, SCI_INT_RXFF);
        DBG_PIN3_CLEAR;
    }

    CLB_clearInterruptTag(RS485_RX_TX_CLB_EOT_BASE);
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_Tx_CLB_GROUP);
}

__attribute__((interrupt)) void sci_isrTx_RS485(void)
{
    uint32_t sci_it_src_test = SCI_getInterruptStatus(COM_SCI_BASE);
    if(sci_it_src_test & SCI_INT_TXFF)
    {
//        DBG_PIN2_SET;
        RS485_msgTransmit(COM_SCI_BASE, &rs485_tx_msg);
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_TXFF);
//        DBG_PIN2_CLEAR;
    }
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_TX_MSG_GROUP);
}

//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
//
//00000000 // start
//11111111
//00000000
//00000000
//00000000
//00000000
//11111111
//11111111
//00000000
//11111111 // Stop
//
//00000000 // start
//00000000
//00000000
//00000000
//00000000
//00000000
//00000000
//00000000
//00000000
//11111111 // Stop
//
//00000000 // start
//10101010
//01010101
//10101010
//01010101
//10101010
//01010101
//10101010
//01010101
//11111111 // Stop
//
//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
//
//00000000 // start
//00000000
//00000000
//00000000
//00000000
//00000000
//11111111
//11111111
//00000000
//11111111 // Stop
//
//00000000 // start
//00000000
//00000000
//00000000
//00000000
//00000000
//00000000
//00000000
//00000000
//11111111 // Stop
//
//00000000 // start
//10101010
//01010101
//10101010
//01010101
//10101010
//01010101
//10101010
//01010101
//11111111 // Stop
//
//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
