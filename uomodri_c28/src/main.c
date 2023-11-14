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
//#if (SPI_BUS_ENABLE)
#pragma DATA_ALIGN(com_spi_rx_msg, 4)
#pragma DATA_SECTION(com_spi_rx_msg, "ramgs0")
mst2slv_msg_t           com_spi_rx_msg;
#pragma DATA_ALIGN(com_spi_rx_msg_swap, 4)
#pragma DATA_SECTION(com_spi_rx_msg_swap, "ramgs0")
mst2slv_msg_t           com_spi_rx_msg_swap;
#pragma DATA_ALIGN(com_spi_tx_msg, 4)
#pragma DATA_SECTION(com_spi_tx_msg, "ramgs0")
slv2mst_msg_t           com_spi_tx_msg[2];
#pragma DATA_ALIGN(com_spi_tx_msg_swap, 4)
#pragma DATA_SECTION(com_spi_tx_msg_swap, "ramgs0")
slv2mst_msg_t           com_spi_tx_msg_swap;
//#elif (RS485_BUS_ENABLE)
#pragma DATA_ALIGN(com_rs485_rx_msg, 4)
#pragma DATA_SECTION(com_rs485_rx_msg, "ramgs0")
rs485_rx_t              com_rs485_rx_msg = {.index = 0, .addr_valid = false, .data_highByte = true};
#pragma DATA_ALIGN(com_rs485_tx_msg, 4)
#pragma DATA_SECTION(com_rs485_tx_msg, "ramgs0")
rs485_tx_t              com_rs485_tx_msg[2] = {{.index = 0, .remaining = 34}, {.index = 0, .remaining = 34}};
//#endif
#pragma DATA_ALIGN(led_spi_tx_msg, 4)
#pragma DATA_SECTION(led_spi_tx_msg, "ramgs0")
uint16_t                led_spi_tx_msg[LED_MSG_TX_16BIT_LENGTH];

// Local variables
uint16_t                com_msg_tx_cr = 0U;
uint16_t                com_msg_tx_nx = 1U;

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
    while(1)
    {
//#if (COM_CRC_TEST == 1)
//        if(COM_msgExtract(&rx_msg, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2], crc))
//        {
//            cmd_uomodri[MOTOR_1].cptTimeout = 0U;
//            cmd_uomodri[MOTOR_2].cptTimeout = 0U;
//            CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
//        }
//#endif
    }
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
    /* Initialize clock and peripherals */
    Device_init();

    /*
     * LSPCLK redefine - SPIx Bit Clock & SCIx Bit Clock.
     * DEVICE_SYSCLK_FREQ (200MHz) instead of DEVICE_SYSCLK_FREQ / 4 (50MHz)
     */
#if (UOMODRI_LSPCLK_FREQ == DEVICE_SYSCLK_FREQ)
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_1);
#ifdef DEBUG
    ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == UOMODRI_LSPCLK_FREQ);
#endif
#endif

    /*
     * SYSCLK to EPWMCLK DIVIDER define - ePWM peripheral & CLB tiles.
     * EPWMCLK_FREQ (100MHz) = DEVICE_SYSCLK_FREQ (200MHz) / PLLSYSCLK_TO_EPWMCLK_DIV (2)
     */
    SysCtl_setEPWMClockDivider(PLLSYSCLK_TO_EPWMCLK_DIV);

    /* Set the CLB/tile clock */
#if (CLB_TILE_ENABLE)
    /*
     * Use SYSCLK / PERCLKDIVSEL.EPWMCLKDIV (2) as Tile clock (100MHz).
     */

    /* Redefined to pass to SysCtl_setAuxClock(). Will configure the clock as follows:
     * AUXPLLCLK = 25MHz (XTAL_OSC) * 48 (IMULT) / (2 (REFDIV) * 5 (ODIV) * 2(AUXPLLDIV) )
     * AUXPLLCLK require a 60MHz clock for USB. AUPLLRAWCLK require a 120MHz clock for CM. */
//#undef DEVICE_AUXSETCLOCK_CFG
//#define DEVICE_AUXSETCLOCK_CFG      (SYSCTL_AUXPLL_OSCSRC_XTAL  | \
                                     SYSCTL_AUXPLL_IMULT(48)    | \
                                     SYSCTL_AUXPLL_REFDIV(2U)   | \
                                     SYSCTL_AUXPLL_ODIV(5U)     | \
                                     SYSCTL_AUXPLL_DIV_2        | \
                                     SYSCTL_AUXPLL_ENABLE       | \
                                     SYSCTL_DCC_BASE_0)
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
    DBG_PIN2_SET;
#endif
    // Create partially the Tx message
#if (SPI_BUS_ENABLE)
    COM_msgCreate(&motor_m1, &com_spi_tx_msg[com_msg_tx_nx], NULL);
#elif (RS485_BUS_ENABLE)
    com_rs485_tx_msg[com_msg_tx_nx].msg.addr = 0xFF;
    com_rs485_tx_msg[com_msg_tx_nx].msg.type = 0x00;
    com_rs485_tx_msg[com_msg_tx_nx].msg.size = COM_MSG_RS485_TX_RX_8BIT_FULL_LENGTH;
    COM_msgCreate(&motor_m1, &com_rs485_tx_msg[com_msg_tx_nx].msg.msg_payload, NULL);
#endif
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR1_GROUP);
#ifdef DEBUG
    DBG_PIN2_CLEAR;
#endif
}

__attribute__((interrupt)) void cla_isrMotor2(void)
{
#ifdef DEBUG
    DBG_PIN2_SET;
#endif
    // Finalize the Tx message
#if (SPI_BUS_ENABLE)
    COM_msgCreate(&motor_m2, &com_spi_tx_msg[com_msg_tx_nx], &com_spi_tx_msg_swap);
    // Check that SPI is free for transmit
    if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && COM_SPI_CS_READ)
    {
        com_msg_tx_cr   = (com_msg_tx_cr) ? (0U) : (1U);
        com_msg_tx_nx   = (com_msg_tx_cr) ? (0U) : (1U);
        SPI_resetTxFIFO(COM_SPI_BASE);
        // Force first write in SPI_O_DAT register
        COM_SPI_DAT(com_spi_tx_msg[com_msg_tx_cr].status.all);
        DMA_configAddresses(DMA_COM_SPI_TX_BASE_ADDR, (uint16_t *) COM_SPI_TX_BASE_REG_ADDR, (uint16_t *) &com_spi_tx_msg[com_msg_tx_cr].timeStamp);
        DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
    }
#elif (RS485_BUS_ENABLE)
    COM_msgCreate(&motor_m2, &com_rs485_tx_msg[com_msg_tx_nx].msg.msg_payload, NULL);
#endif
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR2_GROUP);
#ifdef DEBUG
    DBG_PIN2_CLEAR;
#endif
}

__attribute__((interrupt)) void tmr_isrLed(void)
{
#ifdef DEBUG
    DBG_PIN2_SET;
#endif
    // LED dimming control
    if(!DMA_getRunStatusFlag(DMA_LED_SPI_TX_BASE_ADDR))
    {
        LED_dimming(&motor_m1, &motor_m2, &led_spi_tx_msg[0]);
        DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
    }
    // Timeout management
    cmd_uomodri[MOTOR_1].cptTimeout++;
    cmd_uomodri[MOTOR_2].cptTimeout++;
    CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
    Interrupt_clearACKGroup(INT_TIMER_LED_GROUP);
#ifdef DEBUG
    DBG_PIN2_CLEAR;
#endif
}

__attribute__((interrupt)) void dma_isrComRx(void)
{
#ifdef DEBUG
//    DBG_PIN3_SET;
#endif
    // Extract received message & send new commands to FOC structures if valid
#if (COM_CRC_TEST == 0)
    if(COM_msgExtract(&com_spi_rx_msg, &com_spi_rx_msg_swap, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2]))
#elif (COM_CRC_TEST == 1)
    uint32_t crc = COM_msgCrc((uint16_t *)&com_spi_rx_msg, COM_MSG_SPI_TX_RX_16BIT_PAYLOAD_LENGTH);
    if(COM_msgExtract(&com_spi_rx_msg, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2], crc))
#endif
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
    DBG_PIN2_SET;
    // FIFO IT source test.
    uint32_t sci_it_src_test = SCI_getInterruptStatus(COM_SCI_BASE);
    // FIFO RX IT?
    if(sci_it_src_test & SCI_INT_RXFF)
    {
        // Get contain of FIFO Rx
         RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx_msg);
        // Clear IT source
        HWREGH(COM_SCI_BASE + SCI_O_FFRX) |= (SCI_FFRX_RXFFOVRCLR | SCI_FFRX_RXFFINTCLR);
    }
    // Clear global IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_MSG_GROUP);
    DBG_PIN2_CLEAR;
}

__attribute__((interrupt)) void clb_isrRxTx_RS485(void)
{
    DBG_PIN3_SET;
    uint16_t clb_tag_test   = CLB_getInterruptTag(RS485_RX_TX_CLB_EOT_BASE);

    if(clb_tag_test == 0x01)
    {
        if(com_rs485_rx_msg.addr_valid)
            // Valid address ?
            RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx_msg);
#if (COM_CRC_TEST == 1)
    uint32_t crc = COM_msgCrc((uint16_t *)&com_rs485_rx_msg, COM_MSG_RS485_TX_RX_16BIT_PAYLOAD_LENGTH);
    if(COM_msgExtract(&com_rs485_rx_msg.msg.msg_playload, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2], crc))
    {
        cmd_uomodri[MOTOR_1].cptTimeout = 0U;
        cmd_uomodri[MOTOR_2].cptTimeout = 0U;
        CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
    }
#endif
//        SCI_resetRxFIFO(COM_SCI_BASE);
        HWREGH(COM_SCI_BASE + SCI_O_FFRX) &= ~SCI_FFRX_RXFIFORESET;
        HWREGH(COM_SCI_BASE + SCI_O_FFRX) |= SCI_FFRX_RXFIFORESET;
//        SCI_disableInterrupt(COM_SCI_BASE, SCI_INT_RXFF);
        HWREGH(COM_SCI_BASE + SCI_O_CTL1) &= ~SCI_CTL1_RXERRINTENA;
        HWREGH(COM_SCI_BASE + SCI_O_FFRX) &= ~SCI_FFRX_RXFFIENA;

        com_rs485_rx_msg.index      = 0;
        com_rs485_rx_msg.addr_valid = false;
        // Clear IT source
//        SCI_clearOverflowStatus(COM_SCI_BASE);
//        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
        HWREGH(COM_SCI_BASE + SCI_O_FFRX) |= (SCI_FFRX_RXFFOVRCLR | SCI_FFRX_RXFFINTCLR);
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
        com_rs485_rx_msg.index          = 0;
        com_rs485_rx_msg.addr_valid     = true;
        com_rs485_rx_msg.data_highByte  = true;
        RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx_msg);
        HWREGH(COM_SCI_BASE + SCI_O_FFRX) |= (SCI_FFRX_RXFFIENA | SCI_FFRX_RXFFINTCLR);
    }

    // Clear CLB Flag -> Address valid.
    CLB_clearInterruptTag(RS485_RX_TX_CLB_EOT_BASE);
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_Tx_CLB_GROUP);
    DBG_PIN3_CLEAR;
}

__attribute__((interrupt)) void sci_isrTx_RS485(void)
{
    uint32_t sci_it_src_test = SCI_getInterruptStatus(COM_SCI_BASE);
    if(sci_it_src_test & SCI_INT_TXFF)
    {
//        DBG_PIN2_SET;
        RS485_msgTransmit(COM_SCI_BASE, &com_rs485_tx_msg[com_msg_tx_cr]);
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_TXFF);
//        DBG_PIN2_CLEAR;
    }
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_TX_MSG_GROUP);
}

