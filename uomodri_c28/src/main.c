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
#pragma DATA_ALIGN(com_spi_rx, 4)
#pragma DATA_SECTION(com_spi_rx, "ramgs0")
spi_rx_t                com_spi_rx;
#pragma DATA_ALIGN(com_spi_tx, 4)
#pragma DATA_SECTION(com_spi_tx, "ramgs0")
spi_tx_t                com_spi_tx = {.msg_cr = 0U, .msg_nx = 1U};
//#elif (RS485_BUS_ENABLE)
#pragma DATA_ALIGN(com_rs485_rx, 4)
#pragma DATA_SECTION(com_rs485_rx, "ramgs0")
rs485_rx_t              com_rs485_rx = {.index = 0, .addr1 = false, .addr2 = false};//, .highByte = true};
#pragma DATA_ALIGN(com_rs485_tx, 4)
#pragma DATA_SECTION(com_rs485_tx, "ramgs0")
rs485_tx_t              com_rs485_tx = {.index = 0, .msg_cr = 1U, .msg_nx = 0U};
//#endif
#pragma DATA_ALIGN(led_spi_tx, 4)
#pragma DATA_SECTION(led_spi_tx, "ramgs0")
uint16_t                led_spi_tx[LED_MSG_TX_16BIT_LENGTH];

// External variables
extern motor_t          motor_m1;
extern motor_t          motor_m2;
extern const hal_cfg_t  hal;
extern cmd_t            cmd_uomodri[2];

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void main(void);
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
//    {
//#if (COM_CRC_TEST == 1)
//        if(COM_msgExtract(&rx_msg, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2], crc))
//        {
//            cmd_uomodri[MOTOR_1].cptTimeout = 0U;
//            cmd_uomodri[MOTOR_2].cptTimeout = 0U;
//            CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
//        }
//#endif
//    }
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
/**
 * @brief           Force a clear of all pending interrupts after ADC calibration.
 */
__attribute__((interrupt)) void cla_ClearTask(void)
{
    // Purge pending tasks after ADC calibration.
    CLA_clearTaskFlags(CLA1_BASE, CLA_TASKFLAG_ALL);
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_ADC_CALIB_GROUP);
}

/**
 * @brief           After MOTOR_1 control, generate a part of the Tx message associated to MOTOR_1.
 */
__attribute__((interrupt)) void cla_isrMotor1(void)
{
#ifdef DEBUG
//    DBG_PIN2_SET;
#endif
    // Create partially the Tx message
#if SPI_BUS_ENABLE
    spi_tx_msg_t* p_msg = &com_spi_tx.msg[com_spi_tx.msg_nx];
    COM_msgCreate(&motor_m1, &p_msg->data);
#elif RS485_BUS_ENABLE
    rs485_tx_msg_u* p_msg   = &com_rs485_tx.msg[com_rs485_tx.msg_nx];
    p_msg->header.addr      = COM_MSG_RS485_TX_ADDRESS;
    p_msg->header.type      = COM_MSG_RS485_TX_TYPE_STANDARD;
    p_msg->header.size      = COM_MSG_RS485_TX_8BIT_FULL_LENGTH;
    COM_msgCreate(&motor_m1, &p_msg->data);
#endif
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR1_GROUP);
#ifdef DEBUG
//    DBG_PIN2_CLEAR;
#endif
}

/**
 * @brief           After MOTOR_2 control, generate a part of the Tx message associated to the motor + message CRC.
 */
__attribute__((interrupt)) void cla_isrMotor2(void)
{
#ifdef DEBUG
//    DBG_PIN2_SET;
#endif
#if SPI_BUS_ENABLE
    // Finalize the Tx message for SPI. Aggregate MOTOR_2 values + computed CRC
    spi_tx_msg_t* p_msg     = &com_spi_tx.msg[com_spi_tx.msg_nx];
    COM_msgCreate(&motor_m2, &p_msg->data);
    uint32_t crc            = COM_msgCRC((uint16_t *) &p_msg->data, COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH);
    // swap words in message - Required for Solo
    p_msg->crc.u16_msb      = LSB_32(crc);
    p_msg->crc.u16_lsb      = MSB_32(crc);
    // New message complete
    // Is SPI ready to transmit?
    if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && COM_SPI_CS_READ)
    {
        // swap active message array
        com_spi_tx.msg_cr   = !com_spi_tx.msg_cr;
        com_spi_tx.msg_nx   = !com_spi_tx.msg_cr;
        SPI_resetTxFIFO(COM_SPI_BASE);
        // Force first write in SPI_O_DAT register
        COM_SPI_DAT(p_msg->data.status.all);
        DMA_configAddresses(DMA_COM_SPI_TX_BASE_ADDR, (uint16_t *) COM_SPI_TX_BASE_REG_ADDR, (uint16_t *) &p_msg->data.timeStamp);
        DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
    }
#elif RS485_BUS_ENABLE
    // Finalize the Tx message for RS485. Aggregate MOTOR_2 values + computed CRC
    rs485_tx_msg_u* p_msg   = &com_rs485_tx.msg[com_rs485_tx.msg_nx];
    COM_msgCreate(&motor_m2, &p_msg->data);
    uint32_t crc            = COM_msgCRC((uint16_t *) &p_msg->header, COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH);
    // swap words in message - Required for Solo
    p_msg->crc.u16_msb      = LSB_32(crc);
    p_msg->crc.u16_lsb      = MSB_32(crc);
    // New message complete
    // swap active message array
    if(!com_rs485_tx.index)
    {
        com_rs485_tx.msg_cr = !com_rs485_tx.msg_cr;
        com_rs485_tx.msg_nx = !com_rs485_tx.msg_cr;
    }
#endif
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR2_GROUP);
#ifdef DEBUG
//    DBG_PIN2_CLEAR;
#endif
}

/**
 * @brief           Create the message for LED dimming + increment the timeout counter.
 */
__attribute__((interrupt)) void tmr_isrLed(void)
{
#ifdef DEBUG
//    DBG_PIN2_SET;
#endif
    // LED dimming control
    if(!DMA_getRunStatusFlag(DMA_LED_SPI_TX_BASE_ADDR))
    {
        LED_dimming(&motor_m1, &motor_m2, &led_spi_tx[0]);
        DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
    }
    // Timeout management
    cmd_uomodri[MOTOR_1].cptTimeout++;
    cmd_uomodri[MOTOR_2].cptTimeout++;
    CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
    Interrupt_clearACKGroup(INT_TIMER_LED_GROUP);
#ifdef DEBUG
//    DBG_PIN2_CLEAR;
#endif
}

/**
 * @brief           Check & extract the received message from SPI.
 */
__attribute__((interrupt)) void dma_isrSpiComRx(void)
{
#ifdef DEBUG
//    DBG_PIN3_SET;
#endif
    // Compare computed CRC with the one received in message.
    spi_rx_msg_t* p_msg = &com_spi_rx.msg;
    uint32_t crc = COM_msgCRC((uint16_t *)&p_msg->data, COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH);
    if((p_msg->crc.u16_msb == MSB_32(crc)) && (p_msg->crc.u16_lsb == LSB_32(crc)))
    {
        // Extract received message & send new commands to FOC structures if valid
        COM_msgExtract(&p_msg->data, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2]);
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

/**
 * @brief           SCI Tx FIFO level reached - ready to receive new data.
 */
__attribute__((interrupt)) void sci_isrTx_RS485(void)
{
    DBG_PIN2_SET;
    // FIFO IT source test.
    uint32_t sci_it_src_test = SCI_getInterruptStatus(COM_SCI_BASE);
    // FIFO TX IT?
    if(sci_it_src_test & SCI_INT_TXFF)
    {
        RS485_msgTransmit(COM_SCI_BASE, &com_rs485_tx);
        if(com_rs485_tx.index >= COM_MSG_RS485_TX_8BIT_FULL_LENGTH)
        {
#if USE_FCT
            SCI_disableInterrupt(COM_SCI_BASE, SCI_INT_TXFF);
#else
            HWREGH(COM_SCI_BASE + SCI_O_FFTX) &= ~SCI_FFTX_TXFFIENA;
#endif
            com_rs485_tx.index = 0;
        }
#if USE_FCT
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_TXFF);
#else
        HWREGH(COM_SCI_BASE + SCI_O_FFTX) |= SCI_FFTX_TXFFINTCLR;
#endif
    }
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_TX_MSG_GROUP);
    DBG_PIN2_CLEAR;
}

/**
 *@brief            SCI Rx FIFO level reached + slave address valid (IT enable).
 */
__attribute__((interrupt)) void sci_isrRx_RS485(void)
{
    DBG_PIN2_SET;
    // FIFO IT source test.
    uint32_t sci_it_src_test = SCI_getInterruptStatus(COM_SCI_BASE);
    // FIFO RX IT?
    if(sci_it_src_test & SCI_INT_RXFF)
    {
        // Get contain of FIFO Rx
         RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
        // Clear IT sources (FIFO IT source + FIFO overflow IT source)
#if USE_FCT
         SCI_clearOverflowStatus(COM_SCI_BASE);
         SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
#else
         HWREGH(COM_SCI_BASE + SCI_O_FFRX) |= (SCI_FFRX_RXFFOVRCLR | SCI_FFRX_RXFFINTCLR);
#endif
    }
    /*
     * RX ERROR :
     * FE - SCI framing-error flag.
     * OE - SCI overrun-error flag.
     * PE - SCI parity-error flag.
     */
    if(sci_it_src_test & SCI_INT_RXERR)
    {
#if USE_FCT
        SCI_performSoftwareReset(COM_SCI_BASE);
#else
        HWREGH(COM_SCI_BASE + SCI_O_CTL1)  &= ~SCI_CTL1_SWRESET;
        HWREGH(COM_SCI_BASE + SCI_O_CTL1)  |= SCI_CTL1_SWRESET;
#endif
//        error();
    }
    // Clear global IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_MSG_GROUP);
    DBG_PIN2_CLEAR;
}

/**
 * @brief           CLB - Valid address & EOT ITs.
 */
__attribute__((interrupt)) void clb_isrTxRx_RS485(void)
{
    DBG_PIN3_SET;
#if USE_FCT
    uint16_t clb_tag_test   = CLB_getInterruptTag(RS485_RX_TX_CLB_EOT_BASE);
#else
    uint16_t clb_tag_test   = HWREGH(RS485_RX_TX_CLB_EOT_BASE + CLB_LOGICCTL + CLB_O_INTR_TAG_REG);
#endif

    switch (clb_tag_test)
    {
    case CLB_INT_RS485_RX_ADDRESS_1_VALID:
        // RS485 address #1 valid -> Enable SCI ITs
        com_rs485_rx.index  = 0;
        com_rs485_rx.addr1  = true;
        com_rs485_rx.addr2  = false;
        RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
        // Clear IT source & Enable FIFO Rx IT source
#if USE_FCT
        SCI_clearOverflowStatus(COM_SCI_BASE);
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
        SCI_enableInterrupt(COM_SCI_BASE, SCI_INT_RXERR | SCI_INT_RXFF);
#else
        HWREGH(COM_SCI_BASE + SCI_O_FFRX)  |= (SCI_FFRX_RXFFIENA | SCI_FFRX_RXFFINTCLR | SCI_FFRX_RXFFOVRCLR);
        HWREGH(COM_SCI_BASE + SCI_O_CTL1)  |= SCI_CTL1_RXERRINTENA;
#endif
        break;

    case CLB_INT_RS485_RX_ADDRESS_2_VALID:
        // RS485 address #2 valid -> Enable SCI ITs
        com_rs485_rx.index  = 0;
        com_rs485_rx.addr1  = false;
        com_rs485_rx.addr2  = true;
        RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
        // Clear IT source & Enable FIFO Rx IT source
#if USE_FCT
        SCI_clearOverflowStatus(COM_SCI_BASE);
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
        SCI_enableInterrupt(COM_SCI_BASE, SCI_INT_RXERR | SCI_INT_RXFF);
#else
        HWREGH(COM_SCI_BASE + SCI_O_FFRX)  |= (SCI_FFRX_RXFFIENA | SCI_FFRX_RXFFINTCLR | SCI_FFRX_RXFFOVRCLR);
        HWREGH(COM_SCI_BASE + SCI_O_CTL1)  |= SCI_CTL1_RXERRINTENA;
#endif
        break;

    case CLB_INT_RS485_RX_END_OF_RECEPTION:
        // RS485 End-Of-Reception detection
        if(com_rs485_rx.addr1 || com_rs485_rx.addr2) // Valid address ?
        {
            RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
            rs485_rx_msg_u* p_msg   = &com_rs485_rx.msg;
            uint32_t crc            = COM_msgCRC((uint16_t *) &p_msg->header, COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH);
            if((p_msg->crc.u16_msb == MSB_32(crc)) && (p_msg->crc.u16_lsb == LSB_32(crc)))
            {
                // Message valid - Extract it & transfer to motors structures.
                COM_msgExtract(&p_msg->data, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2]);
                CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
                // prepare reply to master - reset transmit index
                com_rs485_tx.index  = 0;
                // prepare reply to master - reset RS485 transmit FIFO & enable RS485 SCI Tx ITs.
#if (1)
#if USE_FCT
                SCI_resetTxFIFO(COM_SCI_BASE);
                SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_TXFF);
                SCI_enableInterrupt(COM_SCI_BASE, SCI_INT_TXFF);
#else
                HWREGH(COM_SCI_BASE + SCI_O_FFTX) &= ~SCI_FFTX_TXFIFORESET;
                HWREGH(COM_SCI_BASE + SCI_O_FFTX) |= (SCI_FFTX_TXFIFORESET | SCI_FFTX_TXFFIENA | SCI_FFTX_TXFFINTCLR);
#endif
                // Inform CLB that next message is transmitted by slave
#if USE_FCT
                uint32_t clb_gpreg  = CLB_getGPREG(RS485_RX_TX_CLB_EOT_BASE);
                clb_gpreg          |= CLB_GPREG_TX_ENABLE;
                CLB_setGPREG(RS485_RX_TX_CLB_EOT_BASE, clb_gpreg);
#else
                HWREG(RS485_RX_TX_CLB_EOT_BASE + CLB_LOGICCTL + CLB_O_GP_REG) |= CLB_GPREG_TX_ENABLE;
#endif
                // Toggle GP reg.
#if USE_FCT
                clb_gpreg          &= ~CLB_GPREG_TX_ENABLE;
                CLB_setGPREG(RS485_RX_TX_CLB_EOT_BASE, clb_gpreg);
#else
                HWREG(RS485_RX_TX_CLB_EOT_BASE + CLB_LOGICCTL + CLB_O_GP_REG) &= ~CLB_GPREG_TX_ENABLE;
#endif
#endif
            }
        }
        /*
         * Force a reset of SCI Rx FIFO & disable SCI Rx ITs while a valid address is not received.
         * Clear IT sources.
         */
#if USE_FCT
        SCI_resetRxFIFO(COM_SCI_BASE);
        SCI_clearOverflowStatus(COM_SCI_BASE);
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
        SCI_disableInterrupt(COM_SCI_BASE, SCI_INT_RXFF);
#else
        HWREGH(COM_SCI_BASE + SCI_O_FFRX)  &= ~(SCI_FFRX_RXFIFORESET | SCI_FFRX_RXFFIENA);
        HWREGH(COM_SCI_BASE + SCI_O_FFRX)  |= (SCI_FFRX_RXFIFORESET | SCI_FFRX_RXFFOVRCLR | SCI_FFRX_RXFFINTCLR);
#endif
        // Purge pending SCI Rx ITs.
        HWREGH(PIECTRL_BASE + PIE_O_IFR8)  &= ~((uint16_t)INT_RS485_RX_PIE_FLAG);
        HWREGH(PIECTRL_BASE + PIE_O_ACK)    = INT_RS485_RX_MSG_GROUP;
        IFR                                &= ~((uint16_t)INT_RS485_RX_MSG_GROUP);
        // Reset flags & index. Waiting for a new valid message.
        com_rs485_rx.index  = 0;
        com_rs485_rx.addr1  = false;
        com_rs485_rx.addr2  = false;
        break;

    case CLB_INT_RS485_TX_END_OF_TRANSMISSION:
        // RS485 End-Of-Transmission detection
#if USE_FCT
        SCI_resetTxFIFO(COM_SCI_BASE);
#else
        HWREGH(COM_SCI_BASE + SCI_O_FFTX) &= ~SCI_FFTX_TXFIFORESET;
        HWREGH(COM_SCI_BASE + SCI_O_FFTX) |= SCI_FFTX_TXFIFORESET;
#endif
        com_rs485_tx.index = 0;
//        // Purge pending SCI Rx ITs.
//        HWREGH(PIECTRL_BASE + PIE_O_IFR8)  &= ~((uint16_t)INT_RS485_RX_PIE_FLAG);
//        HWREGH(PIECTRL_BASE + PIE_O_ACK)    = INT_RS485_RX_MSG_GROUP;
//        IFR                                &= ~((uint16_t)INT_RS485_RX_MSG_GROUP);
        break;

    default:
        break;
    }
//
//    /*
//     * RS485 address #1 valid -> Enable SCI ITs
//     */
//    if(clb_tag_test == CLB_INT_RS485_RX_ADDRESS_1_VALID)
//    {
//        com_rs485_rx.index  = 0;
//        com_rs485_rx.addr1  = true;
//        com_rs485_rx.addr2  = false;
//        RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
//        // Clear IT source & Enable FIFO Rx IT source
//#if USE_FCT
//        SCI_clearOverflowStatus(COM_SCI_BASE);
//        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
//        SCI_enableInterrupt(COM_SCI_BASE, SCI_INT_RXERR | SCI_INT_RXFF);
//#else
//        HWREGH(COM_SCI_BASE + SCI_O_FFRX)  |= (SCI_FFRX_RXFFIENA | SCI_FFRX_RXFFINTCLR | SCI_FFRX_RXFFOVRCLR);
//        HWREGH(COM_SCI_BASE + SCI_O_CTL1)  |= SCI_CTL1_RXERRINTENA;
//#endif
//    }
//
//    /*
//     * RS485 address #2 valid -> Enable SCI ITs
//     */
//    if(clb_tag_test == CLB_INT_RS485_RX_ADDRESS_2_VALID)
//    {
//        com_rs485_rx.index  = 0;
//        com_rs485_rx.addr1  = false;
//        com_rs485_rx.addr2  = true;
//        RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
//        // Clear IT source & Enable FIFO Rx IT source
//#if USE_FCT
//        SCI_clearOverflowStatus(COM_SCI_BASE);
//        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
//        SCI_enableInterrupt(COM_SCI_BASE, SCI_INT_RXERR | SCI_INT_RXFF);
//#else
//        HWREGH(COM_SCI_BASE + SCI_O_FFRX)  |= (SCI_FFRX_RXFFIENA | SCI_FFRX_RXFFINTCLR | SCI_FFRX_RXFFOVRCLR);
//        HWREGH(COM_SCI_BASE + SCI_O_CTL1)  |= SCI_CTL1_RXERRINTENA;
//#endif
//    }
//
//    /*
//     * RS485 End-Of-Reception detection
//     */
//    if(clb_tag_test == CLB_INT_RS485_RX_END_OF_RECEPTION)
//    {
//        if(com_rs485_rx.addr1 || com_rs485_rx.addr2) // Valid address ?
//        {
//            RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
//            rs485_rx_msg_u* p_msg   = &com_rs485_rx.msg;
//            uint32_t crc            = COM_msgCRC((uint16_t *) &p_msg->header, COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH);
//            if((p_msg->crc.u16_msb == MSB_32(crc)) && (p_msg->crc.u16_lsb == LSB_32(crc)))
//            {
//                // Message valid - Extract it & transfer to motors structures.
//                COM_msgExtract(&p_msg->data, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2]);
//                CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
//                // prepare reply to master - reset transmit index
//                com_rs485_tx.index  = 0;
//                // prepare reply to master - reset RS485 transmit FIFO & enable RS485 SCI Tx ITs.
//#if (1)
//#if USE_FCT
//                SCI_resetTxFIFO(COM_SCI_BASE);
//                SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_TXFF);
//                SCI_enableInterrupt(COM_SCI_BASE, SCI_INT_TXFF);
//#else
//                HWREGH(COM_SCI_BASE + SCI_O_FFTX) &= ~SCI_FFTX_TXFIFORESET;
//                HWREGH(COM_SCI_BASE + SCI_O_FFTX) |= (SCI_FFTX_TXFIFORESET | SCI_FFTX_TXFFIENA | SCI_FFTX_TXFFINTCLR);
//#endif
//                // Inform CLB that next message is transmitted by slave
//#if USE_FCT
//                uint32_t clb_gpreg  = CLB_getGPREG(RS485_RX_TX_CLB_EOT_BASE);
//                clb_gpreg          |= CLB_GPREG_TX_ENABLE;
//                CLB_setGPREG(RS485_RX_TX_CLB_EOT_BASE, clb_gpreg);
//#else
//                HWREG(RS485_RX_TX_CLB_EOT_BASE + CLB_LOGICCTL + CLB_O_GP_REG) |= CLB_GPREG_TX_ENABLE;
//#endif
//                // Toggle GP reg.
//#if USE_FCT
//                clb_gpreg          &= ~CLB_GPREG_TX_ENABLE;
//                CLB_setGPREG(RS485_RX_TX_CLB_EOT_BASE, clb_gpreg);
//#else
//                HWREG(RS485_RX_TX_CLB_EOT_BASE + CLB_LOGICCTL + CLB_O_GP_REG) &= ~CLB_GPREG_TX_ENABLE;
//#endif
//#endif
//            }
//        }
//        /*
//         * Force a reset of SCI Rx FIFO & disable SCI Rx ITs while a valid address is not received.
//         * Clear IT sources.
//         */
//#if USE_FCT
//        SCI_resetRxFIFO(COM_SCI_BASE);
//        SCI_clearOverflowStatus(COM_SCI_BASE);
//        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
//        SCI_disableInterrupt(COM_SCI_BASE, SCI_INT_RXFF);
//#else
//        HWREGH(COM_SCI_BASE + SCI_O_FFRX)  &= ~(SCI_FFRX_RXFIFORESET | SCI_FFRX_RXFFIENA);
//        HWREGH(COM_SCI_BASE + SCI_O_FFRX)  |= (SCI_FFRX_RXFIFORESET | SCI_FFRX_RXFFOVRCLR | SCI_FFRX_RXFFINTCLR);
//#endif
//        // Purge pending SCI Rx ITs.
//        HWREGH(PIECTRL_BASE + PIE_O_IFR8)  &= ~((uint16_t)INT_RS485_RX_PIE_FLAG);
//        HWREGH(PIECTRL_BASE + PIE_O_ACK)    = INT_RS485_RX_MSG_GROUP;
//        IFR                                &= ~((uint16_t)INT_RS485_RX_MSG_GROUP);
//
//        // Reset flags & index. Waiting for a new valid message.
//        com_rs485_rx.index  = 0;
//        com_rs485_rx.addr1  = false;
//        com_rs485_rx.addr2  = false;
//    }
//
//    /*
//     * RS485 End-Of-Transmission detection
//     */
//    if(clb_tag_test == CLB_INT_RS485_TX_END_OF_TRANSMISSION)
//    {
//#if USE_FCT
//        SCI_resetTxFIFO(COM_SCI_BASE);
//#else
//        HWREGH(COM_SCI_BASE + SCI_O_FFTX) &= ~SCI_FFTX_TXFIFORESET;
//        HWREGH(COM_SCI_BASE + SCI_O_FFTX) |= SCI_FFTX_TXFIFORESET;
//#endif
////#if USE_FCT
////        SCI_disableInterrupt(COM_SCI_BASE, SCI_INT_TXFF);
////        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_TXFF);
////#else
////        HWREGH(COM_SCI_BASE + SCI_O_FFTX) &= ~SCI_FFTX_TXFFIENA;
////        HWREGH(COM_SCI_BASE + SCI_O_FFTX) |= SCI_FFTX_TXFFINTCLR;
////#endif
//        com_rs485_tx.index = 0;
//    }

    // Clear CLB Flag
    CLB_clearInterruptTag(RS485_RX_TX_CLB_EOT_BASE);
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_Tx_CLB_GROUP);
    DBG_PIN3_CLEAR;
}
