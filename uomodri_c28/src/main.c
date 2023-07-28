/*
 * File name: main.c
 * Description: Source file containing main function and IT running on CPU1 - C28 processor.
 */

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "driverlib.h"
#include "device.h"

#include "main.h"
#include "hal.h"
#include "drv8353.h"
#include "ws2812b.h"
//#include "encoder.h"
//#include "foc.h"
#include "motor.h"
#include "communication.h"
#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"
#include "uomodri_prj_config_handlers.h"

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
#pragma DATA_ALIGN(cmd_msg_otf, 4)
#pragma DATA_SECTION(cmd_msg_otf, "ramgs0")
mst2slv_msg_otf_t   cmd_msg_otf;
//= {.msg_rx_state = COM_RX_STATE_MODE,
//                               .msg_crc32 = UINT32_MAX,
//                               .msg_tmp = 0U,
//};

#pragma DATA_ALIGN(cmd_msg, 4)
#pragma DATA_SECTION(cmd_msg, "ramgs0")
mst2slv_msg_t       cmd_msg;

#pragma DATA_ALIGN(status_msg, 4)
#pragma DATA_SECTION(status_msg, "ramgs0")
slv2mst_msg_t       status_msg[2];

#pragma DATA_ALIGN(led_msg, 4)
#pragma DATA_SECTION(led_msg, "ramgs0")
uint16_t            led_msg[LED_MSG_TX_16BIT_LENGTH];

//#pragma DATA_ALIGN(tx_msg_cr, 2)
//#pragma DATA_SECTION(tx_msg_cr, "ramgs0");
uint16_t tx_msg_cr      = 0U;
//#pragma DATA_ALIGN(tx_msg_nx, 2)
//#pragma DATA_SECTION(tx_msg_nx, "ramgs0");
uint16_t tx_msg_nx      = 1U;
//#pragma DATA_ALIGN(tx_msg_full, 2)
//#pragma DATA_SECTION(tx_msg_full, "ramgs0");
bool_t tx_msg_full      = false;
//#pragma DATA_ALIGN(tmr_timeout, 2)
//#pragma DATA_SECTION(tmr_timeout, "ramgs0");
//bool_t tmr_timeout      = false;
bool_t rx_msg_valid     = false;

extern motor_t          motor_m1;
extern motor_t          motor_m2;
extern const hal_cfg_t  hal;
extern cmd_t            cmd_uOmodri[2];

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
static void MCU_ini(void);
inline void LED_dimming(motor_t*, motor_t*, uint16_t*);
extern void MOT_initStruct(motor_t*, uint16_t);

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
    // init shared structure used by CLA & C28
//    MOT_initStruct(&motor_m1, MOTOR_1);
//    MOT_initStruct(&motor_m2, MOTOR_2);
    // Hardware Abstraction Layer initialization
    HAL_ini(&hal);
    // DRV8353 initialization
    DRV_ini(&drv_cfg[MOTOR_1], &drv_reg[MOTOR_1]);
    DRV_ini(&drv_cfg[MOTOR_2], &drv_reg[MOTOR_2]);
    // ADC offset calibration for debug
    HAL_ADC_offsetCalib(&periph_motor_cfg[MOTOR_1]);
    HAL_ADC_offsetCalib(&periph_motor_cfg[MOTOR_2]);
    // Interrupts initialization
    HAL_INT_ini(InterruptList);

    COM_resetCmdStruct(&cmd_uOmodri[MOTOR_1]);
    COM_resetCmdStruct(&cmd_uOmodri[MOTOR_2]);
    // Reset the message extraction On-The-Fly structure
    COM_resetMsgExtractOnTheFlyStruct(&cmd_msg_otf);
    // Infinite loop
    while(1);
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
#if (defined USB_BUS_ENABLE_ON_CPU1) || ((defined USB_BUS_ENABLE_ON_CM) && (defined CM_CORE_ENABLE))
    /* Redefined to pass to SysCtl_setAuxClock(). Will configure the clock as follows:
     * AUXPLLCLK = 25MHz (XTAL_OSC) * 48 (IMULT) / (2 (REFDIV) * 5 (ODIV) * 2(AUXPLLDIV) )
     * AUXPLLCLK require a 60MHz clock for USB. AUPLLRAWCLK require a 120MHz clock for CM. */
#undef DEVICE_AUXSETCLOCK_CFG
#define DEVICE_AUXSETCLOCK_CFG      (SYSCTL_AUXPLL_OSCSRC_XTAL  | \
                                     SYSCTL_AUXPLL_IMULT(48)    | \
                                     SYSCTL_AUXPLL_REFDIV(2U)   | \
                                     SYSCTL_AUXPLL_ODIV(5U)     | \
                                     SYSCTL_AUXPLL_DIV_2        | \
                                     SYSCTL_AUXPLL_ENABLE       | \
                                     SYSCTL_DCC_BASE_0)
#undef DEVICE_AUXCLK_FREQ
#define DEVICE_AUXCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 48) / (2 * 5 * 2))
#endif
    /* Initialize clock and peripherals */
    Device_init();
    /* Redefine LSPCLK to run at 200MHz (for SPI & SCI bit clocks) */
//#undef DEVICE_LSPCLK_FREQ
//#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ / 1)
    /* Set LSPCLK divider */
//    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_1);
//    ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == UOMODRI_LSPCLK_FREQ);
#if ((defined USB_BUS_ENABLE_ON_CM) && (defined CM_CORE_ENABLE))
    /* Allocate Shared Peripheral USB to the CM Side. */
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
#endif
    /* Boot CM core */
#if (defined _FLASH) && (defined CM_CORE_ENABLE)
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#elif (defined CM_CORE_ENABLE)
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
    static uint16_t timer_cpt_ms    = 0;
    float32_t       timer_fade_blink= (float32_t)((timer_cpt_ms < 500)  ? (timer_cpt_ms)    : (1000 - timer_cpt_ms)) / 500.0f;
    float32_t       timer_blink     = (timer_cpt_ms < 500)              ? (0.0f)            : (1.0f);
    timer_cpt_ms                    = (timer_cpt_ms < 1000)             ? (timer_cpt_ms + 1): (0);
    uint8_t         motor_state     = MOTOR_STATE_IDLE;

    if((p_motor1->state == MOTOR_STATE_IDLE)            || (p_motor2->state == MOTOR_STATE_IDLE))
        motor_state = MOTOR_STATE_IDLE;
    else if((p_motor1->state == MOTOR_STATE_INIT_FW)    || (p_motor1->state == MOTOR_STATE_INIT_REV)    ||
            (p_motor1->state == MOTOR_STATE_INIT_TEST)  || (p_motor1->state == MOTOR_STATE_INIT_FIX)    ||
            (p_motor2->state == MOTOR_STATE_INIT_FW)    || (p_motor2->state == MOTOR_STATE_INIT_REV)    ||
            (p_motor2->state == MOTOR_STATE_INIT_TEST)  || (p_motor2->state == MOTOR_STATE_INIT_FIX))
        motor_state = MOTOR_STATE_INIT_FIX;
    else if((p_motor1->state == MOTOR_STATE_READY)      || (p_motor1->state == MOTOR_STATE_STOP)        ||
            (p_motor2->state == MOTOR_STATE_READY)      || (p_motor2->state == MOTOR_STATE_STOP))
        motor_state = MOTOR_STATE_READY;
    if((p_motor1->state == MOTOR_STATE_ERROR)           || (p_motor2->state == MOTOR_STATE_ERROR))
        motor_state = MOTOR_STATE_ERROR;

    switch(motor_state)
    {
    case MOTOR_STATE_INIT_FW:
    case MOTOR_STATE_INIT_REV:
    case MOTOR_STATE_INIT_TEST:
    case MOTOR_STATE_INIT_FIX:
        ws2812b_writeLed((uint16_t)(0xff * timer_fade_blink), 0, (uint16_t)(0xff * timer_fade_blink), p_msg); //Magenta fade,
        break;
    case MOTOR_STATE_READY:
    case MOTOR_STATE_STOP:
        ws2812b_writeLed(0, (uint16_t)(0xff * timer_fade_blink), 0, p_msg);//Green fade, Active control
        break;
    case MOTOR_STATE_ERROR:
        ws2812b_writeLed((uint16_t)(0xff * timer_blink), 0, 0, p_msg);//red fade, error control
        break;
    case MOTOR_STATE_IDLE:
    default:
        ws2812b_writeLed(0,  0, (uint16_t)(0xff * timer_fade_blink), p_msg);//blue fade
    }

    return;
}

__attribute__((interrupt)) void cla_isrMotor1(void)
{
    COM_msgCreateLite(&motor_m1, &status_msg[tx_msg_nx]);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);//InterruptList[0].intAckGroup);
}

__attribute__((interrupt)) void cla_isrMotor2(void)
{
    COM_msgCreateLite(&motor_m2, &status_msg[tx_msg_nx]);
    if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && COM_SPI_CS_READ)
    {
        tx_msg_cr   = (tx_msg_cr) ? (0U) : (1U);
        tx_msg_nx   = (tx_msg_cr) ? (0U) : (1U);
        SPI_resetTxFIFO(COM_SPI_BASE);
        // Force first write in SPI_O_DAT
        COM_SPI_DAT(status_msg[tx_msg_cr].status.all);
        DMA_configAddresses(DMA_COM_SPI_TX_BASE_ADDR, (uint16_t *) COM_SPI_TX_BASE_REG_ADDR, (uint16_t *) &status_msg[tx_msg_cr].timeStamp);
        DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
    }
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);//InterruptList[1].intAckGroup);
}

__attribute__((interrupt)) void tmr_isrLed(void)
{
    // LED dimming control
    if(!DMA_getRunStatusFlag(DMA_LED_SPI_TX_BASE_ADDR))
    {
        LED_dimming(&motor_m1, &motor_m2, &led_msg[0]);
        DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
    }
    // Timeout management
    cmd_uOmodri[MOTOR_1].cptTimeout++;
    cmd_uOmodri[MOTOR_2].cptTimeout++;
    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_2);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__attribute__((interrupt)) void spi_isrComRx(void)
{
    while((SPI_getRxFIFOStatus(COM_SPI_BASE)) && (!rx_msg_valid))
        rx_msg_valid = COM_msgExtractOnTheFly(&cmd_msg_otf, &cmd_uOmodri[MOTOR_1], &cmd_uOmodri[MOTOR_2], SPI_readDataNonBlocking(COM_SPI_BASE));
    if(rx_msg_valid)
    {
        cmd_uOmodri[MOTOR_1].cptTimeout = 0U;
        cmd_uOmodri[MOTOR_2].cptTimeout = 0U;
        CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_2);
        rx_msg_valid                    = false;
        COM_resetMsgExtractOnTheFlyStruct(&cmd_msg_otf);
    }
    if(COM_SPI_CS_READ)
        SPI_resetRxFIFO(COM_SPI_BASE);
    SPI_clearInterruptStatus(COM_SPI_BASE, SPI_INT_RXFF);// | SPI_INT_RXFF_OVERFLOW);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
}

__attribute__((interrupt)) void dma_isrComRx(void)
{
//    if(!DMA_getRunStatusFlag(DMA_COM_SPI_RX_BASE_ADDR) && GPIO_readPin(COM_SPI_CS_NUM))
//    {
        // Prepare SPI and associated DMA for a new receive
//        SPI_resetRxFIFO(COM_SPI_BASE);
        DMA_clearTriggerFlag(DMA_COM_SPI_RX_BASE_ADDR);
        // Extract received message & send new commands to FOC structures if valid
        rx_msg_valid       = COM_msgExtract(&cmd_msg, &cmd_uOmodri[MOTOR_1], &cmd_uOmodri[MOTOR_2]);
        // Restart DMA reception
        DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
        if(rx_msg_valid)
            {
                cmd_uOmodri[MOTOR_1].cptTimeout = 0U;
                cmd_uOmodri[MOTOR_2].cptTimeout = 0U;
                CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_2);
                rx_msg_valid                    = false;
//                COM_resetMsgExtractOnTheFlyStruct(&cmd_msg);
            }
            if(COM_SPI_CS_READ)
                SPI_resetRxFIFO(COM_SPI_BASE);
//            SPI_clearInterruptStatus(COM_SPI_BASE, SPI_INT_RXFF);// | SPI_INT_RXFF_OVERFLOW);
            Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
//    }
}

