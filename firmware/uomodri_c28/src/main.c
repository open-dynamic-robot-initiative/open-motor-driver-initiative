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

#include "uomodri_user_defines.h"
#include "config/uomodri_math.h"

#if (USE_UOMODRI_REV == USE_UOMODRI_V2)
#include "config/uomodri_2/uomodri_2_handlers.h"    // uOmodri v2.x configuration include file
#elif (USE_UOMODRI_REV == USE_UOMODRI_V3)
#include "config/uomodri_3/uomodri_3_handlers.h"    // uOmodri v3.x configuration include file
#endif

#include "communication.h"
#include "motor_ctrl.h"
#include "drv8353.h"
#include "ws2812b.h"
#include "main.h"
#include "hal.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void main(void);
static void MCU_ini(void);
inline void LED_dimming(motor_t*, motor_t*, uint16_t*);
void MOT_initStruct(motor_t*, uint16_t);

__attribute__((interrupt)) void cla_isrClearTask(void);
__attribute__((interrupt)) void cla_isrMotor1(void);
__attribute__((interrupt)) void cla_isrMotor2(void);
__attribute__((interrupt)) void tmr_isrLedCmd(void);
__attribute__((interrupt)) void dma_isrSpiComRx(void);
__attribute__((interrupt)) void sci_isrTx_RS485(void);
__attribute__((interrupt)) void sci_isrRx_RS485(void);
__attribute__((interrupt)) void clb_isrTxRx_RS485(void);
__attribute__((interrupt)) void errata_isr(void);

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
/** @var    int_cfg_t   interruptList[]
 *  @brief  List all required interrupts configurations.
 */
static const int_cfg_t interruptList[] =
{
 INT_ERRATA_DEF,
 INT_CLA_ADC_CALIB_DEF,
 INT_CLA_MOTOR1_DEF,
 INT_CLA_MOTOR2_DEF,
 INT_TIMER_LED_DEF,
 INT_DMA_RX_MSG_DEF,
#if (RS485_BUS_ENABLE) && (USE_UOMODRI_REV != USE_UOMODRI_V1)
 INT_RS485_TX_MSG_DEF,
 INT_RS485_RX_MSG_DEF,
 INT_RS485_Rx_Tx_CLB_DEF,
#endif
 {.intNum = UINT32_MAX},        /* END OF ARRAY */
};

#pragma DATA_ALIGN(motor_m1, 4)
#pragma DATA_SECTION(motor_m1, "ClaDataMotor1");
motor_t     motor_m1;

#pragma DATA_ALIGN(motor_m2, 4)
#pragma DATA_SECTION(motor_m2, "ClaDataMotor2");
motor_t     motor_m2;

#pragma DATA_ALIGN(cmd_uomodri, 4)
#pragma DATA_SECTION(cmd_uomodri, "MSGRAM_CPU_TO_CLA")
cmd_t       cmd_uomodri[2];

#pragma DATA_ALIGN(com_spi_rx, 4)
#pragma DATA_SECTION(com_spi_rx, "ramgs0")
spi_rx_t    com_spi_rx;

#pragma DATA_ALIGN(com_spi_tx, 4)
#pragma DATA_SECTION(com_spi_tx, "ramgs0")
spi_tx_t    com_spi_tx      = {.msg_cr = 0U, .msg_nx = 1U,
                               .msg[0].crc.u16_msb = 0xDFFF, .msg[0].crc.u16_lsb = 0x5283,
                               .msg[1].crc.u16_msb = 0xDFFF, .msg[1].crc.u16_lsb = 0x5283};
#pragma DATA_ALIGN(com_rs485_rx, 4)
#pragma DATA_SECTION(com_rs485_rx, "ramgs0")
rs485_rx_t  com_rs485_rx    = {.index = 0U,
                               .addr1 = false, .addr2 = false};

#pragma DATA_ALIGN(com_rs485_tx, 4)
#pragma DATA_SECTION(com_rs485_tx, "ramgs0")
rs485_tx_t  com_rs485_tx    = {.index = 0U,
                               .msg_cr = 0U, .msg_nx = 1U,
                               .msg[0].crc.u16_msb = 0x8CA0, .msg[0].crc.u16_lsb = 0x22CF,
                               .msg[1].crc.u16_msb = 0x8CA0, .msg[1].crc.u16_lsb = 0x22CF};

#pragma DATA_ALIGN(led_spi_tx, 4)
#pragma DATA_SECTION(led_spi_tx, "ramgs0")
uint16_t    led_spi_tx[LED_MSG_TX_16BIT_LENGTH];

#pragma DATA_ALIGN(com_bus_select, 2)
#pragma DATA_SECTION(com_bus_select, "ramgs0")
com_bus_e   com_bus_select      = COM_NO_BUS_DEFINED;

#pragma DATA_ALIGN(global_1ms_tmr_cnt, 2)
#pragma DATA_SECTION(global_1ms_tmr_cnt, "ramgs0")
uint32_t    global_1ms_tmr_cnt  = 0U;

#pragma DATA_ALIGN(global_errata_cnt, 2)
#pragma DATA_SECTION(global_errata_cnt, "ramgs0")
uint32_t    global_errata_cnt   = 0U;

/***********************************************************************
 * EXTERNAL VARIABLES
 ***********************************************************************/
extern const hal_cfg_t  hal;

/***********************************************************************
 * FUNCTIONS DEFINITION
 ***********************************************************************/
/**
 * @brief   	Main function running on CPU1/C28.
 *              - MCU init,
 *              - motors structures init,
 *              - communication structure init,
 *              - MCU peripherals init,
 *              - DRV init,
 *              - ADC task calibration,
 *              - IT init.
 */
void main(void)
{
    // uOmodri low level device initialization
    MCU_ini();
    // Initialization of the shared structure used by CLA & C28 for Motor_1 & Motor_2
    MOT_initStruct(&motor_m1, MOTOR_1);
    MOT_initStruct(&motor_m2, MOTOR_2);
    // Initialization the command structure for CLA.
    COM_initCmdStruct(&cmd_uomodri[MOTOR_1]);
    COM_initCmdStruct(&cmd_uomodri[MOTOR_2]);
    // Hardware Abstraction Layer initialization
    HAL_ini(&hal);
    // DRV8353 initialization
    cmd_uomodri[MOTOR_1].periphField.drv_initErr = DRV_ini(&drv_cfg[MOTOR_1], &drv_reg[MOTOR_1]);
    cmd_uomodri[MOTOR_2].periphField.drv_initErr = DRV_ini(&drv_cfg[MOTOR_2], &drv_reg[MOTOR_2]);
    // Start ADC offset calibration on CLA
    CLA_forceTasks(CLA1_BASE, CLA_ADC_CALIB_TASK_FLAG);
    // Interrupts initialization
    HAL_INT_ini(interruptList);
//    // SPI check parameters
//    if((COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH != COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH) || \
//            (COM_MSG_SPI_TX_16BIT_FULL_LENGTH != COM_MSG_SPI_RX_16BIT_FULL_LENGTH))
//    error "Error : SPI Tx & Rx must have the same length."

    // Infinite loop
    while(1);
}

/**
 * @brief       Microcontroller initialization function
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
#endif /* DEBUG */
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

#if (defined USB_BUS_ENABLE_ON_CM) && (!defined CPU1)
    /* Allocate Shared Peripheral USB to the CM Side. */
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
#endif
    /* Boot CM core */
#if (defined _FLASH) && (!defined CPU1)
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#elif (!defined CPU1)
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif
    // Initialize GPIO
    Device_initGPIO();
    /* Initializes PIE and clears PIE registers. Disables CPU interrupts. */
    Interrupt_initModule();
    /* Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR). */
    Interrupt_initVectorTable();

    return;
}

/**
 * @brief       Control the led dimming
 * @param[in]   p_motor Pointer on data structure array used for motor control.
 * @param[out]  p_msg   Pointer on the message to be sent to led via SPI.
 */
inline void LED_dimming(motor_t* p_motor1, motor_t* p_motor2, uint16_t* p_msg)
{
    static uint16_t timer_cpt_ms    = 0;
//    uint16_t        i               = 0;
    motor_state_e   motor_state     = MOTOR_STATE_IDLE;

    /* Set status of the 1st led */
    if((p_motor1->state & MOTOR_STATE_BOOT)     || (p_motor2->state & MOTOR_STATE_BOOT))
        motor_state = MOTOR_STATE_IDLE;
    else if((p_motor1->state & MOTOR_STATE_INIT)|| (p_motor2->state & MOTOR_STATE_INIT))
        motor_state = MOTOR_STATE_INIT_FIX;
    else if((p_motor1->state & MOTOR_STATE_OK)  || (p_motor2->state & MOTOR_STATE_OK))
        motor_state = MOTOR_STATE_READY;
    if((p_motor1->state == MOTOR_STATE_ERROR)   || (p_motor2->state == MOTOR_STATE_ERROR))
        motor_state = MOTOR_STATE_ERROR;

    // Master LED (1st led)
    p_msg       = ws2812b_dimmingLed(MASTER_LED, motor_state, timer_cpt_ms, p_msg);
    //MOTOR_1 LED (2nd led)
    p_msg       = ws2812b_dimmingLed(MOTOR1_LED, p_motor1->state, timer_cpt_ms, p_msg);
    //MOTOR_2 LED (3th led)
    p_msg       = ws2812b_dimmingLed(MOTOR2_LED, p_motor2->state, timer_cpt_ms, p_msg);
    // millisecond timer increment
    timer_cpt_ms= (timer_cpt_ms < 1000U) ? (timer_cpt_ms + 1): (0U);

    return;
}

/**
 * @brief       Global structure initialization for MOTOR_1 \& MOTOR_2
 * @param[in]   *p_motor    Pointer on the associated motor structure
 * @param[in]   motor_id    Motor identifier.
 */
void MOT_initStruct(motor_t* p_motor, uint16_t motor_id)
{
    if(motor_id == MOTOR_1)
    {
        //--- MOTOR_1 -----------------------------------------------------------
        p_motor->id                                     = MOTOR_1;
        //--- FOC structure -----------------------------------------------------
        //--- MOTOR PARAMS STRUCTURE --------------------------------------------
        p_motor->foc.cfg.Rs                             = MOTOR1_RS;
        p_motor->foc.cfg.Ls                             = MOTOR1_LS;
        p_motor->foc.cfg.kv                             = MOTOR1_KV;
        p_motor->foc.cfg.ke                             = MOTOR1_KE;
        p_motor->foc.cfg.ki                             = MOTOR1_KI;
        p_motor->foc.cfg.polePairs                      = MOTOR1_POLES_PAIRS;
        //--- MOTOR ACQUISITION STRUCTURE ---------------------------------------
        p_motor->foc.acq.iabcCfgBase[CHANNEL_1]         = MOTOR1_IA_ADC_CFG_ADDR;
        p_motor->foc.acq.iabcCfgBase[CHANNEL_2]         = MOTOR1_IB_ADC_CFG_ADDR;
        p_motor->foc.acq.iabcCfgBase[CHANNEL_3]         = MOTOR1_IC_ADC_CFG_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_1]      = MOTOR1_IA_ADC_RESULT_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_2]      = MOTOR1_IB_ADC_RESULT_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_3]      = MOTOR1_IC_ADC_RESULT_ADDR;
        p_motor->foc.acq.vExtResultBase                 = VEXT1_ADC_RESULT_ADDR;
        p_motor->foc.acq.ppbNum.ia                      = MOTOR1_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.ib                      = MOTOR1_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.ic                      = MOTOR1_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.vExt                    = VEXT1_PPB_NUM;
        p_motor->foc.acq.socNum.ia                      = MOTOR1_IABC_SOC_NUM;
        p_motor->foc.acq.socNum.ib                      = MOTOR1_IABC_SOC_NUM;
        p_motor->foc.acq.socNum.ic                      = MOTOR1_IABC_SOC_NUM;
        p_motor->foc.acq.socNum.vExt                    = VEXT1_SOC_NUM;
        p_motor->foc.acq.intNum.adcInt                  = MOTOR1_IA_INT_NUM;
        //--- MOTOR ENCORDER STRUCTURE ------------------------------------------
        p_motor->foc.enc.eqepBase                       = MOTOR1_QEP_BASE;
        p_motor->foc.enc.polePairs                      = MOTOR1_POLES_PAIRS;
        p_motor->foc.enc.thetaMechScaler                = (float32_t)MOTOR1_ENC_RESOLUTION_SCALE;
        p_motor->foc.enc.speed.speedHighScaler          = (float32_t)MOTOR1_ENC_SPEED_HIGH_SCALE;
        p_motor->foc.enc.speed.speedLowScaler           = (float32_t)MOTOR1_ENC_SPEED_LOW_SCALE;
        p_motor->foc.enc.speed.speedFlt[0].a            = (float32_t)MOTOR1_ENC_SPEED_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[0].one_minus_a  = (float32_t)MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].a            = (float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].one_minus_a  = (float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA;
        //--- MOTOR COMMAND STRUCTURE -------------------------------------------
        p_motor->foc.cmd.iSat                           = MOTOR1_CURRENT_CMD_SAT_MAX;
        //-----------------------------------------------------------------------
        p_motor->foc.iParkFlt.a                         = MOTOR1_CURRENT_LPF_ALPHA;
        p_motor->foc.iParkFlt.one_minus_a               = MOTOR1_CURRENT_LPF_ONE_M_ALPHA;
        //--- FOC - Initialize duty cycle saturation ----------------------------
        p_motor->foc.dtcMax                             = MOTOR1_DTC_MAX;
        p_motor->foc.dtcMin                             = MOTOR1_DTC_MIN;
        //--- FOC - Encoder test ------------------------------------------------
        p_motor->foc.initPosCnt                         = MOTOR1_POSITION_INIT_EVT;
        p_motor->foc.initFixCnt                         = MOTOR1_FIX_INIT_EVT;
        p_motor->foc.initErr[MIN]                       = MOTOR1_POSITION_INIT_ERR_MIN;
        p_motor->foc.initErr[MAX]                       = MOTOR1_POSITION_INIT_ERR_MAX;
        //--- FOC - Initialize PI controller (Id) -------------------------------
        p_motor->foc.piId.kp                            = (float32_t)MOTOR1_PI_ID_KP_COEF;
        p_motor->foc.piId.ki                            = (float32_t)MOTOR1_PI_ID_KI_COEF;
        //--- FOC - Initialize PI controller (Iq) -------------------------------
        p_motor->foc.piIq.kp                            = (float32_t)MOTOR1_PI_IQ_KP_COEF;
        p_motor->foc.piIq.ki                            = (float32_t)MOTOR1_PI_IQ_KI_COEF;
        //--- DRV IO Config -----------------------------------------------------
        p_motor->drvFaultNum                            = MOTOR1_DRV_FAULT_NUM;
        p_motor->drvEnNum                               = MOTOR1_DRV_EN_NUM;
        //---- PWM config address -----------------------------------------------
        p_motor->epwmBase[CHANNEL_1]                    = MOTOR1_PWM1_BASE;
        p_motor->epwmBase[CHANNEL_2]                    = MOTOR1_PWM2_BASE;
        p_motor->epwmBase[CHANNEL_3]                    = MOTOR1_PWM3_BASE;
    }
    else
    {
        //--- MOTOR_2 -----------------------------------------------------------
        p_motor->id                                     = MOTOR_2;
        //--- FOC structure -----------------------------------------------------
        //--- MOTOR PARAMS STRUCTURE --------------------------------------------
        p_motor->foc.cfg.Rs                             = MOTOR2_RS;
        p_motor->foc.cfg.Ls                             = MOTOR2_LS;
        p_motor->foc.cfg.kv                             = MOTOR2_KV;
        p_motor->foc.cfg.ke                             = MOTOR2_KE;
        p_motor->foc.cfg.ki                             = MOTOR2_KI;
        p_motor->foc.cfg.polePairs                      = MOTOR2_POLES_PAIRS;
        //--- MOTOR ACQUISITION STRUCTURE ---------------------------------------
        p_motor->foc.acq.iabcCfgBase[CHANNEL_1]         = MOTOR2_IA_ADC_CFG_ADDR;
        p_motor->foc.acq.iabcCfgBase[CHANNEL_2]         = MOTOR2_IB_ADC_CFG_ADDR;
        p_motor->foc.acq.iabcCfgBase[CHANNEL_3]         = MOTOR2_IC_ADC_CFG_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_1]      = MOTOR2_IA_ADC_RESULT_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_2]      = MOTOR2_IB_ADC_RESULT_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_3]      = MOTOR2_IC_ADC_RESULT_ADDR;
        p_motor->foc.acq.vExtResultBase                 = VEXT2_ADC_RESULT_ADDR;
        p_motor->foc.acq.ppbNum.ia                      = MOTOR2_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.ib                      = MOTOR2_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.ic                      = MOTOR2_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.vExt                    = VEXT2_PPB_NUM;
        p_motor->foc.acq.socNum.ia                      = MOTOR2_IABC_SOC_NUM;
        p_motor->foc.acq.socNum.ib                      = MOTOR2_IABC_SOC_NUM;
        p_motor->foc.acq.socNum.ic                      = MOTOR2_IABC_SOC_NUM;
        p_motor->foc.acq.socNum.vExt                    = VEXT2_SOC_NUM;
        p_motor->foc.acq.intNum.adcInt                  = MOTOR2_IA_INT_NUM;
        //--- MOTOR ENCORDER STRUCTURE ------------------------------------------
        p_motor->foc.enc.eqepBase                       = MOTOR2_QEP_BASE;
        p_motor->foc.enc.polePairs                      = MOTOR2_POLES_PAIRS;
        p_motor->foc.enc.thetaMechScaler                = (float32_t)MOTOR2_ENC_RESOLUTION_SCALE;
        p_motor->foc.enc.speed.speedHighScaler          = (float32_t)MOTOR2_ENC_SPEED_HIGH_SCALE;
        p_motor->foc.enc.speed.speedLowScaler           = (float32_t)MOTOR2_ENC_SPEED_LOW_SCALE;
        p_motor->foc.enc.speed.speedFlt[0].a            = (float32_t)MOTOR2_ENC_SPEED_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[0].one_minus_a  = (float32_t)MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].a            = (float32_t)MOTOR2_ENC_SPEED_LOCAL_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].one_minus_a  = (float32_t)MOTOR2_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA;
        //--- MOTOR COMMAND STRUCTURE -------------------------------------------
        p_motor->foc.cmd.iSat                           = MOTOR2_CURRENT_CMD_SAT_MAX;
        //-----------------------------------------------------------------------
        p_motor->foc.iParkFlt.a                         = MOTOR2_CURRENT_LPF_ALPHA;
        p_motor->foc.iParkFlt.one_minus_a               = MOTOR2_CURRENT_LPF_ONE_M_ALPHA;
        //--- FOC - Initialize duty cycle saturation ----------------------------
        p_motor->foc.dtcMax                             = MOTOR2_DTC_MAX;
        p_motor->foc.dtcMin                             = MOTOR2_DTC_MIN;
        //--- FOC - Encoder test ------------------------------------------------
        p_motor->foc.initPosCnt                         = MOTOR2_POSITION_INIT_EVT;
        p_motor->foc.initFixCnt                         = MOTOR2_FIX_INIT_EVT;
        p_motor->foc.initErr[MIN]                       = MOTOR2_POSITION_INIT_ERR_MIN;
        p_motor->foc.initErr[MAX]                       = MOTOR2_POSITION_INIT_ERR_MAX;
        //--- FOC - Initialize PI controller (Id) -------------------------------
        p_motor->foc.piId.kp                            = (float32_t)MOTOR2_PI_ID_KP_COEF;
        p_motor->foc.piId.ki                            = (float32_t)MOTOR2_PI_ID_KI_COEF;
        //--- FOC - Initialize PI controller (Iq) -------------------------------
        p_motor->foc.piIq.kp                            = (float32_t)MOTOR2_PI_IQ_KP_COEF;
        p_motor->foc.piIq.ki                            = (float32_t)MOTOR2_PI_IQ_KI_COEF;
        //--- DRV IO Config -----------------------------------------------------
        p_motor->drvFaultNum                            = MOTOR2_DRV_FAULT_NUM;
        p_motor->drvEnNum                               = MOTOR2_DRV_EN_NUM;
        //---- PWM config address -----------------------------------------------
        p_motor->epwmBase[CHANNEL_1]                    = MOTOR2_PWM1_BASE;
        p_motor->epwmBase[CHANNEL_2]                    = MOTOR2_PWM2_BASE;
        p_motor->epwmBase[CHANNEL_3]                    = MOTOR2_PWM3_BASE;
    }
    //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
    p_motor->foc.acq.vBusResultBase                     = VBUS_ADC_RESULT_ADDR;
    p_motor->foc.acq.ppbNum.vBus                        = VBUS_PPB_NUM;
    p_motor->foc.acq.socNum.vBus                        = VBUS_SOC_NUM;
    p_motor->foc.acq.vBusFlt.a                          = VBUS_LPF_ALPHA;
    p_motor->foc.acq.vBusFlt.one_minus_a                = VBUS_LPF_ONE_M_ALPHA;
    p_motor->foc.acq.vExtFlt.a                          = VEXT_LPF_ALPHA;
    p_motor->foc.acq.vExtFlt.one_minus_a                = VEXT_LPF_ONE_M_ALPHA;
    //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
    p_motor->foc.enc.thetaMech[NEW]                     = 0.0f;
    p_motor->foc.enc.thetaMech[OLD]                     = 0.0f;
    p_motor->foc.enc.thetaIndexRelative                 = 0.0f;
    p_motor->foc.enc.thetaIndexAbsolute                 = 0.0f;
    p_motor->foc.enc.turnNb                             = 0;
    p_motor->foc.enc.flags.all                          = 0U;
    p_motor->foc.enc.speed.speedMech[0]                 = 0.0f;
    p_motor->foc.enc.speed.speedMech[1]                 = 0.0f;
    p_motor->foc.enc.speed.theta[NEW]                   = 0.0f;
    p_motor->foc.enc.speed.theta[OLD]                   = 0.0f;
    //--- MOTOR COMMAND STRUCTURE -----------------------------------------------
    p_motor->foc.cmd.posRef                             = 0.0f;
    p_motor->foc.cmd.velRef                             = 0.0f;
    p_motor->foc.cmd.iqff                               = 0.0f;
    p_motor->foc.cmd.kpCoeff                            = 0.0f;
    p_motor->foc.cmd.kdCoeff                            = 0.0f;
    p_motor->foc.cmd.timeoutRef                         = 0U;
    p_motor->foc.cmd.cptTimeout                         = 0U;
    p_motor->foc.cmd.index                              = 0U;
    p_motor->foc.cmd.cmdField.all                       = 0U;
    p_motor->foc.cmd.periphField.all                    = 0U;
    //--- FOC - Park transform results ------------------------------------------
    p_motor->foc.id                                     = 0.0f;
    p_motor->foc.iq                                     = 0.0f;
    //--- FOC - Encoder test ----------------------------------------------------
    p_motor->foc.initPosStep                            = 0.0f;
    p_motor->foc.initPosEnc[FW]                         = 0.0f;
    p_motor->foc.initPosEnc[REV]                        = 0.0f;
    //--- FOC - Initialize PI controller (Id) -----------------------------------
    p_motor->foc.piId.p_fb.ptr                          = (float32_t *)(&p_motor->foc.id);
    p_motor->foc.piId.p_set.ptr                         = (float32_t *)(&p_motor->foc.idRef);
    p_motor->foc.piId.p_sat.ptr                         = (float32_t *)(&p_motor->foc.acq.vBus);
    p_motor->foc.piId.err                               = 0.0f;
    p_motor->foc.piId.integral                          = 0.0f;
    p_motor->foc.piId.ff                                = 0.0f;
    p_motor->foc.piId.out                               = 0.0f;
    //--- FOC - Initialize PI controller (Iq) -----------------------------------
    p_motor->foc.piIq.p_fb.ptr                          = (float32_t *)(&p_motor->foc.iq);
    p_motor->foc.piIq.p_set.ptr                         = (float32_t *)(&p_motor->foc.iqRef);
    p_motor->foc.piIq.p_sat.ptr                         = (float32_t *)(&p_motor->foc.acq.vBus);
    p_motor->foc.piIq.err                               = 0.0f;
    p_motor->foc.piIq.integral                          = 0.0f;
    p_motor->foc.piIq.ff                                = 0.0f;
    p_motor->foc.piIq.out                               = 0.0f;
    //--- FOC - Initialize PD controller (Speed & position) ---------------------
    p_motor->foc.pdPosVel.p_fbTheta.ptr                 = (float32_t *)(&p_motor->foc.enc.thetaAbsolute);
    p_motor->foc.pdPosVel.p_setTheta.ptr                = (float32_t *)(&p_motor->foc.cmd.posRef);
    p_motor->foc.pdPosVel.errTheta                      = 0.0f;
    p_motor->foc.pdPosVel.p_fbSpeed.ptr                 = (float32_t *)(&p_motor->foc.enc.speed.speedMech[0]);
    p_motor->foc.pdPosVel.p_setSpeed.ptr                = (float32_t *)(&p_motor->foc.cmd.velRef);
    p_motor->foc.pdPosVel.errSpeed                      = 0.0f;
    p_motor->foc.pdPosVel.p_kp.ptr                      = (float32_t *)(&p_motor->foc.cmd.kpCoeff);
    p_motor->foc.pdPosVel.p_kd.ptr                      = (float32_t *)(&p_motor->foc.cmd.kdCoeff);
    p_motor->foc.pdPosVel.derivative                    = 0.0f;
    p_motor->foc.pdPosVel.p_ff.ptr                      = (float32_t *)(&p_motor->foc.cmd.iqff);
    p_motor->foc.pdPosVel.p_sat.ptr                     = (float32_t *)(&p_motor->foc.cmd.iSat);
    p_motor->foc.pdPosVel.out                           = 0.0f;
    //--- FSM motor initial state -----------------------------------------------
    p_motor->state                                      = MOTOR_STATE_IDLE;
    //--- Error message ---------------------------------------------------------
    p_motor->error.all                                  = 0U;
    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------
}

/***********************************************************************
 * INTERRUPT HANDLING FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Force a clear of all pending interrupts after ADC calibration.
 */
__attribute__((interrupt)) void cla_isrClearTask(void)
{
    // Purge pending tasks after ADC calibration.
    CLA_clearTaskFlags(CLA1_BASE, CLA_TASKFLAG_ALL);
    // Disable the task definitely.
    CLA_disableTasks(CLA1_BASE, CLA_ADC_CALIB_TASK_FLAG);
    // Disable the IT - The CLA task is one time called.
    Interrupt_disable(INT_CLA_ADC_CALIB);
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_ADC_CALIB_GROUP);
}

/**
 * @brief       After MOTOR_1 control, generate a part of the Tx message associated to MOTOR_1.
 */
__attribute__((interrupt)) void cla_isrMotor1(void)
{
    // Fill of the M1 motor's elements in the Tx message
    if((com_bus_select == COM_USE_SPI_BUS) && (!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR)) && COM_SPI_CS_READ)
    {
        spi_tx_msg_t* p_msg_spi     = &com_spi_tx.msg[com_spi_tx.msg_nx];
        COM_msgCreate(&motor_m1, &p_msg_spi->data, global_1ms_tmr_cnt);
    }
    else if((com_bus_select == COM_USE_RS485_BUS) && (!com_rs485_tx.index))
    {
        rs485_tx_msg_u* p_msg_rs485 = &com_rs485_tx.msg[com_rs485_tx.msg_nx];
        p_msg_rs485->header.addr    = COM_MSG_RS485_TX_ADDRESS;
        p_msg_rs485->header.type    = COM_MSG_RS485_TX_TYPE_STANDARD;
        p_msg_rs485->header.size    = COM_MSG_RS485_TX_8BIT_FULL_LENGTH;
        COM_msgCreate(&motor_m1, &p_msg_rs485->data, global_1ms_tmr_cnt);
    }
    CLA_clearTaskFlags(CLA1_BASE, CLA_MOTOR1_TASK_FLAG);
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR1_GROUP);
}

/**
 * @brief       After MOTOR_2 control, generate a part of the Tx message associated to the motor + message CRC.
 */
__attribute__((interrupt)) void cla_isrMotor2(void)
{
    // Finalize the Tx message. Aggregate MOTOR_2 values + computed CRC
    if((com_bus_select == COM_USE_SPI_BUS) && (!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR)) && COM_SPI_CS_READ)
    {
        spi_tx_msg_t* p_msg_spi     = &com_spi_tx.msg[com_spi_tx.msg_nx];
        COM_msgCreate(&motor_m2, &p_msg_spi->data, global_1ms_tmr_cnt);
        uint32_t msg_crc            = COM_msgCRC((uint16_t *) &p_msg_spi->data, COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH);
        // swap words in message - Required for Solo
        p_msg_spi->crc.u16_msb      = LSB_32(msg_crc);
        p_msg_spi->crc.u16_lsb      = MSB_32(msg_crc);
        // New message complete
        // Is SPI already transmitting (SPI Tx is slave - no control when Tx start)?
        if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && COM_SPI_CS_READ)
        {
            // swap active message array
            com_spi_tx.msg_cr       = !com_spi_tx.msg_cr;
            com_spi_tx.msg_nx       = !com_spi_tx.msg_cr;
            SPI_resetTxFIFO(COM_SPI_BASE);
            // Force first write in SPI_O_DAT register
            COM_SPI_DAT(p_msg_spi->data.status.all);
            DMA_configAddresses(DMA_COM_SPI_TX_BASE_ADDR, (uint16_t *) COM_SPI_TX_BASE_REG_ADDR, (uint16_t *) &p_msg_spi->data.timeStamp);
            DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
        }
    }
    else if((com_bus_select == COM_USE_RS485_BUS) && (!com_rs485_tx.index))
    {
        rs485_tx_msg_u* p_msg_rs485 = &com_rs485_tx.msg[com_rs485_tx.msg_nx];
        COM_msgCreate(&motor_m2, &p_msg_rs485->data, global_1ms_tmr_cnt);
        uint32_t msg_crc            = COM_msgCRC((uint16_t *) &p_msg_rs485->header, COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH);
        // swap words in message - Required for Solo
        p_msg_rs485->crc.u16_msb    = LSB_32(msg_crc);
        p_msg_rs485->crc.u16_lsb    = MSB_32(msg_crc);
        // New message complete
        // If we are here RS485 has not started.
        com_rs485_tx.msg_cr         = !com_rs485_tx.msg_cr;
        com_rs485_tx.msg_nx         = !com_rs485_tx.msg_cr;
    }
    else if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && COM_SPI_CS_READ)
    {
        spi_tx_msg_t* p_msg_spi     = &com_spi_tx.msg[0];
        SPI_resetTxFIFO(COM_SPI_BASE);
        // Force first write in SPI_O_DAT register
        COM_SPI_DAT(p_msg_spi->data.status.all);
        DMA_configAddresses(DMA_COM_SPI_TX_BASE_ADDR, (uint16_t *) COM_SPI_TX_BASE_REG_ADDR, (uint16_t *) &p_msg_spi->data.timeStamp);
        DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
    }
    CLA_clearTaskFlags(CLA1_BASE, CLA_MOTOR2_TASK_FLAG);
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR2_GROUP);
}

/**
 * @brief       Create the message for LED dimming + increment the timeout counter.
 */
__attribute__((interrupt)) void tmr_isrLedCmd(void)
{
    global_1ms_tmr_cnt = (global_1ms_tmr_cnt < UINT32_MAX) ? (global_1ms_tmr_cnt + 1) : (0);
    // LED dimming control
    if(!DMA_getRunStatusFlag(DMA_LED_SPI_TX_BASE_ADDR))
    {
        LED_dimming(&motor_m1, &motor_m2, &led_spi_tx[0]);
        DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
    }
    // Timeout management
    cmd_uomodri[MOTOR_1].cptTimeout++;
    cmd_uomodri[MOTOR_2].cptTimeout++;
    // Force an update of the motors command structures (Timeout value)
    CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
    // Clear IT
    Interrupt_clearACKGroup(INT_TIMER_LED_CMD_GROUP);
}

/**
 * @brief       Check & extract the received message from SPI.
 */
__attribute__((interrupt)) void dma_isrSpiComRx(void)
{
    // Compare computed CRC with the one received in message.
    spi_rx_msg_t* p_msg_spi = &com_spi_rx.msg;
    uint32_t msg_crc        = COM_msgCRC((uint16_t *)&p_msg_spi->data, COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH);
    if(((com_bus_select == COM_USE_SPI_BUS) || (com_bus_select == COM_NO_BUS_DEFINED)) && \
            (p_msg_spi->crc.u16_msb == MSB_32(msg_crc)) && (p_msg_spi->crc.u16_lsb == LSB_32(msg_crc)))
    {
        // Extract received message & send new commands to FOC structures. Define SPI as communication bus by default.
        COM_msgExtract(&p_msg_spi->data, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2]);
        // Force an update of the motors command structures (commands values)
        CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
        com_bus_select      = COM_USE_SPI_BUS;
        // Disable definitely the ITs linked to RS485.
//        CLB_disableCLB(RS485_RX_TX_CLB_EOT_BASE);
//        CLB_disableCLB(RS485_RX_CLB_ADDR_VALID_BASE);
        Interrupt_disable(INT_RS485_Rx_Tx_CLB);

//        SCI_disableModule(COM_SCI_BASE);
        Interrupt_disable(INT_RS485_RX_MSG);
        Interrupt_disable(INT_RS485_TX_MSG);
    }
    // Restart DMA reception & reset the SPI Rx fifo.
    DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
    SPI_resetRxFIFO(COM_SPI_BASE);
    // Clear IT flag
    Interrupt_clearACKGroup(INT_DMA_RX_MSG_GROUP);
}

/**
 * @brief       SCI Tx FIFO level reached - ready to receive new data.
 */
__attribute__((interrupt)) void sci_isrTx_RS485(void)
{
#ifdef DEBUG
    DBG_PIN2_SET;
#endif
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
            HWREGH(COM_SCI_FIFO_TX_REG_ADDR)   &= ~SCI_FFTX_TXFFIENA;
//            Interrupt_disable(INT_RS485_TX_MSG);
#endif
            com_rs485_tx.index = 0;
        }
#if USE_FCT
        SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_TXFF);
#else
        HWREGH(COM_SCI_FIFO_TX_REG_ADDR)       |= SCI_FFTX_TXFFINTCLR;
#endif
    }
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_TX_MSG_GROUP);
#ifdef DEBUG
    DBG_PIN2_CLEAR;
#endif
}

/**
 *@brief        SCI Rx FIFO level reached + slave address valid (IT enable).
 */
__attribute__((interrupt)) void sci_isrRx_RS485(void)
{
#ifdef DEBUG
    DBG_PIN2_SET;
#endif
    // FIFO IT source test.
    uint32_t sci_it_src_test = SCI_getInterruptStatus(COM_SCI_BASE);
//    if((HWREGH(COM_SCI_BASE + SCI_O_RXBUF) & SCI_RXBUF_SCIFFFE) == SCI_RXBUF_SCIFFFE)
//        sci_it_src_test |= SCI_INT_FIFO_RX_FE;
//    if((HWREGH(COM_SCI_BASE + SCI_O_RXBUF) & SCI_RXBUF_SCIFFPE) == SCI_RXBUF_SCIFFPE)
//        sci_it_src_test |= SCI_INT_FIFO_RX_PE;

    // FIFO RX IT?
    if(sci_it_src_test & SCI_INT_RXFF)
    {
        if(com_rs485_rx.addr1 || com_rs485_rx.addr2)
            // Get contain of FIFO Rx
            RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
        else
        {
            HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   &= ~(SCI_FFRX_RXFIFORESET | SCI_FFRX_RXFFIENA);
            HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   |= (SCI_FFRX_RXFIFORESET);
        }
        // Clear IT sources (FIFO IT source + FIFO overflow IT source)
#if USE_FCT
         SCI_clearOverflowStatus(COM_SCI_BASE);
         SCI_clearInterruptStatus(COM_SCI_BASE, SCI_INT_RXFF);
#else
         HWREGH(COM_SCI_FIFO_RX_REG_ADDR)  |= (SCI_FFRX_RXFFOVRCLR | SCI_FFRX_RXFFINTCLR);
#endif
    }
    /*
     * RX ERROR :
     * FE - SCI framing-error flag.
     * OE - SCI overrun-error flag.
     * PE - SCI parity-error flag.
     */
    if((sci_it_src_test & SCI_INT_RXERR))// || (sci_it_src_test & SCI_RXBUF_SCIFFFE) || (sci_it_src_test & SCI_RXBUF_SCIFFPE))
    {
#if USE_FCT
        SCI_performSoftwareReset(COM_SCI_BASE);
#else
        HWREGH(COM_SCI_CTRL1_REG_ADDR)     &= ~SCI_CTL1_SWRESET;
        HWREGH(COM_SCI_CTRL1_REG_ADDR)     |= SCI_CTL1_SWRESET;
#endif
    }
    // Clear global IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_MSG_GROUP);
#ifdef DEBUG
    DBG_PIN2_CLEAR;
#endif
}

/**
 * @brief       CLB - Valid address & EOT ITs.
 */
__attribute__((interrupt)) void clb_isrTxRx_RS485(void)
{
#ifdef DEBUG
    DBG_PIN3_SET;
#endif
#if USE_FCT
    uint16_t clb_tag_test   = CLB_getInterruptTag(RS485_RX_TX_CLB_EOT_BASE);
#else
    uint16_t clb_tag_test   = HWREGH(CLB_INT_REG_ADDR);
#endif
    switch (clb_tag_test)
    {
        // RS485 address #1 valid
    case CLB_INT_RS485_RX_ADDRESS_1_VALID:
        // Set flags
        com_rs485_rx.index  = 0;
        com_rs485_rx.addr1  = true;
        com_rs485_rx.addr2  = false;
        // First FIFO read.
        RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
        // Clear IT source & Enable FIFO Rx & Rx error IT sources.
        HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   |= (SCI_FFRX_RXFFIENA | SCI_FFRX_RXFFINTCLR | SCI_FFRX_RXFFOVRCLR);
        HWREGH(COM_SCI_CTRL1_REG_ADDR)     |= SCI_CTL1_RXERRINTENA;
        break;

        // RS485 address #2 valid
    case CLB_INT_RS485_RX_ADDRESS_2_VALID:
        // Set flags
        com_rs485_rx.index  = 0;
        com_rs485_rx.addr1  = false;
        com_rs485_rx.addr2  = true;
        // First FIFO read.
        RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
        // Clear IT source & Enable FIFO Rx & Rx error IT sources.
        HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   |= (SCI_FFRX_RXFFIENA | SCI_FFRX_RXFFINTCLR | SCI_FFRX_RXFFOVRCLR);
        HWREGH(COM_SCI_CTRL1_REG_ADDR)     |= SCI_CTL1_RXERRINTENA;
        break;

        // RS485 End-Of-Reception detection
    case CLB_INT_RS485_RX_END_OF_RECEPTION:
        // Disable immediately SCI FIFO Rx & errors ITs while a valid address is not received.
        HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   &= ~SCI_FFRX_RXFFIENA;
        HWREGH(COM_SCI_CTRL1_REG_ADDR)     &= ~SCI_CTL1_RXERRINTENA;
        // Purge pending ITs.
        HWREGH(PIECTRL_BASE + PIE_O_IFR8)  &= ~((uint16_t)INT_RS485_RX_PIE_FLAG);
        HWREGH(PIECTRL_BASE + PIE_O_ACK)    = INT_RS485_RX_MSG_GROUP;
        // Is it a valid address ?
        if(com_rs485_rx.addr1 || com_rs485_rx.addr2)
        {
            // Last FIFO read.
            RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
            rs485_rx_msg_u* p_msg_rs485     = &com_rs485_rx.msg;
            uint32_t msg_crc                = COM_msgCRC((uint16_t *) &p_msg_rs485->header, COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH);
            if((com_bus_select != COM_USE_SPI_BUS) && (p_msg_rs485->crc.u16_msb == MSB_32(msg_crc)) && (p_msg_rs485->crc.u16_lsb == LSB_32(msg_crc)))
            {
                // Message valid - Extract it & transfer to motors structures.
                COM_msgExtract(&p_msg_rs485->data, &cmd_uomodri[MOTOR_1], &cmd_uomodri[MOTOR_2]);
                CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
                // use RS485 as communication bus
                com_bus_select              = COM_USE_RS485_BUS;
                // Disable definitely the IT
//                Interrupt_disable(INT_DMA_RX_MSG);
                // prepare reply to master - reset transmit index
                com_rs485_tx.index          = 0;
                p_msg_rs485->crc.u16_lsb    = 0;
                p_msg_rs485->crc.u16_msb    = 0;
                // prepare reply to master - reset RS485 transmit FIFO & enable RS485 SCI Tx ITs.
                HWREGH(COM_SCI_FIFO_TX_REG_ADDR)   &= ~SCI_FFTX_TXFIFORESET;
                HWREGH(COM_SCI_FIFO_TX_REG_ADDR)   |= (SCI_FFTX_TXFIFORESET | SCI_FFTX_TXFFIENA | SCI_FFTX_TXFFINTCLR);
                // Inform CLB that next message is transmitted by slave
                HWREG(CLB_GP_REG_ADDR)             |= CLB_GPREG_RS485_TX_ENABLE;
                HWREG(CLB_GP_REG_ADDR)             &= ~CLB_GPREG_RS485_TX_ENABLE;
            }
        }
        // Reset of SCI Rx FIFO & Clear IT sources.
        HWREGH(COM_SCI_FIFO_RX_REG_ADDR)           &= ~(SCI_FFRX_RXFIFORESET);
        HWREGH(COM_SCI_FIFO_RX_REG_ADDR)           |= (SCI_FFRX_RXFIFORESET | SCI_FFRX_RXFFOVRCLR | SCI_FFRX_RXFFINTCLR);
        HWREGH(COM_SCI_CTRL1_REG_ADDR)             &= ~SCI_CTL1_SWRESET;
        HWREGH(COM_SCI_CTRL1_REG_ADDR)             |= SCI_CTL1_SWRESET;
        // Reset flags & index. Waiting for a new valid message.
        com_rs485_rx.index                          = 0;
        com_rs485_rx.addr1                          = false;
        com_rs485_rx.addr2                          = false;
        break;

    default:
        break;
    }
    // Clear CLB Flag
    CLB_clearInterruptTag(RS485_RX_TX_CLB_EOT_BASE);
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_Tx_CLB_GROUP);
#ifdef DEBUG
    DBG_PIN3_CLEAR;
#endif
}

/**
 * @brief       Errata workaround interrupt according errata note (sprz458.pdf)
 */
__attribute__((interrupt)) void errata_isr(void)
{
    global_errata_cnt = (global_errata_cnt < UINT32_MAX) ? (global_errata_cnt + 1) : (0U);
}
