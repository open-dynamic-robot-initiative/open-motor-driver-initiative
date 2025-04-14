/*
 * File name: main.c
 * Description: Source file containing main function running on CPU1 - C28 processor.
 */

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "string.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"

#include "uomodri_user_defines.h"
#include "config/uomodri_math.h"

#if (USE_UOMODRI_REV == USE_UOMODRI_V1)
#include "config/uomodri_1/uomodri_1_handlers.h"    // uOmodri v1.x configuration include file
#elif (USE_UOMODRI_REV == USE_UOMODRI_V2)
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
void MOT_initStruct(motor_t*, uint16_t);
void COM_muxCtrl(com_bus_e);
void CLA_adcCalibration(void);
void DMA_configModeUpdate(uint32_t, uint32_t);

__attribute__((interrupt)) void cla_isrMotor1(void);
__attribute__((interrupt)) void cla_isrMotor2(void);
__attribute__((interrupt)) void tmr_isrLedCmd(void);
__attribute__((interrupt)) void gpio_isrSpiComRx(void);
__attribute__((interrupt)) void sci_isrTx_RS485(void);
__attribute__((interrupt)) void sci_isrRx_RS485(void);
__attribute__((interrupt)) void clb_isrTxRx_RS485(void);
__attribute__((interrupt)) void mcan_isr_canfd(void);

// TODO(rafael): Don't keep them all
__attribute__((interrupt)) void fsi_tx1_isr(void);
__attribute__((interrupt)) void fsi_tx2_isr(void);
__attribute__((interrupt)) void fsi_rx1_isr(void);
__attribute__((interrupt)) void fsi_rx2_isr(void);

__attribute__((interrupt)) void errata_isr(void);

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
/** @var    int_cfg_t   interruptList[]
 *  @brief  List all required interrupts configurations.
 */
static const int_cfg_t interruptList[] =
{
 INT_CLA_MOTOR1_DEF,
 INT_CLA_MOTOR2_DEF,
 INT_TIMER_LED_DEF,
 INT_GPIO_SPI_CS_DEF,
 INT_RS485_TX_MSG_DEF,
 INT_RS485_RX_MSG_DEF,
 INT_RS485_RX_TX_CLB_DEF,
 INT_MCAN_RX_TX_MSG_DEF,

 // TODO(rafael): Add FSI interrupts for CPU1

 {.intNum = UINT32_MAX},        /* END OF ARRAY */
};

/***********************************************************************
 * VARIABLES IN CLA SPACE MEMORY
 ***********************************************************************/
extern motor_t        motor_m1;
extern motor_t        motor_m2;
extern robot_state_t  recv_robot_state;

/***********************************************************************
 * VARIABLES IN CPU TO CLA SPACE MEMORY
 ***********************************************************************/
extern cmd_t    cmd_uomodri[2];

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
#pragma DATA_ALIGN(com_bus_select, 4)
#pragma DATA_SECTION(com_bus_select, "ramgs0")
com_bus_e   com_bus_select      = COM_NO_BUS_DEFINED;


// TODO(rafael): remove these!
// ======================================================================================
//#pragma DATA_ALIGN(fsi_test_txint1_recv, 4)
//#pragma DATA_SECTION(fsi_test_txint1_recv, "ramgs0")
//bool_t   fsi_test_txint1_recv      = false;
//
//#pragma DATA_ALIGN(fsi_test_txint2_recv, 4)
//#pragma DATA_SECTION(fsi_test_txint2_recv, "ramgs0")
//bool_t   fsi_test_txint2_recv      = false;
//
//#pragma DATA_ALIGN(fsi_test_rxint1_recv, 4)
//#pragma DATA_SECTION(fsi_test_rxint1_recv, "ramgs0")
//bool_t   fsi_test_rxint1_recv      = false;
//
//#pragma DATA_ALIGN(fsi_test_rxint2_recv, 4)
//#pragma DATA_SECTION(fsi_test_rxint2_recv, "ramgs0")
//bool_t   fsi_test_rxint2_recv      = false;
// ======================================================================================

/***********************************************************************
 * EXTERNAL VARIABLES
 ***********************************************************************/
extern const hal_cfg_t  hal;

/***********************************************************************
 * CONSTANTS IN FLASH
 ***********************************************************************/
#pragma DATA_ALIGN(motor1_kv, 4)
#pragma DATA_SECTION(motor1_kv, "ConstFlashMotor1")
float32_t motor1_kv = MOTOR1_KV;

#pragma DATA_ALIGN(motor1_ke, 4)
#pragma DATA_SECTION(motor1_ke, "ConstFlashMotor1")
float32_t motor1_ke = MOTOR1_KE;

#pragma DATA_ALIGN(motor1_ki, 4)
#pragma DATA_SECTION(motor1_ki, "ConstFlashMotor1")
float32_t motor1_ki = MOTOR1_KI;

#pragma DATA_ALIGN(motor1_rs, 4)
#pragma DATA_SECTION(motor1_rs, "ConstFlashMotor1")
float32_t motor1_rs = MOTOR1_RS;

#pragma DATA_ALIGN(motor1_ls, 4)
#pragma DATA_SECTION(motor1_ls, "ConstFlashMotor1")
float32_t motor1_ls = MOTOR1_LS;

#pragma DATA_ALIGN(motor1_poles_pairs, 4)
#pragma DATA_SECTION(motor1_poles_pairs, "ConstFlashMotor1")
float32_t motor1_poles_pairs = MOTOR1_POLES_PAIRS;

#pragma DATA_ALIGN(motor1_enc_resolution_scale, 4)
#pragma DATA_SECTION(motor1_enc_resolution_scale, "ConstFlashMotor1")
float32_t motor1_enc_resolution_scale = MOTOR1_ENC_RESOLUTION_SCALE;

#pragma DATA_ALIGN(motor1_enc_speed_high_scale, 4)
#pragma DATA_SECTION(motor1_enc_speed_high_scale, "ConstFlashMotor1")
float32_t motor1_enc_speed_high_scale = MOTOR1_ENC_SPEED_HIGH_SCALE;

#pragma DATA_ALIGN(motor1_enc_speed_low_scale, 4)
#pragma DATA_SECTION(motor1_enc_speed_low_scale, "ConstFlashMotor1")
float32_t motor1_enc_speed_low_scale = MOTOR1_ENC_SPEED_LOW_SCALE;

#pragma DATA_ALIGN(motor1_enc_speed_lpf_alpha, 4)
#pragma DATA_SECTION(motor1_enc_speed_lpf_alpha, "ConstFlashMotor1")
float32_t motor1_enc_speed_lpf_alpha = MOTOR1_ENC_SPEED_LPF_ALPHA;

#pragma DATA_ALIGN(motor1_enc_speed_local_lpf_alpha, 4)
#pragma DATA_SECTION(motor1_enc_speed_local_lpf_alpha, "ConstFlashMotor1")
float32_t motor1_enc_speed_local_lpf_alpha  = MOTOR1_ENC_SPEED_LOCAL_LPF_ALPHA;

#pragma DATA_ALIGN(motor1_current_cmd_sat_max, 4)
#pragma DATA_SECTION(motor1_current_cmd_sat_max, "ConstFlashMotor1")
float32_t motor1_current_cmd_sat_max = MOTOR1_CURRENT_CMD_SAT_MAX;

#pragma DATA_ALIGN(motor1_current_lpf_alpha, 4)
#pragma DATA_SECTION(motor1_current_lpf_alpha, "ConstFlashMotor1")
float32_t motor1_current_lpf_alpha = MOTOR1_CURRENT_LPF_ALPHA;

#pragma DATA_ALIGN(motor1_dtc_max, 4)
#pragma DATA_SECTION(motor1_dtc_max, "ConstFlashMotor1")
float32_t motor1_dtc_max = MOTOR1_DTC_MAX;

#pragma DATA_ALIGN(motor1_dtc_min, 4)
#pragma DATA_SECTION(motor1_dtc_min, "ConstFlashMotor1")
float32_t motor1_dtc_min = MOTOR1_DTC_MIN;

#pragma DATA_ALIGN(motor1_position_init_evt, 4)
#pragma DATA_SECTION(motor1_position_init_evt, "ConstFlashMotor1")
float32_t motor1_position_init_evt = MOTOR1_POSITION_INIT_EVT;

#pragma DATA_ALIGN(motor1_fix_init_evt, 4)
#pragma DATA_SECTION(motor1_fix_init_evt, "ConstFlashMotor1")
float32_t motor1_fix_init_evt = MOTOR1_FIX_INIT_EVT;

#pragma DATA_ALIGN(motor1_position_init_err_min, 4)
#pragma DATA_SECTION(motor1_position_init_err_min, "ConstFlashMotor1")
float32_t motor1_position_init_err_min = MOTOR1_POSITION_INIT_ERR_MIN;

#pragma DATA_ALIGN(motor1_position_init_err_max, 4)
#pragma DATA_SECTION(motor1_position_init_err_max, "ConstFlashMotor1")
float32_t motor1_position_init_err_max = MOTOR1_POSITION_INIT_ERR_MAX;

#pragma DATA_ALIGN(motor1_pi_id_kp_coeff, 4)
#pragma DATA_SECTION(motor1_pi_id_kp_coeff, "ConstFlashMotor1")
float32_t motor1_pi_id_kp_coeff = MOTOR1_PI_ID_KP_COEF;

#pragma DATA_ALIGN(motor1_pi_id_ki_coeff, 4)
#pragma DATA_SECTION(motor1_pi_id_ki_coeff, "ConstFlashMotor1")
float32_t motor1_pi_id_ki_coeff = MOTOR1_PI_ID_KI_COEF;

#pragma DATA_ALIGN(motor1_pi_iq_kp_coeff, 4)
#pragma DATA_SECTION(motor1_pi_iq_kp_coeff, "ConstFlashMotor1")
float32_t motor1_pi_iq_kp_coeff = MOTOR1_PI_IQ_KP_COEF;

#pragma DATA_ALIGN(motor1_pi_iq_ki_coeff, 4)
#pragma DATA_SECTION(motor1_pi_iq_ki_coeff, "ConstFlashMotor1")
float32_t motor1_pi_iq_ki_coeff = MOTOR1_PI_IQ_KI_COEF;

//#pragma DATA_ALIGN(motor1_flash, 4)
//#pragma DATA_SECTION(motor1_flash, "ConstFlashMotor1")
//motor_t motor1_flash = {.foc.cfg.Rs                             = MOTOR1_RS,
//                        .foc.cfg.Ls                             = MOTOR1_LS,
//                        .foc.cfg.kv                             = MOTOR1_KV,
//                        .foc.cfg.ke                             = MOTOR1_KE,
//                        .foc.cfg.ki                             = MOTOR1_KI,
//                        .foc.cfg.polePairs                      = MOTOR1_POLES_PAIRS,
//                        //--- MOTOR ENCORDER STRUCTURE ------------------------------------------
//                        .foc.enc.polePairs                      = MOTOR1_POLES_PAIRS,
//                        .foc.enc.thetaMechScaler                = (float32_t)MOTOR1_ENC_RESOLUTION_SCALE,
//                        .foc.enc.speed.speedHighScaler          = (float32_t)MOTOR1_ENC_SPEED_HIGH_SCALE,
//                        .foc.enc.speed.speedLowScaler           = (float32_t)MOTOR1_ENC_SPEED_LOW_SCALE,
//                        .foc.enc.speed.speedFlt[0].a            = (float32_t)MOTOR1_ENC_SPEED_LPF_ALPHA,
//                        .foc.enc.speed.speedFlt[0].one_minus_a  = (float32_t)MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA,
//                        .foc.enc.speed.speedFlt[1].a            = (float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ALPHA,
//                        .foc.enc.speed.speedFlt[1].one_minus_a  = (float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA,
//                        //--- MOTOR COMMAND STRUCTURE -------------------------------------------
//                        .foc.cmd.iSat                           = MOTOR1_CURRENT_CMD_SAT_MAX,
//                        //-----------------------------------------------------------------------
//                        .foc.iParkFlt.a                         = MOTOR1_CURRENT_LPF_ALPHA,
//                        .foc.iParkFlt.one_minus_a               = MOTOR1_CURRENT_LPF_ONE_M_ALPHA,
//                        //--- FOC - Initialize duty cycle saturation ----------------------------
//                        .foc.dtcMax                             = MOTOR1_DTC_MAX,
//                        .foc.dtcMin                             = MOTOR1_DTC_MIN,
//                        //--- FOC - Encoder test ------------------------------------------------
//                        .foc.initPosCnt                         = MOTOR1_POSITION_INIT_EVT,
//                        .foc.initFixCnt                         = MOTOR1_FIX_INIT_EVT,
//                        .foc.initErr[MIN]                       = MOTOR1_POSITION_INIT_ERR_MIN,
//                        .foc.initErr[MAX]                       = MOTOR1_POSITION_INIT_ERR_MAX,
//                        //--- FOC - Initialize PI controller (Id) -------------------------------
//                        .foc.piId.kp                            = (float32_t)MOTOR1_PI_ID_KP_COEF,
//                        .foc.piId.ki                            = (float32_t)MOTOR1_PI_ID_KI_COEF,
//                        //--- FOC - Initialize PI controller (Iq) -------------------------------
//                        .foc.piIq.kp                            = (float32_t)MOTOR1_PI_IQ_KP_COEF,
//                        .foc.piIq.ki                            = (float32_t)MOTOR1_PI_IQ_KI_COEF,
//};

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
    // Hardware Abstraction Layer initialization.
    HAL_ini(&hal);
    // Initialization the command structure for CLA.
    COM_initCmdStruct(&cmd_uomodri[MOTOR_1], MOTOR_1);
    COM_initCmdStruct(&cmd_uomodri[MOTOR_2], MOTOR_2);
    // DRV8353 initialization.
    cmd_uomodri[MOTOR_1].periphField.drv_initErr = DRV_ini(&drv_cfg[MOTOR_1], &drv_reg[MOTOR_1]);
    cmd_uomodri[MOTOR_2].periphField.drv_initErr = DRV_ini(&drv_cfg[MOTOR_2], &drv_reg[MOTOR_2]);
    // Start ADC offset calibration on CLA.
    CLA_adcCalibration();
    // Disable continuous mode for SPI communication
    DMA_configModeUpdate(DMA_COM_SPI_TX_BASE_ADDR, DMA_COM_SPI_TX_CONFIG_NEW);
    // Use SPI, RS485 or CAN-FD as master communication bus.
#if (CAN_FD_BUS_ENABLE)
    com_bus_select = COM_USE_CANFD_BUS;
#elif (RS485_BUS_ENABLE)
    com_bus_select = COM_USE_RS485_BUS;
#elif (SPI_BUS_ENABLE)
    com_bus_select = COM_USE_SPI_BUS;
#else
    com_bus_select = COM_NO_BUS_DEFINED;
#endif

#if (USE_UOMODRI_REV >= USE_UOMODRI_V3)
    // Mux control of the differential pair.
    COM_muxCtrl(com_bus_select);
#endif
    // Interrupts initialization.
    HAL_INT_ini(interruptList, &errata_isr);
    // Infinite loop
    while(1);
}

/**
 * @brief       Microcontroller initialization function
 *                  - Configure master clock to 200MHz
 *                  - Configure auxiliary clock (CM core) to 125MHz (USB � 60MHz)
 *                  - Configure MCAN (clock divider + processor management (CPU1 or CM))
 *                  - Enable GPIO clocking
 *                  - Configure the interruption vector table
 *                  - Boot CM from flash or RAM
 */
static void MCU_ini(void)
{
    /* Initialize clock and peripherals */
    Device_init();
#if (USE_UOMODRI_REV >= USE_UOMODRI_V3)
    // Define which core manage MCAN (a shared peripheral) - CPU1 (C28x) / CM (Cortex M)
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_MCAN_A, MCAN_ALLOCATE_ON_CORE);
    // Configure the divisor for the MCAN bit-clock
    SysCtl_setMCANClk(SYSCTL_MCANCLK_DIV_1);
#endif
    // Define which core manage USB (a shared peripheral) - CPU1 (C28x) / CM (Cortex M)
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, USB_ALLOCATE_ON_CORE);

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
//#if (CLB_TILE_ENABLE)
    /*
     * Use SYSCLK / PERCLKDIVSEL.EPWMCLKDIV (2) as Tile clock (100MHz).
     */

    /* Redefined to pass to SysCtl_setAuxClock(). Will configure the clock as follows:
     * AUXPLLCLK = 25MHz (XTAL_OSC) * 48 (IMULT) / (2 (REFDIV) * 5 (ODIV) * 2(AUXPLLDIV) )
     * AUXPLLCLK require a 60MHz clock for USB. AUPLLRAWCLK require a 120MHz clock for CM. */
#undef  DEVICE_AUXSETCLOCK_CFG
#define DEVICE_AUXSETCLOCK_CFG      (SYSCTL_AUXPLL_OSCSRC_XTAL  | \
                                     SYSCTL_AUXPLL_IMULT(48)    | \
                                     SYSCTL_AUXPLL_REFDIV(2U)   | \
                                     SYSCTL_AUXPLL_ODIV(5U)     | \
                                     SYSCTL_AUXPLL_DIV_2        | \
                                     SYSCTL_AUXPLL_ENABLE       | \
                                     SYSCTL_DCC_BASE_0)

//#define DEVICE_AUXSETCLOCK_CFG      (SYSCTL_AUXPLL_OSCSRC_XTAL  | \
//                                     SYSCTL_AUXPLL_IMULT(72)    | \
//                                     SYSCTL_AUXPLL_REFDIV(5U)   | \
//                                     SYSCTL_AUXPLL_ODIV(3U)     | \
//                                     SYSCTL_AUXPLL_DIV_2        | \
//                                     SYSCTL_AUXPLL_ENABLE       | \
//                                     SYSCTL_DCC_BASE_0)
#undef  DEVICE_AUXCLK_FREQ
#define DEVICE_AUXCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 48) / (2 * 5 * 2))
//#define DEVICE_AUXCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 72) / (5 * 3 * 2))
    // Set up AUXPLL control and clock dividers needed for CMCLK
    SysCtl_setAuxClock(DEVICE_AUXSETCLOCK_CFG);
    // Set up CMCLK to use AUXPLL as the clock source and set the clock divider to 1.
    SysCtl_setCMClk(SYSCTL_CMCLKOUT_DIV_1,SYSCTL_SOURCE_AUXPLL);
//    SysCtl_setCMClk(SYSCTL_CMCLKOUT_DIV_3,SYSCTL_SOURCE_AUXPLL);

//    EALLOW;
//    uint32_t tmp = HWREGH(CLKCFG_BASE + SYSCTL_O_AUXCLKDIVSEL) & 0xFFFFFFF8;
//    HWREGH(CLKCFG_BASE + SYSCTL_O_AUXCLKDIVSEL) = (uint32_t)(tmp | \
//            ((1 & SYSCTL_AUXCLKDIVSEL_AUXPLLDIV_M) >> SYSCTL_AUXCLKDIVSEL_AUXPLLDIV_S));
//    SYSCTL_REGWRITE_DELAY;
//    EDIS;

//#endif
//#if (defined USB_BUS_ENABLE)
//    /* Allocate Shared Peripheral USB to the CM Side. */
//    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
//#endif

#if (defined USB_BUS_ENABLE_ON_CM) && (!defined CPU1)
    /* Allocate Shared Peripheral USB to the CM Side. */
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
#endif
    /* Boot CM core */
#if (defined _FLASH)
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif

//    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
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
    static uint16_t tmr_cpt_ms  = 0;
    uint16_t motor_state        = MOTOR_STATE_IDLE;

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
    p_msg       = ws2812b_dimmingLed(MASTER_LED, motor_state, tmr_cpt_ms, p_msg);
    //MOTOR_1 LED (2nd led)
    p_msg       = ws2812b_dimmingLed(MOTOR1_LED, p_motor1->state, tmr_cpt_ms, p_msg);
    //MOTOR_2 LED (3th led)
    p_msg       = ws2812b_dimmingLed(MOTOR2_LED, p_motor2->state, tmr_cpt_ms, p_msg);
    // millisecond timer increment
    tmr_cpt_ms  = (tmr_cpt_ms < 1000U) ? (tmr_cpt_ms + 1): (0U);

    return;
}

/**
 * @brief       Copy content of motors structure to a debug structure (CM core).
 * @param[in]   p_motor Pointer on data structure array used for motor control.
 * @param[out]  p_dbg   Pointer on debug structure to be passed out to the CM core.
 */
void DBG_copyStruct(motor_t* p_motor, debug_t* p_dbg)
{
    p_dbg->ia                       = p_motor->foc.acq.ia;                  // [A]
    p_dbg->ib                       = p_motor->foc.acq.ib;                  // [A]
    p_dbg->ic                       = p_motor->foc.acq.ic;                  // [A]
    p_dbg->vBus                     = p_motor->foc.acq.vBus;                // [V]
    p_dbg->vExt                     = p_motor->foc.acq.vExt;                // [V]

    p_dbg->enc_thetaElec            = p_motor->foc.enc.thetaElec;           /*!< Angular electrical position  - [pu] */
    p_dbg->enc_thetaMech            = p_motor->foc.enc.thetaMech[0];        /*!< Angular mechanical position  - [pu] */
    p_dbg->enc_thetaIndexRelative   = p_motor->foc.enc.thetaIndexRelative;  /*!< Index mechanical angle */
    p_dbg->enc_thetaIndexAbsolute   = p_motor->foc.enc.thetaIndexAbsolute;  /*!< First index mechanical angle */
    p_dbg->enc_thetaAbsolute        = p_motor->foc.enc.thetaAbsolute;       /*!< Absolute angular position - [rad] */
    p_dbg->enc_turnNb               = p_motor->foc.enc.turnNb;              /*!< Number of full turns (signed) */
    p_dbg->enc_thetaDir             = p_motor->foc.enc.thetaDir;            /*!< Rotor direction - -1 = CCW/reverse, 1 = CW/forward */
    p_dbg->enc_flags.all            = p_motor->foc.enc.flags.all;

    p_dbg->enc_speed_speedHigh      = p_motor->foc.enc.speed.speedHigh;     /*!< High speed estimation -  [rad/s] - diff_angular * scaler = velocity */
    p_dbg->enc_speed_speedMech      = p_motor->foc.enc.speed.speedMech[0];  /*!< Mechanical output speed - [rad/s] */
    p_dbg->enc_speed_speedElec      = p_motor->foc.enc.speed.speedElec;     /*!< Electrical output speed (pole pairs integration) - [rad/s] */
    p_dbg->enc_speed_theta          = p_motor->foc.enc.speed.theta[0];      /*!< Angular position of rotor - [pu] */

    p_dbg->cmd_iSat                 = p_motor->foc.cmd.iSat;
    p_dbg->cmd_cptTimeout           = p_motor->foc.cmd.cptTimeout;
    p_dbg->cmd_index                = p_motor->foc.cmd.index;
    p_dbg->cmd_cmdField.all         = p_motor->foc.cmd.cmdField.all;            /*!< Bit 0      : R/W - Enable system to start. No power on motors.         */
    p_dbg->cmd_periphField.all      = p_motor->foc.cmd.periphField.all;         /*!< Bits 0   : R/W - DRV initialization went wrong             */

    p_dbg->sinTheta                 = p_motor->foc.sinTheta;                /*!< Sinus of the electrical rotor angle */
    p_dbg->cosTheta                 = p_motor->foc.cosTheta;           /*!< Cosine of the electrical rotor angle */
    p_dbg->ialpha                   = p_motor->foc.ialpha;             /*!< Direct Clarke transform result - [A] */
    p_dbg->ibeta                    = p_motor->foc.ibeta;              /*!< Direct Clarke transform result - [A] */
    p_dbg->id                       = p_motor->foc.id;                 /*!< Filtered Direct Park transform result - [A] */
    p_dbg->iq                       = p_motor->foc.iq;                 /*!< Filtered Quadrature Park transform result - [A] */
    p_dbg->ud                       = p_motor->foc.ud;                 /*!< Inverse Park transform result - [V] */
    p_dbg->uq                       = p_motor->foc.uq;                 /*!< Inverse Park transform result - [V] */
    p_dbg->ualpha                   = p_motor->foc.ualpha;             /*!< Inverse Clarke transform result - [V] */
    p_dbg->ubeta                    = p_motor->foc.ubeta;              /*!< Inverse Clarke transform result - [V] */
    p_dbg->ua                       = p_motor->foc.ua;                 /*!< Phase commands - Channel a - [V] */
    p_dbg->ub                       = p_motor->foc.ub;                 /*!< Phase commands - Channel b - [V] */
    p_dbg->uc                       = p_motor->foc.uc;                 /*!< Phase commands - Channel c - [V] */
    p_dbg->dtc_u                    = p_motor->foc.dtc_u;              /*!< Duty cycles on channel u - [pu] between -0.5 and 0.5 */
    p_dbg->dtc_v                    = p_motor->foc.dtc_v;              /*!< Duty cycles on channel v - [pu] between -0.5 and 0.5 */
    p_dbg->dtc_w                    = p_motor->foc.dtc_w;              /*!< Duty cycles on channel w - [pu] between -0.5 and 0.5 */
    p_dbg->idRef                    = p_motor->foc.idRef;              /*!< PI controller set point values for D-axis current - [A] */
    p_dbg->iqRef                    = p_motor->foc.iqRef;              /*!< PI controller set point values for Q-axis current - [A] */

    p_dbg->piId_err                 = p_motor->foc.piId.err;      /*!< Current error */
    p_dbg->piId_integral            = p_motor->foc.piId.integral;           /*!< Integral term partial computation */
    p_dbg->piId_out                 = p_motor->foc.piId.out;                /*!< Controller output */

    p_dbg->piIq_err                 = p_motor->foc.piIq.err;      /*!< Current error */
    p_dbg->piIq_integral            = p_motor->foc.piIq.integral;           /*!< Integral term partial computation */
    p_dbg->piIq_out                 = p_motor->foc.piIq.out;                /*!< Controller output */

//    p_dbg->pdPosVel_errTheta        = p_motor->foc.pdPosVel.errTheta;           /*!< Error position */
//    p_dbg->pdPosVel_errSpeed        = p_motor->foc.pdPosVel.errSpeed;           /*!< Error speed */
//    p_dbg->pdPosVel_derivative      = p_motor->foc.pdPosVel.derivative;         /*!< Derivative term partial computation */
//    p_dbg->pdPosVel_out             = p_motor->foc.pdPosVel.out;                /*!< Controller output */

    p_dbg->itCnt                    = p_motor->itCnt;          /*!< Event counter incrementing on every IT call */
    p_dbg->motor_id                 = p_motor->id;             /*!< Motor identification */
    p_dbg->error.all                = p_motor->error.all;          /*!< Bit field for the error sources. */
    p_dbg->state                    = p_motor->state;      /*!< Motor state for the FSM. */

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
        p_motor->foc.cfg.Rs                             = motor1_rs;//MOTOR1_RS;
        p_motor->foc.cfg.Ls                             = motor1_ls;//MOTOR1_LS;
        p_motor->foc.cfg.kv                             = motor1_kv;//MOTOR1_KV;
        p_motor->foc.cfg.ke                             = motor1_ke;//MOTOR1_KE;
        p_motor->foc.cfg.ki                             = motor1_ki;//MOTOR1_KI;
        p_motor->foc.cfg.polePairs                      = motor1_poles_pairs;//MOTOR1_POLES_PAIRS;
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
        p_motor->foc.enc.polePairs                      = motor1_poles_pairs;//MOTOR1_POLES_PAIRS;
        p_motor->foc.enc.thetaMechScaler                = motor1_enc_resolution_scale;//(float32_t)MOTOR1_ENC_RESOLUTION_SCALE;
        p_motor->foc.enc.speed.speedHighScaler          = motor1_enc_speed_high_scale;//(float32_t)MOTOR1_ENC_SPEED_HIGH_SCALE;
        p_motor->foc.enc.speed.speedLowScaler           = motor1_enc_speed_low_scale;//(float32_t)MOTOR1_ENC_SPEED_LOW_SCALE;
        p_motor->foc.enc.speed.speedFlt[0].a            = motor1_enc_speed_lpf_alpha;//(float32_t)MOTOR1_ENC_SPEED_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[0].one_minus_a  = 1.0f - motor1_enc_speed_lpf_alpha;//(float32_t)MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].a            = motor1_enc_speed_local_lpf_alpha;//(float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].one_minus_a  = 1.0f - motor1_enc_speed_local_lpf_alpha;//(float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA;
        //--- MOTOR COMMAND STRUCTURE -------------------------------------------
        p_motor->foc.cmd.iSat                           = motor1_current_cmd_sat_max;//MOTOR1_CURRENT_CMD_SAT_MAX;
        //-----------------------------------------------------------------------
        p_motor->foc.iParkFlt.a                         = motor1_current_lpf_alpha;//MOTOR1_CURRENT_LPF_ALPHA;
        p_motor->foc.iParkFlt.one_minus_a               = 1.0f - motor1_current_lpf_alpha;//MOTOR1_CURRENT_LPF_ONE_M_ALPHA;
        //--- FOC - Initialize duty cycle saturation ----------------------------
        p_motor->foc.dtcMax                             = motor1_dtc_max;//MOTOR1_DTC_MAX;
        p_motor->foc.dtcMin                             = motor1_dtc_min;//MOTOR1_DTC_MIN;
        //--- FOC - Encoder test ------------------------------------------------
        p_motor->foc.initPosCnt                         = motor1_position_init_evt;//MOTOR1_POSITION_INIT_EVT;
        p_motor->foc.initFixCnt                         = motor1_fix_init_evt;//MOTOR1_FIX_INIT_EVT;
        p_motor->foc.initErr[MIN]                       = motor1_position_init_err_min;//MOTOR1_POSITION_INIT_ERR_MIN;
        p_motor->foc.initErr[MAX]                       = motor1_position_init_err_max;//MOTOR1_POSITION_INIT_ERR_MAX;
        //--- FOC - Initialize PI controller (Id) -------------------------------
        p_motor->foc.piId.kp                            = motor1_pi_id_kp_coeff;//(float32_t)MOTOR1_PI_ID_KP_COEF;
        p_motor->foc.piId.ki                            = motor1_pi_id_ki_coeff;//(float32_t)MOTOR1_PI_ID_KI_COEF;
        //--- FOC - Initialize PI controller (Iq) -------------------------------
        p_motor->foc.piIq.kp                            = motor1_pi_iq_kp_coeff;//(float32_t)MOTOR1_PI_IQ_KP_COEF;
        p_motor->foc.piIq.ki                            = motor1_pi_iq_ki_coeff;//(float32_t)MOTOR1_PI_IQ_KI_COEF;
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
    memset(p_motor->foc.cmd.linCtrlAlpha, 0, sizeof(float32_t) * ROBOT_STATE_DIMS);
    p_motor->foc.cmd.linCtrlBeta                        = 0.0f;
    p_motor->foc.cmd.iSat                               = 0.0f;
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

    //--- FOC - Initialize linear controller ------------------------------------
    p_motor->foc.linControl.p_alpha.ptr                  = (float32_t *)(&p_motor->foc.cmd.linCtrlAlpha[0]);
    p_motor->foc.linControl.p_beta.ptr                   = (float32_t *)(&p_motor->foc.cmd.linCtrlBeta);
    p_motor->foc.linControl.p_iSat.ptr                   = (float32_t *)(&p_motor->foc.cmd.iSat);

    // The linear controller of both motors must access the latest state of all motors
    // in the driver daisy chain, including motors not controlled by this driver.
    // The two pointers to this driver's M1 and M2 positions will be set by the CLA
    // whenever a new command is received. This is to ensure the correct position is
    // used, depending on whether offset compensation is enabled.
    //
    // TODO(rafael): When `recv_robot_state` is updated by C28, make sure the CLA doesn't
    //               read garbage!
    int32_t dim = 0;
    for (dim = 0; dim < ROBOT_STATE_DIMS; dim++)
        p_motor->foc.linControl.p_state[dim].ptr = (float32_t *)(&recv_robot_state.state[dim]);

    // The state components that correspond to our velocities can be set now, as they
    // don't depend on any runtime settings:
    p_motor->foc.linControl.p_state[2*DAISY_CHAIN_DRIVER_COUNT + 2*DAISY_CHAIN_DRIVER_ID].ptr     = \
            (float32_t *)(&motor_m1.foc.enc.speed.speedMech[0]);

    p_motor->foc.linControl.p_state[2*DAISY_CHAIN_DRIVER_COUNT + 2*DAISY_CHAIN_DRIVER_ID + 1].ptr = \
            (float32_t *)(&motor_m2.foc.enc.speed.speedMech[0]);

    //--- FSM motor initial state -----------------------------------------------
    p_motor->state                                      = MOTOR_STATE_IDLE;
    //--- Error message ---------------------------------------------------------
    p_motor->error.all                                  = 0U;
    //--- New Data ready to be send ---------------------------------------------
//    p_motor->newData                                    = 0U;
    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------
    return;
}

#if (USE_UOMODRI_REV >= USE_UOMODRI_V3)
/**
 * @brief       Mux switching for communication protocol section
 * @param[in]   bus_select  Define the communication protocol used.
 */
void COM_muxCtrl(com_bus_e bus_select)
{
    switch(com_bus_select)
    {
        case COM_USE_RS485_BUS:
        {
            // CAN driver OFF, mux on RS485.
            GPIO_writePin(CANFD_SHDN_MODE_NUM, CANFD_SHDN_MODE_ENABLE);
            GPIO_writePin(CANFD_RS485_MUX_SEL_NUM, CANFD_RS485_MUX_SEL_RS485_MODE);
            break;
        }

        case COM_USE_CANFD_BUS:
        {

            // CAN driver ON, mux on CAN.
            GPIO_writePin(CANFD_SHDN_MODE_NUM, CANFD_SHDN_MODE_DISABLE);
            GPIO_writePin(CANFD_RS485_MUX_SEL_NUM, CANFD_RS485_MUX_SEL_CANFD_MODE);
            break;
        }

        case COM_USE_SPI_BUS :
        default:
        {
            // CAN driver OFF, mux on CAN.
            GPIO_writePin(CANFD_SHDN_MODE_NUM, CANFD_SHDN_MODE_ENABLE);
            GPIO_writePin(CANFD_RS485_MUX_SEL_NUM, CANFD_RS485_MUX_SEL_CANFD_MODE);
            break;
        }
    }

    return;
}
#endif

/**
 * @brief       ADC calibration function. Called at startup to initialize ADC bias.
 */
void CLA_adcCalibration(void)
{
    // Start ADC offset calibration on CLA.
    CLA_forceTasks(CLA1_BASE, CLA_ADC_CALIB_TASK_FLAG);
    // Wait the start of ADC offset calibration on CLA.
    while(CLA_getTaskRunStatus(CLA1_BASE, CLA_ADC_CALIB_TASK_NUMBER) == false);
    // Wait the end of ADC offset calibration on CLA.
    while(CLA_getTaskRunStatus(CLA1_BASE, CLA_ADC_CALIB_TASK_NUMBER) == true);
    // Purge pending tasks after ADC calibration.
    CLA_clearTaskFlags(CLA1_BASE, CLA_TASKFLAG_ALL);
    // Disable the task definitely.
    CLA_disableTasks(CLA1_BASE, CLA_ADC_CALIB_TASK_FLAG);

    return;
}

/**
 * @brief       Configures/reconfigures the DMA channel mode. (No trigger config).
 * @param       base    Base address of the DMA channel control registers.
 * @param       config  Bit field of several configuration selections.
 * The \e config parameter is the logical OR of the following values:
 * - \b DMA_CFG_ONESHOT_DISABLE or \b DMA_CFG_ONESHOT_ENABLE. If enabled,
 *      the subsequent burst transfers occur without additional event triggers
 *      after the first event trigger.
 *      If disabled, only one burst transfer is performed per event trigger.
 * - \b DMA_CFG_CONTINUOUS_DISABLE or \b DMA_CFG_CONTINUOUS_ENABLE. If enabled
 *      the DMA reinitializes when the transfer count is zero and waits for the
 *      next interrupt event trigger.
 *      If disabled, the DMA stops and clears the run status bit.
 * - \b DMA_CFG_SIZE_16BIT or \b DMA_CFG_SIZE_32BIT. This setting selects
 *      whether the databus width is 16 or 32 bits.
 * @return      None.
*/
void DMA_configModeUpdate(uint32_t base, uint32_t config)
{
    EALLOW;
    HWREGH(base + DMA_O_MODE) &= ~(DMA_MODE_DATASIZE | DMA_MODE_CONTINUOUS | DMA_MODE_ONESHOT);
    HWREGH(base + DMA_O_MODE) |= config;
    EDIS;

    return;
}

