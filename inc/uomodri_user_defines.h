/*
 * File name: omodri_user_defines.h
 * Description: Header file containing user definitions
 */

#ifndef __UOMODRI_USER_DEFINES_H__
#define __UOMODRI_USER_DEFINES_H__

#include "driverlib.h"

#include "hal.h"
#include "drv8353.h"

/** @defgroup DEV_defines
 *  @brief    Special configurations definitions.
 * @{
 */
#define OMODRI_PAL_SETUP            0   // Set this to 1 to use the PAL pin affectation
#define SPI_DEBUG_CONFIGURATION     0   // Set this to 1 to use debug configuration
// Word number in a full communication
//#define SPI_COM_LENGTH              17
/**
 * @}
 */

/** @defgroup ADC_global_defines
 *  @brief    Configure ADC subsystems.
 * @{
 */
// ADC clock config
#define PERxSYSCLK_TO_ADCCLK_DIV        ADC_CLK_DIV_4_0
#define ADCCLK_FREQ                     (DEVICE_SYSCLK_FREQ / ((PERxSYSCLK_TO_ADCCLK_DIV / 2) + 1))
// ADC defines
#define ADC_SAMPLING_WINDOW             119   // From data sheet Table 20-12
#define ADC_RESOLUTION                  ADC_RESOLUTION_16BIT
#define ADC_SIGNAL_MODE                 ADC_MODE_SINGLE_ENDED
#define ADC_PULSE_END_MODE              ADC_PULSE_END_OF_CONV
#define ADC_SOC_PRIORITY                ADC_PRI_THRU_SOC3_HIPRI
//ADC_PRI_ALL_ROUND_ROBIN
#define ADC_INT_SOC_TRIGGER             ADC_INT_SOC_TRIGGER_NONE
/**
 * @}
 */

/** @defgroup ADC_A config.
 *  @brief    Configure the Analog to Digital Core \& 2 Analog to Digital Wrapper Logic blocks
 * @{
 */
// Channels
#define MOTOR1_VA_CH                        ADC_CH_ADCIN2
#define MOTOR2_VA_CH                        ADC_CH_ADCIN3
#define MOTOR1_IA_CH                        ADC_CH_ADCIN0
#define MOTOR2_IA_CH                        ADC_CH_ADCIN1
// SOC Number
#define MOTOR1_VA_SOC_NUM                   ADC_SOC_NUMBER0
#define MOTOR2_VA_SOC_NUM                   ADC_SOC_NUMBER1
#define MOTOR1_IA_SOC_NUM                   ADC_SOC_NUMBER2
#define MOTOR2_IA_SOC_NUM                   ADC_SOC_NUMBER3
// PPB Number
#define MOTOR1_VA_PPB_NUM                   ADC_PPB_NUMBER1
#define MOTOR2_VA_PPB_NUM                   ADC_PPB_NUMBER2
#define MOTOR1_IA_PPB_NUM                   ADC_PPB_NUMBER3
#define MOTOR2_IA_PPB_NUM                   ADC_PPB_NUMBER4
// ADC Base Address
#define MOTOR1_VA_ADC_ADDR                  ADCA_BASE
#define MOTOR2_VA_ADC_ADDR                  ADCA_BASE
#define MOTOR1_IA_ADC_ADDR                  ADCA_BASE
#define MOTOR2_IA_ADC_ADDR                  ADCA_BASE
#define MOTOR12_IVA_ADC_ADDR                ADCA_BASE
// ADC Results Base Address
#define MOTOR1_VA_ADC_RESULT_ADDR           (ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_VA_SOC_NUM))
#define MOTOR2_VA_ADC_RESULT_ADDR           (ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_VA_SOC_NUM))
#define MOTOR1_IA_ADC_RESULT_ADDR           (ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_IA_SOC_NUM))
#define MOTOR2_IA_ADC_RESULT_ADDR           (ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_IA_SOC_NUM))
#define MOTOR12_IVA_ADC_RESULT_ADDR         ADCARESULT_BASE
// ADC PPB Results Address
#define MOTOR1_VA_ADC_PPB_RESULT_ADDR       (ADCARESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_VA_PPB_NUM))
#define MOTOR2_VA_ADC_PPB_RESULT_ADDR       (ADCARESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_VA_PPB_NUM))
#define MOTOR1_IA_ADC_PPB_RESULT_ADDR       (ADCARESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_IA_PPB_NUM))
#define MOTOR2_IA_ADC_PPB_RESULT_ADDR       (ADCARESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_IA_PPB_NUM))
/**
 * @}
 */

/** @defgroup ADC_B config.
 *  @brief    Configure the Analog to Digital Core \& 2 Analog to Digital Wrapper Logic blocks
 * @{
 */
// Channels
#define MOTOR1_VB_CH                        ADC_CH_ADCIN2
#define MOTOR2_VB_CH                        ADC_CH_ADCIN3
#define MOTOR1_IB_CH                        ADC_CH_ADCIN0
#define MOTOR2_IB_CH                        ADC_CH_ADCIN1
// SOC Number
#define MOTOR1_VB_SOC_NUM                   ADC_SOC_NUMBER0
#define MOTOR2_VB_SOC_NUM                   ADC_SOC_NUMBER1
#define MOTOR1_IB_SOC_NUM                   ADC_SOC_NUMBER2
#define MOTOR2_IB_SOC_NUM                   ADC_SOC_NUMBER3
// PPB Number
#define MOTOR1_VB_PPB_NUM                   ADC_PPB_NUMBER1
#define MOTOR2_VB_PPB_NUM                   ADC_PPB_NUMBER2
#define MOTOR1_IB_PPB_NUM                   ADC_PPB_NUMBER3
#define MOTOR2_IB_PPB_NUM                   ADC_PPB_NUMBER4
// ADC Base Address
#define MOTOR1_VB_ADC_ADDR                  ADCB_BASE
#define MOTOR2_VB_ADC_ADDR                  ADCB_BASE
#define MOTOR1_IB_ADC_ADDR                  ADCB_BASE
#define MOTOR2_IB_ADC_ADDR                  ADCB_BASE
#define MOTOR12_IVB_ADC_ADDR                ADCB_BASE
// ADC Results Base Address
#define MOTOR1_VB_ADC_RESULT_ADDR           (ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_VB_SOC_NUM))
#define MOTOR2_VB_ADC_RESULT_ADDR           (ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_VB_SOC_NUM))
#define MOTOR1_IB_ADC_RESULT_ADDR           (ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_IB_SOC_NUM))
#define MOTOR2_IB_ADC_RESULT_ADDR           (ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_IB_SOC_NUM))
#define MOTOR12_IVB_ADC_RESULT_ADDR         ADCBRESULT_BASE
// ADC PPB Results Address
#define MOTOR1_VB_ADC_PPB_RESULT_ADDR       (ADCBRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_VB_PPB_NUM))
#define MOTOR2_VB_ADC_PPB_RESULT_ADDR       (ADCBRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_VB_PPB_NUM))
#define MOTOR1_IB_ADC_PPB_RESULT_ADDR       (ADCBRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_IB_PPB_NUM))
#define MOTOR2_IB_ADC_PPB_RESULT_ADDR       (ADCBRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_IB_PPB_NUM))
/**
 * @}
 */

/** @defgroup ADC_C config.
 *  @brief    Configure the Analog to Digital Core \& 2 Analog to Digital Wrapper Logic blocks
 * @{
 */
// Channels
#define MOTOR12_VBUS_CH                     ADC_CH_ADCIN2
// SOC Number
#define MOTOR12_VBUS_SOC_NUM                ADC_SOC_NUMBER0
// PPB Number
#define MOTOR12_VBUS_PPB_NUM                ADC_PPB_NUMBER1
// ADC Base Address
#define MOTOR12_VBUS_ADC_ADDR               ADCC_BASE
// ADC Results Base Address
#define MOTOR12_VBUS_ADC_RESULT_ADDR        (ADCCRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR12_VBUS_SOC_NUM))
// ADC PPB Results Address
#define MOTOR12_VBUS_ADC_PPB_RESULT_ADDR    (MOTOR12_VBUS_ADC_RESULT_ADDR + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR12_VBUS_PPB_NUM))
/**
 * @}
 */

/** @defgroup ADC_D config.
 *  @brief    Configure the Analog to Digital Core \& 2 Analog to Digital Wrapper Logic blocks
 * @{
 */
// Channels
#define MOTOR1_VC_CH                        ADC_CH_ADCIN2
#define MOTOR2_VC_CH                        ADC_CH_ADCIN3
#define MOTOR1_IC_CH                        ADC_CH_ADCIN0
#define MOTOR2_IC_CH                        ADC_CH_ADCIN1
// SOC Number
#define MOTOR1_VC_SOC_NUM                   ADC_SOC_NUMBER0
#define MOTOR2_VC_SOC_NUM                   ADC_SOC_NUMBER1
#define MOTOR1_IC_SOC_NUM                   ADC_SOC_NUMBER2
#define MOTOR2_IC_SOC_NUM                   ADC_SOC_NUMBER3
// PPB Number
#define MOTOR1_VC_PPB_NUM                   ADC_PPB_NUMBER1
#define MOTOR2_VC_PPB_NUM                   ADC_PPB_NUMBER2
#define MOTOR1_IC_PPB_NUM                   ADC_PPB_NUMBER3
#define MOTOR2_IC_PPB_NUM                   ADC_PPB_NUMBER4
// ADC Base Address
#define MOTOR1_IC_ADC_ADDR                  ADCD_BASE
#define MOTOR2_IC_ADC_ADDR                  ADCD_BASE
#define MOTOR1_VC_ADC_ADDR                  ADCD_BASE
#define MOTOR2_VC_ADC_ADDR                  ADCD_BASE
#define MOTOR12_IVC_ADC_ADDR                ADCD_BASE
// ADC Results Base Address
#define MOTOR1_VC_ADC_RESULT_ADDR           (ADCDRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_VC_SOC_NUM))
#define MOTOR2_VC_ADC_RESULT_ADDR           (ADCDRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_VC_SOC_NUM))
#define MOTOR1_IC_ADC_RESULT_ADDR           (ADCDRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_IC_SOC_NUM))
#define MOTOR2_IC_ADC_RESULT_ADDR           (ADCDRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_IC_SOC_NUM))
#define MOTOR12_IVC_ADC_RESULT_ADDR         ADCDRESULT_BASE
// ADC PPB Results Address
#define MOTOR1_VC_ADC_PPB_RESULT_ADDR       (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_VC_PPB_NUM))
#define MOTOR2_VC_ADC_PPB_RESULT_ADDR       (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_VC_PPB_NUM))
#define MOTOR1_IC_ADC_PPB_RESULT_ADDR       (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_IC_PPB_NUM))
#define MOTOR2_IC_ADC_PPB_RESULT_ADDR       (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_IC_PPB_NUM))
/**
 * @}
 */

/******************************************************************************************************
*                                               HAL Defines
******************************************************************************************************/
/** @defgroup Debug pins config.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#ifndef UOMODRI
#define DEBUG_INT_PIN1                  35
#define DEBUG_INT_PIN1_CFG              GPIO_35_GPIO35
#define DEBUG_INT_PIN2                  37
#define DEBUG_INT_PIN2_CFG              GPIO_37_GPIO37
#endif
/**
 * @}
 */

/** @defgroup ePWM_Module configuration.
 *  @brief    Define the address of all the ePWM used (2 motors).
 * @{
 */
#define MOTOR1_PWM1_BASE                EPWM1_BASE
#define MOTOR1_PWM2_BASE                EPWM2_BASE
#define MOTOR1_PWM3_BASE                EPWM3_BASE
#define MOTOR2_PWM1_BASE                EPWM4_BASE
#define MOTOR2_PWM2_BASE                EPWM5_BASE
#define MOTOR2_PWM3_BASE                EPWM6_BASE
/**
 * @}
 */

/** @defgroup ePWM_GPIO_pins configuration.
 *  @brief    Configure GPIO pins associated to 12x ePWM (2 motors).
 * @{
 */
#define MOTOR1_PWM1_CHA                 0
#define MOTOR1_PWM1_CHA_CFG             GPIO_0_EPWM1A
#define MOTOR1_PWM1_CHB                 1
#define MOTOR1_PWM1_CHB_CFG             GPIO_1_EPWM1B
#define MOTOR1_PWM2_CHA                 2
#define MOTOR1_PWM2_CHA_CFG             GPIO_2_EPWM2A
#define MOTOR1_PWM2_CHB                 3
#define MOTOR1_PWM2_CHB_CFG             GPIO_3_EPWM2B
#define MOTOR1_PWM3_CHA                 4
#define MOTOR1_PWM3_CHA_CFG             GPIO_4_EPWM3A
#define MOTOR1_PWM3_CHB                 5
#define MOTOR1_PWM3_CHB_CFG             GPIO_5_EPWM3B
#define MOTOR2_PWM1_CHA                 6
#define MOTOR2_PWM1_CHA_CFG             GPIO_6_EPWM4A
#define MOTOR2_PWM1_CHB                 7
#define MOTOR2_PWM1_CHB_CFG             GPIO_7_EPWM4B
#define MOTOR2_PWM2_CHA                 8
#define MOTOR2_PWM2_CHA_CFG             GPIO_8_EPWM5A
#define MOTOR2_PWM2_CHB                 9
#define MOTOR2_PWM2_CHB_CFG             GPIO_9_EPWM5B
#define MOTOR2_PWM3_CHA                 10
#define MOTOR2_PWM3_CHA_CFG             GPIO_10_EPWM6A
#define MOTOR2_PWM3_CHB                 11
#define MOTOR2_PWM3_CHB_CFG             GPIO_11_EPWM6B
/**
 * @}
 */

/** @defgroup ePWM_clock configuration.
 *  @brief    Configure the clock subdivision of SYSCLK associated to 12x ePWM (2 motors).
 * @{
 */
#define PLLSYSCLK_TO_EPWMCLK_DIV        SYSCTL_EPWMCLK_DIV_2
#define EPWMCLK_FREQ                    (DEVICE_SYSCLK_FREQ / (PLLSYSCLK_TO_EPWMCLK_DIV + 1))
#define EPWMCLK_TO_TBCLK_DIV            EPWM_CLOCK_DIVIDER_1
#define HRPWMCLK_TO_TBCLK_DIV           EPWM_HSCLOCK_DIVIDER_1
#define TBCLK_FREQ                      (EPWMCLK_FREQ / ((EPWMCLK_TO_TBCLK_DIV + 1) * (HRPWMCLK_TO_TBCLK_DIV + 1)))
/**
 * @}
 */

/** @defgroup ePWM_general configuration.
 *  @brief    Configure the general registers associated to 12x ePWM (2 motors).
 * @{
 */
#define PWM_FREQ                        (40000) //[Hz]
#define PWM_COUNTER_MODE                EPWM_COUNTER_MODE_UP_DOWN
#define PWM_TBPRD                       (TBCLK_FREQ / PWM_FREQ / ((PWM_COUNTER_MODE == EPWM_COUNTER_MODE_UP_DOWN) ? (2) : (1))) // PWM Period register (max value)
#define PWM_HALF_TBPRD                  (PWM_TBPRD / 2)
#define PWM_TBPHS                       (0) // PWM Phase register (min reload value)
#define PWM_TBCTR                       (0) // PWM initial current counter
#define PWM_CCCMPx                      PWM_TBPRD // PWM initial value for capture/compare register
#define PWM_DBRED                       (0) // PWM rising edge delay. ePWMxA (low to high) & ePWMxB (high to low) - Active High Complementary (AHC)
#define PWM_DBFED                       (0) // PWM falling edge delay. ePWMxA (high to low) & ePWMxB (low to high) - Active High Complementary (AHC)
#define PWM_SHADOW_MODE_EN              (1) // PWM Shadow mode definition
#if PWM_SHADOW_MODE_EN
#define PWM_SHADOW_MODE_ET              EPWM_PERIOD_SHADOW_LOAD
#define PWM_SHADOW_MODE_EVENT_ET        EPWM_SHADOW_LOAD_MODE_COUNTER_SYNC
#define PWM_SHADOW_MODE_EVENT_CC        EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_AQ        EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_DB        EPWM_DB_LOAD_ON_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_DB_RED    EPWM_RED_LOAD_ON_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_DB_FED    EPWM_FED_LOAD_ON_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_GLOBAL    EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD
#else
//#define PWM_SHADOW_MODE_PERIOD          EPWM_PERIOD_DIRECT_LOAD
#define PWM_SHADOW_MODE_ET              EPWM_PERIOD_DIRECT_LOAD
#endif
/**
 * @}
 */

/******************************************************************************************************
*                                               Encoders Defines
*******************************************************************************************************/

#define ENC1_CHA                        20
#define ENC1_CHA_CFG                    GPIO_20_EQEP1A
#define ENC1_CHB                        21
#define ENC1_CHB_CFG                    GPIO_21_EQEP1B
#define ENC1_CHI                        23
#define ENC1_CHI_CFG                    GPIO_23_EQEP1I
#define ENC2_CHA                        54
#define ENC2_CHA_CFG                    GPIO_54_EQEP2A
#define ENC2_CHB                        55
#define ENC2_CHB_CFG                    GPIO_55_EQEP2B
#define ENC2_CHI                        57
#define ENC2_CHI_CFG                    GPIO_57_EQEP2I

/******************************************************************************************************
*                                               SPI Defines
*******************************************************************************************************/
#define DRV_SPI_SIMO                    16
#define DRV_SPI_SIMO_CFG                GPIO_16_SPISIMOA
#define DRV_SPI_SOMI                    17
#define DRV_SPI_SOMI_CFG                GPIO_17_SPISOMIA
#define DRV_SPI_CLK                     18
#define DRV_SPI_CLK_CFG                 GPIO_18_SPICLKA

#ifdef UOMODRI
// DRV1 SPI defines
#define DRV1_GPIO_EN                    89
#define DRV1_GPIO_EN_CFG                GPIO_89_GPIO89
#define DRV1_GPIO_CS                    19
#define DRV1_GPIO_CS_CFG                GPIO_19_GPIO19
#define DRV1_GPIO_NFAULT                90
#define DRV1_GPIO_NFAULT_CFG            GPIO_90_GPIO90
// DRV2 SPI defines
#define DRV2_GPIO_EN                    37
#define DRV2_GPIO_EN_CFG                GPIO_37_GPIO37
#define DRV2_GPIO_CS                    35
#define DRV2_GPIO_CS_CFG                GPIO_35_GPIO35
#define DRV2_GPIO_NFAULT                36
#define DRV2_GPIO_NFAULT_CFG            GPIO_36_GPIO36
#else
// DRV1 SPI defines
#define DRV1_GPIO_EN                    89
#define DRV1_GPIO_EN_CFG                GPIO_89_GPIO89
#define DRV1_GPIO_CS                    19
#define DRV1_GPIO_CS_CFG                GPIO_19_GPIO19
#define DRV1_GPIO_NFAULT                91
#define DRV1_GPIO_NFAULT_CFG            GPIO_91_GPIO91
// DRV2 SPI defines
#define DRV2_GPIO_EN                    133
#define DRV2_GPIO_EN_CFG                GPIO_133_GPIO133
#define DRV2_GPIO_CS                    61
#define DRV2_GPIO_CS_CFG                GPIO_61_GPIO61
#define DRV2_GPIO_NFAULT                67
#define DRV2_GPIO_NFAULT_CFG            GPIO_67_GPIO67
#endif

/** @defgroup eQEP_Module configuration.
 *  @brief    Define the address of all the eQEP used (2 quadrature encoders).
 * @{
 */
#define MOTOR1_QEP_BASE                 EQEP1_BASE
#define MOTOR2_QEP_BASE                 EQEP2_BASE
#define MOTOR1_QEP_COUNTER_ADDR         (MOTOR1_QEP_BASE + EQEP_O_QPOSCNT)
#define MOTOR2_QEP_COUNTER_ADDR         (MOTOR2_QEP_BASE + EQEP_O_QPOSCNT)
/**
 * @}
 */

/** @defgroup eQEP_general configuration.
 *  @brief    Configure the general registers associated to teh eQEP.
 * @{
 */
#define QEP1_SWAP                       EQEP_CONFIG_NO_SWAP         // Set to EQEP_CONFIG_SWAP if increments in the wrong direction
#define QEP2_SWAP                       EQEP_CONFIG_NO_SWAP
//#define QEP_CONFIG                      (EQEP_CONFIG_2X_RESOLUTION | EQEP_CONFIG_UP_COUNT | EQEP_CONFIG_NO_SWAP)
#define QEP_EMUL_MODE                   EQEP_EMULATIONMODE_RUNFREE
#define QEP_LATCH_MODE                  (EQEP_LATCH_RISING_INDEX | EQEP_LATCH_UNIT_TIME_OUT)
//#define QEP_ENCODER_STEPS               5000
//#define QEP_ENCODER_POSITION_MAX        ((4 * QEP_ENCODER_STEPS) - 1)
#define QEP_RESET_POSITION_MODE         EQEP_POSITION_RESET_MAX_POS
#define QEP_CAPTURE_CLOCK_DIV           EQEP_CAPTURE_CLK_DIV_8
#define QEP_UNIT_POS_EVENT_DIV          EQEP_UNIT_POS_EVNT_DIV_16
#define PERxSYSCLK_TO_EQEPCLK_DIV       1
#define EQEPCLK_FREQ                    (DEVICE_SYSCLK_FREQ / pow(2.0, ((QEP_CAPTURE_CLOCK_DIV >> 4) & 0x07)))

//#define QEP_UNIT_TIMER_TICKS            PWM_TBPRD               // Sync QEP with PWM
#define MOTOR1_QEP_SWAP                 EQEP_CONFIG_NO_SWAP         // Set to EQEP_CONFIG_SWAP if increments in the wrong direction
#define MOTOR2_QEP_SWAP                 EQEP_CONFIG_NO_SWAP
#define MOTOR1_ENCODER_LINES_NB         5000
#define MOTOR2_ENCODER_LINES_NB         5000
// Velocity computation
#define QEP_UNIT_TIMER_FREQ             1000 // [Hz] Velocity computation frequency
#define QEP_UNIT_TIMER_TICKS            (DEVICE_SYSCLK_FREQ / QEP_UNIT_TIMER_FREQ)
/**
 * @}
 */

#define DRV_SPI_BASE                    SPIA_BASE
#define COM_SPI_BASE                    SPIB_BASE


// DRV 1 measurement defines
#define MOTOR1_ADC_SOC_TRIGGER1         ADC_TRIGGER_EPWM1_SOCA
#define MOTOR1_ADC_SOC_TRIGGER2         ADC_TRIGGER_EPWM1_SOCB
#define MOTOR1_ADC_PPB_NUM1             ADC_PPB_NUMBER1
#define MOTOR1_ADC_PPB_NUM2             ADC_PPB_NUMBER3
#define MOTOR1_ADC_SPL_WINDOW           ADC_SAMPLING_WINDOW
#define MOTOR1_ADC_SOC_PRI              ADC_PRI_ALL_HIPRI
#define MOTOR1_ADC_INT_NUM              ADC_INT_NUMBER1
#define MOTOR1_ADC_INT_SOC_TRIGGER      ADC_INT_SOC_TRIGGER_NONE

// DRV 2 measurement defines
#define MOTOR2_ADC_SOC_TRIGGER1         ADC_TRIGGER_EPWM4_SOCA
#define MOTOR2_ADC_SOC_TRIGGER2         ADC_TRIGGER_EPWM4_SOCB
#define MOTOR2_ADC_PPB_NUM1             ADC_PPB_NUMBER2
#define MOTOR2_ADC_PPB_NUM2             ADC_PPB_NUMBER4
#define MOTOR2_ADC_SPL_WINDOW           ADC_SAMPLING_WINDOW
#define MOTOR2_ADC_SOC_PRI              ADC_PRI_ALL_HIPRI
#define MOTOR2_ADC_INT_NUM              ADC_INT_NUMBER1
#define MOTOR2_ADC_INT_SOC_TRIGGER      ADC_INT_SOC_TRIGGER_NONE

// Global ADC defines
#define ADC_TRIGGER_EVT1                ADC_TRIGGER_EPWM1_SOCA
#define ADC_TRIGGER_EVT2                ADC_TRIGGER_EPWM4_SOCA
#define ADC_TRIGGER_EVT3                ADC_TRIGGER_EPWM1_SOCB
#define ADC_TRIGGER_EVT4                ADC_TRIGGER_EPWM4_SOCB
//typedef   DRV (*Operation)(void);


//
#define ADC_VREF                    3.0f   // [V]
#define DRV_CSA_GAIN_REG            DRV_CSA_GAIN_10_V_V  // [V/V] Possible values: 5, 10, 20, 40
#define DRV_CSA_GAIN_VAL            (5 << DRV_CSA_GAIN_REG) // base gain
#define DRV_R_SHUNT                 0.01f // [Ohm]
// Conversion defines
//#define ADC_VBUS_SCALE              ((ADC_VREF / 4096.0f) * (4.99f + 82.0f) / 4.99f) // Voltage divider R1:4.99kO R2:82kO
//#define ADC_ISEN_SCALE              ((ADC_VREF / 4096.0f) / ((5 << DRV_CSA_GAIN) * DRV_R_SHUNT)) //0.5 * ADC_VREF / (DRV_CSA_GAIN * DRV_R_SHUNT)
//#define ADC_VSEN_SCALE              ((ADC_VREF / 4096.0f) / (5 << DRV_CSA_GAIN)) //ADC_VREF * (4.99 + 82.0) / 4.99 // Voltage divider R1:4.99kO R2:82kO

#define ADC_CURRENT_SCALE           (ADC_VREF * (-0.5f) / DRV_CSA_GAIN_VAL / DRV_R_SHUNT / ((ADC_RESOLUTION) ? (32768.0f) : (2048.0f)))
#define ADC_VOLTAGE_SCALE           (ADC_VREF * (4.99f + 82.0f) / 4.99f / ((ADC_RESOLUTION) ? (65536.0f) : (4096.0f)))// Voltage divider R1:4.99kO R2:82kO



/******************************************************************************************************
*                                             PWM defines
*******************************************************************************************************/

#define PWM_FREQUENCY                   40000UL //[Hz]

// Initialization define
#define PWM_HALF_PERIOD                 (DEVICE_SYSCLK_FREQ / (2.0 * PWM_FREQUENCY)) / 2.0

/*** Encoder defines ***/
#define MOTOR1_ENC_RESOLUTION_SCALE     (4.0 * MOTOR1_ENCODER_LINES_NB)
#define MOTOR2_ENC_RESOLUTION_SCALE     (4.0 * MOTOR2_ENCODER_LINES_NB)
#define MOTOR1_ENC_HIGH_SPEED_SCALE     (2.0 * M_PI * QEP_UNIT_TIMER_FREQ) // 2pi / dt
#define MOTOR2_ENC_HIGH_SPEED_SCALE     (2.0 * M_PI * QEP_UNIT_TIMER_FREQ) // 2pi / dt
#define MOTOR1_ENC_LOW_SPEED_SCALE      (2.0 * M_PI * MOTOR1_ENC_RESOLUTION_SCALE * DEVICE_SYSCLK_FREQ * QEP_CAPTURE_CLOCK_DIV / QEP_UNIT_POS_EVENT_DIV) // 4 is pos divider and 128 is clock divider
#define MOTOR2_ENC_LOW_SPEED_SCALE      (2.0 * M_PI * MOTOR2_ENC_RESOLUTION_SCALE * DEVICE_SYSCLK_FREQ * QEP_CAPTURE_CLOCK_DIV / QEP_UNIT_POS_EVENT_DIV)
//LPF_setBandwidth(&foc->encoder.speedEst.lpf, PWM_FREQUENCY, 200); // 200Hz low-pass filter for velocity estimation



// Initialization define
//#define QEP_UNIT_TIMER_TICKS            DEVICE_SYSCLK_FREQ / QEP_UNIT_TIMER_FREQ

/*** Motor 1 constants ***/
#define MOTOR1_KV                           300.0     // Motor constant (rpm/V)
#define MOTOR1_KE                           0.0017    //1.0 / (MOTOR1_KV * FM_RPM2RADPS * MOTOR1_POLES_PAIRS) // Motor back EMF constant (V/rad/s)
#define MOTOR1_KI                           0.1       // Motor torque constant (N.m/A)
#define MOTOR1_RS                           0.7       // Stator resistance (ohm)
#define MOTOR1_LD                           134e-6    // Stator d-axis inductance (H)
#define MOTOR1_LQ                           140e-6    // Stator q-axis inductance (H)
#define MOTOR1_POLES_PAIRS                  12        // Number of poles
#define MOTOR1_CUR_LOOP_BANDWIDTH           500.0     // Current loop bandwidth (Hz)

/*** Motor 2 constants ***/
#define MOTOR2_KV                           300.0     // Motor constant (rpm/V)
#define MOTOR2_KE                           0.0017    // Motor back EMF constant (V/rad/s)
#define MOTOR2_KI                           0.1       // Motor torque constant (N.m/A)
#define MOTOR2_RS                           0.7       // Stator resistance (ohm)
#define MOTOR2_LD                           134e-6    // Stator d-axis inductance (H)
#define MOTOR2_LQ                           140e-6    // Stator q-axis inductance (H)
#define MOTOR2_POLES_PAIRS                  12        // Number of poles
#define MOTOR2_CUR_LOOP_BANDWIDTH           500.0     // Current loop bandwidth (Hz)

#define HWSREGH(x)              (*((volatile int16_t *)((uintptr_t)(x))))

#endif
