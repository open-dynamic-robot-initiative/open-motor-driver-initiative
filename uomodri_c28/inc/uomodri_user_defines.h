/*
 * File name: omodri_user_defines.h
 * Description: Header file containing user definitions
 */

#ifndef __UOMODRI_USER_DEFINES_H__
#define __UOMODRI_USER_DEFINES_H__

#include "f2838x_device.h"
#include "driverlib.h"
#include "device.h"

//#include "hal.h"
#include "drv8353.h"

/******************************************************************************************************
*                                               GLOBAL macros
******************************************************************************************************/
/** @defgroup DEV_defines
 *  @brief    Special configurations definitions.
 * @{
 */
#define UOMODRI                                 0
#define CLA                                     0
#define CM                                      1
#define UOMODRI_VBUS_FLT(flt_coef, y, x)        LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_IDQ_FLT(flt_coef, y, x)         LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_RES_ESTIM_FLT(flt_coef, y, x)   LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_SPEED_FLT(flt_coef, y, x)       LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_SPI_DEBUG_CONFIG                1
#define UOMODRI_FEED_FORWARD_EN                 1

#define OMODRI_PAL_SETUP                        0   // Set this to 1 to use the PAL pin affectation
//#define UOMODRI_DEBUG_LOGGED_MOTOR              MOTOR_1
//#define SPI_DEBUG_CONFIGURATION                 0   // Set this to 1 to use debug configuration
// Word number in a full communication
//#define SPI_COM_LENGTH              17
/**
 * @}
 */

/** @defgroup CONSTANT_defines
 *  @brief    General constants definitions.
 * @{
 */
//#ifndef FM_PI
////#define FM_PI                                   3.14159265358979f
//#endif

#ifndef FM_SQRT3
#define FM_SQRT3                                1.7320508075689f    /* sqrt(3) */
#endif

#ifndef FM_1DIVSQRT3
#define FM_1DIVSQRT3                            0.57735026919f      /* 1/sqrt(3) */
#endif

#ifndef FM_1DIV3
#define FM_1DIV3                                0.333333333333f     /* 1/3 */
#endif

#ifndef FM_RPM2RADPS
#define FM_RPM2RADPS                            (2.0 * M_PI / 60.0)
#endif

#define FM_ABS(x)                               ((x) >= 0.0f ? (x) : -(x))

//#define HWSREGH(x)                              (*((volatile int16_t *)((uintptr_t)(x))))
#define LPF_FILTER_1(flt_coef, y, x)            (y) = ((y) + (flt_coef.a * ((x) - (y))))
#define LPF_FILTER_2(flt_coef, y, x)            (y) = ((flt_coef.a * (x)) + (flt_coef.one_minus_a * (y)))
#define LPF_FILTER_3(flt_coef, y, x)            (y) = (x)
#define FMAX3(x, y, z)                          (__fmax(__fmax((x), (y)), (z)))
//(((x) > (y)) ? (((x) > (z)) ? (x) : (z)) : (((y) > (z)) ? (y) : (z)))
#define FMIN3(x, y, z)                          (__fmin(__fmin((x), (y)), (z)))
//(((x) < (y)) ? (((x) < (z)) ? (x) : (z)) : (((y) < (z)) ? (y) : (z)))
//#define FMINMAX(x, y, z)                        (__fmax(__fmin((x), (y)), (z)))
// 16 bits macros
#define LSB_16(x)                               ((x) & 0xFF)
#define MSB_16(x)                               (((x) >> 8) & 0xFF)
#define FUS_16_TO_32(x, y)                      (((uint32_t)(x) << 16) | (uint32_t)(y))
// 32 bits macros
#define LSB_32(x)                               ((x) & 0xFFFF)
#define MSB_32(x)                               (((x) >> 16) & 0xFFFF)

/******************************************************************************************************
*                                               DEBUG Defines
******************************************************************************************************/
/** @defgroup Debug pins configuration.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#if (UOMODRI == 0)
#define DEBUG_LED_PIN1                          31
#define DEBUG_LED_PIN1_CFG                      GPIO_31_GPIO31
#define DEBUG_LED_PIN2                          34
#define DEBUG_LED_PIN2_CFG                      GPIO_34_GPIO34
#define DEBUG_INT_PIN1                          40
#define DEBUG_INT_PIN1_CFG                      GPIO_40_GPIO40
#define DEBUG_INT_PIN2                          41
#define DEBUG_INT_PIN2_CFG                      GPIO_41_GPIO41
#ifdef  USB
#define USB_M                                   42
#define USB_M_CFG                               GPIO_42_GPIO42
#define USB_P                                   43
#define USB_P_CFG                               GPIO_43_GPIO43
#define USB_VBUS                                46
#define USB_VBUS_CFG                            GPIO_46_GPIO46
#else
#define UART_DBG_TX                             42
#define UART_DBG_TX_CFG                         GPIO_42_UARTA_TX
#define UART_DBG_RX                             43
#define UART_DBG_RX_CFG                         GPIO_43_UARTA_RX
#endif
#endif
/**
 * @}
 */

/******************************************************************************************************
*                                               SPI Defines
*******************************************************************************************************/
/** @defgroup SPI_COM_GPIO_pins - COM configuration.
 *  @brief    Configure GPIO pins associated to external communication.
 * @{
 */
#define COM_SPI_SIMO                            (60)
#define COM_SPI_SIMO_CFG                        GPIO_60_SPIB_SIMO
#define COM_SPI_SOMI                            (61)
#define COM_SPI_SOMI_CFG                        GPIO_61_SPIB_SOMI
#define COM_SPI_CLK                             (65)
#define COM_SPI_CLK_CFG                         GPIO_65_SPIB_CLK
#define COM_SPI_CS                              (59)
#define COM_SPI_CS_CFG                          GPIO_59_SPIB_STEN
/**
 * @}
 */

/** @defgroup SPI communication configuration.
 *  @brief    Configure SPI module associated with external communication.
 * @{
 */
#define COM_SPI_BASE                            SPIB_BASE
#define COM_SPI_TX_BASE_ADDR                    (COM_SPI_BASE + SPI_O_TXBUF)
#define COM_SPI_RX_BASE_ADDR                    (COM_SPI_BASE + SPI_O_RXBUF)
/**
 * @}
 */

/** @defgroup SPI_DRV_GPIO_pins - DRV configuration.
 *  @brief    Configure GPIO pins associated to DRVs communication.
 * @{
 */
#define DRV_SPI_SIMO                            (16)
#define DRV_SPI_SIMO_CFG                        GPIO_16_SPIA_SIMO
#define DRV_SPI_SOMI                            (17)
#define DRV_SPI_SOMI_CFG                        GPIO_17_SPIA_SOMI
#define DRV_SPI_CLK                             (18)
#define DRV_SPI_CLK_CFG                         GPIO_18_SPIA_CLK
#define MOTOR1_DRV_SPI_CS                       (19)
#define MOTOR1_DRV_SPI_CS_CFG                   GPIO_19_GPIO19
#define MOTOR2_DRV_SPI_CS                       (35)
#define MOTOR2_DRV_SPI_CS_CFG                   GPIO_35_GPIO35
/**
 * @}
 */

/** @defgroup SPI DRV configuration.
 *  @brief    Configure SPI module associated with DRV communication.
 * @{
 */
#define DRV_SPI_BASE                            SPIA_BASE
/**
 * @}
 */

/******************************************************************************************************
*                                               DMA Defines
*******************************************************************************************************/
/** @defgroup DMA configuration.
 *  @brief    Configure DMA channels associated to SPI communication.
 * @{
 */
#define DMA_COM_SPI_TX_BASE_ADDR                DMA_CH5_BASE
#define DMA_COM_SPI_RX_BASE_ADDR                DMA_CH6_BASE
#define DMA_COM_SPI_TX_RX_LENGTH                (17)
//#define DMA_TX_BASE_ADDR                        DMA_CH5_BASE
//#define DMA_RX_BASE_ADDR                        DMA_CH6_BASE
/**
 * @}
 */

/******************************************************************************************************
*                                               DRV Defines
*******************************************************************************************************/
/** @defgroup DRV_GPIO_pins - DRV GPIO pins configuration.
 *  @brief    Configure the pins necessaries for DRV selection \& fault status.
 * @{
 */
// DRV1 SPI defines
#define MOTOR1_DRV_GPIO_EN                      (89)
#define MOTOR1_DRV_GPIO_EN_CFG                  GPIO_89_GPIO89
#define MOTOR1_DRV_GPIO_NFAULT                  (90)
#define MOTOR1_DRV_GPIO_NFAULT_CFG              GPIO_90_GPIO90
// DRV2 SPI defines
#define MOTOR2_DRV_GPIO_EN                      (37)
#define MOTOR2_DRV_GPIO_EN_CFG                  GPIO_37_GPIO37
#define MOTOR2_DRV_GPIO_NFAULT                  (36)
#define MOTOR2_DRV_GPIO_NFAULT_CFG              GPIO_36_GPIO36
/**
 * @}
 */

/******************************************************************************************************
*                                             ADC defines
*******************************************************************************************************/
/** @defgroup ADC_global_defines
 *  @brief    Configure ADC subsystems.
 * @{
 */
// ADC clock config
#define PERxSYSCLK_TO_ADCCLK_DIV                ADC_CLK_DIV_4_0
#define ADCCLK_FREQ                             (DEVICE_SYSCLK_FREQ / ((PERxSYSCLK_TO_ADCCLK_DIV / 2) + 1))
// ADC defines
#define ADC_SAMPLING_WINDOW                     119   // From data sheet Table 20-12
#define ADC_RESOLUTION_BIT						ADC_RESOLUTION_16BIT
#define ADC_RESOLUTION_MAX                      ((ADC_RESOLUTION_BIT) ? (65536) : (4096))
//#define ADC_RESOLUTION_MID                      (ADC_RESOLUTION_MAX / 2)
#define ADC_SIGNAL_MODE                         ADC_MODE_SINGLE_ENDED
#define ADC_PULSE_END_MODE                      ADC_PULSE_END_OF_CONV
#define ADC_SOC_PRIORITY                        ADC_PRI_THRU_SOC3_HIPRI  //ADC_PRI_ALL_ROUND_ROBIN
#define ADC_INT_SOC_TRIGGER                     ADC_INT_SOC_TRIGGER_NONE

#define ADC_VREF                                3.0f   // [V]
#define DRV_CSA_GAIN_REG                        DRV_CSA_GAIN_10_V_V  // [V/V] Possible values: 5, 10, 20, 40
#define DRV_CSA_GAIN_VAL                        (5 << DRV_CSA_GAIN_REG) // base gain

#if (UOMODRI == 0)
#define R_SHUNT                                 0.007f // [Ohm]
#define BRIDGE_DIVIDER                          ((4.99f + 82.0f) / 4.99f)
#else
#define R_SHUNT                                 0.01f // [Ohm]
#define BRIDGE_DIVIDER                          ((5.1f + 100.0f) / 5.1f)
#endif
#define ADC_VOLTAGE_SCALE                       (ADC_VREF * BRIDGE_DIVIDER / ADC_RESOLUTION_MAX)// Voltage divider R1:4.99kO R2:82kO
#define ADC_CURRENT_SCALE                       (ADC_VREF * (-0.5f) / DRV_CSA_GAIN_VAL / R_SHUNT / ADC_RESOLUTION_MAX / 2) //(0x1000 << (ADC_RESOLUTION >> 4)))
/**
 * @}
 */

/** @defgroup ADC SOC Trigger event source
 *  @brief    Associate the trigger event source to start conversions.
 * @{
 */
#define MOTOR1_ADC_SOC_TRIGGER_EVT1             ADC_TRIGGER_EPWM1_SOCB
#define MOTOR1_ADC_SOC_TRIGGER_EVT3             ADC_TRIGGER_EPWM1_SOCA

#define MOTOR2_ADC_SOC_TRIGGER_EVT2             ADC_TRIGGER_EPWM4_SOCA
#define MOTOR2_ADC_SOC_TRIGGER_EVT4             ADC_TRIGGER_EPWM4_SOCB
/**
 * @}
 */

/** @defgroup ADC physical channels
 *  @brief    Configure the analog channel inputs (U, I \& Vbus of Motor1 \& Motor2).
 * @{
 */
#define MOTOR1_VA_CH                            ADC_CH_ADCIN2
#define MOTOR1_VB_CH                            ADC_CH_ADCIN2
#define MOTOR1_VC_CH                            ADC_CH_ADCIN2
#define MOTOR12_VBUS_CH                         ADC_CH_ADCIN2

#define MOTOR1_IA_CH                            ADC_CH_ADCIN0
#define MOTOR1_IB_CH                            ADC_CH_ADCIN0
#define MOTOR1_IC_CH                            ADC_CH_ADCIN0

#define MOTOR2_VA_CH                            ADC_CH_ADCIN3
#define MOTOR2_VB_CH                            ADC_CH_ADCIN3
#define MOTOR2_VC_CH                            ADC_CH_ADCIN3

#define MOTOR2_IA_CH                            ADC_CH_ADCIN1
#define MOTOR2_IB_CH                            ADC_CH_ADCIN1
#define MOTOR2_IC_CH                            ADC_CH_ADCIN1
/**
 * @}
 */

/** @defgroup ADC SOC (Start-Of-Conversion)
 *  @brief    Associate the SOC for each measure to perform.
 * @{
 */
#define MOTOR1_VA_SOC_NUM                       ADC_SOC_NUMBER0
#define MOTOR1_VB_SOC_NUM                       ADC_SOC_NUMBER0
#define MOTOR1_VC_SOC_NUM                       ADC_SOC_NUMBER0
#define MOTOR12_VBUS_SOC_NUM                    ADC_SOC_NUMBER0

#define MOTOR1_IA_SOC_NUM                       ADC_SOC_NUMBER2
#define MOTOR1_IB_SOC_NUM                       ADC_SOC_NUMBER2
#define MOTOR1_IC_SOC_NUM                       ADC_SOC_NUMBER2

#define MOTOR2_VA_SOC_NUM                       ADC_SOC_NUMBER1
#define MOTOR2_VB_SOC_NUM                       ADC_SOC_NUMBER1
#define MOTOR2_VC_SOC_NUM                       ADC_SOC_NUMBER1

#define MOTOR2_IA_SOC_NUM                       ADC_SOC_NUMBER3
#define MOTOR2_IB_SOC_NUM                       ADC_SOC_NUMBER3
#define MOTOR2_IC_SOC_NUM                       ADC_SOC_NUMBER3
/**
 * @}
 */

/** @defgroup ADC PPB (Post-Processing Blocks)
 *  @brief    Associate the PPB for each measure to perform.
 * @{
 */
#define MOTOR1_VA_PPB_NUM                       ADC_PPB_NUMBER1
#define MOTOR1_VB_PPB_NUM                       ADC_PPB_NUMBER1
#define MOTOR1_VC_PPB_NUM                       ADC_PPB_NUMBER1
#define MOTOR12_VBUS_PPB_NUM                    ADC_PPB_NUMBER1

#define MOTOR1_IA_PPB_NUM                       ADC_PPB_NUMBER3
#define MOTOR1_IB_PPB_NUM                       ADC_PPB_NUMBER3
#define MOTOR1_IC_PPB_NUM                       ADC_PPB_NUMBER3

#define MOTOR2_VA_PPB_NUM                       ADC_PPB_NUMBER2
#define MOTOR2_VB_PPB_NUM                       ADC_PPB_NUMBER2
#define MOTOR2_VC_PPB_NUM                       ADC_PPB_NUMBER2

#define MOTOR2_IA_PPB_NUM                       ADC_PPB_NUMBER4
#define MOTOR2_IB_PPB_NUM                       ADC_PPB_NUMBER4
#define MOTOR2_IC_PPB_NUM                       ADC_PPB_NUMBER4
/**
 * @}
 */

/** @defgroup ADC interrupts source
 *  @brief    Define the interrupt source \& the PIE Interrupt Number associated
 * @{
 */
#define MOTOR1_IA_INT_CH                        INT_ADCA3
#define MOTOR2_IA_INT_CH                        INT_ADCA4

#define MOTOR1_IA_INT_NUM                       ADC_INT_NUMBER3
#define MOTOR2_IA_INT_NUM                       ADC_INT_NUMBER4
/**
 * @}
 */

/** @defgroup ADC Base Address
 *  @brief    Define the ADC base address for all the conversion to be performed.
 * @{
 */
// ADC Base Address
#define MOTOR1_VA_ADC_ADDR                      ADCA_BASE
#define MOTOR1_IA_ADC_ADDR                      ADCA_BASE
#define MOTOR2_VA_ADC_ADDR                      ADCA_BASE
#define MOTOR2_IA_ADC_ADDR                      ADCA_BASE
#define MOTOR12_IVA_ADC_ADDR                    ADCA_BASE

#define MOTOR1_VB_ADC_ADDR                      ADCB_BASE
#define MOTOR1_IB_ADC_ADDR                      ADCB_BASE
#define MOTOR2_VB_ADC_ADDR                      ADCB_BASE
#define MOTOR2_IB_ADC_ADDR                      ADCB_BASE
#define MOTOR12_IVB_ADC_ADDR                    ADCB_BASE

#define MOTOR12_VBUS_ADC_ADDR                   ADCC_BASE

#define MOTOR1_VC_ADC_ADDR                      ADCD_BASE
#define MOTOR1_IC_ADC_ADDR                      ADCD_BASE
#define MOTOR2_VC_ADC_ADDR                      ADCD_BASE
#define MOTOR2_IC_ADC_ADDR                      ADCD_BASE
#define MOTOR12_IVC_ADC_ADDR                    ADCD_BASE
/**
 * @}
 */

/** @defgroup ADC Base result Address
 *  @brief    Define the ADC results base address for all the conversion to be performed.
 * @{
 */
#define MOTOR1_VA_ADC_RESULT_ADDR               (ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_VA_SOC_NUM))
#define MOTOR1_IA_ADC_RESULT_ADDR               (ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_IA_SOC_NUM))
#define MOTOR2_VA_ADC_RESULT_ADDR               (ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_VA_SOC_NUM))
#define MOTOR2_IA_ADC_RESULT_ADDR               (ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_IA_SOC_NUM))
#define MOTOR12_IVA_ADC_RESULT_ADDR             ADCARESULT_BASE

#define MOTOR1_VB_ADC_RESULT_ADDR               (ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_VB_SOC_NUM))
#define MOTOR1_IB_ADC_RESULT_ADDR               (ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_IB_SOC_NUM))
#define MOTOR2_VB_ADC_RESULT_ADDR               (ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_VB_SOC_NUM))
#define MOTOR2_IB_ADC_RESULT_ADDR               (ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_IB_SOC_NUM))
#define MOTOR12_IVB_ADC_RESULT_ADDR             ADCBRESULT_BASE

#define MOTOR12_VBUS_ADC_RESULT_ADDR            (ADCCRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR12_VBUS_SOC_NUM))

#define MOTOR1_VC_ADC_RESULT_ADDR               (ADCDRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_VC_SOC_NUM))
#define MOTOR1_IC_ADC_RESULT_ADDR               (ADCDRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR1_IC_SOC_NUM))
#define MOTOR2_VC_ADC_RESULT_ADDR               (ADCDRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_VC_SOC_NUM))
#define MOTOR2_IC_ADC_RESULT_ADDR               (ADCDRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * MOTOR2_IC_SOC_NUM))
#define MOTOR12_IVC_ADC_RESULT_ADDR             ADCDRESULT_BASE
/**
 * @}
 */

/** @defgroup ADC PPB Base result Address
 *  @brief    Define the ADC PPB results base address for all the conversion to be performed.
 * @{
 */
#define MOTOR1_VA_ADC_PPB_RESULT_ADDR           (ADCARESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_VA_PPB_NUM))
#define MOTOR1_IA_ADC_PPB_RESULT_ADDR           (ADCARESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_IA_PPB_NUM))
#define MOTOR2_VA_ADC_PPB_RESULT_ADDR           (ADCARESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_VA_PPB_NUM))
#define MOTOR2_IA_ADC_PPB_RESULT_ADDR           (ADCARESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_IA_PPB_NUM))

#define MOTOR1_VB_ADC_PPB_RESULT_ADDR           (ADCBRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_VB_PPB_NUM))
#define MOTOR1_IB_ADC_PPB_RESULT_ADDR           (ADCBRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_IB_PPB_NUM))
#define MOTOR2_VB_ADC_PPB_RESULT_ADDR           (ADCBRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_VB_PPB_NUM))
#define MOTOR2_IB_ADC_PPB_RESULT_ADDR           (ADCBRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_IB_PPB_NUM))

#define MOTOR12_VBUS_ADC_PPB_RESULT_ADDR        (ADCCRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR12_VBUS_PPB_NUM))

#define MOTOR1_VC_ADC_PPB_RESULT_ADDR           (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_VC_PPB_NUM))
#define MOTOR1_IC_ADC_PPB_RESULT_ADDR           (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_IC_PPB_NUM))
#define MOTOR2_VC_ADC_PPB_RESULT_ADDR           (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_VC_PPB_NUM))
#define MOTOR2_IC_ADC_PPB_RESULT_ADDR           (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_IC_PPB_NUM))
/**
 * @}
 */

/** @defgroup ADC base \& ADC result registers
 *  @brief    Define the ADC result registers for all the conversion to be performed.
 * @{
 */
#define MOTOR12_IVA_ADC_VAR                     (&AdcaRegs)
#define MOTOR12_IVB_ADC_VAR                     (&AdcbRegs)
#define MOTOR12_VBUS_ADC_VAR                    (&AdccRegs)
#define MOTOR12_IVC_ADC_VAR                     (&AdcdRegs)

#define MOTOR12_IVA_ADC_RESULT_VAR              (&AdcaResultRegs)
#define MOTOR12_IVB_ADC_RESULT_VAR              (&AdcbResultRegs)
#define MOTOR12_VBUS_ADC_RESULT_VAR             (&AdccResultRegs)
#define MOTOR12_IVC_ADC_RESULT_VAR              (&AdcdResultRegs)
/**
 * @}
 */

/******************************************************************************************************
*                                             PWM defines
*******************************************************************************************************/
/** @defgroup ePWM_Module address configuration.
 *  @brief    Define the address of all the ePWM used (2 motors).
 * @{
 */
#define MOTOR1_PWM1_BASE                        EPWM1_BASE
#define MOTOR1_PWM2_BASE                        EPWM2_BASE
#define MOTOR1_PWM3_BASE                        EPWM3_BASE
#define MOTOR2_PWM1_BASE                        EPWM4_BASE
#define MOTOR2_PWM2_BASE                        EPWM5_BASE
#define MOTOR2_PWM3_BASE                        EPWM6_BASE
#define MOTOR1_PWM1_VAR                         (&EPwm1Regs)
#define MOTOR1_PWM2_VAR                         (&EPwm2Regs)
#define MOTOR1_PWM3_VAR                         (&EPwm3Regs)
#define MOTOR2_PWM1_VAR                         (&EPwm4Regs)
#define MOTOR2_PWM2_VAR                         (&EPwm5Regs)
#define MOTOR2_PWM3_VAR                         (&EPwm6Regs)
#define MOTOR1_PWM1_CMD_ADDR                    (MOTOR1_PWM1_BASE + EPWM_O_CMPA + 1U)
#define MOTOR1_PWM2_CMD_ADDR                    (MOTOR1_PWM2_BASE + EPWM_O_CMPA + 1U)
#define MOTOR1_PWM3_CMD_ADDR                    (MOTOR1_PWM3_BASE + EPWM_O_CMPA + 1U)
#define MOTOR2_PWM1_CMD_ADDR                    (MOTOR2_PWM1_BASE + EPWM_O_CMPA + 1U)
#define MOTOR2_PWM2_CMD_ADDR                    (MOTOR2_PWM2_BASE + EPWM_O_CMPA + 1U)
#define MOTOR2_PWM3_CMD_ADDR                    (MOTOR2_PWM3_BASE + EPWM_O_CMPA + 1U)
/**
 * @}
 */

/** @defgroup ePWM_GPIO_pins configuration.
 *  @brief    Configure GPIO pins associated to 12x ePWM (2 motors).
 * @{
 */
#define MOTOR1_PWM1_CHA                         (0)
#define MOTOR1_PWM1_CHA_CFG                     GPIO_0_EPWM1A
#define MOTOR1_PWM1_CHB                         (1)
#define MOTOR1_PWM1_CHB_CFG                     GPIO_1_EPWM1B
#define MOTOR1_PWM2_CHA                         (2)
#define MOTOR1_PWM2_CHA_CFG                     GPIO_2_EPWM2A
#define MOTOR1_PWM2_CHB                         (3)
#define MOTOR1_PWM2_CHB_CFG                     GPIO_3_EPWM2B
#define MOTOR1_PWM3_CHA                         (4)
#define MOTOR1_PWM3_CHA_CFG                     GPIO_4_EPWM3A
#define MOTOR1_PWM3_CHB                         (5)
#define MOTOR1_PWM3_CHB_CFG                     GPIO_5_EPWM3B
#define MOTOR2_PWM1_CHA                         (6)
#define MOTOR2_PWM1_CHA_CFG                     GPIO_6_EPWM4A
#define MOTOR2_PWM1_CHB                         (7)
#define MOTOR2_PWM1_CHB_CFG                     GPIO_7_EPWM4B
#define MOTOR2_PWM2_CHA                         (8)
#define MOTOR2_PWM2_CHA_CFG                     GPIO_8_EPWM5A
#define MOTOR2_PWM2_CHB                         (9)
#define MOTOR2_PWM2_CHB_CFG                     GPIO_9_EPWM5B
#define MOTOR2_PWM3_CHA                         (10)
#define MOTOR2_PWM3_CHA_CFG                     GPIO_10_EPWM6A
#define MOTOR2_PWM3_CHB                         (11)
#define MOTOR2_PWM3_CHB_CFG                     GPIO_11_EPWM6B
/**
 * @}
 */

/** @defgroup ePWM_clock configuration.
 *  @brief    Configure the clock subdivision of SYSCLK associated to 12x ePWM (2 motors).
 * @{
 */
#define PLLSYSCLK_TO_EPWMCLK_DIV                SYSCTL_EPWMCLK_DIV_2
#define EPWMCLK_FREQ                            (DEVICE_SYSCLK_FREQ / (PLLSYSCLK_TO_EPWMCLK_DIV + 1))
#define EPWMCLK_TO_TBCLK_DIV                    EPWM_CLOCK_DIVIDER_1
#define HRPWMCLK_TO_TBCLK_DIV                   EPWM_HSCLOCK_DIVIDER_1
#define TBCLK_FREQ                              (EPWMCLK_FREQ / ((EPWMCLK_TO_TBCLK_DIV + 1) * (HRPWMCLK_TO_TBCLK_DIV + 1)))
/**
 * @}
 */

/** @defgroup ePWM_general configuration.
 *  @brief    Configure the general registers associated to 12x ePWM (2 motors).
 * @{
 */
#define PWM_FREQ                                (40000) //[Hz]
#define PWM_COUNTER_MODE                        EPWM_COUNTER_MODE_UP_DOWN
#define PWM_TIMEBASE_CNT                        (TBCLK_FREQ / PWM_FREQ / ((PWM_COUNTER_MODE == EPWM_COUNTER_MODE_UP_DOWN) ? (2) : (1))) // PWM Period register (max value)
#define PWM_TIMEBASE_HALF_CNT                   (PWM_TIMEBASE_CNT / 2)
#define PWM_INITIAL_PHASE_0                     (0)                     // PWM initial phase offset (MOTOR_1)
#define PWM_INITIAL_PHASE_90                    PWM_TIMEBASE_HALF_CNT   // PWM initial phase offset (MOTOR_2)
#define PWM_INITIAL_CNT_VAL                     (0)
#define PWM_INITIAL_COMP_VAL                    PWM_TIMEBASE_CNT        // PWM initial value for capture/compare register
#define PWM_DEADBAND_RISING_EDGE_DELAY          (0) // PWM rising edge delay. ePWMxA (low to high) & ePWMxB (high to low) - Active High Complementary (AHC)
#define PWM_DEADBAND_FALLING_EDGE_DELAY         (0) // PWM falling edge delay. ePWMxA (high to low) & ePWMxB (low to high) - Active High Complementary (AHC)
//#define PWM_CMD_SCALE                           PWM_TBPRD
#define PWM_SHADOW_MODE_EN                      (1) // PWM Shadow mode definition
#if PWM_SHADOW_MODE_EN
#define PWM_SHADOW_MODE_ET                      EPWM_PERIOD_SHADOW_LOAD
#define PWM_SHADOW_MODE_EVENT_ET                EPWM_SHADOW_LOAD_MODE_COUNTER_SYNC
#define PWM_SHADOW_MODE_EVENT_CC                EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_AQ                EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_DB                EPWM_DB_LOAD_ON_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_DB_RED            EPWM_RED_LOAD_ON_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_DB_FED            EPWM_FED_LOAD_ON_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_GLOBAL            EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD
#else
//#define PWM_SHADOW_MODE_PERIOD                EPWM_PERIOD_DIRECT_LOAD
#define PWM_SHADOW_MODE_ET                      EPWM_PERIOD_DIRECT_LOAD
#endif
/**
 * @}
 */

/******************************************************************************************************
*                                               Encoders Defines
*******************************************************************************************************/
/** @defgroup encoders IOs configuration.
 *  @brief    Configure the IO channels to connect the encoders.
 * @{
 */
#define MOTOR1_ENC_CHA                          (20)
#define MOTOR1_ENC_CHA_CFG                      GPIO_20_EQEP1_A
#define MOTOR1_ENC_CHB                          (21)
#define MOTOR1_ENC_CHB_CFG                      GPIO_21_EQEP1_B
#define MOTOR1_ENC_CHI                          (23)
#define MOTOR1_ENC_CHI_CFG                      GPIO_23_EQEP1_INDEX
#define MOTOR2_ENC_CHA                          (24)
#define MOTOR2_ENC_CHA_CFG                      GPIO_24_EQEP2_A
#define MOTOR2_ENC_CHB                          (25)
#define MOTOR2_ENC_CHB_CFG                      GPIO_25_EQEP2_B
#define MOTOR2_ENC_CHI                          (26)
#define MOTOR2_ENC_CHI_CFG                      GPIO_26_EQEP2_INDEX
/**
 * @}
 */

/** @defgroup encoders general configuration.
 *  @brief    Configure resolution \& speed scaler (low \& high speed).
 * @{
 */
#define MOTOR1_ENC_RESOLUTION                   (5000)
#define MOTOR1_ENC_RESOLUTION_SCALE             (4.0 * MOTOR1_ENC_RESOLUTION)
#define MOTOR1_ENC_SPEED_HIGH_SAMPLING_FREQ     (1000) // [Hz] Velocity computation frequency
#define MOTOR1_ENC_SPEED_HIGH_SAMPLING_TICKS    (DEVICE_SYSCLK_FREQ / MOTOR1_ENC_SPEED_HIGH_SAMPLING_FREQ)
#define MOTOR1_ENC_SPEED_HIGH_SCALE             (2.0 * M_PI * MOTOR1_ENC_SPEED_HIGH_SAMPLING_FREQ) // 2pi / dt
#define MOTOR1_ENC_SPEED_LOW_SCALE              (2.0 * M_PI * DEVICE_SYSCLK_FREQ * (1 << QEP_UNIT_POS_EVENT_DIV) / (1 << (QEP_CAPTURE_CLOCK_DIV >> 4)) / MOTOR1_ENC_RESOLUTION_SCALE)
#define MOTOR1_ENC_SPEED_CUTOFF_FREQ            (200.0)     // cutoff frequency for velocity estimation
#define MOTOR1_ENC_SPEED_LPF_ALPHA              (1.0 / (1.0 + (1.0 / (2.0 * M_PI * MOTOR1_ENC_SPEED_CUTOFF_FREQ / PWM_FREQ))))
#define MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA        (1.0 - MOTOR1_ENC_SPEED_LPF_ALPHA)

#define MOTOR2_ENC_RESOLUTION                   (5000)
#define MOTOR2_ENC_RESOLUTION_SCALE             (4.0 * MOTOR2_ENC_RESOLUTION)
#define MOTOR2_ENC_SPEED_HIGH_SAMPLING_FREQ     (1000) // [Hz] Velocity computation frequency
#define MOTOR2_ENC_SPEED_HIGH_SAMPLING_TICKS    (DEVICE_SYSCLK_FREQ / MOTOR2_ENC_SPEED_HIGH_SAMPLING_FREQ)
#define MOTOR2_ENC_SPEED_HIGH_SCALE             (2.0 * M_PI * MOTOR2_ENC_SPEED_HIGH_SAMPLING_FREQ) // 2pi / dt
#define MOTOR2_ENC_SPEED_LOW_SCALE              (2.0 * M_PI * DEVICE_SYSCLK_FREQ * (1 << QEP_UNIT_POS_EVENT_DIV) / (1 << (QEP_CAPTURE_CLOCK_DIV >> 4)) / MOTOR2_ENC_RESOLUTION_SCALE)
#define MOTOR2_ENC_SPEED_CUTOFF_FREQ            (200.0)     // cutoff frequency for velocity estimation
#define MOTOR2_ENC_SPEED_LPF_ALPHA              (1.0 / (1.0 + (1.0 / (2.0 * M_PI * MOTOR2_ENC_SPEED_CUTOFF_FREQ / PWM_FREQ))))
#define MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA        (1.0 - MOTOR2_ENC_SPEED_LPF_ALPHA)

//#define MOTOR1_ENC_LOW_SPEED_SCALE      (2.0 * M_PI * MOTOR1_ENC_RESOLUTION_SCALE * DEVICE_SYSCLK_FREQ * (1 << QEP_UNIT_POS_EVENT_DIV) / (1 << (QEP_CAPTURE_CLOCK_DIV >> 4)))
//#define MOTOR2_ENC_LOW_SPEED_SCALE      (2.0 * M_PI * MOTOR2_ENC_RESOLUTION_SCALE * DEVICE_SYSCLK_FREQ * (1 << QEP_UNIT_POS_EVENT_DIV) / (1 << (QEP_CAPTURE_CLOCK_DIV >> 4)))
/**
 * @}
 */

/** @defgroup eQEP module general configuration.
 *  @brief    Configure the registers associated to the eQEP modules.
 * @{
 */
#define MOTOR1_QEP_BASE                         EQEP1_BASE
#define MOTOR2_QEP_BASE                         EQEP2_BASE
#define MOTOR1_QEP_VAR                          (&EQep1Regs)
#define MOTOR2_QEP_VAR                          (&EQep2Regs)
#define MOTOR1_QEP_COUNTER_ADDR                 (MOTOR1_QEP_BASE + EQEP_O_QPOSCNT)
#define MOTOR2_QEP_COUNTER_ADDR                 (MOTOR2_QEP_BASE + EQEP_O_QPOSCNT)
#define MOTOR1_QEP_SWAP                         EQEP_CONFIG_NO_SWAP // Set to EQEP_CONFIG_SWAP if increments in the wrong direction
#define MOTOR2_QEP_SWAP                         EQEP_CONFIG_NO_SWAP
#define QEP_EMUL_MODE                           EQEP_EMULATIONMODE_RUNFREE
#define QEP_LATCH_MODE                          (EQEP_LATCH_RISING_INDEX | EQEP_LATCH_UNIT_TIME_OUT)
#define QEP_RESET_POSITION_MODE                 EQEP_POSITION_RESET_MAX_POS
#define QEP_CAPTURE_CLOCK_DIV                   EQEP_CAPTURE_CLK_DIV_128
#define QEP_UNIT_POS_EVENT_DIV                  EQEP_UNIT_POS_EVNT_DIV_4
//#define PERxSYSCLK_TO_EQEPCLK_DIV               (1)
//#define EQEPCLK_FREQ                            (DEVICE_SYSCLK_FREQ / pow(2.0, ((QEP_CAPTURE_CLOCK_DIV >> 4) & 0x07)))
/**
 * @}
 */

/******************************************************************************************************
*                                               Motors Defines
*******************************************************************************************************/
/*** Motor 1 constants ***/
#define MOTOR1_KV                               (300.0)     // Motor constant (rpm/V)
#define MOTOR1_KE                               (0.0017)    //1.0 / (MOTOR1_KV * FM_RPM2RADPS * MOTOR1_POLES_PAIRS) // Motor back EMF constant (V/rad/s)
#define MOTOR1_KI                               (0.1)       // Motor torque constant (N.m/A)
#define MOTOR1_RS                               (0.7)       // Stator resistance (ohm)
#define MOTOR1_LD                               (134e-6)    // Stator d-axis inductance (H)
#define MOTOR1_LQ                               (140e-6)    // Stator q-axis inductance (H)
#define MOTOR1_POLES_PAIRS                      (12)        // Number of poles
#define MOTOR1_CURRENT_CUTOFF_FREQ              (500.0)     // Current loop bandwidth (Hz)
#define MOTOR1_CURRENT_LPF_ALPHA                (1.0 / (1.0 + (1.0 / (2.0 * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ / PWM_FREQ))))
#define MOTOR1_CURRENT_LPF_ONE_M_ALPHA          (1.0 - MOTOR1_CURRENT_LPF_ALPHA)
#define MOTOR1_ID_REGULATOR_KP_COEF             (MOTOR1_LD * 2.0 * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ)
#define MOTOR1_ID_REGULATOR_KI_COEF             (MOTOR1_RS * 2.0 * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ / PWM_FREQ)
#define MOTOR1_IQ_REGULATOR_KP_COEF             (MOTOR1_LQ * 2.0 * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ)
#define MOTOR1_IQ_REGULATOR_KI_COEF             (MOTOR1_RS * 2.0 * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ / PWM_FREQ)
#define MOTOR1_SPEED_REGULATOR_KP_COEF          (1.0f)
#define MOTOR1_SPEED_REGULATOR_KD_COEF          (0.02f)
#define MOTOR1_DTC_MAX                          (0.9f)     // Max duty cycle
#define MOTOR1_DTC_MIN                          (0.1f)     // Min duty cycle
#define MOTOR1_STATOR_RESISTOR_CUTOFF_FREQ      (0.2f)
#define MOTOR1_STATOR_RESISTOR_LPF_ALPHA        (1.0 / (1.0 + (1.0 / (2.0 * M_PI * MOTOR1_STATOR_RESISTOR_CUTOFF_FREQ / PWM_FREQ))))
#define MOTOR1_STATOR_RESISTOR_LPF_ONE_M_ALPHA  (1.0 - MOTOR1_STATOR_RESISTOR_LPF_ALPHA)
//#define MOTOR1_CUR_LOOP_ONE_M_ALPHA         (1.0 / (1.0 + (2.0 * M_PI * MOTOR1_CUR_LOOP_BANDWIDTH / PWM_FREQ)))
//#define MOTOR1_CUR_LOOP_ALPHA                   (1.0 - (MOTOR1_CUR_LOOP_ONE_M_ALPHA))

/*** Motor 2 constants ***/
#define MOTOR2_KV                               (300.0)     // Motor constant (rpm/V)
#define MOTOR2_KE                               (0.0017)    // Motor back EMF constant (V/rad/s)
#define MOTOR2_KI                               (0.1)       // Motor torque constant (N.m/A)
#define MOTOR2_RS                               (0.7)       // Stator resistance (ohm)
#define MOTOR2_LD                               (134e-6)    // Stator d-axis inductance (H)
#define MOTOR2_LQ                               (140e-6)    // Stator q-axis inductance (H)
#define MOTOR2_POLES_PAIRS                      (12)        // Number of poles
#define MOTOR2_CURRENT_CUTOFF_FREQ              (500.0)     // Current loop bandwidth (Hz)
#define MOTOR2_CURRENT_LPF_ALPHA                (1.0 / (1.0 + (1.0 / (2.0 * M_PI * MOTOR2_CURRENT_CUTOFF_FREQ / PWM_FREQ))))
#define MOTOR2_CURRENT_LPF_ONE_M_ALPHA          (1.0 - MOTOR2_CURRENT_LPF_ALPHA)
#define MOTOR2_ID_REGULATOR_KP_COEF             (MOTOR2_LD * 2.0 * M_PI * MOTOR2_CURRENT_CUTOFF_FREQ)
#define MOTOR2_ID_REGULATOR_KI_COEF             (MOTOR2_RS * 2.0 * M_PI * MOTOR2_CURRENT_CUTOFF_FREQ / PWM_FREQ)
#define MOTOR2_IQ_REGULATOR_KP_COEF             (MOTOR2_LQ * 2.0 * M_PI * MOTOR2_CURRENT_CUTOFF_FREQ)
#define MOTOR2_IQ_REGULATOR_KI_COEF             (MOTOR2_RS * 2.0 * M_PI * MOTOR2_CURRENT_CUTOFF_FREQ / PWM_FREQ)
#define MOTOR2_SPEED_REGULATOR_KP_COEF          (1.0f)
#define MOTOR2_SPEED_REGULATOR_KD_COEF          (0.02f)
#define MOTOR2_DTC_MAX                          (0.9f)     // Max duty cycle
#define MOTOR2_DTC_MIN                          (0.1f)     // Min duty cycle
#define MOTOR2_STATOR_RESISTOR_CUTOFF_FREQ      (0.2f)
#define MOTOR2_STATOR_RESISTOR_LPF_ALPHA        (1.0 / (1.0 + (1.0 / (2.0 * M_PI * MOTOR2_STATOR_RESISTOR_CUTOFF_FREQ / PWM_FREQ))))
#define MOTOR2_STATOR_RESISTOR_LPF_ONE_M_ALPHA  (1.0 - MOTOR2_STATOR_RESISTOR_LPF_ALPHA)
//#define MOTOR2_CUR_LOOP_ONE_M_ALPHA         (1.0 / (1.0 + (2.0 * M_PI * MOTOR2_CUR_LOOP_BANDWIDTH / PWM_FREQ)))
//#define MOTOR2_CUR_LOOP_ALPHA                   (1.0 - (MOTOR2_CUR_LOOP_ONE_M_ALPHA))

#define MOTOR12_OVERMODULATION                  (1.15)
#define MOTOR12_VBUS_CUTOFF_FREQ                (500.0)     // vbus voltage loop bandwidth (Hz)
#define MOTOR12_VBUS_LPF_ALPHA                  (1.0 / (1.0 + (1.0 / (2.0 * M_PI * MOTOR12_VBUS_CUTOFF_FREQ / PWM_FREQ))))
#define MOTOR12_VBUS_LPF_ONE_M_ALPHA            (1.0 - MOTOR12_VBUS_LPF_ALPHA)
#define MOTOR12_IMAX                            (20.0)

typedef struct __LowpassFilter_Structure__
{
    float32_t a;
    float32_t one_minus_a;
} LpfStruct;

/**
 * @enum    ArrayID
 * @brief   Easier array management for history.
 */
typedef enum
{
    NEW = 0,    /*!< Current data */
    OLD = 1,    /*!< Previous data */
} ArrayID;

/**
 * @enum    MotorID
 * @brief   Easier motor identification
 */
typedef enum
{
    MOTOR_1 = 0,    /*!< Select Motor_1 */
    MOTOR_2 = 1,    /*!< Select Motor_2 */
    NO_MOT  = -1    /*!< No motor selected */
} ID_ENUM;

/**
 * @enum    itID
 * @brief   Easier interrupts identification
 */
typedef enum
{
    ADC_IT_MOTOR_1  = 0,    /*!< Select the ADC interrupt source of Motor_1 */
    ADC_IT_MOTOR_2  = 1,    /*!< Select the ADC interrupt source of Motor_2 */
//    PWM_IT_MOTOR_1  = 2,    /*!< Select the PWM interrupt source of Motor_1 */
//    PWM_IT_MOTOR_2  = 3,    /*!< Select the PWM interrupt source of Motor_2 */
    DRV_IT_MOTOR_1  = 2,    /*!< Select the GPIO (DRV fault) interrupt source of Motor_1 */
    DRV_IT_MOTOR_2  = 3,    /*!< Select the GPIO (DRV fault) interrupt source of Motor_2 */
    CLA_IT_MOTOR_1  = 4,    /*!< Select the PWM interrupt source of Motor_1 */
} IT_ENUM;

#endif
