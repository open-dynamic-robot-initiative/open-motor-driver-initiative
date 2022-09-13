/*
 * File name: omodri_user_defines.h
 * Description: Header file containing user definitions
 */

#ifndef __UOMODRI_USER_DEFINES_H__
#define __UOMODRI_USER_DEFINES_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "f2838x_device.h"
#include "driverlib.h"
#include "device.h"

#include "drv8353.h"

/***********************************************************************
 * GLOBAL MACROS
 ***********************************************************************/
/** @defgroup DEV_defines
 *  @brief    Special configurations definitions.
 * @{
 */
#define UOMODRI_USE_ENCODER_GET_REG_FUNCTION    0
#define CLA_CORE_ENABLE                         0
#define CM_CORE_ENABLE                          0
#define UOMODRI_VBUS_FLT(flt_coef, y, x)        LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_VEXT_FLT(flt_coef, y, x)        LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_IDQ_FLT(flt_coef, y, x)         LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_RES_ESTIM_FLT(flt_coef, y, x)   LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_SPEED_FLT(flt_coef, y, x)       LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_HAL_ADC_CALIB(flt_coef, y, x)   LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_FEED_FORWARD_ENABLE             1
#define OMODRI_PAL_SETUP                        0   // Set this to 1 to use the PAL pin affectation
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
#define FM_SQRT3                                1.7320508075688772935274463415059f  /* sqrt(3) */
#endif

#ifndef FM_1DIVSQRT3
#define FM_1DIVSQRT3                            0.57735026918962576450914878050196f /* 1/sqrt(3) */
#endif

#ifndef FM_SQRT3DIV2
#define FM_SQRT3DIV2                            0.86602540378443864676372317075294f /* sqrt(3)/2 */
#endif

#ifndef FM_1DIV3
#define FM_1DIV3                                0.33333333333333333333333333333333f /* 1/3 */
#endif

#ifndef FM_ROUND2RAD
#define FM_ROUND2RAD                            (2.0f * M_PI)
#endif

#ifndef FM_RPM2RADPS
#define FM_RPM2RADPS                            (FM_ROUND2RAD / 60.0f)
#endif

#ifndef FM_RADPS2RPM
#define FM_RADPS2RPM                            (1.0f / FM_RPM2RADPS)
#endif

#ifndef FM_RAD2ROUND
#define FM_RAD2ROUND                            (1.0f / FM_ROUND2RAD)
#endif

#ifndef FM_KRPM2RADPS
#define FM_KRPM2RADPS                           (1000.0f * FM_RPM2RADPS)
#endif

#ifndef FM_RADPS2KRPM
#define FM_RADPS2KRPM                           (1.0f / FM_KRPM2RADPS)
#endif

#define FM_ABS(x)                               ((x) >= 0.0f ? (x) : -(x))

//#define HWSREGH(x)                              (*((volatile int16_t *)((uintptr_t)(x))))
/* Initialize control loop low-pass filter
   alpha = 1 / (1 + (1 / (2 * PI * CUTTOFF_FREQ / PWM_FREQ)))
   Simplifying :
   alpha = 1 - (1 / (1 + (2 * PI * CUTTOFF_FREQ / PWM_FREQ)))
   1 - alpha = 1 / (1 + (2 * PI * CUTTOFF_FREQ / PWM_FREQ)) */
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
//#define SEP_32_TO_16(x, y)
// 32 bits macros
#define LSB_32(x)                               ((x) & 0xFFFF)
#define MSB_32(x)                               (((x) >> 16) & 0xFFFF)

/***********************************************************************
 * DEBUG DEFINES
 ***********************************************************************/
/** @defgroup Debug pins configuration.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#if (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE) && (defined USB)
#define USB_M                                   42
#define USB_M_CFG                               GPIO_42_GPIO42
#define USB_P                                   43
#define USB_P_CFG                               GPIO_43_GPIO43
#define USB_VBUS                                46
#define USB_VBUS_CFG                            GPIO_46_GPIO46
#elif (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE)
#define UART_DBG_TX                             (42)
#define UART_DBG_TX_CFG                         GPIO_42_UARTA_TX
#define UART_DBG_RX                             (43)
#define UART_DBG_RX_CFG                         GPIO_43_UARTA_RX
#endif
#define DBG_PIN4                                (149)
#define DBG_PIN4_CFG                            GPIO_149_GPIO149
#define DBG_PIN4_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPESET)   = GPIO_GPESET_GPIO149)
#define DBG_PIN4_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPECLEAR) = GPIO_GPECLEAR_GPIO149)
#define DBG_PIN4_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPETOGGLE)= GPIO_GPETOGGLE_GPIO149)
#define DBG_PIN3                                (81)
#define DBG_PIN3_CFG                            GPIO_81_GPIO81
#define DBG_PIN3_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO81)
#define DBG_PIN3_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO81)
#define DBG_PIN3_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO81)
#define DBG_PIN2                                (82)
#define DBG_PIN2_CFG                            GPIO_82_GPIO82
#define DBG_PIN2_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO82)
#define DBG_PIN2_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO82)
#define DBG_PIN2_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO82)
#define DBG_PIN1                                (77)
#define DBG_PIN1_CFG                            GPIO_77_GPIO77
#define DBG_PIN1_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO77)
#define DBG_PIN1_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO77)
#define DBG_PIN1_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO77)
#define DBG_PIN0                                (78)
#define DBG_PIN0_CFG                            GPIO_78_GPIO78
#define DBG_PIN0_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO78)
#define DBG_PIN0_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO78)
#define DBG_PIN0_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO78)
/**
 * @}
 */

/***********************************************************************
 * UART DEFINES
 ***********************************************************************/
//#define DBG_SCI_TX                              (70)
//#define DBG_SCI_TX_CFG                          GPIO_70_SCIB_TX
//
//#define DBG_SCI_BASE                            SCIB_BASE
//#define DBG_SCI_TX_BASE_REG_ADDR                (DBG_SCI_BASE + SCI_O_TXBUF)

/***********************************************************************
 * CPU TIMER DEFINES
 ***********************************************************************/
/** @defgroup CPU Timer configuration.
 *  @brief    Configure CPU timer for system tick generation.
 * @{
 */
#define CPU_TIMER_0_BASE                        CPUTIMER0_BASE
#define CPU_TIMER_0_PERIOD                      (1000U)                             // Period counter - 1ms
#define CPU_TIMER_0_PRESCALER                   ((DEVICE_SYSCLK_FREQ / 1e6) - 1)    // Prescaler increment - 1us
#define CPU_TIMER_0_INT                         INT_TIMER0
#define CPU_TIMER_0_INT_ACK_GROUP               INTERRUPT_ACK_GROUP1
/**
 * @}
 */

/***********************************************************************
 * IPC DEFINES
 ***********************************************************************/
/** @defgroup Debug pins configuration.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#define IPC_SYNC_FLAG                           IPC_FLAG31
#define IPC_CPU1_TO_CM_FLAG                     IPC_FLAG0
#define IPC_CM_TO_CPU1_FLAG                     IPC_FLAG0

/***********************************************************************
 * SPI DEFINES
 ***********************************************************************/
/** @defgroup SPI DRV configuration.
 *  @brief    Configure SPI module associated with DRV communication.
 * @{
 */
#define DRV_SPI_BASE                            SPIA_BASE
#define DRV_SPI_TX_BASE_REG_ADDR                (DRV_SPI_BASE + SPI_O_TXBUF)
#define DRV_SPI_RX_BASE_REG_ADDR                (DRV_SPI_BASE + SPI_O_RXBUF)
/**
 * @}
 */

/** @defgroup Slave SPI communication configuration.
 *  @brief    Configure SPI module associated with external communication.
 * @{
 */
#define COM_SPI_BASE                            SPIB_BASE
#define COM_SPI_DAT_REG_ADDR                    (COM_SPI_BASE + SPI_O_DAT)
#define COM_SPI_DAT(x)                          (HWREGH(COM_SPI_DAT_REG_ADDR) = (uint16_t)(x))
#define COM_SPI_TX_BASE_REG_ADDR                (COM_SPI_BASE + SPI_O_TXBUF)
#define COM_SPI_RX_BASE_REG_ADDR                (COM_SPI_BASE + SPI_O_RXBUF)
/**
 * @}
 */

/** @defgroup Master SPI communication configuration.
 *  @brief    Configure master SPI module (simulating master board).
 * @{
 */
#define ENC_SPI_BASE                            SPIC_BASE
#define ENC_SPI_TX_BASE_REG_ADDR                (ENC_SPI_BASE + SPI_O_TXBUF)
#define ENC_SPI_RX_BASE_REG_ADDR                (ENC_SPI_BASE + SPI_O_RXBUF)
/**
 * @}
 */

/** @defgroup SPI RGB LED control.
 *  @brief    Configure master SPI module associated with RGB LED.
 * @{
 */
#define LED_SPI_BASE                            SPID_BASE
#define LED_SPI_TX_BASE_REG_ADDR                (LED_SPI_BASE + SPI_O_TXBUF)
#define LED_SPI_RX_BASE_REG_ADDR                (LED_SPI_BASE + SPI_O_RXBUF)
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
#define MOTOR1_DRV_SPI_CS_SET                   (HWREG(GPIODATA_BASE + GPIO_O_GPASET)   = GPIO_GPASET_GPIO19)
#define MOTOR1_DRV_SPI_CS_CLEAR                 (HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR) = GPIO_GPACLEAR_GPIO19)
#define MOTOR1_DRV_SPI_CS_TOGGLE                (HWREG(GPIODATA_BASE + GPIO_O_GPATOGGLE)= GPIO_GPATOGGLE_GPIO19)
#define MOTOR2_DRV_SPI_CS                       (35)
#define MOTOR2_DRV_SPI_CS_CFG                   GPIO_35_GPIO35
#define MOTOR2_DRV_SPI_CS_SET                   (HWREG(GPIODATA_BASE + GPIO_O_GPBSET)   = GPIO_GPBSET_GPIO35)
#define MOTOR2_DRV_SPI_CS_CLEAR                 (HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIO_GPBCLEAR_GPIO35)
#define MOTOR2_DRV_SPI_CS_TOGGLE                (HWREG(GPIODATA_BASE + GPIO_O_GPBTOGGLE)= GPIO_GPBTOGGLE_GPIO35)
/**
 * @}
 */

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
#define COM_SPI_CS_READ                         (HWREG(GPIODATA_BASE + GPIO_O_GPBDAT) & GPIO_GPBDAT_GPIO59)
/**
 * @}
 */

/** @defgroup SPI_ENC_GPIO_pins - ENC communication associated with as5047u configuration.
 *  @brief    Configure GPIO pins associated to master board communication.
 * @{
 */
#define ENC_SPI_SIMO                            (69)
#define ENC_SPI_SIMO_CFG                        GPIO_69_SPIC_SIMO
#define ENC_SPI_SOMI                            (70)
#define ENC_SPI_SOMI_CFG                        GPIO_70_SPIC_SOMI
#define ENC_SPI_CLK                             (71)
#define ENC_SPI_CLK_CFG                         GPIO_71_SPIC_CLK
#define ENC_SPI_CS                              (72)
#define ENC_SPI_CS_CFG                          GPIO_72_GPIO72//SPIC_STEN
#define ENC_SPI_CS_SET                          (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO72)
#define ENC_SPI_CS_CLEAR                        (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO72)
#define ENC_SPI_CS_TOGGLE                       (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO72)
/**
 * @}
 */
//#endif

/** @defgroup SPI_LED_GPIO_pins - RGB LED configuration.
 *  @brief    Configure GPIO pins associated with RGB LED.
 * @{
 */
#define LED_SPI_SIMO                            (91)
#define LED_SPI_SIMO_CFG                        GPIO_91_SPID_SIMO
/**
 * @}
 */

/***********************************************************************
 * DMA DEFINES
 ***********************************************************************/
/** @defgroup DMA configuration.
 *  @brief    Configure DMA channels associated to SPI communication.
 * @{
 */
#define DMA_NO_WRAP                             (0x10000)

#define DMA_CMD_2_FOC_BASE_ADDR                 DMA_CH1_BASE
#define DMA_CMD_2_FOC_INT                       INT_DMA_CH1
#define DMA_CMD_2_FOC_TRIGGER_SRC               DMA_TRIGGER_SOFTWARE

#define DMA_CMD_2_FOC_16BIT_FULL_LENGTH         (20)
#define DMA_CMD_2_FOC_16BIT_BURST_EVT           DMA_CMD_2_FOC_16BIT_FULL_LENGTH
#define DMA_CMD_2_FOC_16BIT_USE_WRAP            (1)
#define DMA_CMD_2_FOC_16BIT_WRAP                (1)
#define DMA_CMD_2_FOC_16BIT_TRANSFER_EVT        (DMA_CMD_2_FOC_16BIT_FULL_LENGTH * DMA_CMD_2_FOC_16BIT_WRAP / DMA_CMD_2_FOC_16BIT_BURST_EVT)
#define DMA_CMD_2_FOC_16BIT_INC_STEP            (1)
#define DMA_CMD_2_FOC_16BIT_CONFIG              (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT)
#define DMA_CMD_2_FOC_16BIT_WRAP                (1)

//#define DMA_ENC_SPI_TX_BASE_ADDR                DMA_CH3_BASE
//#define DMA_ENC_SPI_TX_INT                      INT_DMA_CH3
//#define DMA_ENC_SPI_TX_TRIGGER_SRC              DMA_TRIGGER_SOFTWARE

#define DMA_LED_SPI_TX_BASE_ADDR                DMA_CH4_BASE
#define DMA_LED_SPI_TX_INT                      INT_DMA_CH4
#define DMA_LED_SPI_TX_TRIGGER_SRC              DMA_TRIGGER_SPIDTX

#define DMA_COM_SPI_TX_BASE_ADDR                DMA_CH5_BASE
#define DMA_COM_SPI_TX_INT                      INT_DMA_CH5
#define DMA_COM_SPI_RX_BASE_ADDR                DMA_CH6_BASE
#define DMA_COM_SPI_RX_INT                      INT_DMA_CH6
#define DMA_COM_SPI_TX_TRIGGER_SRC              DMA_TRIGGER_SPIBTX
#define DMA_COM_SPI_RX_TRIGGER_SRC              DMA_TRIGGER_SPIBRX
#define DMA_COM_SPI_RX_INT_ACK_GROUP            INTERRUPT_ACK_GROUP7

#define DMA_LED_SPI_TX_8BIT_LENGTH              (12)
#define DMA_LED_SPI_TX_16BIT_FULL_LENGTH        (DMA_LED_SPI_TX_8BIT_LENGTH / 2)
#define DMA_LED_SPI_TX_16BIT_BURST_EVT          (3)
#define DMA_LED_SPI_TX_16BIT_TRANSFER_EVT       (DMA_LED_SPI_TX_16BIT_FULL_LENGTH / DMA_LED_SPI_TX_16BIT_BURST_EVT)
#define DMA_LED_SPI_TX_16BIT_CONFIG             (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT)

#define DMA_COM_SPI_TX_RX_8BIT_LENGTH           (34)
#define DMA_COM_SPI_TX_RX_16BIT_FULL_LENGTH     (DMA_COM_SPI_TX_RX_8BIT_LENGTH / 2)
#define DMA_COM_SPI_TX_RX_16BIT_CRC_LENGTH      (2)
#define DMA_COM_SPI_TX_RX_16BIT_PAYLOAD_LENGTH  (DMA_COM_SPI_TX_RX_16BIT_FULL_LENGTH - DMA_COM_SPI_TX_RX_16BIT_CRC_LENGTH)
#define DMA_COM_SPI_TX_16BIT_BURST_EVT          (8)
#define DMA_COM_SPI_TX_16BIT_TRANSFER_EVT       ((DMA_COM_SPI_TX_RX_16BIT_FULL_LENGTH - 1) / DMA_COM_SPI_TX_16BIT_BURST_EVT)
#define DMA_COM_SPI_RX_16BIT_BURST_EVT          (1)
#define DMA_COM_SPI_RX_16BIT_TRANSFER_EVT       (DMA_COM_SPI_TX_RX_16BIT_FULL_LENGTH / DMA_COM_SPI_RX_16BIT_BURST_EVT)
#define DMA_COM_SPI_TX_16BIT_CONFIG             (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT)
#define DMA_COM_SPI_RX_16BIT_CONFIG             (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT)
#if ((DMA_COM_SPI_TX_16BIT_CONFIG & DMA_MODE_DATASIZE) == DMA_CFG_SIZE_16BIT)
#define DMA_COM_SPI_TX_16BIT_INC_STEP           (1)
#else
#define DMA_COM_SPI_TX_16BIT_INC_STEP           (2)
#endif
#if ((DMA_COM_SPI_RX_16BIT_CONFIG & DMA_MODE_DATASIZE) == DMA_CFG_SIZE_16BIT)
#define DMA_COM_SPI_RX_16BIT_INC_STEP           (1)
#else
#define DMA_COM_SPI_RX_16BIT_INC_STEP           (2)
#endif
/**
 * @}
 */

/***********************************************************************
 * DRV DEFINES
 ***********************************************************************/
/** @defgroup DRV_GPIO_pins - DRV GPIO pins configuration.
 *  @brief    Configure the pins necessaries for DRV selection \& fault status.
 * @{
 */
// DRV1 SPI defines
#define MOTOR1_DRV_GPIO_EN                      (89)
#define MOTOR1_DRV_GPIO_EN_CFG                  GPIO_89_GPIO89
#define MOTOR1_DRV_GPIO_EN_SET                  (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO89)
#define MOTOR1_DRV_GPIO_EN_CLEAR                (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO89)
#define MOTOR1_DRV_GPIO_EN_TOGGLE               (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO89)
#define MOTOR1_DRV_GPIO_NFAULT                  (90)
#define MOTOR1_DRV_GPIO_NFAULT_CFG              GPIO_90_GPIO90
#define MOTOR1_DRV_GPIO_NFAULT_SET              (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO90)
#define MOTOR1_DRV_GPIO_NFAULT_CLEAR            (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO90)
#define MOTOR1_DRV_GPIO_NFAULT_TOGGLE           (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO90)
// DRV2 SPI defines
#define MOTOR2_DRV_GPIO_EN                      (37)
#define MOTOR2_DRV_GPIO_EN_CFG                  GPIO_37_GPIO37
#define MOTOR2_DRV_GPIO_EN_SET                  (HWREG(GPIODATA_BASE + GPIO_O_GPBSET)   = GPIO_GPBSET_GPIO37)
#define MOTOR2_DRV_GPIO_EN_CLEAR                (HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIO_GPBCLEAR_GPIO37)
#define MOTOR2_DRV_GPIO_EN_TOGGLE               (HWREG(GPIODATA_BASE + GPIO_O_GPBTOGGLE)= GPIO_GPBTOGGLE_GPIO37)
#define MOTOR2_DRV_GPIO_NFAULT                  (36)
#define MOTOR2_DRV_GPIO_NFAULT_CFG              GPIO_36_GPIO36
#define MOTOR2_DRV_GPIO_NFAULT_SET              (HWREG(GPIODATA_BASE + GPIO_O_GPBSET)   = GPIO_GPBSET_GPIO36)
#define MOTOR2_DRV_GPIO_NFAULT_CLEAR            (HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIO_GPBCLEAR_GPIO36)
#define MOTOR2_DRV_GPIO_NFAULT_TOGGLE           (HWREG(GPIODATA_BASE + GPIO_O_GPBTOGGLE)= GPIO_GPBTOGGLE_GPIO36)
/**
 * @}
 */

/***********************************************************************
 * ADC DEFINES
 ***********************************************************************/
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
#define ADC_RESOLUTION_MAX                      ((ADC_RESOLUTION_BIT == ADC_RESOLUTION_16BIT) ? (65536) : (4096))
#define ADC_SIGNAL_MODE                         ADC_MODE_SINGLE_ENDED
#define ADC_PULSE_END_MODE                      ADC_PULSE_END_OF_CONV
#define ADC_SOC_PRIORITY                        ADC_PRI_THRU_SOC3_HIPRI  //ADC_PRI_ALL_ROUND_ROBIN
#define ADC_INT_SOC_TRIGGER                     ADC_INT_SOC_TRIGGER_NONE

#define ADC_VREF                                (3.0f)   // [V]
#define DRV_CSA_GAIN_REG                        DRV_CSA_GAIN_10_V_V  // [V/V] Possible values: 5, 10, 20, 40
#define DRV_CSA_GAIN_VAL                        (5 << DRV_CSA_GAIN_REG) // base gain

#define R_SHUNT                                 (0.005f) // [Ohm]
#define BRIDGE_DIVIDER                          ((5.1f + 100.0f) / 5.1f)
#define ADC_EXTERN_VOLTAGE_SCALE                (ADC_VREF / ADC_RESOLUTION_MAX)
#define ADC_VBUS_VOLTAGE_SCALE                  (ADC_VREF * BRIDGE_DIVIDER / ADC_RESOLUTION_MAX)
#define ADC_MOTOR_VOLTAGE_SCALE                 (ADC_VREF * BRIDGE_DIVIDER / ADC_RESOLUTION_MAX)
#define ADC_MOTOR_CURRENT_SCALE                 (ADC_VREF * (-1.0f) / DRV_CSA_GAIN_VAL / R_SHUNT / ADC_RESOLUTION_MAX)
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

#define VBUS_ADC_SOC_TRIGGER_EVT                ADC_TRIGGER_EPWM1_SOCB
#define EXTERN_ADC_SOC_TRIGGER_EVT1             ADC_TRIGGER_EPWM1_SOCA
#define EXTERN_ADC_SOC_TRIGGER_EVT2             ADC_TRIGGER_EPWM4_SOCB
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
#define VBUS_CH                                 ADC_CH_ADCIN2

#define MOTOR1_IA_CH                            ADC_CH_ADCIN0
#define MOTOR1_IB_CH                            ADC_CH_ADCIN0
#define MOTOR1_IC_CH                            ADC_CH_ADCIN0

#define MOTOR2_VA_CH                            ADC_CH_ADCIN3
#define MOTOR2_VB_CH                            ADC_CH_ADCIN3
#define MOTOR2_VC_CH                            ADC_CH_ADCIN3

#define MOTOR2_IA_CH                            ADC_CH_ADCIN1
#define MOTOR2_IB_CH                            ADC_CH_ADCIN1
#define MOTOR2_IC_CH                            ADC_CH_ADCIN1

#define EXTERN_V1_CH                            ADC_CH_ADCIN14
#define EXTERN_V2_CH                            ADC_CH_ADCIN15
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
#define VBUS_SOC_NUM                            ADC_SOC_NUMBER0

#define MOTOR1_IA_SOC_NUM                       ADC_SOC_NUMBER2
#define MOTOR1_IB_SOC_NUM                       ADC_SOC_NUMBER2
#define MOTOR1_IC_SOC_NUM                       ADC_SOC_NUMBER2
#define EXTERN_V1_SOC_NUM                       ADC_SOC_NUMBER2

#define MOTOR2_VA_SOC_NUM                       ADC_SOC_NUMBER1
#define MOTOR2_VB_SOC_NUM                       ADC_SOC_NUMBER1
#define MOTOR2_VC_SOC_NUM                       ADC_SOC_NUMBER1

#define MOTOR2_IA_SOC_NUM                       ADC_SOC_NUMBER3
#define MOTOR2_IB_SOC_NUM                       ADC_SOC_NUMBER3
#define MOTOR2_IC_SOC_NUM                       ADC_SOC_NUMBER3
#define EXTERN_V2_SOC_NUM                       ADC_SOC_NUMBER3
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
#define VBUS_PPB_NUM                            ADC_PPB_NUMBER1

#define MOTOR1_IA_PPB_NUM                       ADC_PPB_NUMBER3
#define MOTOR1_IB_PPB_NUM                       ADC_PPB_NUMBER3
#define MOTOR1_IC_PPB_NUM                       ADC_PPB_NUMBER3
#define EXTERN_V1_PPB_NUM                       ADC_PPB_NUMBER3

#define MOTOR2_VA_PPB_NUM                       ADC_PPB_NUMBER2
#define MOTOR2_VB_PPB_NUM                       ADC_PPB_NUMBER2
#define MOTOR2_VC_PPB_NUM                       ADC_PPB_NUMBER2

#define MOTOR2_IA_PPB_NUM                       ADC_PPB_NUMBER4
#define MOTOR2_IB_PPB_NUM                       ADC_PPB_NUMBER4
#define MOTOR2_IC_PPB_NUM                       ADC_PPB_NUMBER4
#define EXTERN_V2_PPB_NUM                       ADC_PPB_NUMBER4
/**
 * @}
 */

/** @defgroup ADC interrupts source \& acknowledge group
 *  @brief    Define the interrupt source \& the PIE Interrupt Number associated
 * @{
 */
#define MOTOR1_IA_INT_CH                        INT_ADCA3
#define MOTOR2_IA_INT_CH                        INT_ADCA4

#define MOTOR1_IA_INT_NUM                       ADC_INT_NUMBER3
#define MOTOR2_IA_INT_NUM                       ADC_INT_NUMBER4

#define MOTOR1_IA_INT_ACK_GROUP                 INTERRUPT_ACK_GROUP10
#define MOTOR2_IA_INT_ACK_GROUP                 INTERRUPT_ACK_GROUP10
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

#define VBUS_ADC_ADDR                           ADCC_BASE
#define EXTERN_V1_ADC_ADDR                      ADCC_BASE
#define EXTERN_V2_ADC_ADDR                      ADCC_BASE

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

#define VBUS_ADC_RESULT_ADDR                    (ADCCRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * VBUS_SOC_NUM))
#define EXTERN_V1_ADC_RESULT_ADDR               (ADCCRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * EXTERN_V1_SOC_NUM))
#define EXTERN_V2_ADC_RESULT_ADDR               (ADCCRESULT_BASE + ADC_RESULTx_OFFSET_BASE + (1U * EXTERN_V2_SOC_NUM))

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

#define VBUS_ADC_PPB_RESULT_ADDR                (ADCCRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * VBUS_PPB_NUM))
#define EXTERN_V1_ADC_PPB_RESULT_ADDR           (ADCCRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * EXTERN_V1_PPB_NUM))
#define EXTERN_V2_ADC_PPB_RESULT_ADDR           (ADCCRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * EXTERN_V2_PPB_NUM))

#define MOTOR1_VC_ADC_PPB_RESULT_ADDR           (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_VC_PPB_NUM))
#define MOTOR1_IC_ADC_PPB_RESULT_ADDR           (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR1_IC_PPB_NUM))
#define MOTOR2_VC_ADC_PPB_RESULT_ADDR           (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_VC_PPB_NUM))
#define MOTOR2_IC_ADC_PPB_RESULT_ADDR           (ADCDRESULT_BASE + ADC_PPBxRESULT_OFFSET_BASE + (2U * MOTOR2_IC_PPB_NUM))
/**
 * @}
 */

/***********************************************************************
 * PWM DEFINES
 ***********************************************************************/
/** @defgroup ePWM_Module address configuration.
 *  @brief    Define the address of all the ePWM used (2 motors).
 * @{
 */
#define MOTOR1_PWMx_CMD_ADDR_OFFSET             1U // bits 31-16 of CMPx registers
#define MOTOR2_PWMx_CMD_ADDR_OFFSET             1U // bits 31-16 of CMPx registers
#define MOTOR1_PWM1_BASE                        EPWM1_BASE
#define MOTOR1_PWM2_BASE                        EPWM2_BASE
#define MOTOR1_PWM3_BASE                        EPWM3_BASE
#define MOTOR2_PWM1_BASE                        EPWM4_BASE
#define MOTOR2_PWM2_BASE                        EPWM5_BASE
#define MOTOR2_PWM3_BASE                        EPWM6_BASE
#define MOTOR1_PWM1_CMD_ADDR                    (MOTOR1_PWM1_BASE + EPWM_O_CMPA + MOTOR1_PWMx_CMD_ADDR_OFFSET)
#define MOTOR1_PWM2_CMD_ADDR                    (MOTOR1_PWM2_BASE + EPWM_O_CMPA + MOTOR1_PWMx_CMD_ADDR_OFFSET)
#define MOTOR1_PWM3_CMD_ADDR                    (MOTOR1_PWM3_BASE + EPWM_O_CMPA + MOTOR1_PWMx_CMD_ADDR_OFFSET)
#define MOTOR2_PWM1_CMD_ADDR                    (MOTOR2_PWM1_BASE + EPWM_O_CMPA + MOTOR2_PWMx_CMD_ADDR_OFFSET)
#define MOTOR2_PWM2_CMD_ADDR                    (MOTOR2_PWM2_BASE + EPWM_O_CMPA + MOTOR2_PWMx_CMD_ADDR_OFFSET)
#define MOTOR2_PWM3_CMD_ADDR                    (MOTOR2_PWM3_BASE + EPWM_O_CMPA + MOTOR2_PWMx_CMD_ADDR_OFFSET)
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
#define PWM_PERIOD                              (1.0f / (float32_t)PWM_FREQ)
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

/***********************************************************************
 * ENCODER DEFINES
 ***********************************************************************/
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
#define MOTOR1_ENC_RESOLUTION                   (5000.0f)
#define MOTOR1_ENC_QUADRATURE_SCALE             (4.0f * MOTOR1_ENC_RESOLUTION)
#define MOTOR1_ENC_RESOLUTION_SCALE             (1.0f / MOTOR1_ENC_QUADRATURE_SCALE)
//#define MOTOR1_ENC_RESOLUTION_SCALE_PU          (4.0f * MOTOR1_ENC_RESOLUTION)
//#define MOTOR1_ENC_RESOLUTION_SCALE_RAD         (2.0f * M_PI / MOTOR1_ENC_RESOLUTION_SCALE_PU) // [rad]
#define MOTOR1_ENC_SPEED_HIGH_SAMPLING_FREQ     (1000) // [Hz] Velocity computation frequency
#define MOTOR1_ENC_SPEED_HIGH_SAMPLING_TICKS    (DEVICE_SYSCLK_FREQ / MOTOR1_ENC_SPEED_HIGH_SAMPLING_FREQ)
#define MOTOR1_ENC_SPEED_HIGH_SCALE             (MOTOR1_ENC_SPEED_HIGH_SAMPLING_FREQ) // 2pi / dt
#define MOTOR1_ENC_SPEED_LOW_SCALE              (2.0f * M_PI * DEVICE_SYSCLK_FREQ * (1 << QEP_UNIT_POS_EVENT_DIV) / (1 << (QEP_CAPTURE_CLOCK_DIV >> 4)) / MOTOR1_ENC_RESOLUTION_SCALE)
#define MOTOR1_ENC_SPEED_CUTOFF_FREQ            (200.0f)     // cutoff frequency for velocity estimation
#define MOTOR1_ENC_SPEED_TIME_CONST             (2.0f * M_PI * MOTOR1_ENC_SPEED_CUTOFF_FREQ * PWM_PERIOD)
#define MOTOR1_ENC_SPEED_LPF_ALPHA              (MOTOR1_ENC_SPEED_TIME_CONST / (1.0f + MOTOR1_ENC_SPEED_TIME_CONST))
#define MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA        (1.0f - MOTOR1_ENC_SPEED_LPF_ALPHA)

#define MOTOR2_ENC_RESOLUTION                   (1024.0f)
#define MOTOR2_ENC_QUADRATURE_SCALE             (4.0f * MOTOR2_ENC_RESOLUTION)
#define MOTOR2_ENC_RESOLUTION_SCALE             (1.0f / MOTOR2_ENC_QUADRATURE_SCALE)
//#define MOTOR2_ENC_RESOLUTION_SCALE             (M_PI / 2.0f / MOTOR2_ENC_RESOLUTION)
#define MOTOR2_ENC_SPEED_HIGH_SAMPLING_FREQ     (1000) // [Hz] Velocity computation frequency
#define MOTOR2_ENC_SPEED_HIGH_SAMPLING_TICKS    (DEVICE_SYSCLK_FREQ / MOTOR2_ENC_SPEED_HIGH_SAMPLING_FREQ)
#define MOTOR2_ENC_SPEED_HIGH_SCALE             (MOTOR2_ENC_SPEED_HIGH_SAMPLING_FREQ) // 2pi / dt
#define MOTOR2_ENC_SPEED_LOW_SCALE              (2.0f * M_PI * DEVICE_SYSCLK_FREQ * (1 << QEP_UNIT_POS_EVENT_DIV) / (1 << (QEP_CAPTURE_CLOCK_DIV >> 4)) / MOTOR2_ENC_RESOLUTION_SCALE)
#define MOTOR2_ENC_SPEED_CUTOFF_FREQ            (200.0f)     // cutoff frequency for velocity estimation
#define MOTOR2_ENC_SPEED_TIME_CONST             (2.0f * M_PI * MOTOR2_ENC_SPEED_CUTOFF_FREQ * PWM_PERIOD)
#define MOTOR2_ENC_SPEED_LPF_ALPHA              (MOTOR2_ENC_SPEED_TIME_CONST / (1.0f + MOTOR2_ENC_SPEED_TIME_CONST))
#define MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA        (1.0f - MOTOR2_ENC_SPEED_LPF_ALPHA)

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
//#define MOTOR1_QEP_REGS                         (&EQep1Regs)
//#define MOTOR2_QEP_REGS                         (&EQep2Regs)
#define MOTOR1_QEP_COUNTER_ADDR                 (MOTOR1_QEP_BASE + EQEP_O_QPOSCNT)
#define MOTOR2_QEP_COUNTER_ADDR                 (MOTOR2_QEP_BASE + EQEP_O_QPOSCNT)
#define MOTOR1_QEP_SWAP                         EQEP_CONFIG_NO_SWAP // Set to EQEP_CONFIG_SWAP if increments in the wrong direction
#define MOTOR2_QEP_SWAP                         EQEP_CONFIG_NO_SWAP
#define QEP_EMUL_MODE                           EQEP_EMULATIONMODE_RUNFREE
#define QEP_LATCH_MODE                          (EQEP_LATCH_SW_INDEX_MARKER | EQEP_LATCH_UNIT_TIME_OUT)//(EQEP_LATCH_RISING_INDEX | EQEP_LATCH_UNIT_TIME_OUT)
#define QEP_RESET_POSITION_MODE                 EQEP_POSITION_RESET_MAX_POS
#define QEP_CAPTURE_CLOCK_DIV                   EQEP_CAPTURE_CLK_DIV_64
#define QEP_UNIT_POS_EVENT_DIV                  EQEP_UNIT_POS_EVNT_DIV_4
//#define PERxSYSCLK_TO_EQEPCLK_DIV               (1)
//#define EQEPCLK_FREQ                            (DEVICE_SYSCLK_FREQ / pow(2.0, ((QEP_CAPTURE_CLOCK_DIV >> 4) & 0x07)))
/**
 * @}
 */

/***********************************************************************
 * MOTORS DEFINES
 ***********************************************************************/
/*** Motor 1 constants ***/
#define MOTOR1_KV                               (300.0f)    // Motor constant (rpm/V)
#define MOTOR1_KE                               (FM_SQRT3 / (MOTOR1_KV * FM_RPM2RADPS))   //1.0 / (MOTOR1_KV * FM_RPM2RAD * MOTOR1_POLES_PAIRS) // Motor back EMF constant (V/rad/s)
#define MOTOR1_KI                               (0.1f)      // Motor torque constant (N.m/A)
#define MOTOR1_RS                               (0.52f)//(0.7f)      // Stator resistance (ohm)
#define MOTOR1_LS                               (260e-6)//(134e-6f)   // Stator d-axis inductance (H)
#define MOTOR1_POLES_PAIRS                      (12.0f)        // Number of poles
#define MOTOR1_CURRENT_CUTOFF_FREQ              (500.0f)    // Current loop bandwidth (Hz)
#define MOTOR1_CURRENT_TIME_CONST               (2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ * PWM_PERIOD)
#define MOTOR1_CURRENT_LPF_ALPHA                (MOTOR1_CURRENT_TIME_CONST / (1.0f + MOTOR1_CURRENT_TIME_CONST))
#define MOTOR1_CURRENT_LPF_ONE_M_ALPHA          (1.0f - MOTOR1_CURRENT_LPF_ALPHA)
#define MOTOR1_CURRENT_ALIGN_MAX                (2.0f)      // Current on d-axis for motor alignment (A)
#define MOTOR1_PI_ID_KP_COEF                    (MOTOR1_LS * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ)
#define MOTOR1_PI_ID_KI_COEF                    (MOTOR1_RS * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ)
#define MOTOR1_PI_IQ_KP_COEF                    (MOTOR1_LS * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ)
#define MOTOR1_PI_IQ_KI_COEF                    (MOTOR1_RS * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ)
#define MOTOR1_CURRENT_CMD_SAT_MAX              (10.0f)
//#define MOTOR1_SPEED_REGULATOR_KP_COEF          (1.0f)
//#define MOTOR1_SPEED_REGULATOR_KD_COEF          (0.02f)
#define MOTOR1_DTC_MAX                          (0.9f)     // Max duty cycle
#define MOTOR1_DTC_MIN                          (0.1f)     // Min duty cycle
#define MOTOR1_STATOR_RESISTOR_CUTOFF_FREQ      (0.2f)
#define MOTOR1_STATOR_RESISTOR_TIME_CONST       (2.0f * M_PI * MOTOR1_STATOR_RESISTOR_CUTOFF_FREQ * PWM_PERIOD)
#define MOTOR1_STATOR_RESISTOR_LPF_ALPHA        (MOTOR1_STATOR_RESISTOR_TIME_CONST / (1.0f + MOTOR1_STATOR_RESISTOR_TIME_CONST))
#define MOTOR1_STATOR_RESISTOR_LPF_ONE_M_ALPHA  (1.0f - MOTOR1_STATOR_RESISTOR_LPF_ALPHA)

/*** Motor 2 constants ***/
#define MOTOR2_KV                               MOTOR1_KV                   // Motor constant (rpm/V)
#define MOTOR2_KE                               MOTOR1_KE                   // Motor back EMF constant (V/rad/s)
#define MOTOR2_KI                               MOTOR1_KI                   // Motor torque constant (N.m/A)
#define MOTOR2_RS                               MOTOR1_RS                   // Stator resistance (ohm)
#define MOTOR2_LS                               MOTOR1_LS                   // Stator d-axis inductance (H)
#define MOTOR2_POLES_PAIRS                      MOTOR1_POLES_PAIRS          // Number of poles
#define MOTOR2_CURRENT_CUTOFF_FREQ              MOTOR1_CURRENT_CUTOFF_FREQ  // Current loop bandwidth (Hz)
#define MOTOR2_CURRENT_TIME_CONST               MOTOR1_CURRENT_TIME_CONST
#define MOTOR2_CURRENT_LPF_ALPHA                MOTOR1_CURRENT_LPF_ALPHA
#define MOTOR2_CURRENT_LPF_ONE_M_ALPHA          MOTOR1_CURRENT_LPF_ONE_M_ALPHA
#define MOTOR2_CURRENT_ALIGN_MAX                MOTOR1_CURRENT_ALIGN_MAX     // Current on d-axis for motor alignment (A)
#define MOTOR2_PI_ID_KP_COEF                    MOTOR1_PI_ID_KP_COEF
#define MOTOR2_PI_ID_KI_COEF                    MOTOR1_PI_ID_KI_COEF
#define MOTOR2_PI_IQ_KP_COEF                    MOTOR1_PI_IQ_KP_COEF
#define MOTOR2_PI_IQ_KI_COEF                    MOTOR1_PI_IQ_KI_COEF
#define MOTOR2_CURRENT_CMD_SAT_MAX              MOTOR1_CURRENT_CMD_SAT_MAX
//#define MOTOR2_SPEED_REGULATOR_KP_COEF          MOTOR1_SPEED_REGULATOR_KP_COEF
//#define MOTOR2_SPEED_REGULATOR_KD_COEF          MOTOR1_SPEED_REGULATOR_KD_COEF
#define MOTOR2_DTC_MAX                          MOTOR1_DTC_MAX              // Max duty cycle
#define MOTOR2_DTC_MIN                          MOTOR1_DTC_MIN              // Min duty cycle
#define MOTOR2_STATOR_RESISTOR_CUTOFF_FREQ      MOTOR1_STATOR_RESISTOR_CUTOFF_FREQ
#define MOTOR2_STATOR_RESISTOR_TIME_CONST       MOTOR1_STATOR_RESISTOR_TIME_CONST
#define MOTOR2_STATOR_RESISTOR_LPF_ALPHA        MOTOR1_STATOR_RESISTOR_LPF_ALPHA
#define MOTOR2_STATOR_RESISTOR_LPF_ONE_M_ALPHA  MOTOR1_STATOR_RESISTOR_LPF_ONE_M_ALPHA

#define MOTOR12_OVERMODULATION                  (1.15f)
#define VBUS_CUTOFF_FREQ                        (400.0f)    // vbus voltage loop bandwidth (Hz)
#define VBUS_CUTOFF_TIME_CONST                  (2.0f * M_PI * VBUS_CUTOFF_FREQ * PWM_PERIOD)
#define VBUS_LPF_ALPHA                          (VBUS_CUTOFF_TIME_CONST / (1.0f + VBUS_CUTOFF_TIME_CONST))
#define VBUS_LPF_ONE_M_ALPHA                    (1.0f - VBUS_LPF_ALPHA)
#define VEXT_CUTOFF_FREQ                        (4000.0f)   // vbus voltage loop bandwidth (Hz)
#define VEXT_CUTOFF_TIME_CONST                  (2.0f * M_PI * VEXT_CUTOFF_FREQ * PWM_PERIOD)
#define VEXT_LPF_ALPHA                          (VEXT_CUTOFF_TIME_CONST / (1.0f + VEXT_CUTOFF_TIME_CONST))
#define VEXT_LPF_ONE_M_ALPHA                    (1.0f - VEXT_LPF_ALPHA)

/***********************************************************************
 * COMMUNICATION DEFINES
 ***********************************************************************/
#define COM_MSG_TX_RX_8BIT_FULL_LENGTH          (34)
#define COM_MSG_TX_RX_16BIT_FULL_LENGTH         (COM_MSG_TX_RX_8BIT_FULL_LENGTH / 2)
#define COM_MSG_TX_RX_8BIT_CRC_LENGTH           (4)
#define COM_MSG_TX_RX_16BIT_CRC_LENGTH          (COM_MSG_TX_RX_8BIT_CRC_LENGTH / 2)
#define COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH      (COM_MSG_TX_RX_16BIT_FULL_LENGTH - COM_MSG_TX_RX_16BIT_CRC_LENGTH)
#define COM_MSG_TX_RX_8BIT_PAYLOAD_LENGTH       (COM_MSG_TX_RX_8BIT_FULL_LENGTH - COM_MSG_TX_RX_8BIT_CRC_LENGTH)

#define LED_MSG_TX_16BIT_LENGTH                 (6)

/*** Communication constants ***/
#define POSITION_LSB                            (5.960464477539063e-08f)    // 2**(-24)
#define VELOCITY_LSB                            (4.8828125e-04f)            // 2**(-11)
#define IQ_LSB                                  (9.765625e-04f)             // 2**(-10)
#define CURRENT_SAT_LSB                         (1.25e-01f)                 // 2**(-3)
#define RESISTANCE_LSB                          (3.0517578125e-05f)         // 2**(-15)
#define VOLTAGE_LSB                             (6.103515625e-05f)          // 2**(-14)
#define KP_LSB                                  (4.8828125e-04f)            // 2**(-11)
#define KD_LSB                                  (9.765625e-04f)             // 2**(-10)

typedef struct __lowPassFilter_t__
{
    float32_t a;
    float32_t one_minus_a;
} lpf_t;

/**
 * @enum    ArrayID
 * @brief   Easier array management for history.
 */
typedef enum
{
    NEW = 0,    /*!< Current data */
    OLD = 1,    /*!< Previous data */
} array_id_e;

/**
 * @enum    MotorID
 * @brief   Easier motor identification
 */
typedef enum
{
    MOTOR_1 = 0,    /*!< Select Motor_1 */
    MOTOR_2 = 1,    /*!< Select Motor_2 */
    NO_MOT  = -1,   /*!< No motor selected */
} motor_id_e;

#endif
