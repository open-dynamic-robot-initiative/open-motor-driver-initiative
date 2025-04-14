/*
 * File name: omodri_user_defines.h
 * Description: Header file containing user definitions
 */

#ifndef __UOMODRI_1_HAL_DEFINES_H__
#define __UOMODRI_1_HAL_DEFINES_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "device.h"                 // F2838x Headerfile include File

/***********************************************************************
 * GPIO DEFINES
 ***********************************************************************/
#define MOTOR1_PWM1_CHA_NUM                     (0U)
#define MOTOR1_PWM1_CHA_CFG                     GPIO_0_EPWM1A
#define MOTOR1_PWM1_CHA_DEF                     { \
                                                    .pinNum         = MOTOR1_PWM1_CHA_NUM, \
                                                    .fctMux         = MOTOR1_PWM1_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_PWM1_CHB_NUM                     (1U)
#define MOTOR1_PWM1_CHB_CFG                     GPIO_1_EPWM1B
#define MOTOR1_PWM1_CHB_DEF                     { \
                                                    .pinNum         = MOTOR1_PWM1_CHB_NUM, \
                                                    .fctMux         = MOTOR1_PWM1_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_PWM2_CHA_NUM                     (2U)
#define MOTOR1_PWM2_CHA_CFG                     GPIO_2_EPWM2A
#define MOTOR1_PWM2_CHA_DEF                     { \
                                                    .pinNum         = MOTOR1_PWM2_CHA_NUM, \
                                                    .fctMux         = MOTOR1_PWM2_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_PWM2_CHB_NUM                     (3U)
#define MOTOR1_PWM2_CHB_CFG                     GPIO_3_EPWM2B
#define MOTOR1_PWM2_CHB_DEF                     { \
                                                    .pinNum         = MOTOR1_PWM2_CHB_NUM, \
                                                    .fctMux         = MOTOR1_PWM2_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_PWM3_CHA_NUM                     (4U)
#define MOTOR1_PWM3_CHA_CFG                     GPIO_4_EPWM3A
#define MOTOR1_PWM3_CHA_DEF                     { \
                                                    .pinNum         = MOTOR1_PWM3_CHA_NUM, \
                                                    .fctMux         = MOTOR1_PWM3_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_PWM3_CHB_NUM                     (5U)
#define MOTOR1_PWM3_CHB_CFG                     GPIO_5_EPWM3B
#define MOTOR1_PWM3_CHB_DEF                     { \
                                                    .pinNum         = MOTOR1_PWM3_CHB_NUM, \
                                                    .fctMux         = MOTOR1_PWM3_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_PWM1_CHA_NUM                     (6U)
#define MOTOR2_PWM1_CHA_CFG                     GPIO_6_EPWM4A
#define MOTOR2_PWM1_CHA_DEF                     { \
                                                    .pinNum         = MOTOR2_PWM1_CHA_NUM, \
                                                    .fctMux         = MOTOR2_PWM1_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_PWM1_CHB_NUM                     (7U)
#define MOTOR2_PWM1_CHB_CFG                     GPIO_7_EPWM4B
#define MOTOR2_PWM1_CHB_DEF                     { \
                                                    .pinNum         = MOTOR2_PWM1_CHB_NUM, \
                                                    .fctMux         = MOTOR2_PWM1_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_PWM2_CHA_NUM                     (8U)
#define MOTOR2_PWM2_CHA_CFG                     GPIO_8_EPWM5A
#define MOTOR2_PWM2_CHA_DEF                     { \
                                                    .pinNum         = MOTOR2_PWM2_CHA_NUM, \
                                                    .fctMux         = MOTOR2_PWM2_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_PWM2_CHB_NUM                     (9U)
#define MOTOR2_PWM2_CHB_CFG                     GPIO_9_EPWM5B
#define MOTOR2_PWM2_CHB_DEF                     { \
                                                    .pinNum         = MOTOR2_PWM2_CHB_NUM, \
                                                    .fctMux         = MOTOR2_PWM2_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_PWM3_CHA_NUM                     (10U)
#define MOTOR2_PWM3_CHA_CFG                     GPIO_10_EPWM6A
#define MOTOR2_PWM3_CHA_DEF                     { \
                                                    .pinNum         = MOTOR2_PWM3_CHA_NUM, \
                                                    .fctMux         = MOTOR2_PWM3_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_PWM3_CHB_NUM                     (11U)
#define MOTOR2_PWM3_CHB_CFG                     GPIO_11_EPWM6B
#define MOTOR2_PWM3_CHB_DEF                     { \
                                                    .pinNum         = MOTOR2_PWM3_CHB_NUM, \
                                                    .fctMux         = MOTOR2_PWM3_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#if (RS485_BUS_ENABLE && !CAN_BUS_ENABLE)
#define RS485_TX_NUM                            (12U)
#define RS485_TX_CFG                            GPIO_12_SCIC_TX//GPIO_12_EPWM7A//
#define RS485_TX_DEF                            { \
                                                    .pinNum         = RS485_TX_NUM, \
                                                    .fctMux         = RS485_TX_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define RS485_RX_NUM                            (13U)
#define RS485_RX_CFG                            GPIO_13_SCIC_RX
#define RS485_RX_DEF                            { \
                                                    .pinNum         = RS485_RX_NUM, \
                                                    .fctMux         = RS485_RX_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }
#elif (!RS485_BUS_ENABLE && CAN_BUS_ENABLE)
#define CAN_TX_NUM                              (12U)
#define CAN_TX_CFG                              GPIO_12_CANB_TX
#define CAN_TX_DEF                              { \
                                                    .pinNum         = CAN_TX_NUM, \
                                                    .fctMux         = CAN_TX_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define CAN_RX_NUM                              (13U)
#define CAN_RX_CFG                              GPIO_13_CANB_RX
#define CAN_RX_DEF                              { \
                                                    .pinNum         = CAN_RX_NUM, \
                                                    .fctMux         = CAN_RX_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }
#endif

#define DRV_SPI_SIMO_NUM                        (16U)
#define DRV_SPI_SIMO_CFG                        GPIO_16_SPIA_SIMO
#define DRV_SPI_SIMO_DEF                        { \
                                                    .pinNum         = DRV_SPI_SIMO_NUM, \
                                                    .fctMux         = DRV_SPI_SIMO_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define DRV_SPI_SOMI_NUM                        (17U)
#define DRV_SPI_SOMI_CFG                        GPIO_17_SPIA_SOMI
#define DRV_SPI_SOMI_DEF                        { \
                                                    .pinNum         = DRV_SPI_SOMI_NUM, \
                                                    .fctMux         = DRV_SPI_SOMI_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define DRV_SPI_CLK_NUM                         (18U)
#define DRV_SPI_CLK_CFG                         GPIO_18_SPIA_CLK
#define DRV_SPI_CLK_DEF                         { \
                                                    .pinNum         = DRV_SPI_CLK_NUM, \
                                                    .fctMux         = DRV_SPI_CLK_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_DRV_SPI_CS_NUM                   (19U)
#define MOTOR1_DRV_SPI_CS_CFG                   GPIO_19_GPIO19
#define MOTOR1_DRV_SPI_CS_SET                   (HWREG(GPIODATA_BASE + GPIO_O_GPASET)   = GPIO_GPASET_GPIO19)
#define MOTOR1_DRV_SPI_CS_CLEAR                 (HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR) = GPIO_GPACLEAR_GPIO19)
#define MOTOR1_DRV_SPI_CS_TOGGLE                (HWREG(GPIODATA_BASE + GPIO_O_GPATOGGLE)= GPIO_GPATOGGLE_GPIO19)
#define MOTOR1_DRV_SPI_CS_DEF                   { \
                                                    .pinNum         = MOTOR1_DRV_SPI_CS_NUM, \
                                                    .fctMux         = MOTOR1_DRV_SPI_CS_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_ENC_CHA_NUM                      (20U)
#define MOTOR1_ENC_CHA_CFG                      GPIO_20_EQEP1_A
#define MOTOR1_ENC_CHA_DEF                      { \
                                                    .pinNum         = MOTOR1_ENC_CHA_NUM, \
                                                    .fctMux         = MOTOR1_ENC_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_ENC_CHB_NUM                      (21U)
#define MOTOR1_ENC_CHB_CFG                      GPIO_21_EQEP1_B
#define MOTOR1_ENC_CHB_DEF                      { \
                                                    .pinNum         = MOTOR1_ENC_CHB_NUM, \
                                                    .fctMux         = MOTOR1_ENC_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_ENC_CHI_NUM                      (23U)
#define MOTOR1_ENC_CHI_CFG                      GPIO_23_EQEP1_INDEX
#define MOTOR1_ENC_CHI_DEF                      { \
                                                    .pinNum         = MOTOR1_ENC_CHI_NUM, \
                                                    .fctMux         = MOTOR1_ENC_CHI_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_ENC_CHA_NUM                      (24U)
#define MOTOR2_ENC_CHA_CFG                      GPIO_24_EQEP2_A
#define MOTOR2_ENC_CHA_DEF                      { \
                                                    .pinNum         = MOTOR2_ENC_CHA_NUM, \
                                                    .fctMux         = MOTOR2_ENC_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_ENC_CHB_NUM                      (25U)
#define MOTOR2_ENC_CHB_CFG                      GPIO_25_EQEP2_B
#define MOTOR2_ENC_CHB_DEF                      { \
                                                    .pinNum         = MOTOR2_ENC_CHB_NUM, \
                                                    .fctMux         = MOTOR2_ENC_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_ENC_CHI_NUM                      (26U)
#define MOTOR2_ENC_CHI_CFG                      GPIO_26_EQEP2_INDEX
#define MOTOR2_ENC_CHI_DEF                      { \
                                                    .pinNum         = MOTOR2_ENC_CHI_NUM, \
                                                    .fctMux         = MOTOR2_ENC_CHI_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_DRV_SPI_CS_NUM                   (35U)
#define MOTOR2_DRV_SPI_CS_CFG                   GPIO_35_GPIO35
#define MOTOR2_DRV_SPI_CS_SET                   (HWREG(GPIODATA_BASE + GPIO_O_GPBSET)   = GPIO_GPBSET_GPIO35)
#define MOTOR2_DRV_SPI_CS_CLEAR                 (HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIO_GPBCLEAR_GPIO35)
#define MOTOR2_DRV_SPI_CS_TOGGLE                (HWREG(GPIODATA_BASE + GPIO_O_GPBTOGGLE)= GPIO_GPBTOGGLE_GPIO35)
#define MOTOR2_DRV_SPI_CS_DEF                   { \
                                                    .pinNum         = MOTOR2_DRV_SPI_CS_NUM, \
                                                    .fctMux         = MOTOR2_DRV_SPI_CS_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_DRV_FAULT_NUM                    (36U)
#define MOTOR2_DRV_FAULT_CFG                    GPIO_36_GPIO36
#define MOTOR2_DRV_FAULT_READ                   (HWREG(GPIODATA_BASE + GPIO_O_GPBDAT) & GPIO_GPBDAT_GPIO36)
#define MOTOR2_DRV_FAULT_DEF                    { \
                                                    .pinNum         = MOTOR2_DRV_FAULT_NUM, \
                                                    .fctMux         = MOTOR2_DRV_FAULT_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_3SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_DRV_EN_NUM                       (37U)
#define MOTOR2_DRV_EN_CFG                       GPIO_37_GPIO37
#define MOTOR2_DRV_EN_SET                       (HWREG(GPIODATA_BASE + GPIO_O_GPBSET)   = GPIO_GPBSET_GPIO37)
#define MOTOR2_DRV_EN_CLEAR                     (HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIO_GPBCLEAR_GPIO37)
#define MOTOR2_DRV_EN_TOGGLE                    (HWREG(GPIODATA_BASE + GPIO_O_GPBTOGGLE)= GPIO_GPBTOGGLE_GPIO37)
#define MOTOR2_DRV_EN_DEF                       { \
                                                    .pinNum         = MOTOR2_DRV_EN_NUM, \
                                                    .fctMux         = MOTOR2_DRV_EN_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#if (USB_BUS_ENABLE) && (!UART_BUS_ENABLE)
#define USB_DM_NUM                              (42U)
#define USB_DM_DEF                              { \
                                                    .pinNum             = USB_DM_NUM, \
                                                    .coreSelect         = GPIO_CORE_CM, \
                                                    .analogModeEnable   = GPIO_ANALOG_ENABLED, \
                                                }

#define USB_DP_NUM                              (43U)
#define USB_DP_DEF                              { \
                                                    .pinNum             = USB_DP_NUM, \
                                                    .coreSelect         = GPIO_CORE_CM, \
                                                    .analogModeEnable   = GPIO_ANALOG_ENABLED, \
                                                }
#elif (!USB_BUS_ENABLE) && (UART_BUS_ENABLE)
#define UART_ON_USB_TX_NUM                      (42U)
#define UART_ON_USB_TX_CFG                      GPIO_42_UARTA_TX
#define UART_ON_USB_TX_DEF                      { \
                                                    .pinNum         = UART_ON_USB_TX_NUM, \
                                                    .fctMux         = UART_ON_USB_TX_CFG, \
                                                    .coreSelect     = GPIO_CORE_CM, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define UART_ON_USB_RX_NUM                      (43U)
#define UART_ON_USB_RX_CFG                      GPIO_43_UARTA_RX
#define UART_ON_USB_RX_DEF                      { \
                                                    .pinNum         = UART_ON_USB_RX_NUM, \
                                                    .fctMux         = UART_ON_USB_RX_CFG, \
                                                    .coreSelect     = GPIO_CORE_CM, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }
#endif

#define COM_SPI_CS_NUM                          (59U)
#define COM_SPI_CS_CFG                          GPIO_59_SPIB_STEN
#define COM_SPI_CS_READ                         (HWREG(GPIODATA_BASE + GPIO_O_GPBDAT) & GPIO_GPBDAT_GPIO59)
#define COM_SPI_CS_DEF                          { \
                                                    .pinNum         = COM_SPI_CS_NUM, \
                                                    .fctMux         = COM_SPI_CS_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define COM_SPI_SIMO_NUM                        (60U)
#define COM_SPI_SIMO_CFG                        GPIO_60_SPIB_SIMO
#define COM_SPI_SIMO_DEF                        { \
                                                    .pinNum         = COM_SPI_SIMO_NUM, \
                                                    .fctMux         = COM_SPI_SIMO_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define COM_SPI_SOMI_NUM                        (61U)
#define COM_SPI_SOMI_CFG                        GPIO_61_SPIB_SOMI
#define COM_SPI_SOMI_DEF                        { \
                                                    .pinNum         = COM_SPI_SOMI_NUM, \
                                                    .fctMux         = COM_SPI_SOMI_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define COM_SPI_CLK_NUM                         (65U)
#define COM_SPI_CLK_CFG                         GPIO_65_SPIB_CLK
#define COM_SPI_CLK_DEF                         { \
                                                    .pinNum         = COM_SPI_CLK_NUM, \
                                                    .fctMux         = COM_SPI_CLK_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define DBG_PIN1_NUM                            (77U)
#define DBG_PIN1_CFG                            GPIO_77_GPIO77
#define DBG_PIN1_CORE                           GPIO_CORE_CPU1_CLA1
#define DBG_PIN1_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO77)
#define DBG_PIN1_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO77)
#define DBG_PIN1_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO77)
#define DBG_PIN1_READ                           (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO77)
#define DBG_PIN1_DEF                            { \
                                                    .pinNum         = DBG_PIN1_NUM, \
                                                    .fctMux         = DBG_PIN1_CFG, \
                                                    .coreSelect     = DBG_PIN1_CORE, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define DBG_PIN0_NUM                            (78U)
#define DBG_PIN0_CFG                            GPIO_78_GPIO78
#define DBG_PIN0_CORE                           GPIO_CORE_CPU1_CLA1
#define DBG_PIN0_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO78)
#define DBG_PIN0_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO78)
#define DBG_PIN0_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO78)
#define DBG_PIN0_READ                           (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO78)
#define DBG_PIN0_DEF                            { \
                                                    .pinNum         = DBG_PIN0_NUM, \
                                                    .fctMux         = DBG_PIN0_CFG, \
                                                    .coreSelect     = DBG_PIN0_CORE, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define DBG_PIN3_NUM                            (81U)
#define DBG_PIN3_CFG                            GPIO_81_GPIO81
#define DBG_PIN3_CORE                           GPIO_CORE_CPU1
#define DBG_PIN3_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO81)
#define DBG_PIN3_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO81)
#define DBG_PIN3_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO81)
#define DBG_PIN3_READ                           (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO81)
#define DBG_PIN3_DEF                            { \
                                                    .pinNum         = DBG_PIN3_NUM, \
                                                    .fctMux         = DBG_PIN3_CFG, \
                                                    .coreSelect     = DBG_PIN3_CORE, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define DBG_PIN2_NUM                            (82U)
#define DBG_PIN2_CFG                            GPIO_82_GPIO82
#define DBG_PIN2_CORE                           GPIO_CORE_CPU1
#define DBG_PIN2_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO82)
#define DBG_PIN2_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO82)
#define DBG_PIN2_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO82)
#define DBG_PIN2_READ                           (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO82)
#define DBG_PIN2_DEF                            { \
                                                    .pinNum         = DBG_PIN2_NUM, \
                                                    .fctMux         = DBG_PIN2_CFG, \
                                                    .coreSelect     = DBG_PIN2_CORE, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_DRV_EN_NUM                       (89U)
#define MOTOR1_DRV_EN_CFG                       GPIO_89_GPIO89
#define MOTOR1_DRV_EN_SET                       (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO89)
#define MOTOR1_DRV_EN_CLEAR                     (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO89)
#define MOTOR1_DRV_EN_TOGGLE                    (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO89)
#define MOTOR1_DRV_EN_DEF                       { \
                                                    .pinNum         = MOTOR1_DRV_EN_NUM, \
                                                    .fctMux         = MOTOR1_DRV_EN_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_DRV_FAULT_NUM                    (90U)
#define MOTOR1_DRV_FAULT_CFG                    GPIO_90_GPIO90
#define MOTOR1_DRV_FAULT_READ                   (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO90)
#define MOTOR1_DRV_FAULT_DEF                    { \
                                                    .pinNum         = MOTOR1_DRV_FAULT_NUM, \
                                                    .fctMux         = MOTOR1_DRV_FAULT_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_3SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define LED_SPI_SIMO_NUM                        (91U)
#define LED_SPI_SIMO_CFG                        GPIO_91_SPID_SIMO
#define LED_SPI_SIMO_DEF                        { \
                                                    .pinNum         = LED_SPI_SIMO_NUM, \
                                                    .fctMux         = LED_SPI_SIMO_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }


/***********************************************************************
 * ADC DEFINES
 ***********************************************************************/
/** @defgroup ADC_global_defines
 *  @brief    Configure ADC subsystems.
 * @{
 */
// ADC clock config
#define PERxSYSCLK_TO_ADCCLK_DIV                ADC_CLK_DIV_4_0
#define ADC_CLK_FREQ                            (DEVICE_SYSCLK_FREQ / ((PERxSYSCLK_TO_ADCCLK_DIV / 2) + 1))
// ADC defines
#define ADC_SAMPLING_WINDOW                     (119U)   // From data sheet Table 20-12
#define ADC_RESOLUTION_BIT                      ADC_RESOLUTION_16BIT
#define ADC_SIGNAL_MODE                         ADC_MODE_SINGLE_ENDED
#define ADC_PULSE_END_MODE                      ADC_PULSE_END_OF_CONV
#define ADC_SOC_PRIORITY                        ADC_PRI_ALL_ROUND_ROBIN
#define ADC_INT_SOC_TRIGGER                     ADC_INT_SOC_TRIGGER_NONE
// Reference voltage
#define ADC_VREF                                (3.0f)   // [V]
// DRV analog config
#define DRV_CSA_GAIN_REG                        DRV_CSA_GAIN_10_V_V  // [V/V] Possible values: 5, 10, 20, 40
// Shunt & Vbus bridge divider
#define R_SHUNT                                 (0.005f) // [Ohm]
#define BRIDGE_DIVIDER                          ((5.1f + 100.0f) / 5.1f)
/**
 * @}
 */

// Analog IOs
#define MOTOR1_IA_CH                            ADC_CH_ADCIN0
#define MOTOR1_IB_CH                            ADC_CH_ADCIN0
#define MOTOR1_IC_CH                            ADC_CH_ADCIN0

#define MOTOR2_IA_CH                            ADC_CH_ADCIN1
#define MOTOR2_IB_CH                            ADC_CH_ADCIN1
#define MOTOR2_IC_CH                            ADC_CH_ADCIN1

#define VBUS_CH                                 ADC_CH_ADCIN2
#define VEXT1_CH                                ADC_CH_ADCIN14
#define VEXT2_CH                                ADC_CH_ADCIN15

/** @defgroup ADC SOC Trigger event source
 *  @brief    Associate the trigger event source to start conversions.
 * @{
 */
#define MOTOR1_IABC_SOC_TRIGGER_EVT             ADC_TRIGGER_EPWM1_SOCA
#define MOTOR2_IABC_SOC_TRIGGER_EVT             ADC_TRIGGER_EPWM4_SOCB
#define VBUS_SOC_TRIGGER_EVT                    ADC_TRIGGER_EPWM1_SOCA
#define VEXT1_SOC_TRIGGER_EVT                   ADC_TRIGGER_EPWM4_SOCB
#define VEXT2_SOC_TRIGGER_EVT                   ADC_TRIGGER_EPWM4_SOCB

#define MOTOR1_IA_SOC_TRIGGER_EVT               MOTOR1_IABC_SOC_TRIGGER_EVT
#define MOTOR1_IB_SOC_TRIGGER_EVT               MOTOR1_IABC_SOC_TRIGGER_EVT
#define MOTOR1_IC_SOC_TRIGGER_EVT               MOTOR1_IABC_SOC_TRIGGER_EVT
#define MOTOR2_IA_SOC_TRIGGER_EVT               MOTOR2_IABC_SOC_TRIGGER_EVT
#define MOTOR2_IB_SOC_TRIGGER_EVT               MOTOR2_IABC_SOC_TRIGGER_EVT
#define MOTOR2_IC_SOC_TRIGGER_EVT               MOTOR2_IABC_SOC_TRIGGER_EVT
/**
 * @}
 */

/** @defgroup ADC SOC (Start-Of-Conversion)
 *  @brief    Associate the SOC for each measure to perform.
 * @{
 */
#define MOTOR1_IABC_SOC_NUM                     ADC_SOC_NUMBER0
#define MOTOR2_IABC_SOC_NUM                     ADC_SOC_NUMBER1
#define VBUS_SOC_NUM                            ADC_SOC_NUMBER0
#define VEXT1_SOC_NUM                           ADC_SOC_NUMBER1
#define VEXT2_SOC_NUM                           ADC_SOC_NUMBER2

#define MOTOR1_IA_SOC_NUM                       MOTOR1_IABC_SOC_NUM
#define MOTOR1_IB_SOC_NUM                       MOTOR1_IABC_SOC_NUM
#define MOTOR1_IC_SOC_NUM                       MOTOR1_IABC_SOC_NUM
#define MOTOR2_IA_SOC_NUM                       MOTOR2_IABC_SOC_NUM
#define MOTOR2_IB_SOC_NUM                       MOTOR2_IABC_SOC_NUM
#define MOTOR2_IC_SOC_NUM                       MOTOR2_IABC_SOC_NUM
/**
 * @}
 */

/** @defgroup ADC PPB (Post-Processing Blocks)
 *  @brief    Associate the PPB for each measure to perform.
 * @{
 */
#define MOTOR1_IABC_PPB_NUM                     ADC_PPB_NUMBER1
#define MOTOR2_IABC_PPB_NUM                     ADC_PPB_NUMBER2
#define VBUS_PPB_NUM                            ADC_PPB_NUMBER1
#define VEXT1_PPB_NUM                           ADC_PPB_NUMBER2
#define VEXT2_PPB_NUM                           ADC_PPB_NUMBER3

#define MOTOR1_IA_PPB_NUM                       MOTOR1_IABC_PPB_NUM
#define MOTOR1_IB_PPB_NUM                       MOTOR1_IABC_PPB_NUM
#define MOTOR1_IC_PPB_NUM                       MOTOR1_IABC_PPB_NUM
#define MOTOR2_IA_PPB_NUM                       MOTOR2_IABC_PPB_NUM
#define MOTOR2_IB_PPB_NUM                       MOTOR2_IABC_PPB_NUM
#define MOTOR2_IC_PPB_NUM                       MOTOR2_IABC_PPB_NUM
/**
 * @}
 */

/** @defgroup ADC interrupts source \& acknowledge group
 *  @brief    Define the interrupt source \& the PIE Interrupt Number associated
 * @{
 */
#define MOTOR1_IA_INT_NUM                       ADC_INT_NUMBER1
#define MOTOR2_IA_INT_NUM                       ADC_INT_NUMBER2
/**
 * @}
 */

/** @defgroup ADC Base Address
 *  @brief    Define the ADC base address for all the conversion to be performed.
 * @{
 */
// ADC Base Address
#define MOTOR12_IA_ADC_CFG_ADDR                 ADCA_BASE
#define MOTOR12_IB_ADC_CFG_ADDR                 ADCB_BASE
#define MOTOR12_IC_ADC_CFG_ADDR                 ADCD_BASE
#define VBUS_VEXT12_ADC_CFG_ADDR                ADCC_BASE

#define MOTOR1_IA_ADC_CFG_ADDR                  MOTOR12_IA_ADC_CFG_ADDR
#define MOTOR1_IB_ADC_CFG_ADDR                  MOTOR12_IB_ADC_CFG_ADDR
#define MOTOR1_IC_ADC_CFG_ADDR                  MOTOR12_IC_ADC_CFG_ADDR
#define MOTOR2_IA_ADC_CFG_ADDR                  MOTOR12_IA_ADC_CFG_ADDR
#define MOTOR2_IB_ADC_CFG_ADDR                  MOTOR12_IB_ADC_CFG_ADDR
#define MOTOR2_IC_ADC_CFG_ADDR                  MOTOR12_IC_ADC_CFG_ADDR
#define VBUS_ADC_CFG_ADDR                       VBUS_VEXT12_ADC_CFG_ADDR
#define VEXT1_ADC_CFG_ADDR                      VBUS_VEXT12_ADC_CFG_ADDR
#define VEXT2_ADC_CFG_ADDR                      VBUS_VEXT12_ADC_CFG_ADDR
/**
 * @}
 */

/** @defgroup ADC Base result Address
 *  @brief    Define the ADC results base address for all the conversion to be performed.
 * @{
 */
#define MOTOR12_IA_ADC_RESULT_ADDR              ADCARESULT_BASE
#define MOTOR12_IB_ADC_RESULT_ADDR              ADCBRESULT_BASE
#define MOTOR12_IC_ADC_RESULT_ADDR              ADCDRESULT_BASE
#define VBUS_VEXT12_ADC_RESULT_ADDR             ADCCRESULT_BASE

#define MOTOR1_IA_ADC_RESULT_ADDR               MOTOR12_IA_ADC_RESULT_ADDR
#define MOTOR1_IB_ADC_RESULT_ADDR               MOTOR12_IB_ADC_RESULT_ADDR
#define MOTOR1_IC_ADC_RESULT_ADDR               MOTOR12_IC_ADC_RESULT_ADDR
#define MOTOR2_IA_ADC_RESULT_ADDR               MOTOR12_IA_ADC_RESULT_ADDR
#define MOTOR2_IB_ADC_RESULT_ADDR               MOTOR12_IB_ADC_RESULT_ADDR
#define MOTOR2_IC_ADC_RESULT_ADDR               MOTOR12_IC_ADC_RESULT_ADDR
#define VBUS_ADC_RESULT_ADDR                    VBUS_VEXT12_ADC_RESULT_ADDR
#define VEXT1_ADC_RESULT_ADDR                   VBUS_VEXT12_ADC_RESULT_ADDR
#define VEXT2_ADC_RESULT_ADDR                   VBUS_VEXT12_ADC_RESULT_ADDR
/**
 * @}
 */

/* MOTOR1 - Ia ADC Configs */
#define MOTOR1_IA_ADC_INIT_DEF                  { \
                                                    .adcBase        = MOTOR1_IA_ADC_CFG_ADDR, \
                                                    .adcChannel     = MOTOR1_IA_CH, \
                                                    .adcSOCNum      = MOTOR1_IA_SOC_NUM, \
                                                    .adcTrigSrc     = MOTOR1_IA_SOC_TRIGGER_EVT, \
                                                }
#define MOTOR1_IA_ADC_ACQ_DEF                   { \
                                                    .adcBase        = MOTOR1_IA_ADC_CFG_ADDR, \
                                                    .adcResultBase  = MOTOR1_IA_ADC_RESULT_ADDR, \
                                                    .adcSOCNum      = MOTOR1_IA_SOC_NUM, \
                                                    .adcPPBNum      = MOTOR1_IA_PPB_NUM, \
                                                }
#define MOTOR1_IA_ADC_INT_DEF                   { \
                                                    .adcBase        = MOTOR1_IA_ADC_CFG_ADDR, \
                                                    .intTrig        = MOTOR1_IA_SOC_NUM, \
                                                    .adcIntNum      = MOTOR1_IA_INT_NUM, \
                                                }
/* MOTOR2 - Ia ADC Configs */
#define MOTOR2_IA_ADC_INIT_DEF                  { \
                                                    .adcBase          = MOTOR2_IA_ADC_CFG_ADDR, \
                                                    .adcChannel       = MOTOR2_IA_CH, \
                                                    .adcSOCNum        = MOTOR2_IA_SOC_NUM, \
                                                    .adcTrigSrc       = MOTOR2_IA_SOC_TRIGGER_EVT, \
                                                }
#define MOTOR2_IA_ADC_ACQ_DEF                   { \
                                                    .adcBase          = MOTOR2_IA_ADC_CFG_ADDR, \
                                                    .adcResultBase    = MOTOR2_IA_ADC_RESULT_ADDR, \
                                                    .adcSOCNum        = MOTOR2_IA_SOC_NUM, \
                                                    .adcPPBNum        = MOTOR2_IA_PPB_NUM, \
                                                }
#define MOTOR2_IA_ADC_INT_DEF                   { \
                                                    .adcBase        = MOTOR2_IA_ADC_CFG_ADDR, \
                                                    .intTrig        = MOTOR2_IA_SOC_NUM, \
                                                    .adcIntNum      = MOTOR2_IA_INT_NUM, \
                                                }
/* MOTOR1 - Ib ADC Configs */
#define MOTOR1_IB_ADC_INIT_DEF                  { \
                                                    .adcBase          = MOTOR1_IB_ADC_CFG_ADDR, \
                                                    .adcChannel       = MOTOR1_IB_CH, \
                                                    .adcSOCNum        = MOTOR1_IB_SOC_NUM, \
                                                    .adcTrigSrc       = MOTOR1_IB_SOC_TRIGGER_EVT, \
                                                }
#define MOTOR1_IB_ADC_ACQ_DEF                   { \
                                                    .adcBase          = MOTOR1_IB_ADC_CFG_ADDR, \
                                                    .adcResultBase    = MOTOR1_IB_ADC_RESULT_ADDR, \
                                                    .adcSOCNum        = MOTOR1_IB_SOC_NUM, \
                                                    .adcPPBNum        = MOTOR1_IB_PPB_NUM, \
                                                }
/* MOTOR2 - Ib ADC Configs */
#define MOTOR2_IB_ADC_INIT_DEF                  { \
                                                    .adcBase          = MOTOR2_IB_ADC_CFG_ADDR, \
                                                    .adcChannel       = MOTOR2_IB_CH, \
                                                    .adcSOCNum        = MOTOR2_IB_SOC_NUM, \
                                                    .adcTrigSrc       = MOTOR2_IB_SOC_TRIGGER_EVT, \
                                                }
#define MOTOR2_IB_ADC_ACQ_DEF                   { \
                                                    .adcBase          = MOTOR2_IB_ADC_CFG_ADDR, \
                                                    .adcResultBase    = MOTOR2_IB_ADC_RESULT_ADDR, \
                                                    .adcSOCNum        = MOTOR2_IB_SOC_NUM, \
                                                    .adcPPBNum        = MOTOR2_IB_PPB_NUM, \
                                                }
/* MOTOR1 - Ic ADC Configs */
#define MOTOR1_IC_ADC_INIT_DEF                  { \
                                                    .adcBase          = MOTOR1_IC_ADC_CFG_ADDR, \
                                                    .adcChannel       = MOTOR1_IC_CH, \
                                                    .adcSOCNum        = MOTOR1_IC_SOC_NUM, \
                                                    .adcTrigSrc       = MOTOR1_IC_SOC_TRIGGER_EVT, \
                                                }
#define MOTOR1_IC_ADC_ACQ_DEF                   { \
                                                    .adcBase          = MOTOR1_IC_ADC_CFG_ADDR, \
                                                    .adcResultBase    = MOTOR1_IC_ADC_RESULT_ADDR, \
                                                    .adcSOCNum        = MOTOR1_IC_SOC_NUM, \
                                                    .adcPPBNum        = MOTOR1_IC_PPB_NUM, \
                                                }
/* MOTOR2 - Ic ADC Configs */
#define MOTOR2_IC_ADC_INIT_DEF                  { \
                                                    .adcBase          = MOTOR2_IC_ADC_CFG_ADDR, \
                                                    .adcChannel       = MOTOR2_IC_CH, \
                                                    .adcSOCNum        = MOTOR2_IC_SOC_NUM, \
                                                    .adcTrigSrc       = MOTOR2_IC_SOC_TRIGGER_EVT, \
                                                }
#define MOTOR2_IC_ADC_ACQ_DEF                   { \
                                                    .adcBase          = MOTOR2_IC_ADC_CFG_ADDR, \
                                                    .adcResultBase    = MOTOR2_IC_ADC_RESULT_ADDR, \
                                                    .adcSOCNum        = MOTOR2_IC_SOC_NUM, \
                                                    .adcPPBNum        = MOTOR2_IC_PPB_NUM, \
                                                }
/* VBus ADC Configs */
#define VBUS_ADC_INIT_DEF                       { \
                                                    .adcBase          = VBUS_ADC_CFG_ADDR, \
                                                    .adcChannel       = VBUS_CH, \
                                                    .adcSOCNum        = VBUS_SOC_NUM, \
                                                    .adcTrigSrc       = VBUS_SOC_TRIGGER_EVT, \
                                                }
#define VBUS_ADC_ACQ_DEF                        { \
                                                    .adcBase          = VBUS_ADC_CFG_ADDR, \
                                                    .adcResultBase    = VBUS_ADC_RESULT_ADDR, \
                                                    .adcSOCNum        = VBUS_SOC_NUM, \
                                                    .adcPPBNum        = VBUS_PPB_NUM, \
                                                }
/* Vext1 ADC Configs */
#define VEXT1_ADC_INIT_DEF                      { \
                                                    .adcBase          = VEXT1_ADC_CFG_ADDR, \
                                                    .adcChannel       = VEXT1_CH, \
                                                    .adcSOCNum        = VEXT1_SOC_NUM, \
                                                    .adcTrigSrc       = VEXT1_SOC_TRIGGER_EVT, \
                                                }
#define VEXT1_ADC_ACQ_DEF                       { \
                                                    .adcBase          = VEXT1_ADC_CFG_ADDR, \
                                                    .adcResultBase    = VEXT1_ADC_RESULT_ADDR, \
                                                    .adcSOCNum        = VEXT1_SOC_NUM, \
                                                    .adcPPBNum        = VEXT1_PPB_NUM, \
                                                }
/* Vext2 ADC Configs */
#define VEXT2_ADC_INIT_DEF                      { \
                                                    .adcBase          = VEXT2_ADC_CFG_ADDR, \
                                                    .adcChannel       = VEXT2_CH, \
                                                    .adcSOCNum        = VEXT2_SOC_NUM, \
                                                    .adcTrigSrc       = VEXT2_SOC_TRIGGER_EVT, \
                                                }
#define VEXT2_ADC_ACQ_DEF                       { \
                                                    .adcBase          = VEXT2_ADC_CFG_ADDR, \
                                                    .adcResultBase    = VEXT2_ADC_RESULT_ADDR, \
                                                    .adcSOCNum        = VEXT2_SOC_NUM, \
                                                    .adcPPBNum        = VEXT2_PPB_NUM, \
                                                }
/* MOTOR1 & MOTOR2 - Ia ADC Global Configs */
#define MOTOR12_IA_ADC_DEF                      { \
                                                    .adcBase          = MOTOR12_IA_ADC_CFG_ADDR, \
                                                    .p_adcIni         = &adcInitList[0], \
                                                    .p_adcAcq         = &adcAcqList[0], \
                                                    .p_adcInt         = &AdcIntList[0], \
                                                }
/* MOTOR1 & MOTOR2 - Ib ADC Global Configs */
#define MOTOR12_IB_ADC_DEF                      { \
                                                    .adcBase          = MOTOR12_IB_ADC_CFG_ADDR, \
                                                    .p_adcIni         = &adcInitList[2], \
                                                    .p_adcAcq         = &adcAcqList[2], \
                                                    .p_adcInt         = NULL, \
                                                }
/* MOTOR1 & MOTOR2 - Ic ADC Global Configs */
#define MOTOR12_IC_ADC_DEF                      { \
                                                    .adcBase          = MOTOR12_IC_ADC_CFG_ADDR, \
                                                    .p_adcIni         = &adcInitList[4], \
                                                    .p_adcAcq         = &adcAcqList[4], \
                                                    .p_adcInt         = NULL, \
                                                }
/* MOTOR1 & MOTOR2 - Vbus, Vext1 &Vext2 ADC Global Configs */
#define VBUS_VEXT12_ADC_DEF                     { \
                                                    .adcBase          = VBUS_VEXT12_ADC_CFG_ADDR, \
                                                    .p_adcIni         = &adcInitList[6], \
                                                    .p_adcAcq         = &adcAcqList[6], \
                                                    .p_adcInt         = NULL, \
                                                }

/***********************************************************************
 * PWM DEFINES
 ***********************************************************************/
/** @defgroup ePWM_Module address configuration.
 *  @brief    Define the address of all the ePWM used (2 motors).
 * @{
 */
#define MOTOR12_PWMx_CMD_REG_OFFSET             0x01U // bits 31-16 of CMPx registers
#define MOTOR12_PWMx_CMD_ADDR_OFFSET            (EPWM_O_CMPA + MOTOR12_PWMx_CMD_REG_OFFSET)
#define MOTOR1_PWM1_BASE                        EPWM1_BASE
#define MOTOR1_PWM2_BASE                        EPWM2_BASE
#define MOTOR1_PWM3_BASE                        EPWM3_BASE
#define MOTOR2_PWM1_BASE                        EPWM4_BASE
#define MOTOR2_PWM2_BASE                        EPWM5_BASE
#define MOTOR2_PWM3_BASE                        EPWM6_BASE
#define MOTOR1_PWM1_CMD_ADDR                    (MOTOR1_PWM1_BASE + MOTOR12_PWMx_CMD_ADDR_OFFSET)
#define MOTOR1_PWM2_CMD_ADDR                    (MOTOR1_PWM2_BASE + MOTOR12_PWMx_CMD_ADDR_OFFSET)
#define MOTOR1_PWM3_CMD_ADDR                    (MOTOR1_PWM3_BASE + MOTOR12_PWMx_CMD_ADDR_OFFSET)
#define MOTOR2_PWM1_CMD_ADDR                    (MOTOR2_PWM1_BASE + MOTOR12_PWMx_CMD_ADDR_OFFSET)
#define MOTOR2_PWM2_CMD_ADDR                    (MOTOR2_PWM2_BASE + MOTOR12_PWMx_CMD_ADDR_OFFSET)
#define MOTOR2_PWM3_CMD_ADDR                    (MOTOR2_PWM3_BASE + MOTOR12_PWMx_CMD_ADDR_OFFSET)
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
#if (PWM_COUNTER_MODE == EPWM_COUNTER_MODE_UP_DOWN)
#define PWM_TIMEBASE_CNT                        (TBCLK_FREQ / PWM_FREQ / (2)) // PWM Period register (max value)
#else
#define PWM_TIMEBASE_CNT                        (TBCLK_FREQ / PWM_FREQ / (1)) // PWM Period register (max value)
#endif
#define PWM_TIMEBASE_HALF_CNT                   (PWM_TIMEBASE_CNT / 2)
#define PWM_INITIAL_PHASE_0                     (0)                     // PWM initial phase offset (MOTOR_1)
#define PWM_INITIAL_PHASE_90                    PWM_TIMEBASE_HALF_CNT   // PWM initial phase offset (MOTOR_2)
#define PWM_INITIAL_PHASE_180                   PWM_TIMEBASE_CNT        // PWM initial phase offset (MOTOR_2) - V2
#define PWM_INITIAL_CNT_VAL                     (0)
#define PWM_INITIAL_COMP_VAL                    PWM_TIMEBASE_CNT        // PWM initial value for capture/compare register
#define PWM_DEADBAND_RISING_EDGE_DELAY          (0) // PWM rising edge delay. ePWMxA (low to high) & ePWMxB (high to low) - Active High Complementary (AHC)
#define PWM_DEADBAND_FALLING_EDGE_DELAY         (0) // PWM falling edge delay. ePWMxA (high to low) & ePWMxB (low to high) - Active High Complementary (AHC)
#define PWM_SHADOW_MODE_ET                      EPWM_PERIOD_SHADOW_LOAD
#define PWM_SHADOW_MODE_EVENT_ET                EPWM_SHADOW_LOAD_MODE_COUNTER_SYNC
#define PWM_SHADOW_MODE_EVENT_CC                EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_AQ                EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_DB                EPWM_DB_LOAD_ON_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_DB_RED            EPWM_RED_LOAD_ON_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_DB_FED            EPWM_FED_LOAD_ON_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_EVENT_GLOBAL            EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD
#define PWM_SHADOW_MODE_ENABLE                  (1) // Enable/Disable the PWM shadow mode register.
/**
 * @}
 */

/* MOTOR1 - PWM1 Configs */
#define MOTOR1_PWM1_TIME_BASE_DEF               { \
                                                    .epwmBase         = MOTOR1_PWM1_BASE, \
                                                    .phaseCount       = PWM_INITIAL_PHASE_0, \
                                                    .syncOutputMode   = EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO, \
                                                    .syncInputSrc     = EPWM_SYNC_IN_PULSE_SRC_DISABLE, \
                                                }
#define MOTOR1_PWM1_CAPTURE_COMPARE_DEF         { \
                                                    .epwmBase         = MOTOR1_PWM1_BASE, \
                                                    .compCount        = PWM_INITIAL_COMP_VAL, \
                                                    .compModule       = EPWM_COUNTER_COMPARE_A, \
                                                }
#define MOTOR1_PWM1_ACTION_QUALIFIER_DEF        { \
                                                    .epwmBase         = MOTOR1_PWM1_BASE, \
                                                    .actionsEvt       = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), \
                                                    .epwmOutput       = EPWM_AQ_OUTPUT_A, \
                                                }
#define MOTOR1_PWM1_DEAD_BAND_DEF               { \
                                                    .epwmBase         = MOTOR1_PWM1_BASE, \
                                                    .redCount         = PWM_DEADBAND_RISING_EDGE_DELAY, \
                                                    .fedCount         = PWM_DEADBAND_FALLING_EDGE_DELAY, \
                                                    .DelayMode        = true, \
                                                }
#define MOTOR1_PWM1_ADC_SOC_EVT_TRIG_DEF        { \
                                                    .epwmBase         = MOTOR1_PWM1_BASE, \
                                                    .adcSOCEvtCount   = 1, \
                                                    .adcSOCType       = EPWM_SOC_A, \
                                                    .adcSOCSource     = EPWM_SOC_TBCTR_ZERO, \
                                                }
#define MOTOR1_PWM1_DEF                         { \
                                                    .epwmBase                   = MOTOR1_PWM1_BASE, \
                                                    .p_epwmTimeBase             = &pwmTimeBaseCfgList[0], \
                                                    .p_epwmCounterCompare       = &pwmCounterCompareCfgList[0], \
                                                    .p_epwmActionQualifier      = &pwmActionQualifierCfgList[0], \
                                                    .p_epwmDeadband             = &pwmDeadBandCfgList[0], \
                                                    .p_epwmProgChopper          = NULL, \
                                                    .p_epwmAdcSocEventTrigger   = &pwmAdcSocEventTriggerCfgList[0], \
                                                    .p_epwmInterruptsEvent      = NULL, \
                                                    .epwmAdcSocSrcExtern        = SYSCTL_ADCSOC_SRC_PWM1SOCA, \
                                                }
/* MOTOR1 - PWM2 Configs */
#define MOTOR1_PWM2_TIME_BASE_DEF               { \
                                                    .epwmBase         = MOTOR1_PWM2_BASE, \
                                                    .phaseCount       = PWM_INITIAL_PHASE_0, \
                                                    .syncOutputMode   = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE, \
                                                    .syncInputSrc     = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1, \
                                                }
#define MOTOR1_PWM2_CAPTURE_COMPARE_DEF         { \
                                                    .epwmBase         = MOTOR1_PWM2_BASE, \
                                                    .compCount        = PWM_INITIAL_COMP_VAL, \
                                                    .compModule       = EPWM_COUNTER_COMPARE_A, \
                                                }
#define MOTOR1_PWM2_ACTION_QUALIFIER_DEF        { \
                                                    .epwmBase         = MOTOR1_PWM2_BASE, \
                                                    .actionsEvt       = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), \
                                                    .epwmOutput       = EPWM_AQ_OUTPUT_A, \
                                                }
#define MOTOR1_PWM2_DEAD_BAND_DEF               { \
                                                    .epwmBase         = MOTOR1_PWM2_BASE, \
                                                    .redCount         = PWM_DEADBAND_RISING_EDGE_DELAY, \
                                                    .fedCount         = PWM_DEADBAND_FALLING_EDGE_DELAY, \
                                                    .DelayMode        = true, \
                                                }
#define MOTOR1_PWM2_DEF                         { \
                                                    .epwmBase                   = MOTOR1_PWM2_BASE, \
                                                    .p_epwmTimeBase             = &pwmTimeBaseCfgList[1], \
                                                    .p_epwmCounterCompare       = &pwmCounterCompareCfgList[1], \
                                                    .p_epwmActionQualifier      = &pwmActionQualifierCfgList[1], \
                                                    .p_epwmDeadband             = &pwmDeadBandCfgList[1], \
                                                    .p_epwmProgChopper          = NULL, \
                                                    .p_epwmAdcSocEventTrigger   = NULL, \
                                                    .p_epwmInterruptsEvent      = NULL, \
                                                    .epwmAdcSocSrcExtern        = 0U, \
                                                }
/* MOTOR1 - PWM3 Configs */
#define MOTOR1_PWM3_TIME_BASE_DEF               { \
                                                    .epwmBase         = MOTOR1_PWM3_BASE, \
                                                    .phaseCount       = PWM_INITIAL_PHASE_0, \
                                                    .syncOutputMode   = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE, \
                                                    .syncInputSrc     = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1, \
                                                }
#define MOTOR1_PWM3_CAPTURE_COMPARE_DEF         { \
                                                    .epwmBase         = MOTOR1_PWM3_BASE, \
                                                    .compCount        = PWM_INITIAL_COMP_VAL, \
                                                    .compModule       = EPWM_COUNTER_COMPARE_A, \
                                                }
#define MOTOR1_PWM3_ACTION_QUALIFIER_DEF        { \
                                                    .epwmBase         = MOTOR1_PWM3_BASE, \
                                                    .actionsEvt       = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), \
                                                    .epwmOutput       = EPWM_AQ_OUTPUT_A, \
                                                }
#define MOTOR1_PWM3_DEAD_BAND_DEF               { \
                                                    .epwmBase         = MOTOR1_PWM3_BASE, \
                                                    .redCount         = PWM_DEADBAND_RISING_EDGE_DELAY, \
                                                    .fedCount         = PWM_DEADBAND_FALLING_EDGE_DELAY, \
                                                    .DelayMode        = true, \
                                                }
#define MOTOR1_PWM3_DEF                         { \
                                                    .epwmBase                   = MOTOR1_PWM3_BASE, \
                                                    .p_epwmTimeBase             = &pwmTimeBaseCfgList[2], \
                                                    .p_epwmCounterCompare       = &pwmCounterCompareCfgList[2], \
                                                    .p_epwmActionQualifier      = &pwmActionQualifierCfgList[2], \
                                                    .p_epwmDeadband             = &pwmDeadBandCfgList[2], \
                                                    .p_epwmProgChopper          = NULL, \
                                                    .p_epwmAdcSocEventTrigger   = NULL, \
                                                    .p_epwmInterruptsEvent      = NULL, \
                                                    .epwmAdcSocSrcExtern        = 0U, \
                                                }
/* MOTOR2 - PWM1 Configs */
#define MOTOR2_PWM1_TIME_BASE_DEF               { \
                                                    .epwmBase         = MOTOR2_PWM1_BASE, \
                                                    .phaseCount       = PWM_INITIAL_PHASE_180, \
                                                    .syncOutputMode   = EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO, \
                                                    .syncInputSrc     = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1, \
                                                }
#define MOTOR2_PWM1_CAPTURE_COMPARE_DEF         { \
                                                    .epwmBase         = MOTOR2_PWM1_BASE, \
                                                    .compCount        = PWM_INITIAL_COMP_VAL, \
                                                    .compModule       = EPWM_COUNTER_COMPARE_A, \
                                                }
#define MOTOR2_PWM1_ACTION_QUALIFIER_DEF        { \
                                                    .epwmBase         = MOTOR2_PWM1_BASE, \
                                                    .actionsEvt       = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), \
                                                    .epwmOutput       = EPWM_AQ_OUTPUT_A, \
                                                }
#define MOTOR2_PWM1_DEAD_BAND_DEF               { \
                                                    .epwmBase         = MOTOR2_PWM1_BASE, \
                                                    .redCount         = PWM_DEADBAND_RISING_EDGE_DELAY, \
                                                    .fedCount         = PWM_DEADBAND_FALLING_EDGE_DELAY, \
                                                    .DelayMode        = true, \
                                                }
#define MOTOR2_PWM1_ADC_SOC_EVT_TRIG_DEF        { \
                                                    .epwmBase         = MOTOR2_PWM1_BASE, \
                                                    .adcSOCEvtCount   = 1, \
                                                    .adcSOCType       = EPWM_SOC_B, \
                                                    .adcSOCSource     = EPWM_SOC_TBCTR_ZERO, \
                                                }
#define MOTOR2_PWM1_DEF                         { \
                                                    .epwmBase                   = MOTOR2_PWM1_BASE, \
                                                    .p_epwmTimeBase             = &pwmTimeBaseCfgList[3], \
                                                    .p_epwmCounterCompare       = &pwmCounterCompareCfgList[3], \
                                                    .p_epwmActionQualifier      = &pwmActionQualifierCfgList[3], \
                                                    .p_epwmDeadband             = &pwmDeadBandCfgList[3], \
                                                    .p_epwmProgChopper          = NULL, \
                                                    .p_epwmAdcSocEventTrigger   = &pwmAdcSocEventTriggerCfgList[1], \
                                                    .p_epwmInterruptsEvent      = NULL, \
                                                    .epwmAdcSocSrcExtern        = SYSCTL_ADCSOC_SRC_PWM4SOCB, \
                                                }
/* MOTOR2 - PWM2 Configs */
#define MOTOR2_PWM2_TIME_BASE_DEF               { \
                                                    .epwmBase         = MOTOR2_PWM2_BASE, \
                                                    .phaseCount       = PWM_INITIAL_PHASE_180, \
                                                    .syncOutputMode   = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE, \
                                                    .syncInputSrc     = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1, \
                                                }
#define MOTOR2_PWM2_CAPTURE_COMPARE_DEF         { \
                                                    .epwmBase         = MOTOR2_PWM2_BASE, \
                                                    .compCount        = PWM_INITIAL_COMP_VAL, \
                                                    .compModule       = EPWM_COUNTER_COMPARE_A, \
                                                }
#define MOTOR2_PWM2_ACTION_QUALIFIER_DEF        { \
                                                    .epwmBase         = MOTOR2_PWM2_BASE, \
                                                    .actionsEvt       = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), \
                                                    .epwmOutput       = EPWM_AQ_OUTPUT_A, \
                                                }
#define MOTOR2_PWM2_DEAD_BAND_DEF               { \
                                                    .epwmBase         = MOTOR2_PWM2_BASE, \
                                                    .redCount         = PWM_DEADBAND_RISING_EDGE_DELAY, \
                                                    .fedCount         = PWM_DEADBAND_FALLING_EDGE_DELAY, \
                                                    .DelayMode        = true, \
                                                }
#define MOTOR2_PWM2_DEF                         { \
                                                    .epwmBase                   = MOTOR2_PWM2_BASE, \
                                                    .p_epwmTimeBase             = &pwmTimeBaseCfgList[4], \
                                                    .p_epwmCounterCompare       = &pwmCounterCompareCfgList[4], \
                                                    .p_epwmActionQualifier      = &pwmActionQualifierCfgList[4], \
                                                    .p_epwmDeadband             = &pwmDeadBandCfgList[4], \
                                                    .p_epwmProgChopper          = NULL, \
                                                    .p_epwmAdcSocEventTrigger   = NULL, \
                                                    .p_epwmInterruptsEvent      = NULL, \
                                                    .epwmAdcSocSrcExtern        = 0U, \
                                                }
/* MOTOR2 - PWM3 Configs */
#define MOTOR2_PWM3_TIME_BASE_DEF               { \
                                                    .epwmBase         = MOTOR2_PWM3_BASE, \
                                                    .phaseCount       = PWM_INITIAL_PHASE_180, \
                                                    .syncOutputMode   = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE, \
                                                    .syncInputSrc     = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1, \
                                                }
#define MOTOR2_PWM3_CAPTURE_COMPARE_DEF         { \
                                                    .epwmBase         = MOTOR2_PWM3_BASE, \
                                                    .compCount        = PWM_INITIAL_COMP_VAL, \
                                                    .compModule       = EPWM_COUNTER_COMPARE_A, \
                                                }
#define MOTOR2_PWM3_ACTION_QUALIFIER_DEF        { \
                                                    .epwmBase         = MOTOR2_PWM3_BASE, \
                                                    .actionsEvt       = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), \
                                                    .epwmOutput       = EPWM_AQ_OUTPUT_A, \
                                                }
#define MOTOR2_PWM3_DEAD_BAND_DEF               { \
                                                    .epwmBase         = MOTOR2_PWM3_BASE, \
                                                    .redCount         = PWM_DEADBAND_RISING_EDGE_DELAY, \
                                                    .fedCount         = PWM_DEADBAND_FALLING_EDGE_DELAY, \
                                                    .DelayMode        = true, \
                                                }
#define MOTOR2_PWM3_DEF                         { \
                                                    .epwmBase                   = MOTOR2_PWM3_BASE, \
                                                    .p_epwmTimeBase             = &pwmTimeBaseCfgList[5], \
                                                    .p_epwmCounterCompare       = &pwmCounterCompareCfgList[5], \
                                                    .p_epwmActionQualifier      = &pwmActionQualifierCfgList[5], \
                                                    .p_epwmDeadband             = &pwmDeadBandCfgList[5], \
                                                    .p_epwmProgChopper          = NULL, \
                                                    .p_epwmAdcSocEventTrigger   = NULL, \
                                                    .p_epwmInterruptsEvent      = NULL, \
                                                    .epwmAdcSocSrcExtern        = 0U, \
                                                }

/***********************************************************************
 * ENCODER DEFINES
 ***********************************************************************/
/** @defgroup eQEP module general configuration.
 *  @brief    Configure the registers associated to the eQEP modules.
 * @{
 */
#define MOTOR1_QEP_BASE                         EQEP1_BASE
#define MOTOR2_QEP_BASE                         EQEP2_BASE

//#define MOTOR1_QEP_COUNTER_ADDR                 (MOTOR1_QEP_BASE + EQEP_O_QPOSCNT)
//#define MOTOR2_QEP_COUNTER_ADDR                 (MOTOR2_QEP_BASE + EQEP_O_QPOSCNT)

#define MOTOR1_QEP_SWAP                         EQEP_CONFIG_NO_SWAP // Set to EQEP_CONFIG_SWAP if increments in the wrong direction
#define MOTOR2_QEP_SWAP                         MOTOR1_QEP_SWAP

#define QEP_EMUL_MODE                           EQEP_EMULATIONMODE_RUNFREE
#define QEP_LATCH_MODE                          (EQEP_LATCH_SW_INDEX_MARKER | EQEP_LATCH_UNIT_TIME_OUT)//(EQEP_LATCH_RISING_INDEX | EQEP_LATCH_UNIT_TIME_OUT)
#define QEP_RESET_POSITION_MODE                 EQEP_POSITION_RESET_MAX_POS
#define QEP_CAPTURE_CLOCK_DIV                   EQEP_CAPTURE_CLK_DIV_16
#define QEP_UNIT_POS_EVENT_DIV                  EQEP_UNIT_POS_EVNT_DIV_4
/**
 * @}
 */

#define MOTOR1_QEP_DEF                          { \
                                                    .eqepBase           = MOTOR1_QEP_BASE, \
                                                    .eqepMaxResolution  = MOTOR1_ENC_QUADRATURE_SCALE_MAX, \
                                                    .eqepUnitTimerPeriod= MOTOR1_ENC_SPEED_HIGH_SAMPLE_TICKS_MAX, \
                                                    .eqepConfig         = (MOTOR1_ENC_CONFIG | MOTOR1_QEP_SWAP), \
                                                    .eqepStrobeSrc      = EQEP_STROBE_OR_ADCSOCA, \
                                                }
#define MOTOR2_QEP_DEF                          { \
                                                    .eqepBase           = MOTOR2_QEP_BASE, \
                                                    .eqepMaxResolution  = MOTOR2_ENC_QUADRATURE_SCALE_MAX, \
                                                    .eqepUnitTimerPeriod= MOTOR2_ENC_SPEED_HIGH_SAMPLE_TICKS_MAX, \
                                                    .eqepConfig         = (MOTOR2_ENC_CONFIG | MOTOR2_QEP_SWAP), \
                                                    .eqepStrobeSrc      = EQEP_STROBE_OR_ADCSOCB, \
                                                }

/***********************************************************************
 * SPI DEFINES
 ***********************************************************************/
/** @defgroup SPI DRV configuration.
 *  @brief    Configure SPI module associated with DRV communication.
 * @{
 */
#define DRV_SPI_BASE                            SPIA_BASE
//#define DRV_SPI_TX_BASE_REG_ADDR                (DRV_SPI_BASE + SPI_O_TXBUF)
//#define DRV_SPI_RX_BASE_REG_ADDR                (DRV_SPI_BASE + SPI_O_RXBUF)
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

/** @defgroup Debug \& IMU SPI communication configuration.
 *  @brief    Configure debug \& IMU SPI module.
 * @{
 */
#define DBG_SPI_BASE                            SPIC_BASE
//#define DBG_SPI_TX_BASE_REG_ADDR                (DBG_SPI_BASE + SPI_O_TXBUF)
//#define DBG_SPI_RX_BASE_REG_ADDR                (DBG_SPI_BASE + SPI_O_RXBUF)
/**
 * @}
 */

/** @defgroup SPI RGB LED control.
 *  @brief    Configure master SPI module associated with RGB LED.
 * @{
 */
#define LED_SPI_BASE                            SPID_BASE
#define LED_SPI_TX_BASE_REG_ADDR                (LED_SPI_BASE + SPI_O_TXBUF)
//#define LED_SPI_RX_BASE_REG_ADDR                (LED_SPI_BASE + SPI_O_RXBUF)
/**
 * @}
 */

#define DRV_SPI_DEF                             { \
                                                    .spiBase        = DRV_SPI_BASE, \
                                                    .bitRate        = 2.0e6, \
                                                    .dataWidth      = 16U, \
                                                    .mode           = SPI_MODE_MASTER, \
                                                    .protocol       = SPI_PROT_POL0PHA0, \
                                                    .txLevel        = SPI_FIFO_TXEMPTY, \
                                                    .rxLevel        = SPI_FIFO_RXEMPTY, \
                                                    .intSrc         = 0U, \
                                                    .highSpeedMode  = false, \
                                                    .intEnable      = false, \
                                                }
#define COM_SPI_DEF                             { \
                                                    .spiBase        = COM_SPI_BASE, \
                                                    .bitRate        = 6.0e6, \
                                                    .dataWidth      = 16U, \
                                                    .mode           = SPI_MODE_SLAVE, \
                                                    .protocol       = SPI_PROT_POL0PHA1, \
                                                    .txLevel        = SPI_FIFO_TXFULL, \
                                                    .rxLevel        = SPI_FIFO_RX1, \
                                                    .intSrc         = 0U, \
                                                    .highSpeedMode  = false, \
                                                    .intEnable      = false, \
                                                }
#define DBG_SPI_DEF                             { \
                                                    .spiBase        = DBG_SPI_BASE, \
                                                    .bitRate        = 10.0e6, \
                                                    .dataWidth      = 16U, \
                                                    .mode           = SPI_MODE_MASTER, \
                                                    .protocol       = SPI_PROT_POL0PHA0, \
                                                    .txLevel        = SPI_FIFO_TX8, \
                                                    .rxLevel        = SPI_FIFO_RX8, \
                                                    .intSrc         = 0U, \
                                                    .highSpeedMode  = false, \
                                                    .intEnable      = false, \
                                                }
#define LED_SPI_DEF                             { \
                                                    .spiBase        = LED_SPI_BASE, \
                                                    .bitRate        = 2.38e6, \
                                                    .dataWidth      = 16U, \
                                                    .mode           = SPI_MODE_MASTER, \
                                                    .protocol       = SPI_PROT_POL0PHA0, \
                                                    .txLevel        = SPI_FIFO_TX6, \
                                                    .rxLevel        = SPI_FIFO_RXEMPTY, \
                                                    .intSrc         = 0U, \
                                                    .highSpeedMode  = false, \
                                                    .intEnable      = false, \
                                                }

/***********************************************************************
 * DRV DEFINES
 ***********************************************************************/
#define MOTOR1_DRV_CFG_DEF                      { \
                                                    .spiHandle          = DRV_SPI_BASE, \
                                                    .gpioNumber_CS      = MOTOR1_DRV_SPI_CS_NUM, \
                                                    .gpioNumber_EN      = MOTOR1_DRV_EN_NUM, \
                                                    .gpioNumber_FAULT   = MOTOR1_DRV_FAULT_NUM, \
                                                }
#define MOTOR2_DRV_CFG_DEF                      { \
                                                    .spiHandle          = DRV_SPI_BASE, \
                                                    .gpioNumber_CS      = MOTOR2_DRV_SPI_CS_NUM, \
                                                    .gpioNumber_EN      = MOTOR2_DRV_EN_NUM, \
                                                    .gpioNumber_FAULT   = MOTOR2_DRV_FAULT_NUM, \
                                                }

#define MOTOR1_DRV_REG_DEF                      { \
                                                    .ctrl_reg_02.CLR_FLT    = false, \
                                                    .ctrl_reg_02.BRAKE      = false, \
                                                    .ctrl_reg_02.COAST      = false, \
                                                    .ctrl_reg_02.PWM1_DIR   = false, \
                                                    .ctrl_reg_02.PWM1_COM   = false, \
                                                    .ctrl_reg_02.PWM_MODE   = DRV_PWMMODE_6, \
                                                    .ctrl_reg_02.OTW_REP    = true, \
                                                    .ctrl_reg_02.DIS_GDF    = false, \
                                                    .ctrl_reg_02.DIS_GDUV   = false, \
                                                    .ctrl_reg_02.OCP_ACT    = true, \
                                                    .ctrl_reg_03.IDRIVEN_HS = DRV_ISINK_HS_0P300_A, \
                                                    .ctrl_reg_03.IDRIVEP_HS = DRV_ISOUR_HS_0P150_A, \
                                                    .ctrl_reg_03.LOCK       = DRV_REG_UNLOCK, \
                                                    .ctrl_reg_04.IDRIVEN_LS = DRV_ISINK_LS_0P300_A, \
                                                    .ctrl_reg_04.IDRIVEP_LS = DRV_ISOUR_LS_0P150_A, \
                                                    .ctrl_reg_04.TDRIVE     = DRV_TSOUR_1000_NS, \
                                                    .ctrl_reg_04.CBC        = false, \
                                                    .ctrl_reg_05.VDS_LVL    = DRV_VDS_LVL_0P080_V, \
                                                    .ctrl_reg_05.OCP_DEG    = DRV_OCPDEG_4_US, \
                                                    .ctrl_reg_05.OCP_MODE   = DRV_LATCHED_SHUTDOWN, \
                                                    .ctrl_reg_05.DEAD_TIME  = DRV_DEADTIME_50_NS, \
                                                    .ctrl_reg_05.TRETRY     = true, \
                                                    .ctrl_reg_06.SEN_LVL    = DRV_SENLVL_0P25_V, \
                                                    .ctrl_reg_06.CSA_CAL_C  = false, \
                                                    .ctrl_reg_06.CSA_CAL_B  = false, \
                                                    .ctrl_reg_06.CSA_CAL_A  = false, \
                                                    .ctrl_reg_06.DIS_SEN    = false, \
                                                    .ctrl_reg_06.CSA_GAIN   = DRV_CSA_GAIN_REG, \
                                                    .ctrl_reg_06.LS_REF     = false, \
                                                    .ctrl_reg_06.VREF_DIV   = true, \
                                                    .ctrl_reg_06.CSA_FET    = false, \
                                                    .ctrl_reg_07.CAL_MODE   = false, \
                                                }
#define MOTOR2_DRV_REG_DEF                      { \
                                                    .ctrl_reg_02.CLR_FLT    = false, \
                                                    .ctrl_reg_02.BRAKE      = false, \
                                                    .ctrl_reg_02.COAST      = false, \
                                                    .ctrl_reg_02.PWM1_DIR   = false, \
                                                    .ctrl_reg_02.PWM1_COM   = false, \
                                                    .ctrl_reg_02.PWM_MODE   = DRV_PWMMODE_6, \
                                                    .ctrl_reg_02.OTW_REP    = true, \
                                                    .ctrl_reg_02.DIS_GDF    = false, \
                                                    .ctrl_reg_02.DIS_GDUV   = false, \
                                                    .ctrl_reg_02.OCP_ACT    = true, \
                                                    .ctrl_reg_03.IDRIVEN_HS = DRV_ISINK_HS_0P300_A, \
                                                    .ctrl_reg_03.IDRIVEP_HS = DRV_ISOUR_HS_0P150_A, \
                                                    .ctrl_reg_03.LOCK       = DRV_REG_UNLOCK, \
                                                    .ctrl_reg_04.IDRIVEN_LS = DRV_ISINK_LS_0P300_A, \
                                                    .ctrl_reg_04.IDRIVEP_LS = DRV_ISOUR_LS_0P150_A, \
                                                    .ctrl_reg_04.TDRIVE     = DRV_TSOUR_1000_NS, \
                                                    .ctrl_reg_04.CBC        = false, \
                                                    .ctrl_reg_05.VDS_LVL    = DRV_VDS_LVL_0P080_V, \
                                                    .ctrl_reg_05.OCP_DEG    = DRV_OCPDEG_4_US, \
                                                    .ctrl_reg_05.OCP_MODE   = DRV_LATCHED_SHUTDOWN, \
                                                    .ctrl_reg_05.DEAD_TIME  = DRV_DEADTIME_50_NS, \
                                                    .ctrl_reg_05.TRETRY     = true, \
                                                    .ctrl_reg_06.SEN_LVL    = DRV_SENLVL_0P25_V, \
                                                    .ctrl_reg_06.CSA_CAL_C  = false, \
                                                    .ctrl_reg_06.CSA_CAL_B  = false, \
                                                    .ctrl_reg_06.CSA_CAL_A  = false, \
                                                    .ctrl_reg_06.DIS_SEN    = false, \
                                                    .ctrl_reg_06.CSA_GAIN   = DRV_CSA_GAIN_REG, \
                                                    .ctrl_reg_06.LS_REF     = false, \
                                                    .ctrl_reg_06.VREF_DIV   = true, \
                                                    .ctrl_reg_06.CSA_FET    = false, \
                                                    .ctrl_reg_07.CAL_MODE   = false, \
                                                }

/***********************************************************************
 * DMA DEFINES
 ***********************************************************************/
/** @defgroup DMA configuration.
 *  @brief    Configure DMA channels associated to SPI communication.
 * @{
 */
#define DMA_NO_WRAP                             (0x10000)

//#define DMA_DBG_CLA2CM_BASE_ADDR                DMA_CH3_BASE
#define DMA_LED_SPI_TX_BASE_ADDR                DMA_CH4_BASE
#define DMA_COM_SPI_TX_BASE_ADDR                DMA_CH5_BASE
#define DMA_COM_SPI_RX_BASE_ADDR                DMA_CH6_BASE

//#define DMA_DBG_CLA2CM_INT                      INT_DMA_CH3
#define DMA_LED_SPI_TX_INT                      INT_DMA_CH4
#define DMA_COM_SPI_TX_INT                      INT_DMA_CH5
#define DMA_COM_SPI_RX_INT                      INT_DMA_CH6

//#define DMA_DBG_CLA2CM_TRIGGER_SRC              DMA_TRIGGER_SOFTWARE
#define DMA_LED_SPI_TX_TRIGGER_SRC              DMA_TRIGGER_SPIDTX
#define DMA_COM_SPI_TX_TRIGGER_SRC              DMA_TRIGGER_SPIBTX
#define DMA_COM_SPI_RX_TRIGGER_SRC              DMA_TRIGGER_SPIBRX

//#define DMA_DBG_CLA2CM_FULL_LENGTH              (128U)
//#define DMA_DBG_CLA2CM_BURST_EVT                (32U)
//#define DMA_DBG_CLA2CM_TRANSFER_EVT             (DMA_DBG_CLA2CM_FULL_LENGTH / DMA_DBG_CLA2CM_BURST_EVT)
//#define DMA_DBG_CLA2CM_CONFIG                   (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT)
//#if ((DMA_DBG_CLA2CM_CONFIG & DMA_MODE_DATASIZE) == DMA_CFG_SIZE_16BIT)
//#define DMA_DBG_CLA2CM_INC_STEP               (1)
//#else
//#define DMA_DBG_CLA2CM_INC_STEP               (2)
//#endif

#define DMA_LED_SPI_TX_FULL_LENGTH              LED_MSG_TX_16BIT_LENGTH
#define DMA_LED_SPI_TX_BURST_EVT                LED_MSG_TX_16BIT_LENGTH
#define DMA_LED_SPI_TX_TRANSFER_EVT             (DMA_LED_SPI_TX_FULL_LENGTH / DMA_LED_SPI_TX_BURST_EVT)
#define DMA_LED_SPI_TX_CONFIG                   (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT)
#if ((DMA_LED_SPI_TX_CONFIG & DMA_MODE_DATASIZE) == DMA_CFG_SIZE_16BIT)
#define DMA_LED_SPI_TX_INC_STEP                 (1)
#else
#define DMA_LED_SPI_TX_INC_STEP                 (2)
#endif

#define DMA_COM_SPI_TX_FULL_LENGTH              (COM_MSG_SPI_TX_16BIT_FULL_LENGTH - 1)
#define DMA_COM_SPI_TX_BURST_EVT                (8)
#define DMA_COM_SPI_TX_TRANSFER_EVT             (DMA_COM_SPI_TX_FULL_LENGTH / DMA_COM_SPI_TX_BURST_EVT)

#define DMA_COM_SPI_RX_FULL_LENGTH              COM_MSG_SPI_RX_16BIT_FULL_LENGTH
#define DMA_COM_SPI_RX_BURST_EVT                (1)
#define DMA_COM_SPI_RX_TRANSFER_EVT             (DMA_COM_SPI_RX_FULL_LENGTH / DMA_COM_SPI_RX_BURST_EVT)
#define DMA_COM_SPI_RX_CONFIG                   (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT)
#define DMA_COM_SPI_TX_CONFIG                   (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT)
#define DMA_COM_SPI_TX_CONFIG_NEW               (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT)
#if ((DMA_COM_SPI_TX_CONFIG & DMA_MODE_DATASIZE) == DMA_CFG_SIZE_16BIT)
#define DMA_COM_SPI_TX_INC_STEP                 (1)
#else
#define DMA_COM_SPI_TX_INC_STEP                 (2)
#endif
#if ((DMA_COM_SPI_RX_CONFIG & DMA_MODE_DATASIZE) == DMA_CFG_SIZE_16BIT)
#define DMA_COM_SPI_RX_INC_STEP                 (1)
#else
#define DMA_COM_SPI_RX_INC_STEP                 (2)
#endif
/**
 * @}
 */

#define DMA_LED_SPI_TX_DEF                      { \
                                                    .dmaChBase      = DMA_LED_SPI_TX_BASE_ADDR, \
                                                    .p_dstAddr      = (uint16_t *)LED_SPI_TX_BASE_REG_ADDR, \
                                                    .p_srcAddr      = (uint16_t *)&led_spi_tx[0], \
                                                    .burstSize      = DMA_LED_SPI_TX_BURST_EVT, \
                                                    .transferSize   = DMA_LED_SPI_TX_TRANSFER_EVT, \
                                                    .dstWrapSize    = DMA_NO_WRAP, \
                                                    .srcWrapSize    = DMA_NO_WRAP, \
                                                    .dstStep        = 0, \
                                                    .srcStep        = DMA_LED_SPI_TX_INC_STEP, \
                                                    .dstWrapStep    = 0, \
                                                    .srcWrapStep    = 0, \
                                                    .config         = DMA_LED_SPI_TX_CONFIG, \
                                                    .trigger        = DMA_LED_SPI_TX_TRIGGER_SRC, \
                                                    .intMode        = DMA_INT_AT_END, \
                                                    .intEnable      = false, \
                                                }
#define DMA_COM_SPI_TX_DEF                      { \
                                                    .dmaChBase      = DMA_COM_SPI_TX_BASE_ADDR, \
                                                    .p_dstAddr      = (uint16_t *)COM_SPI_TX_BASE_REG_ADDR, \
                                                    .p_srcAddr      = (uint16_t *)&com_spi_tx.msg[0].data.timeStamp, \
                                                    .burstSize      = DMA_COM_SPI_TX_BURST_EVT, \
                                                    .transferSize   = DMA_COM_SPI_TX_TRANSFER_EVT, \
                                                    .dstWrapSize    = DMA_NO_WRAP, \
                                                    .srcWrapSize    = DMA_NO_WRAP, \
                                                    .dstStep        = 0, \
                                                    .srcStep        = DMA_COM_SPI_TX_INC_STEP, \
                                                    .dstWrapStep    = 0, \
                                                    .srcWrapStep    = 0, \
                                                    .config         = DMA_COM_SPI_TX_CONFIG, \
                                                    .trigger        = DMA_COM_SPI_TX_TRIGGER_SRC, \
                                                    .intMode        = DMA_INT_AT_END, \
                                                    .intEnable      = false, \
                                                }
#define DMA_COM_SPI_RX_DEF                      { \
                                                    .dmaChBase      = DMA_COM_SPI_RX_BASE_ADDR, \
                                                    .p_dstAddr      = (uint16_t *)&com_spi_rx, \
                                                    .p_srcAddr      = (uint16_t *)COM_SPI_RX_BASE_REG_ADDR, \
                                                    .burstSize      = DMA_COM_SPI_RX_BURST_EVT, \
                                                    .transferSize   = DMA_COM_SPI_RX_TRANSFER_EVT, \
                                                    .dstWrapSize    = DMA_NO_WRAP, \
                                                    .srcWrapSize    = DMA_NO_WRAP, \
                                                    .dstStep        = DMA_COM_SPI_RX_INC_STEP, \
                                                    .srcStep        = 0, \
                                                    .dstWrapStep    = 0, \
                                                    .srcWrapStep    = 0, \
                                                    .config         = DMA_COM_SPI_RX_CONFIG, \
                                                    .trigger        = DMA_COM_SPI_RX_TRIGGER_SRC, \
                                                    .intMode        = DMA_INT_AT_END, \
                                                    .intEnable      = true, \
                                                }

/***********************************************************************
 * CLA DEFINES
 ***********************************************************************/
#define CLA_PROG_RAM                            (MEMCFG_SECT_LS3 | MEMCFG_SECT_LS4 | \
                                                 MEMCFG_SECT_LS5 | MEMCFG_SECT_LS6 | MEMCFG_SECT_LS7)
#define CLA_DATA_RAM                            (MEMCFG_SECT_LS0 | MEMCFG_SECT_LS1 | MEMCFG_SECT_LS2)

#define CLA_ADC_CALIB_TASK_FLAG                 CLA_TASKFLAG_1
#define CLA_CMD_NEW_TASK_FLAG                   CLA_TASKFLAG_2
#define CLA_MOTOR1_TASK_FLAG                    CLA_TASKFLAG_3
#define CLA_MOTOR2_TASK_FLAG                    CLA_TASKFLAG_4

#define CLA_ADC_CALIB_TASK_NUMBER               CLA_TASK_1
#define CLA_CMD_NEW_TASK_NUMBER                 CLA_TASK_2
#define CLA_MOTOR1_TASK_NUMBER                  CLA_TASK_3
#define CLA_MOTOR2_TASK_NUMBER                  CLA_TASK_4

#define CLA_ADC_CALIB_INTERRUPT_VECTOR          CLA_MVECT_1
#define CLA_CMD_NEW_INTERRUPT_VECTOR            CLA_MVECT_2
#define CLA_MOTOR1_INTERRUPT_VECTOR             CLA_MVECT_3
#define CLA_MOTOR2_INTERRUPT_VECTOR             CLA_MVECT_4

#define CLA_ADC_CALIB_TRIGGER_SRC               CLA_TRIGGER_SOFTWARE
#define CLA_CMD_NEW_TRIGGER_SRC                 CLA_TRIGGER_SOFTWARE
#define CLA_MOTOR1_TRIGGER_SRC                  CLA_TRIGGER_ADCA1
#define CLA_MOTOR2_TRIGGER_SRC                  CLA_TRIGGER_ADCA2

#define CLA_ADC_CALIB_TASK_HANDLER              &adcCalib_Task
#define CLA_CMD_NEW_TASK_HANDLER                &cmdNew_Task
#define CLA_MOTOR1_TASK_HANDLER                 &motor1_Task
#define CLA_MOTOR2_TASK_HANDLER                 &motor2_Task

#define CLA_ADC_CALIB_DEF                       { \
                                                    .progRAM        = CLA_PROG_RAM, \
                                                    .dataRAM        = CLA_DATA_RAM, \
                                                    .claTaskFlg     = CLA_ADC_CALIB_TASK_FLAG, \
                                                    .claTaskNum     = CLA_ADC_CALIB_TASK_NUMBER, \
                                                    .claIntVect     = CLA_ADC_CALIB_INTERRUPT_VECTOR, \
                                                    .claTrigSrc     = CLA_ADC_CALIB_TRIGGER_SRC, \
                                                    .p_claFunc      = CLA_ADC_CALIB_TASK_HANDLER, \
                                                    .claForceTask   = false, \
                                                }
#define CLA_CMD_NEW_DEF                         { \
                                                    .progRAM        = CLA_PROG_RAM, \
                                                    .dataRAM        = CLA_DATA_RAM, \
                                                    .claTaskFlg     = CLA_CMD_NEW_TASK_FLAG, \
                                                    .claTaskNum     = CLA_CMD_NEW_TASK_NUMBER, \
                                                    .claIntVect     = CLA_CMD_NEW_INTERRUPT_VECTOR, \
                                                    .claTrigSrc     = CLA_CMD_NEW_TRIGGER_SRC, \
                                                    .p_claFunc      = CLA_CMD_NEW_TASK_HANDLER, \
                                                    .claForceTask   = false, \
                                                }
#define CLA_MOTOR1_DEF                          { \
                                                    .progRAM        = CLA_PROG_RAM, \
                                                    .dataRAM        = CLA_DATA_RAM, \
                                                    .claTaskFlg     = CLA_MOTOR1_TASK_FLAG, \
                                                    .claTaskNum     = CLA_MOTOR1_TASK_NUMBER, \
                                                    .claIntVect     = CLA_MOTOR1_INTERRUPT_VECTOR, \
                                                    .claTrigSrc     = CLA_MOTOR1_TRIGGER_SRC, \
                                                    .p_claFunc      = CLA_MOTOR1_TASK_HANDLER, \
                                                    .claForceTask   = false, \
                                                }
#define CLA_MOTOR2_DEF                          { \
                                                    .progRAM        = CLA_PROG_RAM, \
                                                    .dataRAM        = CLA_DATA_RAM, \
                                                    .claTaskFlg     = CLA_MOTOR2_TASK_FLAG, \
                                                    .claTaskNum     = CLA_MOTOR2_TASK_NUMBER, \
                                                    .claIntVect     = CLA_MOTOR2_INTERRUPT_VECTOR, \
                                                    .claTrigSrc     = CLA_MOTOR2_TRIGGER_SRC, \
                                                    .p_claFunc      = CLA_MOTOR2_TASK_HANDLER, \
                                                    .claForceTask   = false, \
                                                }

/***********************************************************************
 * IPC DEFINES
 ***********************************************************************/
/** @defgroup Debug pins configuration.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#define CPU1TOCMMSGRAM0_BASE                    (0x20080000U)
#define IPC_CPU1_TO_CM_ADDR_TRANSLATE(addr_cpu) ((((addr_cpu) - CPUXTOCMMSGRAM0_BASE) * 2U) + CPU1TOCMMSGRAM0_BASE)

#define IPC_SYNC_FLAG                           IPC_FLAG31
#define IPC_CPU1_TO_CM_FLAG                     IPC_FLAG0
#define IPC_CM_TO_CPU1_FLAG                     IPC_FLAG0
/**
 * @}
 */

#define IPC_CPU1_2_CM_DEF                       { \
                                                    .ipcType          = IPC_CPU1_L_CM_R, \
                                                    .ipcFlag          = IPC_CPU1_TO_CM_FLAG, \
                                                    .ipcMsgQEnable    = false, \
                                                    .ipcIntEnable     = false, \
                                                }

/***********************************************************************
 * USB DEFINES
 ***********************************************************************/
// Specify the clock source for the MCAN.
//#define USB_CLOCK_SOURCE                        USB_CLOCK_SOURCE_SYS

#define USB_ALLOCATE_ON_CPU1_CORE               (0U)
#define USB_ALLOCATE_ON_CM_CORE                 (1U)

// Specify which core is used to manage the MCAN.
#define USB_ALLOCATE_ON_CORE                    USB_ALLOCATE_ON_CM_CORE

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
/**
 * @}
 */

#define CPU_TIMER0_DEF                          { \
                                                    .cpuTimerBase     = CPU_TIMER_0_BASE, \
                                                    .periodCount      = CPU_TIMER_0_PERIOD, \
                                                    .prescaler        = CPU_TIMER_0_PRESCALER, \
                                                    .intEnable        = true, \
                                                }

/***********************************************************************
 *  DEVICE INITIALIZATION DEFINES
 ***********************************************************************/
// Redefine LSPCLK to run at 200MHz (for SPI & SCI bit clocks)
#define UOMODRI_LSPCLK_FREQ                      DEVICE_SYSCLK_FREQ //DEVICE_LSPCLK_FREQ
//#undef DEVICE_LSPCLK_FREQ
//#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ / 1)

#endif /* __UOMODRI_1_HAL_DEFINES_H__ */
