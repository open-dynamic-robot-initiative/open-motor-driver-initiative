/*
 * File name: omodri_user_defines.h
 * Description: Header file containing user definitions
 */

#ifndef __UOMODRI_3_HAL_DEFINES_H__
#define __UOMODRI_3_HAL_DEFINES_H__

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

#define CANFD_TX_NUM                            (22U)
#define CANFD_TX_CFG                            GPIO_22_MCAN_TX
#define CANFD_TX_DEF                            { \
                                                    .pinNum         = CANFD_TX_NUM, \
                                                    .fctMux         = CANFD_TX_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define CANFD_RX_NUM                            (23U)
#define CANFD_RX_CFG                            GPIO_23_MCAN_RX
#define CANFD_RX_DEF                            { \
                                                    .pinNum         = CANFD_RX_NUM, \
                                                    .fctMux         = CANFD_RX_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define EXT_SPI_CS2_NUM                         (28U)
#define EXT_SPI_CS2_CFG                         GPIO_28_GPIO28
#define EXT_SPI_CS2_SET                         (HWREG(GPIODATA_BASE + GPIO_O_GPASET)   = GPIO_GPASET_GPIO28)
#define EXT_SPI_CS2_CLEAR                       (HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR) = GPIO_GPACLEAR_GPIO28)
#define EXT_SPI_CS2_TOGGLE                      (HWREG(GPIODATA_BASE + GPIO_O_GPATOGGLE)= GPIO_GPATOGGLE_GPIO28)

#define MOTOR2_DRV_SPI_CS_NUM                   (30U)
#define MOTOR2_DRV_SPI_CS_CFG                   GPIO_30_GPIO30
#define MOTOR2_DRV_SPI_CS_SET                   (HWREG(GPIODATA_BASE + GPIO_O_GPASET)   = GPIO_GPASET_GPIO30)
#define MOTOR2_DRV_SPI_CS_CLEAR                 (HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR) = GPIO_GPACLEAR_GPIO30)
#define MOTOR2_DRV_SPI_CS_TOGGLE                (HWREG(GPIODATA_BASE + GPIO_O_GPATOGGLE)= GPIO_GPATOGGLE_GPIO30)
#define MOTOR2_DRV_SPI_CS_DEF                   { \
                                                    .pinNum         = MOTOR2_DRV_SPI_CS_NUM, \
                                                    .fctMux         = MOTOR2_DRV_SPI_CS_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_DRV_EN_NUM                       (31U)
#define MOTOR2_DRV_EN_CFG                       GPIO_31_GPIO31
#define MOTOR2_DRV_EN_SET                       (HWREG(GPIODATA_BASE + GPIO_O_GPASET)   = GPIO_GPASET_GPIO31)
#define MOTOR2_DRV_EN_CLEAR                     (HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR) = GPIO_GPACLEAR_GPIO31)
#define MOTOR2_DRV_EN_TOGGLE                    (HWREG(GPIODATA_BASE + GPIO_O_GPATOGGLE)= GPIO_GPATOGGLE_GPIO31)
#define MOTOR2_DRV_EN_DEF                       { \
                                                    .pinNum         = MOTOR2_DRV_EN_NUM, \
                                                    .fctMux         = MOTOR2_DRV_EN_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define IMU_SPI_ACCEL_CS_NUM                    (32U)
#define IMU_SPI_ACCEL_CS_CFG                    GPIO_32_GPIO32
#define IMU_SPI_ACCEL_CS_SET                    (HWREG(GPIODATA_BASE + GPIO_O_GPBSET)   = GPIO_GPBSET_GPIO32)
#define IMU_SPI_ACCEL_CS_CLEAR                  (HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIO_GPBCLEAR_GPIO32)
#define IMU_SPI_ACCEL_CS_TOGGLE                 (HWREG(GPIODATA_BASE + GPIO_O_GPBTOGGLE)= GPIO_GPBTOGGLE_GPIO32)

#define IMU_IT1_NUM                             (37U)
#define IMU_IT1_CFG                             GPIO_37_GPIO37
#define IMU_IT1_READ                            (HWREG(GPIODATA_BASE + GPIO_O_GPBDAT) & GPIO_GPBDAT_GPIO37)

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

#define SELECT_PIN_1_NUM                        (45U)
#define SELECT_PIN_1_CFG                        GPIO_45_GPIO45
#define SELECT_PIN_1_READ                       (HWREG(GPIODATA_BASE + GPIO_O_GPBDAT) & GPIO_GPBDAT_GPIO45)

#define SELECT_PIN_2_NUM                        (46U)
#define SELECT_PIN_2_CFG                        GPIO_46_GPIO46
#define SELECT_PIN_2_READ                       (HWREG(GPIODATA_BASE + GPIO_O_GPBDAT) & GPIO_GPBDAT_GPIO46)

#define SELECT_PIN_3_NUM                        (47U)
#define SELECT_PIN_3_CFG                        GPIO_47_GPIO47
#define SELECT_PIN_3_READ                       (HWREG(GPIODATA_BASE + GPIO_O_GPBDAT) & GPIO_GPBDAT_GPIO47)

#define FSI_TX_D0_NUM                           (49U)
#define FSI_TX_D0_CFG                           GPIO_49_FSITXA_D0

#define FSI_TX_D1_NUM                           (50U)
#define FSI_TX_D1_CFG                           GPIO_50_FSITXA_D1

#define FSI_TX_CLK_NUM                          (51U)
#define FSI_TX_CLK_CFG                          GPIO_51_FSITXA_CLK

#define SELECT_PIN_0_NUM                        (55U)
#define SELECT_PIN_0_CFG                        GPIO_55_GPIO55
#define SELECT_PIN_0_READ                       (HWREG(GPIODATA_BASE + GPIO_O_GPBDAT) & GPIO_GPBDAT_GPIO55)

#define COM_SPI_SIMO_NUM                        (63U)
#define COM_SPI_SIMO_CFG                        GPIO_63_SPIB_SIMO
#define COM_SPI_SIMO_DEF                        { \
                                                    .pinNum         = COM_SPI_SIMO_NUM, \
                                                    .fctMux         = COM_SPI_SIMO_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define COM_SPI_SOMI_NUM                        (64U)
#define COM_SPI_SOMI_CFG                        GPIO_64_SPIB_SOMI
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

#define COM_SPI_CS_NUM                          (66U)
#define COM_SPI_CS_CFG                          GPIO_66_SPIB_STEN
#define COM_SPI_CS_READ                         (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO66)
#define COM_SPI_CS_DEF                          { \
                                                    .pinNum         = COM_SPI_CS_NUM, \
                                                    .fctMux         = COM_SPI_CS_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define USB_DETECT_NUM                          (67U)
#define USB_DETECT_CFG                          GPIO_67_GPIO67
#define USB_DETECT_READ                         (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO67)
#define USB_DETECT_DEF                          { \
                                                    .pinNum         = USB_DETECT_NUM, \
                                                    .fctMux         = USB_DETECT_CFG, \
                                                    .coreSelect     = GPIO_CORE_CM, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define ESC_PHY1_LINKSTATUS_NUM                 (68U)
#define ESC_PHY1_LINKSTATUS_CFG                 GPIO_68_ESC_PHY1_LINKSTATUS

#define IMU_EXT_DBG_SPI_SIMO_NUM                (69U)
#define IMU_EXT_DBG_SPI_SIMO_CFG                GPIO_69_SPIC_SIMO

#define IMU_EXT_DBG_SPI_SOMI_NUM                (70U)
#define IMU_EXT_DBG_SPI_SOMI_CFG                GPIO_70_SPIC_SOMI

#define IMU_EXT_DBG_SPI_CLK_NUM                 (71U)
#define IMU_EXT_DBG_SPI_CLK_CFG                 GPIO_71_SPIC_CLK

#define DBG_PIN3_NUM                            (77U)
#define DBG_PIN3_CFG                            GPIO_77_GPIO77
#define DBG_PIN3_CORE                           GPIO_CORE_CPU1
#define DBG_PIN3_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO77)
#define DBG_PIN3_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO77)
#define DBG_PIN3_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO77)
#define DBG_PIN3_READ                           (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO77)
#define DBG_PIN3_DEF                            { \
                                                    .pinNum         = DBG_PIN3_NUM, \
                                                    .fctMux         = DBG_PIN3_CFG, \
                                                    .coreSelect     = DBG_PIN3_CORE, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define DBG_PIN2_NUM                            (78U)
#define DBG_PIN2_CFG                            GPIO_78_GPIO78
#define DBG_PIN2_CORE                           GPIO_CORE_CPU1
#define DBG_PIN2_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO78)
#define DBG_PIN2_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO78)
#define DBG_PIN2_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO78)
#define DBG_PIN2_READ                           (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO78)
#define DBG_PIN2_DEF                            { \
                                                    .pinNum         = DBG_PIN2_NUM, \
                                                    .fctMux         = DBG_PIN2_CFG, \
                                                    .coreSelect     = DBG_PIN2_CORE, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define DBG_PIN1_NUM                            (81U)
#define DBG_PIN1_CFG                            GPIO_81_GPIO81
#define DBG_PIN1_CORE                           GPIO_CORE_CPU1_CLA1
#define DBG_PIN1_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO81)
#define DBG_PIN1_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO81)
#define DBG_PIN1_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO81)
#define DBG_PIN1_READ                           (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO81)
#define DBG_PIN1_DEF                            { \
                                                    .pinNum         = DBG_PIN1_NUM, \
                                                    .fctMux         = DBG_PIN1_CFG, \
                                                    .coreSelect     = DBG_PIN1_CORE, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define DBG_PIN0_NUM                            (82U)
#define DBG_PIN0_CFG                            GPIO_82_GPIO82
#define DBG_PIN0_CORE                           GPIO_CORE_CPU1_CLA1
#define DBG_PIN0_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO82)
#define DBG_PIN0_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO82)
#define DBG_PIN0_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO82)
#define DBG_PIN0_READ                           (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO82)
#define DBG_PIN0_DEF                            { \
                                                    .pinNum         = DBG_PIN0_NUM, \
                                                    .fctMux         = DBG_PIN0_CFG, \
                                                    .coreSelect     = DBG_PIN0_CORE, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_SYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define ESC_INT0_NUM                            (86U)
#define ESC_INT0_CFG                            GPIO_86_GPIO86

#define MOTOR1_DRV_EN_NUM                       (90U)
#define MOTOR1_DRV_EN_CFG                       GPIO_90_GPIO90
#define MOTOR1_DRV_EN_SET                       (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO90)
#define MOTOR1_DRV_EN_CLEAR                     (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO90)
#define MOTOR1_DRV_EN_TOGGLE                    (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO90)
#define MOTOR1_DRV_EN_DEF                       { \
                                                    .pinNum         = MOTOR1_DRV_EN_NUM, \
                                                    .fctMux         = MOTOR1_DRV_EN_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
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
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_DRV_FAULT_NUM                    (92U)
#define MOTOR1_DRV_FAULT_CFG                    GPIO_92_GPIO92
#define MOTOR1_DRV_FAULT_READ                   (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO92)
#define MOTOR1_DRV_FAULT_DEF                    { \
                                                    .pinNum         = MOTOR1_DRV_FAULT_NUM, \
                                                    .fctMux         = MOTOR1_DRV_FAULT_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_3SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_DRV_SPI_CS_NUM                   (93U)
#define MOTOR1_DRV_SPI_CS_CFG                   GPIO_93_GPIO93
#define MOTOR1_DRV_SPI_CS_SET                   (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO93)
#define MOTOR1_DRV_SPI_CS_CLEAR                 (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO93)
#define MOTOR1_DRV_SPI_CS_TOGGLE                (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO93)
#define MOTOR1_DRV_SPI_CS_DEF                   { \
                                                    .pinNum         = MOTOR1_DRV_SPI_CS_NUM, \
                                                    .fctMux         = MOTOR1_DRV_SPI_CS_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (1U), \
                                                    .intEnable      = false, \
                                                }

#define RS485_TX_EN_NUM                         (94U)
#define RS485_TX_EN_CFG                         GPIO_94_CLB_OUTPUTXBAR4
#define RS485_TX_EN_SET                         (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO94)
#define RS485_TX_EN_CLEAR                       (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO94)
#define RS485_TX_EN_TOGGLE                      (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO94)
#define RS485_TX_EN_DEF                         { \
                                                    .pinNum         = RS485_TX_EN_NUM, \
                                                    .fctMux         = RS485_TX_EN_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define CANFD_RS485_MUX_SEL_NUM                 (99U)
#define CANFD_RS485_MUX_SEL_CFG                 GPIO_99_GPIO99
#define CANFD_RS485_MUX_SEL_CANFD_MODE          (0U)
#define CANFD_RS485_MUX_SEL_RS485_MODE          (1U)
#define CANFD_RS485_MUX_SEL_SET                 (HWREG(GPIODATA_BASE + GPIO_O_GPDSET)   = GPIO_GPDSET_GPIO99)
#define CANFD_RS485_MUX_SEL_CLEAR               (HWREG(GPIODATA_BASE + GPIO_O_GPDCLEAR) = GPIO_GPDCLEAR_GPIO99)
#define CANFD_RS485_MUX_SEL_TOGGLE              (HWREG(GPIODATA_BASE + GPIO_O_GPDTOGGLE)= GPIO_GPDTOGGLE_GPIO99)
#define CANFD_RS485_MUX_SEL_DEF                 { \
                                                    .pinNum         = CANFD_RS485_MUX_SEL_NUM, \
                                                    .fctMux         = CANFD_RS485_MUX_SEL_CFG, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = CANFD_RS485_MUX_SEL_RS485_MODE, \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_ENC_CHA_NUM                      (100U)
#define MOTOR1_ENC_CHA_CFG                      GPIO_100_EQEP2_A
#define MOTOR1_ENC_CHA_DEF                      { \
                                                    .pinNum         = MOTOR1_ENC_CHA_NUM, \
                                                    .fctMux         = MOTOR1_ENC_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_ENC_CHB_NUM                      (101U)
#define MOTOR1_ENC_CHB_CFG                      GPIO_101_EQEP2_B
#define MOTOR1_ENC_CHB_DEF                      { \
                                                    .pinNum         = MOTOR1_ENC_CHB_NUM, \
                                                    .fctMux         = MOTOR1_ENC_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR1_ENC_CHI_NUM                      (103U)
#define MOTOR1_ENC_CHI_CFG                      GPIO_103_EQEP2_INDEX
#define MOTOR1_ENC_CHI_DEF                      { \
                                                    .pinNum         = MOTOR1_ENC_CHI_NUM, \
                                                    .fctMux         = MOTOR1_ENC_CHI_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_ENC_CHA_NUM                      (104U)
#define MOTOR2_ENC_CHA_CFG                      GPIO_104_EQEP3_A
#define MOTOR2_ENC_CHA_DEF                      { \
                                                    .pinNum         = MOTOR2_ENC_CHA_NUM, \
                                                    .fctMux         = MOTOR2_ENC_CHA_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_ENC_CHB_NUM                      (105U)
#define MOTOR2_ENC_CHB_CFG                      GPIO_105_EQEP3_B
#define MOTOR2_ENC_CHB_DEF                      { \
                                                    .pinNum         = MOTOR2_ENC_CHB_NUM, \
                                                    .fctMux         = MOTOR2_ENC_CHB_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define MOTOR2_ENC_CHI_NUM                      (107U)
#define MOTOR2_ENC_CHI_CFG                      GPIO_107_EQEP3_INDEX
#define MOTOR2_ENC_CHI_DEF                      { \
                                                    .pinNum         = MOTOR2_ENC_CHI_NUM, \
                                                    .fctMux         = MOTOR2_ENC_CHI_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_6SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define CANFD_SHDN_MODE_NUM                    (108U)
#define CANFD_SHDN_MODE_CFG                    GPIO_108_GPIO108
#define CANFD_SHDN_MODE_ENABLE                 (1U)
#define CANFD_SHDN_MODE_DISABLE                (0U)
#define CANFD_SHDN_MODE_SET                    (HWREG(GPIODATA_BASE + GPIO_O_GPDSET)   = GPIO_GPDSET_GPIO108)
#define CANFD_SHDN_MODE_CLEAR                  (HWREG(GPIODATA_BASE + GPIO_O_GPDCLEAR) = GPIO_GPDCLEAR_GPIO108)
#define CANFD_SHDN_MODE_TOGGLE                 (HWREG(GPIODATA_BASE + GPIO_O_GPDTOGGLE)= GPIO_GPDTOGGLE_GPIO108)
#define CANFD_SHDN_MODE_DEF                    { \
                                                    .pinNum         = CANFD_SHDN_MODE_NUM, \
                                                    .fctMux         = CANFD_SHDN_MODE_CFG, \
                                                    .pinType        = GPIO_PIN_TYPE_OD, \
                                                    .direction      = GPIO_DIR_MODE_OUT, \
                                                    .samplingInMode = GPIO_QUAL_ASYNC, \
                                                    .setValue       = CANFD_SHDN_MODE_ENABLE, \
                                                    .intEnable      = false, \
                                                }

#define FSI_RX_D0_NUM                           (115U)
#define FSI_RX_D0_CFG                           GPIO_115_FSIRXC_D0

#define FSI_RX_D1_NUM                           (116U)
#define FSI_RX_D1_CFG                           GPIO_116_FSIRXC_D1

#define FSI_RX_CLK_NUM                          (117U)
#define FSI_RX_CLK_CFG                          GPIO_117_FSIRXC_CLK

#define MOTOR2_DRV_FAULT_NUM                    (118U)
#define MOTOR2_DRV_FAULT_CFG                    GPIO_118_GPIO118
#define MOTOR2_DRV_FAULT_READ                   (HWREG(GPIODATA_BASE + GPIO_O_GPDDAT) & GPIO_GPDDAT_GPIO118)
#define MOTOR2_DRV_FAULT_DEF                    { \
                                                    .pinNum         = MOTOR2_DRV_FAULT_NUM, \
                                                    .fctMux         = MOTOR2_DRV_FAULT_CFG, \
                                                    .direction      = GPIO_DIR_MODE_IN, \
                                                    .samplingInMode = GPIO_QUAL_3SAMPLE, \
                                                    .setValue       = (0U), \
                                                    .intEnable      = false, \
                                                }

#define IMU_SPI_GYRO_CS_NUM                     (120U)
#define IMU_SPI_GYRO_CS_CFG                     GPIO_120_GPIO120
#define IMU_SPI_GYRO_CS_SET                     (HWREG(GPIODATA_BASE + GPIO_O_GPDSET)   = GPIO_GPDSET_GPIO120)
#define IMU_SPI_GYRO_CS_CLEAR                   (HWREG(GPIODATA_BASE + GPIO_O_GPDCLEAR) = GPIO_GPDCLEAR_GPIO120)
#define IMU_SPI_GYRO_CS_TOGGLE                  (HWREG(GPIODATA_BASE + GPIO_O_GPDTOGGLE)= GPIO_GPDTOGGLE_GPIO120)

#define EXT_DBG_SPI_CS1_NUM                     (125U)
#define EXT_DBG_SPI_CS1_CFG                     GPIO_125_GPIO125//SPIC_STEN
#define EXT_DBG_SPI_CS1_SET                     (HWREG(GPIODATA_BASE + GPIO_O_GPDSET)   = GPIO_GPDSET_GPIO125)
#define EXT_DBG_SPI_CS1_CLEAR                   (HWREG(GPIODATA_BASE + GPIO_O_GPDCLEAR) = GPIO_GPDCLEAR_GPIO125)
#define EXT_DBG_SPI_CS1_TOGGLE                  (HWREG(GPIODATA_BASE + GPIO_O_GPDTOGGLE)= GPIO_GPDTOGGLE_GPIO125)

#define ESC_INT1_NUM                            (128U)
#define ESC_INT1_CFG                            GPIO_128_GPIO128

#define ESC_TX1_ENA_NUM                         (129U)
#define ESC_TX1_ENA_CFG                         GPIO_129_ESC_TX1_ENA

#define ESC_TX1_CLK_NUM                         (130U)
#define ESC_TX1_CLK_CFG                         GPIO_130_ESC_TX1_CLK

#define ESC_TX1_D0_NUM                          (131U)
#define ESC_TX1_D0_CFG                          GPIO_131_ESC_TX1_DATA0

#define ESC_TX1_D1_NUM                          (132U)
#define ESC_TX1_D1_CFG                          GPIO_132_ESC_TX1_DATA1

#define ESC_TX1_D2_NUM                          (134U)
#define ESC_TX1_D2_CFG                          GPIO_134_ESC_TX1_DATA2

#define ESC_TX1_D3_NUM                          (135U)
#define ESC_TX1_D3_CFG                          GPIO_135_ESC_TX1_DATA3

#define ESC_RX1_DV_NUM                          (136U)
#define ESC_RX1_DV_CFG                          GPIO_136_ESC_RX1_DV

#define ESC_RX1_CLK_NUM                         (137U)
#define ESC_RX1_CLK_CFG                         GPIO_137_ESC_RX1_CLK

#define ESC_RX1_ERR_NUM                         (138U)
#define ESC_RX1_ERR_CFG                         GPIO_138_ESC_RX1_ERR

#define ESC_RX1_D0_NUM                          (139U)
#define ESC_RX1_D0_CFG                          GPIO_139_ESC_RX1_DATA0

#define ESC_RX1_D1_NUM                          (140U)
#define ESC_RX1_D1_CFG                          GPIO_140_ESC_RX1_DATA1

#define ESC_RX1_D2_NUM                          (141U)
#define ESC_RX1_D2_CFG                          GPIO_141_ESC_RX1_DATA2

#define ESC_RX1_D3_NUM                          (142U)
#define ESC_RX1_D3_CFG                          GPIO_142_ESC_RX1_DATA3

#define ESC_LED_LINK0_ACTIVE_NUM                (143U)
#define ESC_LED_LINK0_ACTIVE_CFG                GPIO_143_ESC_LED_LINK0_ACTIVE

#define ESC_LED_LINK1_ACTIVE_NUM                (144U)
#define ESC_LED_LINK1_ACTIVE_CFG                GPIO_144_ESC_LED_LINK1_ACTIVE

#define ESC_LED_ERR_NUM                         (145U)
#define ESC_LED_ERR_CFG                         GPIO_145_ESC_LED_ERR

#define ESC_LED_RUN_NUM                         (146U)
#define ESC_LED_RUN_CFG                         GPIO_146_ESC_LED_RUN

#define ESC_PHY0_LINKSTATUS_NUM                 (148U)
#define ESC_PHY0_LINKSTATUS_CFG                 GPIO_148_ESC_PHY0_LINKSTATUS

#define ESC_I2C_SDA_NUM                         (150U)
#define ESC_I2C_SDA_CFG                         GPIO_150_ESC_I2C_SDA

#define ESC_I2C_SCL_NUM                         (151U)
#define ESC_I2C_SCL_CFG                         GPIO_151_ESC_I2C_SCL

#define ESC_MDIO_CLK_NUM                        (152U)
#define ESC_MDIO_CLK_CFG                        GPIO_152_ESC_MDIO_CLK

#define ESC_MDIO_DATA_NUM                       (153U)
#define ESC_MDIO_DATA_CFG                       GPIO_153_ESC_MDIO_DATA

#define ESC_PHY_RESET_NUM                       (155U)
#define ESC_PHY_RESET_CFG                       GPIO_155_ESC_PHY_RESETN

#define ESC_TX0_ENA_NUM                         (156U)
#define ESC_TX0_ENA_CFG                         GPIO_156_ESC_TX0_ENA

#define ESC_TX0_CLK_NUM                         (157U)
#define ESC_TX0_CLK_CFG                         GPIO_157_ESC_TX0_CLK

#define ESC_TX0_D0_NUM                          (158U)
#define ESC_TX0_D0_CFG                          GPIO_158_ESC_TX0_DATA0

#define ESC_TX0_D1_NUM                          (159U)
#define ESC_TX0_D1_CFG                          GPIO_159_ESC_TX0_DATA1

#define ESC_TX0_D2_NUM                          (160U)
#define ESC_TX0_D2_CFG                          GPIO_160_ESC_TX0_DATA2

#define ESC_TX0_D3_NUM                          (161U)
#define ESC_TX0_D3_CFG                          GPIO_161_ESC_TX0_DATA3

#define ESC_RX0_DV_NUM                          (162U)
#define ESC_RX0_DV_CFG                          GPIO_162_ESC_RX0_DV

#define ESC_RX0_CLK_NUM                         (163U)
#define ESC_RX0_CLK_CFG                         GPIO_163_ESC_RX0_CLK

#define ESC_RX0_ERR_NUM                         (164U)
#define ESC_RX0_ERR_CFG                         GPIO_164_ESC_RX0_ERR

#define ESC_RX0_D0_NUM                          (165U)
#define ESC_RX0_D0_CFG                          GPIO_165_ESC_RX0_DATA0

#define ESC_RX0_D1_NUM                          (166U)
#define ESC_RX0_D1_CFG                          GPIO_166_ESC_RX0_DATA1

#define ESC_RX0_D2_NUM                          (167U)
#define ESC_RX0_D2_CFG                          GPIO_167_ESC_RX0_DATA2

#define ESC_RX0_D3_NUM                          (168U)
#define ESC_RX0_D3_CFG                          GPIO_168_ESC_RX0_DATA3

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
#define MOTOR1_IA_CH                            ADC_CH_ADCIN1
#define MOTOR1_IB_CH                            ADC_CH_ADCIN1
#define MOTOR1_IC_CH                            ADC_CH_ADCIN3

#define MOTOR2_IA_CH                            ADC_CH_ADCIN0
#define MOTOR2_IB_CH                            ADC_CH_ADCIN0
#define MOTOR2_IC_CH                            ADC_CH_ADCIN2

#define VBUS_CH                                 ADC_CH_ADCIN0
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
#define MOTOR12_IC_ADC_CFG_ADDR                 ADCC_BASE
#define VBUS_VEXT12_ADC_CFG_ADDR                ADCD_BASE

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
#define MOTOR12_IC_ADC_RESULT_ADDR              ADCCRESULT_BASE
#define VBUS_VEXT12_ADC_RESULT_ADDR             ADCDRESULT_BASE

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
#define MOTOR1_QEP_BASE                         EQEP2_BASE
#define MOTOR2_QEP_BASE                         EQEP3_BASE

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
 * SCI DEFINES
 ***********************************************************************/
#define COM_SCI_BASE                            SCIC_BASE
#define COM_SCI_CFG                             (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE)
#define COM_SCI_TX_FIFO_LVL                     SCI_FIFO_TX6
#define COM_SCI_RX_FIFO_LVL                     SCI_FIFO_RX8
#define SCI_INT_ALL                             (SCI_INT_RXERR | \
                                                 SCI_INT_RXRDY_BRKDT | \
                                                 SCI_INT_TXRDY | \
                                                 SCI_INT_TXFF | \
                                                 SCI_INT_RXFF)
#define SCI_INT_ALL_ERR                         (SCI_INT_RXERR | \
                                                 SCI_INT_FE | \
                                                 SCI_INT_OE | \
                                                 SCI_INT_PE)
#define SCI_INT_FIFO_RX_FE                      (0x0100)
#define SCI_INT_FIFO_RX_PE                      (0x0200)
#define COM_SCI_IT_SRC                          (0)

#define COM_SCI_DEF                             { \
                                                    .sciBase        = COM_SCI_BASE, \
                                                    .bitRate        = 12500000U, \
                                                    .sciMode        = COM_SCI_CFG, \
                                                    .txLevel        = COM_SCI_TX_FIFO_LVL, \
                                                    .rxLevel        = COM_SCI_RX_FIFO_LVL, \
                                                    .intSrc         = COM_SCI_IT_SRC, \
                                                    .intEnable      = false, \
                                                    .multiProcMode  = SCI_MULTI_PROC_NONE, \
                                                }

/***********************************************************************
 * MCAN DEFINES
 ***********************************************************************/
// Specify the clock source for the MCAN.
#define MCAN_CLOCK_SOURCE                       MCAN_CLOCK_SOURCE_SYS
// Specify which core is used to manage the MCAN.
#define MCAN_ALLOCATE_ON_CORE                   MCAN_ALLOCATE_ON_CPU1_CORE
// Enable/Disable the loopback mode
#define MCAN_LOOPBACK_MODE_CTRL                 MCAN_LOOPBACK_MODE_CTRL_DISABLE
// Define the loopback mode (Internal/external)
#define MCAN_LOOPBACK_MODE                      MCAN_LPBK_MODE_EXTERNAL

// MCAN initialization
// Enable/Disable the CAN-FD (CAN with Flexible Data-Rate) mode.
#define MCAN_FLEXIBLE_DATA_RATE_MODE            MCAN_FLEXIBLE_DATA_RATE_MODE_ENABLE
// Use single or dual data rate mode (after arbitration)
#define MCAN_BIT_RATE_SWITCHING_MODE            MCAN_BIT_RATE_SWITCHING_MODE_ENABLE
// Enable/Disable transmit pause during retry.
#define MCAN_TX_PAUSE                           MCAN_TX_PAUSE_DISABLE
// Enable/Disable Edge Filtering during Bus Integration
#define MCAN_EDGE_FILTERING_4_HARD_SYNC         MCAN_EDGE_FILTERING_4_HARD_SYNC_DISABLE
// Enable/Disable protocol exception handling.
#define MCAN_PROTOCOL_EXCEPTION_HANDLE          MCAN_PROTOCOL_EXCEPTION_HANDLE_ENABLE
// Enable/Disable automatic retransmission of messages not transmitted successfully.
#define MCAN_AUTOMATIC_RETRANSMIT               MCAN_AUTOMATIC_RETRANSMIT_DISABLE
// Enable/Disable wakeup request.
#define MCAN_WAKEUP_REQUEST                     MCAN_WAKEUP_REQUEST_ENABLE
// Enable/Disable auto-Wakeup
#define MCAN_AUTO_WAKEUP                        MCAN_AUTO_WAKEUP_ENABLE
// Enable/Disable emulation/debug suspend mode
#define MCAN_DEBUG_SUSPEND                      MCAN_DEBUG_SUSPEND_ENABLE
// Enable/Disable  Transmitter Delay Compensation
#define MCAN_TX_DELAY_COMPENSATION              MCAN_TX_DELAY_COMPENSATION_ENABLE
// structure associated to MCAN initialization
#define MCAN_INIT_PARAMS                        { \
                                                    .fdMode                 = MCAN_FLEXIBLE_DATA_RATE_MODE, \
                                                    .brsEnable              = MCAN_BIT_RATE_SWITCHING_MODE, \
                                                    .txpEnable              = MCAN_TX_PAUSE, \
                                                    .efbi                   = MCAN_EDGE_FILTERING_4_HARD_SYNC, \
                                                    .pxhddisable            = MCAN_PROTOCOL_EXCEPTION_HANDLE, \
                                                    .darEnable              = MCAN_AUTOMATIC_RETRANSMIT, \
                                                    .wkupReqEnable          = MCAN_WAKEUP_REQUEST, \
                                                    .autoWkupEnable         = MCAN_AUTO_WAKEUP, \
                                                    .emulationEnable        = MCAN_DEBUG_SUSPEND, \
                                                    .tdcEnable              = MCAN_TX_DELAY_COMPENSATION, \
                                                    .wdcPreload             = 0xFFU, \
                                                    .tdcConfig.tdcf         = 0xAU, \
                                                    .tdcConfig.tdco         = 0x6U, \
                                                }

// MCAN configuration
// Enable/Disable the Bus Monitoring Mode
#define MCAN_BUS_MONITORING_MODE                MCAN_BUS_MONITORING_MODE_DISABLE
// Enable/Disable the Restricted Operation Mode
#define MCAN_OPERATION_MODE                     MCAN_NORMAL_OPERATION_MODE_ACTIVE
// Enable/Disable the Timeout Counter
#define MCAN_TIMEOUT_COUNTER                    MCAN_TIMEOUT_COUNTER_DISABLE
// Timeout counter configuration
#define MCAN_TIMEOUT_SELECT                     MCAN_TIMEOUT_SELECT_CONT
// Timestamp source selection
#define MCAN_TIMESTAMP_COUNTER                  MCAN_TIMESTAMP_COUNTER_SET_TO_ZERO
// Accept/Reject remote Frames messages with 29-bit extended IDs
#define MCAN_REMOTE_FRAMES_W_EXT_ID             MCAN_REJECT_REMOTE_FRAMES_W_EXT_ID
// Accept/Reject remote Frames messages with 11-bit standard IDs.
#define MCAN_REMOTE_FRAMES_W_STD_ID             MCAN_REJECT_REMOTE_FRAMES_W_STD_ID
// Accept/Reject Non-matching Frames messages with 29-bit IDs
#define MCAN_NO_MATCH_EXT_ID                    MCAN_REJECT_NO_MATCH_EXT_ID
// Accept/Reject Non-matching Frames messages with 11-bit IDs
#define MCAN_NO_MATCH_STD_ID                    MCAN_REJECT_NO_MATCH_STD_ID
// structure associated to MCAN configuration
#define MCAN_CONFIG_PARAMS                      { \
                                                    .monEnable              = MCAN_BUS_MONITORING_MODE, \
                                                    .asmEnable              = MCAN_OPERATION_MODE, \
                                                    .tsPrescalar            = 0xFU, \
                                                    .tsSelect               = MCAN_TIMESTAMP_COUNTER, \
                                                    .timeoutSelect          = MCAN_TIMEOUT_SELECT, \
                                                    .timeoutPreload         = 0xFFFFU, \
                                                    .timeoutCntEnable       = MCAN_TIMEOUT_COUNTER, \
                                                    .filterConfig.rrfe      = MCAN_REMOTE_FRAMES_W_EXT_ID, \
                                                    .filterConfig.rrfs      = MCAN_REMOTE_FRAMES_W_STD_ID, \
                                                    .filterConfig.anfe      = MCAN_NO_MATCH_EXT_ID, \
                                                    .filterConfig.anfs      = MCAN_NO_MATCH_STD_ID, \
                                                }

// MCAN bit timing
// see MCAN_BitTimingCalculator.xls in C2000Ware
#define MCAN_USE_NOMINAL_RATE_1M                (1U)
#define MCAN_USE_NOMINAL_RATE_2M                (2U)
#define MCAN_USE_NOMINAL_RATE_4M                (4U)
#define MCAN_USE_NOMINAL_RATE_5M                (5U)
#define MCAN_USE_NOMINAL_RATE_8M                (8U)
#define MCAN_USE_NOMINAL_RATE_10M               (10U)
#define MCAN_USE_DATA_RATE_1M                   (1U)
#define MCAN_USE_DATA_RATE_2M                   (2U)
#define MCAN_USE_DATA_RATE_5M                   (5U)
#define MCAN_USE_DATA_RATE_8M                   (8U)
#define MCAN_USE_DATA_RATE_10M                  (10U)
#define MCAN_USE_NOMINAL_RATE                   MCAN_USE_NOMINAL_RATE_4M
#define MCAN_USE_DATA_RATE                      MCAN_USE_DATA_RATE_8M
// According MCAN_BitTimingCalculator.xls in C2000Ware here are the timing
#if (MCAN_USE_NOMINAL_RATE == MCAN_USE_NOMINAL_RATE_1M)
// 1Mb/s NOMINAL
#define MCAN_NOMINAL_BIT_RATE_PRESCALER         (10U)
#define MCAN_NOMINAL_PROPAGATION_SEG            (1U)
#define MCAN_NOMINAL_TIME_SEG1_BEFORE_SP        (9U)
#define MCAN_NOMINAL_TIME_SEG2_AFTER_SP         (9U)
#define MCAN_NOMINAL_SYNC_JUMP_WIDTH            (9U)
#elif (MCAN_USE_NOMINAL_RATE == MCAN_USE_NOMINAL_RATE_2M)
// 2Mb/s NOMINAL
#define MCAN_NOMINAL_BIT_RATE_PRESCALER         (5U)
#define MCAN_NOMINAL_PROPAGATION_SEG            (1U)
#define MCAN_NOMINAL_TIME_SEG1_BEFORE_SP        (9U)
#define MCAN_NOMINAL_TIME_SEG2_AFTER_SP         (9U)
#define MCAN_NOMINAL_SYNC_JUMP_WIDTH            (9U)
#elif (MCAN_USE_NOMINAL_RATE == MCAN_USE_NOMINAL_RATE_4M)
// 4Mb/s NOMINAL
#define MCAN_NOMINAL_BIT_RATE_PRESCALER         (2U)
#define MCAN_NOMINAL_PROPAGATION_SEG            (1U)
#define MCAN_NOMINAL_TIME_SEG1_BEFORE_SP        (11U)
#define MCAN_NOMINAL_TIME_SEG2_AFTER_SP         (12U)
#define MCAN_NOMINAL_SYNC_JUMP_WIDTH            (11U)
#elif (MCAN_USE_NOMINAL_RATE == MCAN_USE_NOMINAL_RATE_5M)
// 5Mb/s NOMINAL
#define MCAN_NOMINAL_BIT_RATE_PRESCALER         (2U)
#define MCAN_NOMINAL_PROPAGATION_SEG            (1U)
#define MCAN_NOMINAL_TIME_SEG1_BEFORE_SP        (9U)
#define MCAN_NOMINAL_TIME_SEG2_AFTER_SP         (9U)
#define MCAN_NOMINAL_SYNC_JUMP_WIDTH            (9U)
#elif (MCAN_USE_NOMINAL_RATE == MCAN_USE_NOMINAL_RATE_8M)
// 8Mb/s NOMINAL
#define MCAN_NOMINAL_BIT_RATE_PRESCALER         (1U)
#define MCAN_NOMINAL_PROPAGATION_SEG            (1U)
#define MCAN_NOMINAL_TIME_SEG1_BEFORE_SP        (11U)
#define MCAN_NOMINAL_TIME_SEG2_AFTER_SP         (12U)
#define MCAN_NOMINAL_SYNC_JUMP_WIDTH            (11U)
#elif (MCAN_USE_NOMINAL_RATE == MCAN_USE_NOMINAL_RATE_10M)
// 10Mb/s NOMINAL
#define MCAN_NOMINAL_BIT_RATE_PRESCALER         (1U)
#define MCAN_NOMINAL_PROPAGATION_SEG            (1U)
#define MCAN_NOMINAL_TIME_SEG1_BEFORE_SP        (9U)
#define MCAN_NOMINAL_TIME_SEG2_AFTER_SP         (9U)
#define MCAN_NOMINAL_SYNC_JUMP_WIDTH            (9U)
#endif

#if (MCAN_USE_DATA_RATE == MCAN_USE_DATA_RATE_1M)
// 1Mb/s DATA
#define MCAN_DATA_BIT_RATE_PRESCALER            (10U)
#define MCAN_DATA_PROPAGATION_SEG               (1U)
#define MCAN_DATA_TIME_SEG1_BEFORE_SP           (9U)
#define MCAN_DATA_TIME_SEG2_AFTER_SP            (9U)
#define MCAN_DATA_SYNC_JUMP_WIDTH               (9U)
#elif (MCAN_USE_DATA_RATE == MCAN_USE_DATA_RATE_2M)
// 2Mb/s DATA
#define MCAN_DATA_BIT_RATE_PRESCALER            (5U)
#define MCAN_DATA_PROPAGATION_SEG               (1U)
#define MCAN_DATA_TIME_SEG1_BEFORE_SP           (9U)
#define MCAN_DATA_TIME_SEG2_AFTER_SP            (9U)
#define MCAN_DATA_SYNC_JUMP_WIDTH               (9U)
#elif (MCAN_USE_DATA_RATE == MCAN_USE_DATA_RATE_5M)
// 5Mb/s DATA
#define MCAN_DATA_BIT_RATE_PRESCALER            (2U)
#define MCAN_DATA_PROPAGATION_SEG               (1U)
#define MCAN_DATA_TIME_SEG1_BEFORE_SP           (9U)
#define MCAN_DATA_TIME_SEG2_AFTER_SP            (9U)
#define MCAN_DATA_SYNC_JUMP_WIDTH               (9U)
#elif (MCAN_USE_DATA_RATE == MCAN_USE_DATA_RATE_8M)
// 8Mb/s DATA
#define MCAN_DATA_BIT_RATE_PRESCALER            (1U)
#define MCAN_DATA_PROPAGATION_SEG               (1U)
#define MCAN_DATA_TIME_SEG1_BEFORE_SP           (11U)
#define MCAN_DATA_TIME_SEG2_AFTER_SP            (12U)
#define MCAN_DATA_SYNC_JUMP_WIDTH               (11U)
#elif (MCAN_USE_DATA_RATE == MCAN_USE_DATA_RATE_10M)
// 10Mb/s DATA
#define MCAN_DATA_BIT_RATE_PRESCALER            (1U)
#define MCAN_DATA_PROPAGATION_SEG               (1U)
#define MCAN_DATA_TIME_SEG1_BEFORE_SP           (9U)
#define MCAN_DATA_TIME_SEG2_AFTER_SP            (9U)
#define MCAN_DATA_SYNC_JUMP_WIDTH               (9U)
#endif
// Output Nominal Bit Timing
#define MCAN_NOMINAL_BIT_RATE_PRESCALER_REG     (MCAN_NOMINAL_BIT_RATE_PRESCALER - 1U)
#define MCAN_NOMINAL_TIME_SEG1_BEFORE_SP_REG    (MCAN_NOMINAL_TIME_SEG1_BEFORE_SP + \
                                                 MCAN_NOMINAL_PROPAGATION_SEG - 1U)
#define MCAN_NOMINAL_TIME_SEG2_AFTER_SP_REG     (MCAN_NOMINAL_TIME_SEG2_AFTER_SP - 1U)
#define MCAN_NOMINAL_SYNC_JUMP_WIDTH_REG        (MCAN_NOMINAL_SYNC_JUMP_WIDTH - 1U)
// Output Data Bit Timing
#define MCAN_DATA_BIT_RATE_PRESCALER_REG        (MCAN_DATA_BIT_RATE_PRESCALER - 1U)
#define MCAN_DATA_TIME_SEG1_BEFORE_SP_REG       (MCAN_DATA_TIME_SEG1_BEFORE_SP + \
                                                 MCAN_DATA_PROPAGATION_SEG - 1U)
#define MCAN_DATA_TIME_SEG2_AFTER_SP_REG        (MCAN_DATA_TIME_SEG2_AFTER_SP - 1U)
#define MCAN_DATA_SYNC_JUMP_WIDTH_REG           (MCAN_DATA_SYNC_JUMP_WIDTH - 1U)
// structure associated to MCAN bit timing
#define MCAN_BIT_TIMING_PARAMS                  { \
                                                    .nomRatePrescalar       = MCAN_NOMINAL_BIT_RATE_PRESCALER_REG, \
                                                    .nomTimeSeg1            = MCAN_NOMINAL_TIME_SEG1_BEFORE_SP_REG, \
                                                    .nomTimeSeg2            = MCAN_NOMINAL_TIME_SEG2_AFTER_SP_REG, \
                                                    .nomSynchJumpWidth      = MCAN_NOMINAL_SYNC_JUMP_WIDTH_REG, \
                                                    .dataRatePrescalar      = MCAN_DATA_BIT_RATE_PRESCALER_REG, \
                                                    .dataTimeSeg1           = MCAN_DATA_TIME_SEG1_BEFORE_SP_REG, \
                                                    .dataTimeSeg2           = MCAN_DATA_TIME_SEG2_AFTER_SP_REG, \
                                                    .dataSynchJumpWidth     = MCAN_DATA_SYNC_JUMP_WIDTH_REG, \
                                                }

// MCAN Message RAM Sections configuration
// Tx FIFO/Queue Mode
#define MCAN_TX_BUFFER_MODE                     MCAN_TX_FIFO_OP
// RX FIFO_0 full management - Blocking or overwrite
#define MCAN_RX_FIFO0_OP_MODE                   MCAN_RX_FIFO0_OVERWRITE_MODE
// RX FIFO_1 full management - Blocking or overwrite
#define MCAN_RX_FIFO1_OP_MODE                   MCAN_RX_FIFO1_OVERWRITE_MODE
// Tx Event FIFO - Level for Tx Event FIFO watermark interrupt
#define MCAN_TX_EVENT_FIFO_WATERMARK_IT         (0U)
// Rx FIFO0 - Level for Rx FIFO 0 watermark interrupt
#define MCAN_RX_FIFO0_WATERMARK_IT              (0U)
// Rx FIFO1 - Level for Rx FIFO 1 watermark interrupt
#define MCAN_RX_FIFO1_WATERMARK_IT              (0U)
#define MCAN_RX_LINE_SRC_IT                     (MCAN_INTR_LINE_NUM_1)
#if (MCAN_RX_LINE_SRC_IT == MCAN_INTR_LINE_NUM_1)
#define MCAN_RX_LINE_IT_CLEAR                   (MCAN_IT_LINE_1_CLEAR)
#elif (MCAN_RX_LINE_SRC_IT == MCAN_INTR_LINE_NUM_0)
#define MCAN_RX_LINE_IT_CLEAR                   (MCAN_IT_LINE_0_CLEAR)
#else
#define MCAN_RX_LINE_IT_CLEAR                   (MCAN_IT_EXT_TS_CLEAR)
#endif

// Standard ID - Number of standard Message ID filter elements / Memory space used.
#define MCAN_STD_ID_FILTER_NUM                  (1U)
#define MCAN_STD_ID_FILTER_MEM_SPACE            (MCAN_STD_ID_FILTER_NUM * MCANSS_STD_ID_FILTER_SIZE_WORDS * 4U)
// Extended ID - Number of extended Message ID filter elements / Memory space used.
#define MCAN_EXT_ID_FILTER_NUM                  (0U)
#define MCAN_EXT_ID_FILTER_MEM_SPACE            (MCAN_EXT_ID_FILTER_NUM * MCANSS_EXT_ID_FILTER_SIZE_WORDS * 4U)
// RX FIFO_0 - Number of Rx FIFO_0 elements / Memory space used.
#define MCAN_RX_FIFO0_NUM                       (0U)
#define MCAN_RX_FIFO0_ELEM_SIZE                 (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_RX_FIFO0_MEM_SPACE                 (MCAN_RX_FIFO0_NUM * MCAN_STRUCT_SIZE(MCAN_RX_FIFO0_ELEM_SIZE))
// RX FIFO_1 - Number of Rx FIFO_1 elements / Memory space used.
#define MCAN_RX_FIFO1_NUM                       (0U)
#define MCAN_RX_FIFO1_ELEM_SIZE                 (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_RX_FIFO1_MEM_SPACE                 (MCAN_RX_FIFO1_NUM * MCAN_STRUCT_SIZE(MCAN_RX_FIFO1_ELEM_SIZE))
// Rx Buffer - Number of Rx buffer elements / Memory space used.
#define MCAN_RX_BUFFER_NUM                      (1U)
#define MCAN_RX_BUFFER_ELEM_SIZE                (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_RX_BUFFER_MEM_SPACE                (MCAN_RX_BUFFER_NUM * MCAN_STRUCT_SIZE(MCAN_RX_BUFFER_ELEM_SIZE))
// Tx Buffer - Number of Tx buffer elements / Memory space used.
#define MCAN_TX_BUFFER_NUM                      (2U)
#define MCAN_TX_FIFO_QUEUE_NUM                  (0U)
#define MCAN_TX_BUFFER_AND_FIFO_NUM             (MCAN_TX_BUFFER_NUM + MCAN_TX_FIFO_QUEUE_NUM)
#define MCAN_TX_BUFFER_ELEM_SIZE                (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_TX_BUFFER_MEM_SPACE                (MCAN_TX_BUFFER_AND_FIFO_NUM * MCAN_STRUCT_SIZE(MCAN_TX_BUFFER_ELEM_SIZE))
// TX EVENTS Buffer - Number of TX event buffer elements / Memory space used.
#define MCAN_TX_EVENT_FIFO_NUM                  (0U)
// Standard ID Filter List Start Address
#define MCAN_STD_ID_FILT_START_ADDR             (0x0U)
// Extended ID Filter List Start Address
#define MCAN_EXT_ID_FILT_START_ADDR             (MCAN_STD_ID_FILT_START_ADDR+ MCAN_STD_ID_FILTER_MEM_SPACE)
// Rx FIFO0 Start Address
#define MCAN_RX_FIFO0_START_ADDR                (MCAN_EXT_ID_FILT_START_ADDR+ MCAN_EXT_ID_FILTER_MEM_SPACE)
// Rx FIFO1 Start Address
#define MCAN_RX_FIFO1_START_ADDR                (MCAN_RX_FIFO0_START_ADDR   + MCAN_RX_FIFO0_MEM_SPACE)
// Rx Buffer Start Address
#define MCAN_RX_BUFFER_START_ADDR               (MCAN_RX_FIFO1_START_ADDR   + MCAN_RX_FIFO1_MEM_SPACE)
// Tx Buffers Start Address
#define MCAN_TX_BUFFER_START_ADDR               (MCAN_RX_BUFFER_START_ADDR  + MCAN_RX_BUFFER_MEM_SPACE)
// Tx Event FIFO Start Address
#define MCAN_TX_EVENT_FIFO_START_ADDR           (MCAN_TX_BUFFER_START_ADDR  + MCAN_TX_BUFFER_MEM_SPACE)
// structure associated to MCAN Message RAM Sections configuration
#define MCAN_MSG_RAM_CONFIG_PARAMS              { \
                                                    .flssa                  = MCAN_STD_ID_FILT_START_ADDR, \
                                                    .flesa                  = MCAN_EXT_ID_FILT_START_ADDR, \
                                                    .rxFIFO0startAddr       = MCAN_RX_FIFO0_START_ADDR, \
                                                    .rxFIFO1startAddr       = MCAN_RX_FIFO1_START_ADDR, \
                                                    .rxBufStartAddr         = MCAN_RX_BUFFER_START_ADDR, \
                                                    .txStartAddr            = MCAN_TX_BUFFER_START_ADDR, \
                                                    .txEventFIFOStartAddr   = MCAN_TX_EVENT_FIFO_START_ADDR, \
                                                    \
                                                    .lss                    = MCAN_STD_ID_FILTER_NUM, \
                                                    .lse                    = MCAN_EXT_ID_FILTER_NUM, \
                                                    .rxFIFO0size            = MCAN_RX_FIFO0_NUM, \
                                                    .rxFIFO1size            = MCAN_RX_FIFO1_NUM, \
                                                    .txBufNum               = MCAN_TX_BUFFER_NUM, \
                                                    .txEventFIFOSize        = MCAN_TX_EVENT_FIFO_NUM, \
                                                    \
                                                    .rxFIFO0ElemSize        = MCAN_RX_FIFO0_ELEM_SIZE, \
                                                    .rxFIFO1ElemSize        = MCAN_RX_FIFO1_ELEM_SIZE, \
                                                    .rxBufElemSize          = MCAN_RX_BUFFER_ELEM_SIZE, \
                                                    .txFIFOSize             = MCAN_TX_FIFO_QUEUE_NUM, \
                                                    .txBufElemSize          = MCAN_TX_BUFFER_ELEM_SIZE, \
                                                    \
                                                    .rxFIFO0OpMode          = MCAN_RX_FIFO0_OP_MODE, \
                                                    .rxFIFO1OpMode          = MCAN_RX_FIFO1_OP_MODE, \
                                                    .txBufMode              = MCAN_TX_BUFFER_MODE, \
                                                    \
                                                    .txEventFIFOWaterMark   = MCAN_TX_EVENT_FIFO_WATERMARK_IT, \
                                                    .rxFIFO0waterMark       = MCAN_RX_FIFO0_WATERMARK_IT, \
                                                    .rxFIFO1waterMark       = MCAN_RX_FIFO1_WATERMARK_IT, \
                                                }

/* filter for Rx Buffer #0
 * standard ID that filter will accept (UID_unique_8b).
 * SFID2[10, 9] -> Received message stored in an Rx buffer.
 * SFID2[0,5]   -> Index of the dedicated Rx buffer.
 * Store message into dedicated Rx buffer if filter matches.
 * Use range mode to define valid ID (is ignored). Filtering to store into dedicated RX BUFFER.
 */
#define MCAN_STD_ID_FLT_RX_BUF_0_ID             CANFD_ID(COM_MSG_CANFD_RX_SLAVE_ID, COM_MSG_CANFD_SUB_ID_CMD)
#define MCAN_STD_ID_FLT_RX_BUF_0_NUM            (CANFD_OP_IN_RX_BUF | MCAN_RX_BUF_0_INDEX)
#define MCAN_STD_ID_FLT_RX_BUF_0_ELEM           (MCAN_STDFILTEC_RXBUFF)
#define MCAN_STD_ID_FLT_RX_BUF_0_TYPE           (MCAN_STDFILT_RANGE)
#define MCAN_STD_ID_FLT_RX_BUF_0_CFG            { \
                                                    .sfid1                  = MCAN_STD_ID_FLT_RX_BUF_0_ID, \
                                                    .sfid2                  = MCAN_STD_ID_FLT_RX_BUF_0_NUM, \
                                                    .sfec                   = MCAN_STD_ID_FLT_RX_BUF_0_ELEM, \
                                                    .sft                    = MCAN_STD_ID_FLT_RX_BUF_0_TYPE, \
                                                }

//#define MCAN_STD_ID_FLT_RX_FIFO_0_ID_FILTER     CANFD_ID(CANFD_SLV_ID, CANFD_SLV_CMD_MSG)
//#define MCAN_STD_ID_FLT_RX_FIFO_0_ID_MASK       (0x7FFU)
//#define MCAN_STD_ID_FLT_RX_FIFO_0_ELEM          (MCAN_STDFILTEC_REJECT)
//#define MCAN_STD_ID_FLT_RX_FIFO_0_TYPE          (MCAN_STDFILT_CLASSIC)
//#define MCAN_STD_ID_FLT_RX_FIFO_0_CFG           { \
//                                                    .sfid1                  = MCAN_STD_ID_FLT_RX_FIFO_0_ID_FILTER, \
//                                                    .sfid2                  = MCAN_STD_ID_FLT_RX_FIFO_0_ID_MASK, \
//                                                    .sfec                   = MCAN_STD_ID_FLT_RX_FIFO_0_ELEM, \
//                                                    .sft                    = MCAN_STD_ID_FLT_RX_FIFO_0_TYPE, \
//                                                }

#define MCAN_STD_ID_FILTER_CFG                  { \
                                                    MCAN_STD_ID_FLT_RX_BUF_0_CFG, \
                                                }
#define MCAN_EXT_ID_FILTER_CFG                  { \
                                                }

// Structure associated to MCAN HAL configuration.
#define COM_MCAN_DEF                            { \
                                                    .mCanBase               = MCANA_DRIVER_BASE, \
                                                    .initParams             = MCAN_INIT_PARAMS, \
                                                    .configParams           = MCAN_CONFIG_PARAMS, \
                                                    .bitTimesParams         = MCAN_BIT_TIMING_PARAMS, \
                                                }

/***********************************************************************
 * SPI DEFINES
 ***********************************************************************/
/** @defgroup SPI DRV configuration.
 *  @brief    Configure SPI module associated with DRV communication.
 * @{
 */
#define DRV_SPI_BASE                            SPIA_BASE
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
/**
 * @}
 */

/** @defgroup SPI RGB LED control.
 *  @brief    Configure master SPI module associated with RGB LED.
 * @{
 */
#define LED_SPI_BASE                            SPID_BASE
#define LED_SPI_TX_BASE_REG_ADDR                (LED_SPI_BASE + SPI_O_TXBUF)
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
                                                    .txLevel        = SPI_FIFO_TXFULL, \
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

#define DMA_DBG_CLA2CM_M1_BASE_ADDR             DMA_CH2_BASE
#define DMA_DBG_CLA2CM_M2_BASE_ADDR             DMA_CH3_BASE
#define DMA_LED_SPI_TX_BASE_ADDR                DMA_CH4_BASE
#define DMA_COM_SPI_TX_BASE_ADDR                DMA_CH5_BASE
#define DMA_COM_SPI_RX_BASE_ADDR                DMA_CH6_BASE

#define DMA_DBG_CLA2CM_M1_INT                   INT_DMA_CH2
#define DMA_DBG_CLA2CM_M2_INT                   INT_DMA_CH3
#define DMA_LED_SPI_TX_INT                      INT_DMA_CH4
#define DMA_COM_SPI_TX_INT                      INT_DMA_CH5
#define DMA_COM_SPI_RX_INT                      INT_DMA_CH6

#define DMA_DBG_CLA2CM_TRIGGER_SRC              DMA_TRIGGER_SOFTWARE
//#define DMA_DBG_CLA2CM_M2_TRIGGER_SRC           DMA_TRIGGER_SOFTWARE
#define DMA_LED_SPI_TX_TRIGGER_SRC              DMA_TRIGGER_SPIDTX
#define DMA_COM_SPI_TX_TRIGGER_SRC              DMA_TRIGGER_SPIBTX
#define DMA_COM_SPI_RX_TRIGGER_SRC              DMA_TRIGGER_SPIBRX

#define DMA_DBG_CLA2CM_FULL_LENGTH              (128U)
#define DMA_DBG_CLA2CM_BURST_EVT                (32U)
#define DMA_DBG_CLA2CM_TRANSFER_EVT             (DMA_DBG_CLA2CM_FULL_LENGTH / DMA_DBG_CLA2CM_BURST_EVT)
#define DMA_DBG_CLA2CM_CONFIG                   (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT)
#if ((DMA_DBG_CLA2CM_CONFIG & DMA_MODE_DATASIZE) == DMA_CFG_SIZE_16BIT)
#define DMA_DBG_CLA2CM_INC_STEP               (1)
#else
#define DMA_DBG_CLA2CM_INC_STEP               (2)
#endif

#define DMA_LED_SPI_TX_FULL_LENGTH              LED_MSG_TX_16BIT_LENGTH
#define DMA_LED_SPI_TX_BURST_EVT                LED_MSG_TX_16BIT_LENGTH//(LED_MSG_TX_16BIT_LENGTH / 2)
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
#define DMA_COM_SPI_TX_CONFIG                   (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT)
#define DMA_COM_SPI_RX_CONFIG                   (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT)
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

#define DMA_DBG_CLA2CM_M1_DEF                   { \
                                                    .dmaChBase      = DMA_DBG_CLA2CM_M1_BASE_ADDR, \
                                                    .p_dstAddr      = (debug_t *) &dbg_cpu2cm_m1, \
                                                    .p_srcAddr      = (debug_t *) &dbg_cla2dma_mx, \
                                                    .burstSize      = DMA_DBG_CLA2CM_BURST_EVT, \
                                                    .transferSize   = DMA_DBG_CLA2CM_TRANSFER_EVT, \
                                                    .dstWrapSize    = DMA_NO_WRAP, \
                                                    .srcWrapSize    = DMA_NO_WRAP, \
                                                    .dstStep        = DMA_DBG_CLA2CM_INC_STEP, \
                                                    .srcStep        = DMA_DBG_CLA2CM_INC_STEP, \
                                                    .dstWrapStep    = 0, \
                                                    .srcWrapStep    = 0, \
                                                    .config         = DMA_DBG_CLA2CM_CONFIG, \
                                                    .trigger        = DMA_DBG_CLA2CM_TRIGGER_SRC, \
                                                    .intMode        = DMA_INT_AT_END, \
                                                    .intEnable      = false, \
                                                }
#define DMA_DBG_CLA2CM_M2_DEF                   { \
                                                    .dmaChBase      = DMA_DBG_CLA2CM_M2_BASE_ADDR, \
                                                    .p_dstAddr      = (debug_t *) &dbg_cpu2cm_m2, \
                                                    .p_srcAddr      = (debug_t *) &dbg_cla2dma_mx, \
                                                    .burstSize      = DMA_DBG_CLA2CM_BURST_EVT, \
                                                    .transferSize   = DMA_DBG_CLA2CM_TRANSFER_EVT, \
                                                    .dstWrapSize    = DMA_NO_WRAP, \
                                                    .srcWrapSize    = DMA_NO_WRAP, \
                                                    .dstStep        = DMA_DBG_CLA2CM_INC_STEP, \
                                                    .srcStep        = DMA_DBG_CLA2CM_INC_STEP, \
                                                    .dstWrapStep    = 0, \
                                                    .srcWrapStep    = 0, \
                                                    .config         = DMA_DBG_CLA2CM_CONFIG, \
                                                    .trigger        = DMA_DBG_CLA2CM_TRIGGER_SRC, \
                                                    .intMode        = DMA_INT_AT_END, \
                                                    .intEnable      = false, \
                                                }
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
#define CLA_BKG_DEF                             { \
                                                    .progRAM        = CLA_PROG_RAM, \
                                                    .dataRAM        = CLA_DATA_RAM, \
                                                    .claTaskFlg     = CLA_TASKFLAG_8, \
                                                    .claTaskNum     = CLA_TASK_8, \
                                                    .claIntVect     = CLA_MVECT_8, \
                                                    .claTrigSrc     = CLA_TRIGGER_SOFTWARE, \
                                                    .p_claFunc      = &Cla1Task8, \
                                                    .claForceTask   = true, \
                                                }

/***********************************************************************
 * CLB DEFINES
 ***********************************************************************/
//#define CLBCLK_SRC                              DEVICE_SYSCLK_FREQ AUXPLLCLK
#define CLBCLK_FREQ                             EPWMCLK_FREQ
//#define CLB_GPREG_ADDR_VALID                    (1 << 3)
#define CLB_GPREG_RS485_TX_ENABLE               (1 << 1)
#define CLB_GP_REG_ADDR                         (RS485_RX_TX_CLB_EOT_BASE + CLB_LOGICCTL + CLB_O_GP_REG)
#define CLB_INT_REG_ADDR                        (RS485_RX_TX_CLB_EOT_BASE + CLB_LOGICCTL + CLB_O_INTR_TAG_REG)

#define CLB_INT_RS485_RX_ADDRESS_1_VALID        (0x10)
#define CLB_INT_RS485_RX_ADDRESS_2_VALID        (0x20)
#define CLB_INT_RS485_RX_END_OF_RECEPTION       (0x01)

#define CLB_RS485_DEF                           { \
                                                    .clb_Addr1Spec     = RS485_CLB_FORCE_ADDRESS, \
                                                    .clb_Addr2Spec     = RS485_CLB_USE_NO_ADDRESS, \
                                                    .clb_Addr1Force    = 0x3F, \
                                                    .clb_Addr2Force    = 0, \
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
#define UOMODRI_LSPCLK_FREQ                      DEVICE_SYSCLK_FREQ

#endif /* __UOMODRI_3_HAL_DEFINES_H__ */
