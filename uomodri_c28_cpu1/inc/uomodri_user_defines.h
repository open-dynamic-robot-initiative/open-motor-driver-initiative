/*
 * File name: omodri_user_defines.h
 * Description: Header file containing user definitions
 */

#ifndef __UOMODRI_USER_DEFINES_H__
#define __UOMODRI_USER_DEFINES_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include <daisy_chain.h>
#include "device.h"
#include "drv8353.h"


/***********************************************************************
 * GLOBAL MACROS
 ***********************************************************************/
/** @defgroup DEV_defines
 *  @brief    Special configurations definitions.
 * @{
 */
#define USE_FCT                                 (0)

#define USE_UOMODRI_V1                          (1)
#define USE_UOMODRI_V2                          (2)
#define USE_UOMODRI_V3                          (3)
#define USE_UOMODRI_SA_V1                       (4)
#define USE_UOMODRI_REV                         USE_UOMODRI_V3
#if (USE_UOMODRI_REV == USE_UOMODRI_SA_V1)
#define USE_DUAL_AXIS_CONFIG                    (0)
#else
#define USE_DUAL_AXIS_CONFIG                    (1)
#endif
//#ifndef __TMS320C28XX_CLA__
//#define __TMS320C28XX_CLA__
//#error "__TMS320C28XX_CLA__ must be removed for compilation."
//#endif
// CM peripherals ENABLE/DISABLE
#define USB_BUS_ENABLE                          (0)
#define UART_BUS_ENABLE                         (1)
//#if ((USB_BUS_ENABLE) || (UART_BUS_ENABLE)) && (defined CPU1)
//#error "In order to use USB or UART, CM must be active."
//#endif
#if (UART_BUS_ENABLE) && (USB_BUS_ENABLE)
#error "USB & UART share the same pins. They cannot run @ the same time"
#endif
// Communication peripherals ENABLE/DISABLE
#define CAN_FD_BUS_ENABLE                       (0)
#define RS485_BUS_ENABLE                        (0)
#define SPI_BUS_ENABLE                          (1)
#if (RS485_BUS_ENABLE) && (CAN_FD_BUS_ENABLE)
#error "RS-485 & CAN share the same pins. They cannot run @ the same time"
//#elif ((RS485_BUS_ENABLE) || (CAN_BUS_ENABLE) ) && (SPI_BUS_ENABLE)
//#error "RS-485, CAN & SPI share the same data. They cannot run @ the same time"
#endif


#ifdef DEBUG
#define DBG_CLA_M1_ISR                          (0)
#define DBG_CLA_M2_ISR                          (0)
#define DBG_TIMER_ISR                           (0)
#define DBG_GPIO_SPI_RX_ISR                     (0)
#define DBG_SCI_RS485_TX_ISR                    (0)
#define DBG_SCI_RS485_RX_ISR                    (0)
#define DBG_CLB_RS485_TX_RX_ISR                 (0)
#define DBG_MCAN_CANFD_TX_RX_ISR                (0)

#define DBG_CMD_NEW_TASK                        (0)
#define DBG_CALIB_TASK                          (0)
//#define DBG_M1_TASK                             (1)
//#define DBG_M2_TASK                             (1)
#endif

#define DBG_M1_TASK                             (0)
#define DBG_M2_TASK                             (0)

#if (DBG_CLA_M1_ISR)
#define DBG_CLA_M1_ISR_SET                      DBG_PIN2_SET
#define DBG_CLA_M1_ISR_UNSET                    DBG_PIN2_CLEAR
#else
#define DBG_CLA_M1_ISR_SET                      asm(" nop")
#define DBG_CLA_M1_ISR_UNSET                    asm(" nop")
#endif

#if (DBG_CLA_M2_ISR)
#define DBG_CLA_M2_ISR_SET                      DBG_PIN2_SET
#define DBG_CLA_M2_ISR_UNSET                    DBG_PIN2_CLEAR
#else
#define DBG_CLA_M2_ISR_SET                      asm(" nop")
#define DBG_CLA_M2_ISR_UNSET                    asm(" nop")
#endif

#if (DBG_TIMER_ISR)
#define DBG_TIMER_ISR_SET                       DBG_PIN1_SET
#define DBG_TIMER_ISR_UNSET                     DBG_PIN1_CLEAR
#else
#define DBG_TIMER_ISR_SET                       asm(" nop")
#define DBG_TIMER_ISR_UNSET                     asm(" nop")
#endif

#if (DBG_GPIO_SPI_RX_ISR)
#define DBG_GPIO_SPI_RX_ISR_SET                 DBG_PIN3_SET
#define DBG_GPIO_SPI_RX_ISR_UNSET               DBG_PIN3_CLEAR
#else
#define DBG_GPIO_SPI_RX_ISR_SET                 asm(" nop")
#define DBG_GPIO_SPI_RX_ISR_UNSET               asm(" nop")
#endif

#if (DBG_SCI_RS485_TX_ISR)
#define DBG_SCI_RX485_TX_ISR_SET                DBG_PIN3_SET
#define DBG_SCI_RX485_TX_ISR_UNSET              DBG_PIN3_CLEAR
#else
#define DBG_SCI_RX485_TX_ISR_SET                asm(" nop")
#define DBG_SCI_RX485_TX_ISR_UNSET              asm(" nop")
#endif

#if (DBG_SCI_RS485_RX_ISR)
#define DBG_SCI_RX485_RX_ISR_SET                DBG_PIN3_SET
#define DBG_SCI_RX485_RX_ISR_UNSET              DBG_PIN3_CLEAR
#else
#define DBG_SCI_RX485_RX_ISR_SET                asm(" nop")
#define DBG_SCI_RX485_RX_ISR_UNSET              asm(" nop")
#endif

#if (DBG_CLB_RS485_TX_RX_ISR)
#define DBG_CLB_RS485_TX_RX_ISR_SET             DBG_PIN3_SET
#define DBG_CLB_RS485_TX_RX_ISR_UNSET           DBG_PIN3_CLEAR
#else
#define DBG_CLB_RS485_TX_RX_ISR_SET             asm(" nop")
#define DBG_CLB_RS485_TX_RX_ISR_UNSET           asm(" nop")
#endif

#if (DBG_MCAN_CANFD_TX_RX_ISR)
#define DBG_MCAN_CANFD_TX_RX_ISR_SET            DBG_PIN3_SET
#define DBG_MCAN_CANFD_TX_RX_ISR_UNSET          DBG_PIN3_CLEAR
#else
#define DBG_MCAN_CANFD_TX_RX_ISR_SET            asm(" nop")
#define DBG_MCAN_CANFD_TX_RX_ISR_UNSET          asm(" nop")
#endif

#if (DBG_CMD_NEW_TASK)
#define DBG_CMD_NEW_TASK_SET                    DBG_PIN1_SET
#define DBG_CMD_NEW_TASK_UNSET                  DBG_PIN1_CLEAR
#else
#define DBG_CMD_NEW_TASK_SET                    __mnop()
#define DBG_CMD_NEW_TASK_UNSET                  __mnop()
#endif

#if (DBG_CALIB_TASK)
#define DBG_CALIB_TASK_SET                      DBG_PIN1_SET
#define DBG_CALIB_TASK_UNSET                    DBG_PIN1_CLEAR
#else
#define DBG_CALIB_TASK_SET                      __mnop()
#define DBG_CALIB_TASK_UNSET                    __mnop()
#endif

#if (DBG_M1_TASK)
#define DBG_M1_TASK_SET                         DBG_PIN0_SET
#define DBG_M1_TASK_UNSET                       DBG_PIN0_CLEAR
#else
#define DBG_M1_TASK_SET                         __mnop()
#define DBG_M1_TASK_UNSET                       __mnop()
#endif

#if (DBG_M2_TASK)
#define DBG_M2_TASK_SET                         DBG_PIN3_SET
#define DBG_M2_TASK_UNSET                       DBG_PIN3_CLEAR
#else
#define DBG_M2_TASK_SET                         __mnop()
#define DBG_M2_TASK_UNSET                       __mnop()
#endif

/***********************************************************************
 * FILTERS PARAMETERS
 ***********************************************************************/
//#define MOTOR12_OVERMODULATION                  (1.15f)
//#define IABC_CUTOFF_FREQ                        (15000.0f)   // vbus voltage loop bandwidth (Hz)
//#define IABC_CUTOFF_TIME_CONST                  (FM_2MULTPI * IABC_CUTOFF_FREQ * PWM_PERIOD)
//#define IABC_LPF_ALPHA                          (IABC_CUTOFF_TIME_CONST / (1.0f + IABC_CUTOFF_TIME_CONST))
//#define IABC_LPF_ONE_M_ALPHA                    (1.0f - IABC_LPF_ALPHA)
#define VBUS_CUTOFF_FREQ                        (400.0f)    // vbus voltage loop bandwidth (Hz)
#define VBUS_CUTOFF_TIME_CONST                  (FM_2MULTPI * VBUS_CUTOFF_FREQ * PWM_PERIOD)
#define VBUS_LPF_ALPHA                          (VBUS_CUTOFF_TIME_CONST / (1.0f + VBUS_CUTOFF_TIME_CONST))
#define VBUS_LPF_ONE_M_ALPHA                    (1.0f - VBUS_LPF_ALPHA)
#define VEXT_CUTOFF_FREQ                        (4000.0f)   // vbus voltage loop bandwidth (Hz)
#define VEXT_CUTOFF_TIME_CONST                  (FM_2MULTPI * VEXT_CUTOFF_FREQ * PWM_PERIOD)
#define VEXT_LPF_ALPHA                          (VEXT_CUTOFF_TIME_CONST / (1.0f + VEXT_CUTOFF_TIME_CONST))
#define VEXT_LPF_ONE_M_ALPHA                    (1.0f - VEXT_LPF_ALPHA)

/***********************************************************************
 * STRUCTURES DEFINES
 ***********************************************************************/
/*
 * Motors config
 */
#define USE_TMOTOR_MN4004_KV300                 (0)
#define USE_MYACTUATOR_RMDX8H                   (1)
#define USE_TMOTOR_MN4004_KV400                 (2)
/*
 * Encoders config
 */
#define USE_BROADCOM_AEDT_9810_Z00              (-1)
#define USE_CUI_AMT112S_4096_5000_S             (-2)
/**
 * Motor & encoder selection for uOmodri
 */
#define MOTOR1_PARAMS                           USE_TMOTOR_MN4004_KV300
#define MOTOR2_PARAMS                           USE_TMOTOR_MN4004_KV300
#define ENCODER1_PARAMS                         USE_BROADCOM_AEDT_9810_Z00
#define ENCODER2_PARAMS                         USE_BROADCOM_AEDT_9810_Z00
/**
 * @}
 */

/***********************************************************************
 * MOTORS \& ENCODERS INCLUDES
 ***********************************************************************/
#if (MOTOR1_PARAMS == USE_TMOTOR_MN4004_KV300) || (MOTOR2_PARAMS == USE_TMOTOR_MN4004_KV300)
#include "config/motor/tmotor_mn4004_kv300.h"           // T-MOTOR MN4000 KV300 motor definitions
#endif
#if (MOTOR1_PARAMS == USE_MYACTUATOR_RMDX8H) || (MOTOR2_PARAMS == USE_MYACTUATOR_RMDX8H)
#include "config/motor/myactuator_rmdx8h.h"             // MYACTUATOR RMDX8H motor definitions
#endif
#if (MOTOR1_PARAMS == USE_TMOTOR_MN4004_KV400) || (MOTOR2_PARAMS == USE_TMOTOR_MN4004_KV400)
#include "config/motor/tmotor_mn4004_kv400.h"           // T-MOTOR MN4000 KV400 motor definitions
#endif

#if (ENCODER1_PARAMS == USE_BROADCOM_AEDT_9810_Z00) || (ENCODER2_PARAMS == USE_BROADCOM_AEDT_9810_Z00)
#include "config/encoder/broadcom_aedt_9810_z00.h"      // BROADCOM AEDT-9810-Z00 encoder definitions
#endif
#if (ENCODER1_PARAMS == USE_CUI_AMT112S_4096_5000_S) || (ENCODER2_PARAMS == USE_CUI_AMT112S_4096_5000_S)
#include "config/encoder/cui_amt112s_4096_5000_s.h"     // CUI AMT112S-4096-5000-s encoder definitions
#endif

#if (USE_UOMODRI_REV == USE_UOMODRI_V1)
#include "config/uomodri_1/uomodri_1_hal_defines.h"     // uOmodri v1.x definitions
#elif (USE_UOMODRI_REV == USE_UOMODRI_V2)
#include "config/uomodri_2/uomodri_2_hal_defines.h"     // uOmodri v2.x definitions
#elif (USE_UOMODRI_REV == USE_UOMODRI_V3)
#include "config/uomodri_3/uomodri_3_hal_defines.h"     // uOmodri v3.x definitions
#else
#error "@ least one config must be set @ a time."
#endif

/***********************************************************************
 * MOTORS DEFINES
 ***********************************************************************/
#define MIN                                     (0)
#define MAX                                     (1)
#define FW                                      (0)     /*!< Forward direction */
#define REV                                     (1)     /*!< Reverse direction */
#define NEW                                     (0)     /*!< Current data */
#define OLD                                     (1)     /*!< Previous data */
#define MOTOR_1                                 (0)     /*!< Select Motor_1 */
#define MOTOR_2                                 (1)     /*!< Select Motor_2 */
#define CHANNEL_1                               (0)     /*!< Select Channel_1 in PWM */
#define CHANNEL_2                               (1)     /*!< Select Channel_2 in PWM */
#define CHANNEL_3                               (2)     /*!< Select Channel_3 in PWM */
#define CURRENT                                 (0)     /*!< Current state of state machine */
#define NEXT                                    (1)     /*!< Next state of state machine */
#endif /* __UOMODRI_USER_DEFINES_H__ */
