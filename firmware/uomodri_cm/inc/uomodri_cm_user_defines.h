/*
 * File name: omodri_cm_user_defines.h
 * Description: Header file containing user definitions
 */

#ifndef __UOMODRI_CM_USER_DEFINES_H__
#define __UOMODRI_CM_USER_DEFINES_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "driverlib_cm.h"
#include "cm.h"

/***********************************************************************
 * GENERAL DEFINES
 ***********************************************************************/
#define UOMODRI_CONFIG_ENABLE                   1
#define UOMODRI_USE_REV_1_0_CONFIG              1

#ifndef FM_RPM2RAD
#define FM_RPM2RAD                              (2.0f * M_PI / 60.0f)
#endif

#ifndef FM_RAD2RPM
#define FM_RAD2RPM                              (60.0f / (2.0f * M_PI))
#endif

/***********************************************************************
 * GPIO DEBUG DEFINES
 ***********************************************************************/
#define DBG_PIN0                                (78)
#define DBG_PIN0_CFG                            GPIO_78_GPIO78
#define DBG_PIN0_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO78)
#define DBG_PIN0_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO78)
#define DBG_PIN0_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO78)

#define DBG_PIN1                                (81)
#define DBG_PIN1_CFG                            GPIO_81_GPIO81
#define DBG_PIN1_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO81)
#define DBG_PIN1_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO81)
#define DBG_PIN1_TOGGLE                         (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO81)
#define DBG_PIN1_READ                           (HWREG(GPIODATA_BASE + GPIO_O_GPCDAT) & GPIO_GPCDAT_GPIO81)

#define EXT_DBG_SPI_SIMO                        (69)
#define EXT_DBG_SPI_SIMO_CFG                    GPIO_69_GPIO69//GPIO_69_SPIC_SIMO
#define EXT_DBG_SPI_SIMO_SET                    (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO69)
#define EXT_DBG_SPI_SIMO_CLEAR                  (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO69)
#define EXT_DBG_SPI_SIMO_TOGGLE                 (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO69)

#define EXT_DBG_SPI_SOMI                        (70)
#define EXT_DBG_SPI_SOMI_CFG                    GPIO_70_GPIO70//GPIO_70_SPIC_SOMI
#define EXT_DBG_SPI_SOMI_SET                    (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO70)
#define EXT_DBG_SPI_SOMI_CLEAR                  (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO70)
#define EXT_DBG_SPI_SOMI_TOGGLE                 (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO70)

#define EXT_DBG_SPI_CS1                         (72)
#define EXT_DBG_SPI_CS1_CFG                     GPIO_72_GPIO72//SPIC_STEN
#define EXT_DBG_SPI_CS1_SET                     (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO72)
#define EXT_DBG_SPI_CS1_CLEAR                   (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO72)
#define EXT_DBG_SPI_CS1_TOGGLE                  (HWREG(GPIODATA_BASE + GPIO_O_GPCTOGGLE)= GPIO_GPCTOGGLE_GPIO72)

/************************************************************************
 * IPC DEFINES
 ************************************************************************/
/** @defgroup Debug pins configuration.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#define IPC_SYNC_FLAG                           IPC_FLAG31
#define IPC_CPU1_TO_CM_FLAG                     IPC_FLAG0
#define IPC_CM_TO_CPU1_FLAG                     IPC_FLAG0
/**
 * @}
 */

/************************************************************************
 * DEBUG DEFINES
 ************************************************************************/
/** @defgroup Debug pins configuration.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
#define DEBUG_LED_PIN1                          31
#define DEBUG_LED_PIN1_CFG                      GPIO_31_GPIO31
#define DEBUG_LED_PIN2                          34
#define DEBUG_LED_PIN2_CFG                      GPIO_34_GPIO34
#define DEBUG_INT_PIN1                          40
#define DEBUG_INT_PIN1_CFG                      GPIO_40_GPIO40
#define DEBUG_INT_PIN2                          41
#define DEBUG_INT_PIN2_CFG                      GPIO_41_GPIO41
#define USB_M                                   42
#define USB_M_CFG                               GPIO_42_GPIO42
#define USB_P                                   43
#define USB_P_CFG                               GPIO_43_GPIO43
#define USB_VBUS                                46
#define USB_VBUS_CFG                            GPIO_46_GPIO46
#endif
#ifdef  USB
#define USB_M                                   42
#define USB_M_CFG                               GPIO_42_GPIO42
#define USB_P                                   43
#define USB_P_CFG                               GPIO_43_GPIO43
#define USB_VBUS                                46
#define USB_VBUS_CFG                            GPIO_46_GPIO46
#else
#define UART_DBG_TX                             (42)
#define UART_DBG_TX_CFG                         GPIO_42_UARTA_TX
#define UART_DBG_RX                             (43)
#define UART_DBG_RX_CFG                         GPIO_43_UARTA_RX
#endif
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (UOMODRI_USE_REV_1_0_CONFIG) && (defined DEBUG)
#define DBG_PIN2                                (70)
#define DBG_PIN2_CFG                            GPIO_70_GPIO70
#endif
/**
 * @}
 */

/** @defgroup Message configuration.
 *  @brief    Configure the message send by DMA channel.
 * @{
 */
//#define DBG_TX_MSG_HEADER_8BIT_SIZE             (2)
//#define DBG_TX_MSG_COUNTER_8BIT_SIZE            (4)
////#define DBG_TX_MSG_STATUS_8BIT_SIZE             (2)
//#define DBG_TX_MSG_POSITION_32BIT_SIZE          (1)
//#define DBG_TX_MSG_POSITION_8BIT_SIZE           (4 * DBG_TX_MSG_POSITION_32BIT_SIZE)
//#define DBG_TX_MSG_IQREF_CMD_16BIT_SIZE         (1)
//#define DBG_TX_MSG_IQREF_CMD_8BIT_SIZE          (2 * DBG_TX_MSG_IQREF_CMD_16BIT_SIZE)
////#define DBG_TX_MSG_IQREF_PD_OUT_16BIT_SIZE      (1)
////#define DBG_TX_MSG_IQREF_PD_OUT_8BIT_SIZE       (2 * DBG_TX_MSG_IQREF_PD_OUT_16BIT_SIZE)
//#define DBG_TX_MSG_IQ_MEAS_16BIT_SIZE           (1)
//#define DBG_TX_MSG_IQ_MEAS_8BIT_SIZE            (2 * DBG_TX_MSG_IQ_MEAS_16BIT_SIZE)
//#define DBG_TX_MSG_ID_MEAS_16BIT_SIZE           (1)
//#define DBG_TX_MSG_ID_MEAS_8BIT_SIZE            (2 * DBG_TX_MSG_ID_MEAS_16BIT_SIZE)
//#define DBG_TX_MSG_DTC_32BIT_SIZE               (2)
//#define DBG_TX_MSG_DTC_8BIT_SIZE                (4 * DBG_TX_MSG_DTC_32BIT_SIZE)
////#define DBG_TX_MSG_STATE_MACHINE_16BIT_SIZE     (0)
////#define DBG_TX_MSG_STATE_MACHINE_8BIT_SIZE      (1)
//#define DBG_TX_MSG_PAYLOAD_8BIT_SIZE            (DBG_TX_MSG_HEADER_8BIT_SIZE + \
//                                                 DBG_TX_MSG_COUNTER_8BIT_SIZE + \
//                                                 DBG_TX_MSG_POSITION_8BIT_SIZE + \
//                                                 DBG_TX_MSG_IQREF_CMD_8BIT_SIZE + \
//                                                 DBG_TX_MSG_IQ_MEAS_8BIT_SIZE + \
//                                                 DBG_TX_MSG_ID_MEAS_8BIT_SIZE + \
//                                                 DBG_TX_MSG_DTC_8BIT_SIZE)
//#define DBG_TX_MSG_PAYLOAD_16BIT_SIZE           (DBG_TX_MSG_PAYLOAD_8BIT_SIZE / 2)
//#define DBG_TX_MSG_CRC_8BIT_SIZE                (1)
//#define DBG_TX_MSG_8BIT_SIZE                    (DBG_TX_MSG_PAYLOAD_8BIT_SIZE + DBG_TX_MSG_CRC_8BIT_SIZE)
/**
 * @}
 */

/************************************************************************
 * COMMUNICATION DEFINES
 ************************************************************************/
/*** Communication constants ***/
#define POSITION_LSB                            (5.960464477539063e-08f)    // 2**(-24)
#define VELOCITY_LSB                            (4.8828125e-04f)// 2**(-11) // Added rad/s to krpm conversion
#define IQ_LSB                                  (9.765625e-04f)             // 2**(-10)
#define CURRENT_SAT_LSB                         (1.25e-01f)                 // 2**(-3)
#define RESISTANCE_LSB                          (3.0517578125e-05f)         // 2**(-15)
#define VOLTAGE_LSB                             (6.103515625e-05f)          // 2**(-14)
#define KP_LSB                                  (4.8828125e-04f)            // 2**(-11)
#define KD_LSB                                  (9.765625e-04f)             // 2**(-10)

/************************************************************************
 * TYPDEF DEFINES
 ************************************************************************/
//typedef _Bool   bool_t;
//typedef  float  float32_t;

typedef struct __LOWPASSFILTER_STRUCT__
{
    float   a;
    float   one_minus_a;
} lpf_t;

/************************************************************************
 * TYPDEF ENUM
 ************************************************************************/
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
} motor_id_e;

/**
 * @enum    StateID
 * @brief   Easier state management for history.
 */
typedef enum
{
    NEXT    = 0,    /*!< Next state */
    CURRENT = 1,    /*!< Current state */
} state_id_e;

#endif /* __UOMODRI_CM_USER_DEFINES_H__ */
