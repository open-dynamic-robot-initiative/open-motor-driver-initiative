/*
 * File name: omodri_cm_user_defines.h
 * Description: Header file containing user definitions
 */

#ifndef __UOMODRI_CM_USER_DEFINES_H__
#define __UOMODRI_CM_USER_DEFINES_H__

#include "driverlib_cm.h"
#include "cm.h"

/************************************************************************
 * GENERAL DEFINES
 ************************************************************************/
#define UOMODRI_CONFIG_ENABLE                   1

/************************************************************************
 * IPC DEFINES
 ************************************************************************/
/** @defgroup Debug pins configuration.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#define IPC_SYNC_FLAG                           IPC_FLAG31
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
/**
 * @}
 */

/** @defgroup Message configuration.
 *  @brief    Configure the message send by DMA channel.
 * @{
 */
#define DBG_TX_MSG_HEADER_8BIT_SIZE             (4)
#define DBG_TX_MSG_COUNTER_8BIT_SIZE            (4)
#define DBG_TX_MSG_DATA_32BIT_SIZE              (4)
#define DBG_TX_MSG_DATA_8BIT_SIZE               (4 * DBG_TX_MSG_DATA_32BIT_SIZE)
#define DBG_TX_MSG_PAYLOAD_8BIT_SIZE            (DBG_TX_MSG_HEADER_8BIT_SIZE + DBG_TX_MSG_COUNTER_8BIT_SIZE + DBG_TX_MSG_DATA_8BIT_SIZE)
#define DBG_TX_MSG_CRC_SIZE                     (4)
#define DBG_TX_MSG_SIZE                         (DBG_TX_MSG_PAYLOAD_8BIT_SIZE + DBG_TX_MSG_CRC_SIZE)
/**
 * @}
 */

/************************************************************************
 * COMMUNICATION DEFINES
 ************************************************************************/
/*** Communication constants ***/
#define POSITION_LSB                            (5.960464477539063e-08)                 // 2**(-24)
#define VELOCITY_LSB                            (0.00048828125 * 1000.0 * FM_RPM2RADPS) // 2**(-11) // Added rad/s to krpm conversion
#define IQ_LSB                                  (0.0009765625)                          // 2**(-10)
#define CURRENT_SAT_LSB                         (0.125)                                 // 2**(-3)
#define RESISTANCE_LSB                          (3.0517578125e-05)                      // 2**(-15)
#define VOLTAGE_LSB                             (6.103515625e-05)                       // 2**(-14)
#define KP_LSB                                  (0.00048828125)                         // 2**(-11)
#define KD_LSB                                  (0.0009765625)                          // 2**(-10)

/************************************************************************
 * TYPDEF DEFINES
 ************************************************************************/
typedef _Bool   bool_t;
typedef  float  float32_t;
typedef struct __LOWPASSFILTER_STRUCT__
{
    float32_t a;
    float32_t one_minus_a;
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

#endif /* __UOMODRI_CM_USER_DEFINES_H__ */
