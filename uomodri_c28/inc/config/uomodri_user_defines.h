/*
 * File name: omodri_user_defines.h
 * Description: Header file containing user definitions
 */

#ifndef __UOMODRI_USER_DEFINES_H__
#define __UOMODRI_USER_DEFINES_H__

/***********************************************************************
 * DEFINES
 ***********************************************************************/
/** @defgroup DEV_defines
 *  @brief    Special configurations definitions.
 * @{
 */
#define USE_FCT                                 (0)

// boards revision include
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

// Cores activation ENABLE/DISABLE
#define CPU1_CORE_ENABLE                        (1)
#define CPU2_CORE_ENABLE                        (0)
#define CM_CORE_ENABLE                          (0)
#define CLA_CORE_ENABLE                         (1)
#define CLB_TILE_ENABLE                         (1)
#if (CLA_CORE_ENABLE)
//#define __TMS320C28XX_CLA__
//#define _TMS320C28XX_CLA_
//#define __TMS320C28XX_CLA2__
#endif
// CM peripherals ENABLE/DISABLE
#define USB_BUS_ENABLE                          (0)
#define UART_BUS_ENABLE                         (0)
#if ((USB_BUS_ENABLE) || (UART_BUS_ENABLE)) && (!CM_CORE_ENABLE)
#error "In order to use USB or UART, CM must be active."
#endif
#if (UART_BUS_ENABLE) && (USB_BUS_ENABLE)
#error "USB & UART share the same pins. They cannot run @ the same time"
#endif
// Communication peripherals ENABLE/DISABLE
#define CAN_BUS_ENABLE                          (0)
#define RS485_BUS_ENABLE                        (1)
#define SPI_BUS_ENABLE                          (1)
#if (RS485_BUS_ENABLE) && (CAN_BUS_ENABLE)
#error "RS-485 & CAN share the same pins. They cannot run @ the same time"
//#elif ((RS485_BUS_ENABLE) || (CAN_BUS_ENABLE) ) && (SPI_BUS_ENABLE)
//#error "RS-485, CAN & SPI share the same data. They cannot run @ the same time"
#endif

// Filters definition
#define UOMODRI_IABC_FLT(flt_coef, y, x)        LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_VBUS_FLT(flt_coef, y, x)        LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_VEXT_FLT(flt_coef, y, x)        LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_IDQ_FLT(flt_coef, y, x)         LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_RES_ESTIM_FLT(flt_coef, y, x)   LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_SPEED_FLT(flt_coef, y, x)       LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_HAL_ADC_CALIB(flt_coef, y, x)   LPF_FILTER_1(flt_coef, y, x)
#define UOMODRI_FEED_FORWARD_ENABLE             (0)

/*
 * Motors configs
 */
#define USE_TMOTOR_MN4004_KV300                 (0)
#define USE_MYACTUATOR_RMDX8H                   (1)
/*
 * Encoders configs
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

/***********************************************************************
 * MOTORS DEFINES
 ***********************************************************************/
#define MOTOR12_OVERMODULATION                  (1.15f)
#define IABC_CUTOFF_FREQ                        (15000.0f)   // Ia, Ib & Ic loop bandwidth (Hz)
#define IABC_CUTOFF_TIME_CONST                  (2.0f * UOMODRI_PI * IABC_CUTOFF_FREQ * PWM_PERIOD)
#define IABC_LPF_ALPHA                          (IABC_CUTOFF_TIME_CONST / (1.0f + IABC_CUTOFF_TIME_CONST))
#define IABC_LPF_ONE_M_ALPHA                    (1.0f - IABC_LPF_ALPHA)
#define VBUS_CUTOFF_FREQ                        (400.0f)    // vbus voltage loop bandwidth (Hz)
#define VBUS_CUTOFF_TIME_CONST                  (2.0f * UOMODRI_PI * VBUS_CUTOFF_FREQ * PWM_PERIOD)
#define VBUS_LPF_ALPHA                          (VBUS_CUTOFF_TIME_CONST / (1.0f + VBUS_CUTOFF_TIME_CONST))
#define VBUS_LPF_ONE_M_ALPHA                    (1.0f - VBUS_LPF_ALPHA)
#define VEXT_CUTOFF_FREQ                        (4000.0f)   // vext voltage loop bandwidth (Hz)
#define VEXT_CUTOFF_TIME_CONST                  (2.0f * UOMODRI_PI * VEXT_CUTOFF_FREQ * PWM_PERIOD)
#define VEXT_LPF_ALPHA                          (VEXT_CUTOFF_TIME_CONST / (1.0f + VEXT_CUTOFF_TIME_CONST))
#define VEXT_LPF_ONE_M_ALPHA                    (1.0f - VEXT_LPF_ALPHA)

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "motor/tmotor_mn4004_kv300.h"
#include "motor/myactuator_rmdx8h.h"
#include "encoder/broadcom_aedt_9810_z00.h"
#include "encoder/cui_amt112s_4096_5000_s.h"

//#if(SPI_BUS_ENABLE) || (RS485_BUS_ENABLE)
//#include "uomodri_communication_defines.h"
//#endif
#include "uomodri_math.h"

#if (USE_UOMODRI_REV == USE_UOMODRI_V1)
//
#elif (USE_UOMODRI_REV == USE_UOMODRI_V2)
//
#elif (USE_UOMODRI_REV == USE_UOMODRI_V3)
#include "uomodri_3_1/uomodri_3_1_hal_defines.h"
#include "uomodri_3_1/uomodri_3_1_hal_handlers.h"
#elif (USE_UOMODRI_REV == USE_UOMODRI_SA_V1)
//
#else
#error "@ least one config must be set @ a time."
#endif
#include "hw_types.h"

///***********************************************************************
// * STRUCTURES
// ***********************************************************************/
//typedef struct __lowPassFilter_t__
//{
//    float32_t a;
//    float32_t one_minus_a;
//} lpf_t;
//#endif
