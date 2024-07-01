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

#ifndef __TMS320C28XX__
#define __TMS320C28XX__  // These enums are only accessible by C28x
#endif

// Cores activation ENABLE/DISABLE
#define CPU1_CORE_ENABLE                        (1)
#if (CPU1_CORE_ENABLE) && (!defined CPU1)
#define CPU1
#endif
#define CPU2_CORE_ENABLE                        (0)
#if (CPU2_CORE_ENABLE) && (!defined CPU2)
#define CPU2
#endif
#define CM_CORE_ENABLE                          (0)
#if (CM_CORE_ENABLE) && (!defined CM)
#define CM
#endif
#define CLA_CORE_ENABLE                         (1)
#define CLB_TILE_ENABLE                         (1)
#if (CLA_CORE_ENABLE) && (!defined __TMS320C28XX_CLA__)
#define __TMS320C28XX_CLA__
#endif//#define _TMS320C28XX_CLA_
//#define __TMS320C28XX_CLA2__
//#endif
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
#ifdef __TMS320C28XX_CLA__
//#include "f2838x_cla_typedefs.h"        //
#include "CLAmath.h"
#endif

#include "f2838x_device.h"              // F2838x Headerfile Include File
//#include "f2838x_examples.h"          // F2838x Examples Include File
//#include "driverlib.h"
#include "device.h"                     // F2838x Headerfile Include File
#include "board.h"                      //

#include "tmotor_mn4004_kv300.h"        // T-MOTOR MN4000 KV300 motor definitions
#include "myactuator_rmdx8h.h"          // MYACTUATOR RMDX8H motor definitions
#include "broadcom_aedt_9810_z00.h"     // BROADCOM AEDT-9810-Z00 encoder definitions
#include "cui_amt112s_4096_5000_s.h"    // CUI AMT112S-4096-5000-s encoder definitions

#include "uomodri_math.h"               // uOmodri math parameters include file

#include "../motor.h"
#include "../communication.h"
#include "../hal.h"

#if (USE_UOMODRI_REV == USE_UOMODRI_V1)
//
#elif (USE_UOMODRI_REV == USE_UOMODRI_V2)
//
#elif (USE_UOMODRI_REV == USE_UOMODRI_V3)
#include "uomodri_3_1_hal_defines.h"    // uOmodri v3.1 definitions
#include "uomodri_3_1_hal_handlers.h"   // uOmodri v3.1 configuration include file
#elif (USE_UOMODRI_REV == USE_UOMODRI_SA_V1)
//
#else
#error "@ least one config must be set @ a time."
#endif

#include "uomodri_drv.h"                // uOmodri DRV configuration include file
#include "uomodri_interrupts.h"         // uOmodri interrupts configuration include file

//#if(SPI_BUS_ENABLE) || (RS485_BUS_ENABLE)
//#include "uomodri_communication_defines.h"
//#endif
//#include "uomodri_math.h"

//#include "hw_types.h"


//#include "f2838x_device.h"        // F2838x Headerfile Include File
//#include "driverlib.h"
//#include "f2838x_examples.h"      // F2838x Examples Include File

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
//#pragma DATA_ALIGN(status_tmp_msg, 4)
//#pragma DATA_SECTION(status_tmp_msg, "ramgs1")
//slv2mst_msg_t   status_tmp_msg;

/** @var    MOTOR_STRUCT    motor[]
 *  @brief  General structure array configuration structure for the Hardware Abstraction Layer.
 */
#pragma DATA_ALIGN(motor_m1, 4)
#pragma DATA_SECTION(motor_m1, "ClaDataMotor1");
motor_t     motor_m1;

#pragma DATA_ALIGN(motor_m2, 4)
#pragma DATA_SECTION(motor_m2, "ClaDataMotor2");
motor_t     motor_m2;

#pragma DATA_ALIGN(cmd_uomodri, 4)
#pragma DATA_SECTION(cmd_uomodri, "MSGRAM_CPU_TO_CLA")
cmd_t       cmd_uomodri[2];

#pragma DATA_ALIGN(com_spi_rx, 4)
#pragma DATA_SECTION(com_spi_rx, "ramgs0")
spi_rx_t    com_spi_rx;
#pragma DATA_ALIGN(com_spi_tx, 4)
#pragma DATA_SECTION(com_spi_tx, "ramgs0")
spi_tx_t    com_spi_tx          = {.msg_cr = 0U, .msg_nx = 1U,
                                   .msg[0].crc.u16_msb = 0xDFFF, .msg[0].crc.u16_lsb = 0x5283,
                                   .msg[1].crc.u16_msb = 0xDFFF, .msg[1].crc.u16_lsb = 0x5283};
#pragma DATA_ALIGN(com_rs485_rx, 4)
#pragma DATA_SECTION(com_rs485_rx, "ramgs0")
rs485_rx_t  com_rs485_rx        = {.index = 0U, .addr1 = false, .addr2 = false};
#pragma DATA_ALIGN(com_rs485_tx, 4)
#pragma DATA_SECTION(com_rs485_tx, "ramgs0")
rs485_tx_t  com_rs485_tx        = {.index = 0U, .msg_cr = 0U, .msg_nx = 1U,
                                   .msg[0].crc.u16_msb = 0x8CA0, .msg[0].crc.u16_lsb = 0x22CF,
                                   .msg[1].crc.u16_msb = 0x8CA0, .msg[1].crc.u16_lsb = 0x22CF};
#pragma DATA_ALIGN(led_spi_tx, 4)
#pragma DATA_SECTION(led_spi_tx, "ramgs0")
uint16_t    led_spi_tx[LED_MSG_TX_16BIT_LENGTH];
#pragma DATA_ALIGN(com_bus_select, 2)
#pragma DATA_SECTION(com_bus_select, "ramgs0")
com_bus_e   com_bus_select      = COM_NO_BUS_DEFINED;
#pragma DATA_ALIGN(global_1ms_tmr_cnt, 2)
#pragma DATA_SECTION(global_1ms_tmr_cnt, "ramgs0")
uint32_t    global_1ms_tmr_cnt  = 0U;

/***********************************************************************
 * STRUCTURES
 ***********************************************************************/
typedef struct __lowPassFilter_t__
{
    float32_t   a;
    float32_t   one_minus_a;
} lpf_t;

#endif /* __UOMODRI_USER_DEFINES_H__ */
