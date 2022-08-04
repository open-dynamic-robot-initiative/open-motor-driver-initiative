#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
//#include "f2838x_device.h"
#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
#include "f2838x_cla_defines.h"
#endif
//#include "f2838x_examples.h"
#include "driverlib.h"
#include "device.h"
#include "hw_types.h"
#include "communication.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
#pragma DATA_ALIGN(cmd_msg, 8)
#pragma DATA_SECTION(cmd_msg, "ramgs1")
mst2slv_msg_t   cmd_msg;

#pragma DATA_ALIGN(status_msg, 8)
#pragma DATA_SECTION(status_msg, "ramgs1")
slv2mst_msg_t   status_msg;
// = {  .status.all                  = 0x0000,
//                                .timeStamp                   = 0x1111,
//                                .position[MOTOR_1].u16_lsb   = 0x2222,
//                                .position[MOTOR_1].u16_msb   = 0x3333,
//                                .position[MOTOR_2].u16_lsb   = 0x4444,
//                                .position[MOTOR_2].u16_msb   = 0x5555,
//                                .velocity[MOTOR_1]           = 0x6666,
//                                .velocity[MOTOR_2]           = 0x7777,
//                                .current[MOTOR_1]            = 0x8888,
//                                .current[MOTOR_2]            = 0x9999,
//                                .coilRes[MOTOR_1]            = 0xAAAA,
//                                .coilRes[MOTOR_2]            = 0xBBBB,
//                                .adcSamples[MOTOR_1]         = 0xCCCC,
//                                .adcSamples[MOTOR_2]         = 0xDDDD,
//                                .lastCmdIndex                = 0xEEEE,
//                                .crc.u16_msb                 = 0xFFFF,
//                                .crc.u16_lsb                 = 0x0000};

#pragma DATA_SECTION(led_msg, "ramgs1")
uint16_t        led_msg[LED_MSG_TX_16BIT_LENGTH];
//{0x0000, 0x9B6D, 0xA492, 0x4924, 0x9249, 0x2400}; // green
//{0x0000, 0x9249, 0x24DB, 0x6DB6, 0x9249, 0x2400}; // red
//{0x0000, 0x9249, 0x2492, 0x4924, 0xDB6D, 0xB600}; // blue

#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
//#pragma DATA_ALIGN ( cmd_Master_msg , 2 )
#pragma DATA_SECTION(cmd_Master_msg, "ramgs0");
cmd_msg_t       cmd_Master_msg;
#pragma DATA_SECTION(status_Master_msg, "ramgs0");
status_msg_t    status_Master_msg;
#endif

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void adc_isrMotor1(void);
__attribute__((interrupt)) void adc_isrMotor2(void);
#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
__attribute__((interrupt)) void cla1_isr1(void);
#endif

#endif /* __MAIN_H__ */
