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

#include "hal.h"
#include "drv8353.h"
#include "ws2812b.h"
#include "encoder.h"
#include "foc.h"
#include "motor.h"
#include "communication.h"

#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"
#include "uomodri_prj_config_handlers.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define IPC_CMD_M1      0x1001
#define IPC_CMD_M2      0x2001

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
// <Step 1>: Assign the CLA variables to their respective memory spaces
//  - Input/Output variables should go into the Cla1ToCpuMsgRam/CpuToCla1MsgRam
//  - Global variables in the .cla file are automatically placed in the ".bss_cla" so
//    no manual placement is required.
//  - The same goes for constants in the .cla file; they are placed in the
//    ".const_cla" section but you will need to load it to FLASH and then copy
//    to RAM at runtime
//  - You can manually place shared variables in a user defined section, e.g. Cla1DataRam0,
//     to allow the C28x to also access them
//Task 1 (C) Variables
//#pragma DATA_SECTION(polePairs, "CpuToCla1MsgRAM");
//uint8_t     polePairs;    // Number of pole pairs of the motor
//#pragma DATA_SECTION(thetaMechScaler, "CpuToCla1MsgRAM");
//float   thetaMechScaler; // [pu/register]
//#pragma DATA_SECTION(thetaMech, "CpuToCla1MsgRAM");
//float32_t   thetaMech; // [pu]
#pragma DATA_SECTION(thetaDir, "Cla1ToCpuMsgRAM");
uint8_t thetaDir; //0 = CCW/reverse, 1 = CW/forward
#pragma DATA_SECTION(thetaMech, "Cla1ToCpuMsgRAM");
float   thetaMech[2]; // [pu]
#pragma DATA_SECTION(thetaElec, "Cla1ToCpuMsgRAM");
float   thetaElec; // [pu]
//#pragma DATA_SECTION(thetaMechCal, "Cla1ToCpuMsgRAM");
//float32_t   thetaMechCal; // [pu]
#endif
#pragma DATA_ALIGN (cmd_msg, 16)
#pragma DATA_SECTION(cmd_msg, "ramgs1");
mst2slv_msg_t   cmd_msg;
#pragma DATA_SECTION(status_msg, "ramgs1");
slv2mst_msg_t   status_msg;
#pragma DATA_SECTION(led_msg, "ramgs1");
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

extern motor_t          motor[];
extern hal_cfg_t        hal;
extern cmd_uomodri_t    cmd_uOmodri;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void adc_isrMotor1(void);
__attribute__((interrupt)) void adc_isrMotor2(void);
#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
__attribute__((interrupt)) void cla1_isr1(void);
#endif

#endif /* __MAIN_H__ */
