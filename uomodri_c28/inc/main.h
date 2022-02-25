#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "f2838x_device.h"
#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
#include "f2838x_cla_defines.h"
#endif
#include "f2838x_examples.h"
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
//#include "uOmodriClaTasksShare.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define IPC_CMD_M1      0x1001
#define IPC_CMD_M2      0x2001

/*****************************************************************************
 * globals
 *****************************************************************************/
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

#pragma DATA_SECTION(cmd_msg, "ramgs0");
mst2slv_msg_t   cmd_msg;//[2];
#pragma DATA_SECTION(status_msg, "ramgs0");
slv2mst_msg_t   status_msg;// =
//{
// {
//  .status.bit.SE            = true,
//  .status.bit.M1E           = false,    // MOTOR_1 enabled
//  .status.bit.M2E           = false,    // MOTOR_2 enabled
//  .status.bit.M1R           = false,    // MOTOR_1 ready
//  .status.bit.M2R           = false,    // MOTOR_2 ready
//  .status.bit.IDX1T         = false,    // MOTOR_1 index toggle
//  .status.bit.IDX2T         = false,    // MOTOR_2 index toggle
//  .status.bit.IDX1D         = false,    // MOTOR_1 index detected
//  .status.bit.IDX2D         = false,    // MOTOR_2 index detected
//  .status.bit.ERROR_CODE    = STATUS_ERROR_NO_ERROR,
//  .position                 = {{ 0, 0}, { 0, 0}},
////  .position[MOTOR_1].u16_lsb    = 0,
////  .position[MOTOR_2].u16_msb    = 0,
////  .position[MOTOR_2].u16_lsb    = 0,
//  .velocity                 = { 0, 0},
////  .velocity[MOTOR_2]            = 0,
//  .current                  = { 0, 0},
////  .current[MOTOR_2]             = 0,
//  .coilRes                  = { 0, 0},
////  .coilRes[MOTOR_2]             = 0,
//  .adcSamples               = { 0, 0},
////  .adcSamples[MOTOR_2]          = 0,
//  .crc                      = {0xD9C3, 0x4244},//{ 0xDFFF, 0x5283},
// // required for Solo0x5283DFFF // 0x4244D9C3(ES true)
////  .crc.u16_lsb                  = 0,
// },
// {
//  .status.bit.SE                = true,
//  .status.bit.M1E               = false,    // MOTOR_1 enabled
//  .status.bit.M2E               = false,    // MOTOR_2 enabled
//  .status.bit.M1R               = false,    // MOTOR_1 ready
//  .status.bit.M2R               = false,    // MOTOR_2 ready
//  .status.bit.IDX1T             = false,    // MOTOR_1 index toggle
//  .status.bit.IDX2T             = false,    // MOTOR_2 index toggle
//  .status.bit.IDX1D             = false,    // MOTOR_1 index detected
//  .status.bit.IDX2D             = false,    // MOTOR_2 index detected
//  .status.bit.ERROR_CODE        = STATUS_ERROR_NO_ERROR,
//  .position[MOTOR_1].u16_msb    = 0,
//  .position[MOTOR_1].u16_lsb    = 0,
//  .position[MOTOR_2].u16_msb    = 0,
//  .position[MOTOR_2].u16_lsb    = 0,
//  .velocity[MOTOR_1]            = 0,
//  .velocity[MOTOR_2]            = 0,
//  .current[MOTOR_1]             = 0,
//  .current[MOTOR_2]             = 0,
//  .coilRes[MOTOR_1]             = 0,
//  .coilRes[MOTOR_2]             = 0,
//  .adcSamples[MOTOR_1]          = 0,
//  .adcSamples[MOTOR_2]          = 0,
//  .crc.u16_msb                  = 0xD9C3,//{ 0xDFFF, 0x5283},0xDFFF,// required for Solo
//  .crc.u16_lsb                  = 0x4244,//{ 0xDFFF, 0x5283},0x5283,
// }
//};

#pragma DATA_SECTION(cmd_uOmodri, "ramgs0");
cmd_uomodri_t   cmd_uOmodri =
{
 .mode.bit.TIMEOUT  = 0,
 .mode.bit.EI2OC    = true,
 .mode.bit.EI1OC    = true,
 .mode.bit.EPRE     = true,
 .mode.bit.EM2      = false,
 .mode.bit.EM1      = false,
 .mode.bit.ES       = true,
 .position          = {0.0, 0.0},
 .velocity          = {0.0, 0.0},
 .current           = {0.0, 0.0},
 .kpCoeff           = {1.0, 1.0},
 .kdCoeff           = {0.02, 0.02},
 .iSat              = {1.0, 1.0},
 .index             = 0
};
#if (defined UOMODRI_DEBUG_MASTER_BOARD_ENABLE) && (!UOMODRI_DEBUG_MASTER_BOARD_ENABLE)
#pragma DATA_SECTION(cmd_master_msg, "ramgs0");
mst2slv_msg_t   cmd_master_msg;
#endif
#pragma DATA_SECTION(led_msg, "ramgs0");
uint16_t        led_msg[6];//          = {0x0000, 0x9B6D, 0xA492, 0x4924, 0x9249, 0x2400}; // green
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

extern motor_t      motor[];
extern hal_cfg_t    hal;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void dma_isrCmdMsg(void);
__attribute__((interrupt)) void adc_isrMotor1(void);
__attribute__((interrupt)) void adc_isrMotor2(void);
__attribute__((interrupt)) void gpio_isrFaultMotor1(void);
__attribute__((interrupt)) void gpio_isrFaultMotor2(void);
#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
__attribute__((interrupt)) void cla1_isr1(void);
#endif

#endif /* __MAIN_H__ */
