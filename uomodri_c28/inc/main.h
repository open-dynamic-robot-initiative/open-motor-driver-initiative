#ifndef __MAIN_H__
#define __MAIN_H__

/*****************************************************************************
 * includes
 *****************************************************************************/
// Project includes
//#include "communication.h"
#include "f2838x_device.h"
//#include "f2838x_cla_defines.h"
#include "f2838x_examples.h"
#include "driverlib.h"
#include "device.h"
//#include "sysctl.h"

#include "hal.h"
#include "drv8353.h"
#include "encoder.h"
#include "foc.h"
#include "motor.h"

#include "communication_interface.h"
#include "legacy_spi.h"

#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"
#include "uomodri_prj_config_handlers.h"
#include "uOmodriClaTasksShare.h"

/*****************************************************************************
 * defines
 *****************************************************************************/
#define IPC_CMD_READ_MEM   0x1001
#define IPC_CMD_RESP       0x2001

#define TIME_M1_S   ((float32_t)motor[MOTOR_1].interruptCnt * dt)
#define TIME_M1_MS  ((float32_t)motor[motor_1].interruptCnt * 1000.0f * dt)
#define TIME_M2_S   ((float32_t)motor[MOTOR_2].interruptCnt * dt)
#define TIME_M2_MS  ((float32_t)motor[motor_2].interruptCnt * 1000.0f * dt)

/*****************************************************************************
 * globals
 *****************************************************************************/
#if (CM == 1)
#pragma DATA_SECTION(motorCpuToCM, "MSGRAM_CPU_TO_CM")
FOC_STRUCT motorCpuToCM[2];
#pragma DATA_SECTION(motorCMToCpu, "MSGRAM_CM_TO_CPU")
FOC_STRUCT motorCMToCpu[2];
#endif

//#pragma DATA_SECTION(motor_enc_in, "CpuToCla1MsgRAM");
//ENC_STRUCT motor_enc_in;// = motor[MOTOR_1].p_motorFOC->motor_enc;
#if (CLA == 1)
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

// Sine for debug
float32_t           debugFreq = 1.0f;
float32_t           pdSat = 4.0;
// Time macro
const float32_t     dt = 1.0 / PWM_FREQ;
bool                isrMotor1Ready = false;
bool                isrMotor2Ready = false;

// <Step 2>: The CLA can only operate out of RAM i.e. its program and data spaces are restricted
//           to RAM. Depending on the device, the CLA may be configured to access the entire lower
//           64K of RAM (F2837x) or its prorgam and data spaces are restricted to certain RAMs
//           (F2806x, F2803x, F2805x program -> RAML3, data -> RAML0,1,/2)
//  - In standalone mode, you must load the CLA code and constants to FLASH  and then copy
//    these over to RAM at runtime
//  - The linker defines placement variables for the section "Cla1Prog" and ".const_cla"
//  - In order to use these variables in a memcpy() we need to tell the C compiler that
//    these variables are declared externally (by the linker). We use the "extern" storage
//    qualifier to do just that
//extern uint32_t Cla1ProgRunStart, Cla1ProgLoadStart, Cla1ProgLoadSize;
//extern uint32_t Cla1ConstRunStart, Cla1ConstLoadStart, Cla1ConstLoadSize;

extern MOTOR_STRUCT motor[];
extern const HAL    hal;

/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/
static void MCU_init();
// Step 3: Setup the CLA in initCLA() and its memory resources in
//         configCLAMemory()
//  - see the initCLA() and configCLAMemory() for more steps
//  - add the End-of-Task 1 prototype here
//void CLA_initCpu1Cla1( void );
//void CLA_configClaMemory(void);
__attribute__((interrupt)) void adc_isrMotor1(void);
__attribute__((interrupt)) void adc_isrMotor2(void);
//extern __interrupt void pwm_isrMotor1(void);
//extern __interrupt void pwm_isrMotor2(void);
__attribute__((interrupt)) void gpio_isrFault1(void);
__attribute__((interrupt)) void gpio_isrFault2(void);
#if (CLA == 1)
__attribute__((interrupt)) void cla1_isr1(void);
#endif
#endif // end of MAIN_H definition
