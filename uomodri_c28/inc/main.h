#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdint.h>
#include <hw_types.h>
#include <driverlib.h>
#include <device.h>

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

#pragma DATA_SECTION(led_msg, "ramgs1")
uint16_t        led_msg[LED_MSG_TX_16BIT_LENGTH];

#if (defined DEBUG) && (CM_CORE_ENABLE) && (0)
#pragma DATA_SECTION(cpu2cm_dbg_msg, "MSGRAM_CPU_TO_CM");
com_cpu_2_cm_t  cpu2cm_dbg_msg;
#endif

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void adc_isrMotor1(void);
__attribute__((interrupt)) void adc_isrMotor2(void);

#endif /* __MAIN_H__ */
