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
 * ENUM TYPES
 ***********************************************************************/

/***********************************************************************
 * UNION TYPES
 ***********************************************************************/
typedef union __uart_addr_u__
{
    uint8_t             addr;
    struct
    {
        uint8_t         addr_0          : 1;    /*!< Bit 0      : R/W - Address bit 0. */
        uint8_t         addr_1          : 1;    /*!< Bit 1      : R/W - Address bit 1. */
        uint8_t         addr_2          : 1;    /*!< Bit 2      : R/W - Address bit 2. */
    };
} uart_addr_u;

/***********************************************************************
 * DEFINES
 ***********************************************************************/

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
#pragma DATA_ALIGN(cmd_msg, 4)
#pragma DATA_SECTION(cmd_msg, "ramgs1")
mst2slv_msg_t   cmd_msg;

#pragma DATA_ALIGN(status_msg, 4)
#pragma DATA_SECTION(status_msg, "ramgs1")
slv2mst_msg_t   status_msg[2];
//#pragma DATA_ALIGN(status_tmp_msg, 4)
//#pragma DATA_SECTION(status_tmp_msg, "ramgs1")
//slv2mst_msg_t   status_tmp_msg;

#pragma DATA_ALIGN(led_msg, 4)
#pragma DATA_SECTION(led_msg, "ramgs1")
uint16_t        led_msg[LED_MSG_TX_16BIT_LENGTH];

#if (defined DEBUG) && (CM_CORE_ENABLE) && (0)
#pragma DATA_SECTION(cpu2cm_dbg_msg, "MSGRAM_CPU_TO_CM");
com_cpu_2_cm_t  cpu2cm_dbg_msg;
#endif

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
//__attribute__((interrupt)) void adc_isrMotor1(void);
//__attribute__((interrupt)) void adc_isrMotor2(void);

#endif /* __MAIN_H__ */
