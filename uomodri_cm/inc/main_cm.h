#ifndef __MAIN_CM_H__
#define __MAIN_CM_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "driverlib_cm.h"
#include "cm.h"

#include "foc.h"
#include "communication.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_hal_config_handlers.h"

/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
#pragma DATA_ALIGN(ucControlTable, 1024)
UDMA_ControlTable ucControlTable[32];
dbg_uart_msg_t dbg_uart_tx_msg =
{
 .header[0]         = 'a',
 .header[1]         = 'b',
 .count             = 0,
 .position          = 0.0,
 .iqRef             = 0.0,
 .iq                = 0.0,
 .id                = 0.0,
 .speed             = 0.0,
 .speedMech         = 0.0,
 .crc               = 0,
};

dbg_uart_addr_t dbg_uart_addr =
{
 .p_position        = NULL,
 .p_iqRef           = NULL,
 .p_iq              = NULL,
 .p_id              = NULL,
 .p_speed           = NULL,
 .p_speedMech       = NULL,
};

extern const hal_cm_t hal_cm;

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
//__attribute__((interrupt)) void ipc_isr0(void);

#endif /*__MAIN_CM_H__*/
