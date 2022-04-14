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
// .status            = 0,
 .position.u16_lsb  = 0,
 .position.u16_msb  = 0,
 .iqRef             = 0,
// .iqPdOut           = 0,
 .iqMeas            = 0,
 .idMeas            = 0,
// .stateMachine      = 0,
 .dtc_u             = 0,
 .dtc_v             = 0,
 .dtc_w             = 0,
 .crc               = 0,
};//[DBG_TX_MSG_SIZE];

dbg_uart_addr_t dbg_uart_addr =
{
//    .status         = NULL,
    .position_msb   = NULL,
    .position_lsb   = NULL,
    .iqRef          = NULL,
//    .iqPdOut        = NULL,
    .iqMeas         = NULL,
    .idMeas         = NULL,
    .dtc_u          = NULL,
    .dtc_v          = NULL,
    .dtc_w          = NULL,
//    .stateMachine   = NULL,
//    .stateMachine[1]= NULL,
};
extern const hal_cm_t hal_cm;

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
//__attribute__((interrupt)) void ipc_isr0(void);

#endif /*__MAIN_CM_H__*/
