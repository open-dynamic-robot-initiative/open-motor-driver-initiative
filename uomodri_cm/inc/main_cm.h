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
#include "communication_cm.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_hal_config_handlers.h"

/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
#pragma DATA_ALIGN(ucControlTable, 1024)
UDMA_ControlTable ucControlTable[32];
uint8_t dbg_uart_tx_msg[DBG_TX_MSG_SIZE];
extern const hal_cm_t hal_cm;

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
__attribute__((interrupt)) void ipc_isr0(void);

#endif /*__MAIN_CM_H__*/
