#ifndef __MAIN_CM_H__
#define __MAIN_CM_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "driverlib_cm.h"
#include "cm.h"

#include "foc.h"
#include "communication_cm.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_hal_config_handlers.h"

//#pragma DATA_SECTION(uomodri_cmd, "MSGRAM_CM_TO_CPU1");
//uomodri_cmd uomodri_cmd[];

// Structure that contains useful variables for master communication
//typedef struct __ComInterface__
//{
//    // Enabled system flag
//    bool systemEnabled;
//
//    // Last received command index
//    uint16_t lastCommandIndex;
//
//    // Time variables
//    float32_t time[2];             // Time in seconds since startup
//    uint64_t lastMessageMs;     // Last time stamp (milliseconds) at which a message has been received
//    uint16_t timeoutThreshold;  // Duration in milliseconds before triggering timeout
//
//    // Position roll-over error flags
//    bool rolloverErrorFlag;
//
//    // CAN error
//    uint16_t canError;
//
//    // Controller references
//    // Position references
//    float32_t positionRef[2];
//
//    // Velocity references
//    float32_t velocityRef[2];
//
//    // Quadrature current references
//    float32_t iqRef[2];
//
//    // Data resolutions
//    const float32_t positionLsb;
//    const float32_t velocityLsb;
//    const float32_t iqLsb;
//    const float32_t currentSatLsb;
//    const float32_t resistanceLsb;
//    const float32_t voltageLsb;
//    const float32_t kpLsb;
//    const float32_t kdLsb;
//
//} ComInterface;

__attribute__((interrupt)) void ipc_isr0(void);
__attribute__((interrupt)) void ipc_isr1(void);

#pragma DATA_ALIGN(ucControlTable, 1024)
UDMA_ControlTable ucControlTable[32];
uint8_t dbg_tx_msg[DBG_TX_MSG_SIZE];
#pragma DATA_SECTION(msgArrayRx, "MSGRAM_CM_TO_CPU1");
uint8_t msgArrayRx[MSG_RX_SIZE];

//#pragma DATA_SECTION(com_rx_cmd, "MSGRAM_CM_TO_CPU1");
rx_uomodri_t __attribute__((section (".ComRxCmdSection"))) com_rx_cmd;
//#pragma DATA_SECTION(com_tx_msg, "MSGRAM_CM_TO_CPU1");
//tx_msg_t com_tx_msg;
tx_msg_t __attribute__((section (".ComTxMsgSection"))) com_tx_msg;

extern const HAL_CM hal_cm;

#endif /*__MAIN_CM_H__*/
