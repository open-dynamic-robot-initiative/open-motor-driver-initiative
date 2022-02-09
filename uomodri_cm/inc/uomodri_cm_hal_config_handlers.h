#ifndef __UOMODRI_CM_HAL_CONFIG_HANDLERS_H__
#define __UOMODRI_CM_HAL_CONFIG_HANDLERS_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "driverlib_cm.h"
#include "cm.h"

#include "hal_cm.h"
#include "uomodri_cm_user_defines.h"

/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
extern uint8_t dbg_uart_tx_msg[DBG_TX_MSG_SIZE];

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
extern __interrupt void ipc_isr0(void);

/************************************************************************
 * VARIABLES INITIALIZATION
 ************************************************************************/
static const uart_cfg_t cm_uartCfg =
{.uartCfg           = (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE),
 .baudRate          = 12000000,
 .dmaFlags          = UART_DMA_TX,
 .dmaEnable         = true,
 .loopBackEnable    = false,
};

static const udma_cfg_t cm_udmaCfgList[] =
{
 {.dmaChannel       = UDMA_CHANNEL_UART0_TX,
  .dmaCfg           = (UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4 | UDMA_NEXT_USEBURST),
  .srcAddr          = (void *)dbg_uart_tx_msg,
  .dstAddr          = (void *)(UART0_BASE + UART_O_DR),
  .transferMode     = UDMA_MODE_BASIC,
  .transferSize     = DBG_TX_MSG_SIZE,
 },
 {.dmaChannel       = UINT32_MAX},
};

static const ipc_cfg_t  cm_ipcCfgList[] =
{
 {.ipcType          = IPC_CM_L_CPU1_R,
  .p_ipcMsgQ        = NULL,
  .ipcIntMsgQL      = 0,
  .ipcIntMsgQR      = 0,
  .ipcFlag          = IPC_FLAG0,
  .ipcInt           = IPC_INT0,
  .p_ipcIntHandler  = &ipc_isr0,
  .ipcMsgQEnable    = false,
  .ipcIntEnable     = true,
 },
 {.ipcType          = IPC_TOTAL_NUM},
};

static const gcrc_cfg_t cm_crcCfg =
{.poly              = 0x04C11DB7,
 .polysize          = 32,
 .seed              = 0xffffffff,
 .endianness        = GCRC_ENDIANNESS_BIG,
 .dataType          = GCRC_DATATYPE_32BIT,
 .dataMask          = 0,
 .bitRev            = false,
};


/** @var    HAL     hal
 *  @brief  General configuration structure for the Hardware Abstraction Layer.
 */
static const hal_cm_t   hal_cm =
{.p_uartHandle      = &cm_uartCfg,
 .p_udmaHandle      = cm_udmaCfgList,
 .p_ipcHandle       = cm_ipcCfgList,
 .p_crcHandle       = &cm_crcCfg,
};

#endif  /*__UOMODRI_CM_HAL_CONFIG_HANDLERS_H__*/
