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
extern dbg_uart_msg_t dbg_uart_tx_msg;

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
//extern __interrupt void ipc_isr0(void);
extern __interrupt UART_RX_IntHandler(void);

/************************************************************************
 * VARIABLES INITIALIZATION
 ************************************************************************/
static const uart_cfg_t cm_uartCfg =
{
 .uartCfg           = (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE),
 .baudRate          = 12000000,
 .dmaFlags          = UART_DMA_TX,// | UART_DMA_RX,
// .intFlags          = UART_INT_DMATX | UART_INT_TX, //UART_INT_DMARX | UART_INT_TX | UART_INT_RX,
 .dmaEnable         = true,
// .intEnable         = true,
// .p_fnHandler       = NULL,//UART_RX_IntHandler(),
};

static const udma_cfg_t cm_udmaCfgList[] =
{
// {
//  .dmaChannel       = UDMA_CHANNEL_UART0_RX,
//  .dmaCfg           = (UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4 | UDMA_NEXT_USEBURST),
//  .srcAddr          = (void *)(UART0_BASE + UART_O_DR),//(void *)&dbg_uart_tx_msg,
////  .dstAddr          = (void *)(UART0_BASE + UART_O_DR),
//  .transferMode     = UDMA_MODE_BASIC,
//  .transferSize     = sizeof(dbg_uart_msg_t),
// },
 {
  .dmaChannel       = UDMA_CHANNEL_UART0_TX,
  .dmaCfg           = (UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_8 | UDMA_NEXT_USEBURST),
  .srcAddr          = (void *)&dbg_uart_tx_msg,
  .dstAddr          = (void *)(UART0_BASE + UART_O_DR),
  .transferMode     = UDMA_MODE_BASIC,
  .transferSize     = sizeof(dbg_uart_msg_t),
 },
 {.dmaChannel       = UINT32_MAX},
};

static const ipc_cfg_t  cm_ipcCfgList[] =
{
 {
  .ipcType          = IPC_CM_L_CPU1_R,
  .ipcFlag          = IPC_CPU1_TO_CM_FLAG,
  //  .ipcInt           = IPC_INT0,
  //  .p_ipcIntHandler  = &ipc_isr0,
  .ipcMsgQEnable    = false,
  .ipcIntEnable     = false,
 },
 {.ipcType          = IPC_TOTAL_NUM},
};

static const gcrc_cfg_t cm_crcCfg =
{
 .poly              = GCRC_FIXEDPATH_POLY,
 .polysize          = GCRC_FIXEDPATH_POLYSIZE,
 .seed              = UINT32_MAX,
 .endianness        = GCRC_ENDIANNESS_BIG,
 .dataType          = GCRC_DATATYPE_32BIT,
 .dataMask          = GCRC_FIXEDPATH_DATAMASK,
 .bitRev            = GCRC_FIXEDPATH_BITREVERSE,
};


/** @var    HAL     hal
 *  @brief  General configuration structure for the Hardware Abstraction Layer.
 */
static const hal_cm_t   hal_cm =
{
 .p_uartHandle      = &cm_uartCfg,
 .p_udmaHandle      = cm_udmaCfgList,
 .p_ipcHandle       = cm_ipcCfgList,
 .p_crcHandle       = NULL,//&cm_crcCfg,
};

#endif  /*__UOMODRI_CM_HAL_CONFIG_HANDLERS_H__*/
