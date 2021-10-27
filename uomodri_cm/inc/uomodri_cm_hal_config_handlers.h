#ifndef __UOMODRI_CM_HAL_CONFIG_HANDLERS_H__
#define __UOMODRI_CM_HAL_CONFIG_HANDLERS_H__

// drivers
#include "driverlib_cm.h"
#include "cm.h"

#include "hal_cm.h"
#include "uomodri_cm_user_defines.h"

//extern float32_t    arrayTx[4];
//extern float32_t    arrayRx[4];
#pragma DATA_SECTION(msgArrayTx, "MSGRAM_CM_TO_CPU1");
extern uint8_t msgArrayTx[MSG_TX_SIZE];
#pragma DATA_SECTION(msgArrayRx, "MSGRAM_CM_TO_CPU1");
extern uint8_t msgArrayRx[MSG_RX_SIZE];


extern __interrupt void IPC_ISR0(void);

static const UART_TypeDef uartCfg =
{.uartCfg           = (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE),
 .baudRate          = 12000000,//7372800, //3686400, //115200,
 .dmaFlags          = (UART_DMA_TX | UART_DMA_RX),
 .dmaEnable         = true,
 .loopBackEnable    = true,
};

static const uDMA_TypeDef cm_udmaCfgList[] =
{
 {.dmaChannel   = UDMA_CHANNEL_UART0_RX,
  .dmaCfg       = (UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4 | UDMA_NEXT_USEBURST),
//  .flagAttr     = UDMA_CH_ATTR_ALL,
//  .index        = ,
  .srcAddr      = (void *)(UART0_BASE + UART_O_DR),
  .dstAddr      = (void *)msgArrayRx,
  .transferMode = UDMA_MODE_BASIC,
  .transferSize = MSG_RX_SIZE,
 },
 {.dmaChannel   = UDMA_CHANNEL_UART0_TX,
  .dmaCfg       = (UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4 | UDMA_NEXT_USEBURST),
//  .flagAttr     = UDMA_CH_ATTR_ALL,
  .srcAddr      = (void *)msgArrayTx,
  .dstAddr      = (void *)(UART0_BASE + UART_O_DR),
  .transferMode = UDMA_MODE_BASIC,
  .transferSize = MSG_TX_SIZE,
 },
 {.dmaChannel   = 0xFFFFFFFF},
};

static const IPC_TypeDef cm_ipcCfgList[] =
{
 {.ipcType          = IPC_CM_L_CPU1_R,
  .p_ipcMsgQ        = NULL,
  .ipcFlag          = IPC_FLAG0,
  .ipcInt           = IPC_INT0,
  .p_ipcIntHandler  = &IPC_ISR0,
  .ipcMsgQEnable    = false,
  .ipcIntEnable     = true,
 },
 {.ipcType          = IPC_CM_L_CPU1_R,
  .p_ipcMsgQ        = NULL,
  .ipcFlag          = IPC_FLAG1,
  .p_ipcIntHandler  = NULL,
  .ipcMsgQEnable    = false,
  .ipcIntEnable     = false
 },
 {.ipcType          = IPC_TOTAL_NUM},
};

static const CRC_TypeDef crcCfg =
{.poly          = 0x04C11DB7,
 .width         = 32,
 .seed          = 0xffffffff,
 .endianness    = GCRC_ENDIANNESS_BIG,
 .dataType      = GCRC_DATATYPE_32BIT,
 .bitRev        = false,
 .dataMask      = 0,
};


/** @var    HAL     hal
 *  @brief  General configuration structure for the Hardware Abstraction Layer.
 */
static const HAL_CM hal_cm =
{.p_uartHandler = &uartCfg,
 .p_udmaHandler = cm_udmaCfgList,
 .p_ipcHandler  = cm_ipcCfgList,
 .p_crcHandler  = NULL,//&crcCfg,
};

#endif  /*__UOMODRI_HAL_CM_CONFIG_HANDLERS_H__*/
