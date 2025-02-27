#ifndef __UOMODRI_CM_HAL_DEFINES_H__
#define __UOMODRI_CM_HAL_DEFINES_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "driverlib_cm.h"
#include "cm.h"

#include "communication.h"
#include "hal_cm.h"
#include "uomodri_cm_user_defines.h"

/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
extern dbg_msg_tx_t dbg_msg_tx;
extern tUSBDCDCDevice usb_CDCDevice;
extern tUSBBuffer usb_RxBuffer;
extern tUSBBuffer usb_TxBuffer;

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
extern __attribute__((interrupt)) void ipc_isr0(void);
extern __attribute__((interrupt)) void UART_RX_IntHandler(void);
extern __attribute__((interrupt)) void CM_USB0DeviceIntHandler(void);

/************************************************************************
 * DMA DEFINES
 ************************************************************************/
/** @defgroup DMA configuration.
 *  @brief    Configure DMA channels associated to UART & USB peripherals.
 * @{
 */
// {
//  .dmaChannel       = UDMA_CHANNEL_UART0_RX,
//  .dmaCfg           = (UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4 | UDMA_NEXT_USEBURST),
//  .srcAddr          = (void *)(UART0_BASE + UART_O_DR),//(void *)&dbg_uart_tx_msg,
////  .dstAddr          = (void *)(UART0_BASE + UART_O_DR),
//  .transferMode     = UDMA_MODE_BASIC,
//  .transferSize     = sizeof(dbg_uart_msg_t),
// },

#define DMA_UART_TX_PRI_CHANNEL                 (UDMA_CHANNEL_UART0_TX | UDMA_PRI_SELECT)
#define DMA_UART_TX_PRI_CONFIG                  (UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1 | UDMA_NEXT_USEBURST)
#define DMA_UART_TX_PRI_SRC_ADDR                ((uint8_t *) &dbg_msg_tx.dbg_msg[0])
#define DMA_UART_TX_PRI_DST_ADDR                ((uint8_t *) (UART0_BASE + UART_O_DR))
#define DMA_UART_TX_PRI_TRANSFER_SIZE           (sizeof(dbg_msg_t))

#define DMA_UART_TX_ALT_CHANNEL                 (UDMA_CHANNEL_UART0_TX | UDMA_ALT_SELECT)
#define DMA_UART_TX_ALT_CONFIG                  (UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1 | UDMA_NEXT_USEBURST)
#define DMA_UART_TX_ALT_SRC_ADDR                ((uint8_t *) &dbg_msg_tx.dbg_msg[1])
#define DMA_UART_TX_ALT_DST_ADDR                ((uint8_t *) (UART0_BASE + UART_O_DR))
#define DMA_UART_TX_ALT_TRANSFER_SIZE           (sizeof(dbg_msg_t))

#define DMA_UART_TX_PRI_DEF                     { \
                                                    .dmaChannel     = DMA_UART_TX_PRI_CHANNEL, \
                                                    .dmaCfg         = DMA_UART_TX_PRI_CONFIG, \
                                                    .srcAddr        = DMA_UART_TX_PRI_SRC_ADDR, \
                                                    .dstAddr        = DMA_UART_TX_PRI_DST_ADDR, \
                                                    .transferMode   = UDMA_MODE_PINGPONG, \
                                                    .transferSize   = DMA_UART_TX_PRI_TRANSFER_SIZE, \
                                                }
#define DMA_UART_TX_ALT_DEF                     { \
                                                    .dmaChannel     = DMA_UART_TX_ALT_CHANNEL, \
                                                    .dmaCfg         = DMA_UART_TX_ALT_CONFIG, \
                                                    .srcAddr        = DMA_UART_TX_ALT_SRC_ADDR, \
                                                    .dstAddr        = DMA_UART_TX_ALT_DST_ADDR, \
                                                    .transferMode   = UDMA_MODE_PINGPONG, \
                                                    .transferSize   = DMA_UART_TX_ALT_TRANSFER_SIZE, \
                                                }

/************************************************************************
 * IPC DEFINES
 ************************************************************************/
/** @defgroup Debug pins configuration.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#define IPC_SYNC_FLAG                           IPC_FLAG31
#define IPC_CPU1_TO_CM_FLAG                     IPC_FLAG0
#define IPC_CM_TO_CPU1_FLAG                     IPC_FLAG0
/**
 * @}
 */


//
///************************************************************************
// * VARIABLES INITIALIZATION
// ************************************************************************/
//static const uart_cfg_t cm_uartCfg =
//{
// .uartCfg           = (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE),
// .baudRate          = 12000000,
// .dmaFlags          = UART_DMA_TX,// | UART_DMA_RX,
//// .intFlags          = UART_INT_DMATX | UART_INT_TX, //UART_INT_DMARX | UART_INT_TX | UART_INT_RX,
// .dmaEnable         = true,
//// .intEnable         = true,
//// .p_fnHandler       = NULL,//UART_RX_IntHandler(),
//};


#endif  /*__UOMODRI_CM_HAL_DEFINES_H__*/
