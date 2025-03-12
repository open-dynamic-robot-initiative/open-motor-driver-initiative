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
extern dbg_msg_t dbg_msg_tx;
extern tUSBDCDCDevice usb_CDCDevice;
extern tUSBBuffer usb_RxBuffer;
extern tUSBBuffer usb_TxBuffer;

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
extern __attribute__((interrupt)) void ipc0_isr(void);
extern __attribute__((interrupt)) void udma_isr(void);
extern __attribute__((interrupt)) void UART_RX_IntHandler(void);
extern __attribute__((interrupt)) void CM_USB0DeviceIntHandler(void);

/************************************************************************
 * UART TX DEFINES
 ************************************************************************/
#define UART_CONFIG                     (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)
#define UART_BAUDRATE                   (12e6)
#define UART_DMA_CONFIG                 (UART_DMA_TX)
#define UART_INT_ALL                    (UART_INT_DMATX | UART_INT_DMARX | \
                                         UART_INT_9BIT | UART_INT_EOT | \
                                         UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE | \
                                         UART_INT_RT | UART_INT_TX | UART_INT_RX)
// | UART_DMA_RX,
// .intFlags          = UART_INT_DMATX | UART_INT_TX, //UART_INT_DMARX | UART_INT_TX | UART_INT_RX,
//.dmaEnable         = true,
// .intEnable         = true,
// .p_fnHandler       = NULL,//UART_RX_IntHandler(),
#define UART_DEF                        { \
                                            .uartCfg        = UART_CONFIG, \
                                            .baudRate       = UART_BAUDRATE, \
                                            .txFifoLevel    = UART_FIFO_TX4_8, \
                                            .rxFifoLevel    = UART_FIFO_RX4_8, \
                                            .dmaEnable      = true, \
                                            .dmaFlags       = UART_DMA_CONFIG, \
                                            .intEnable      = false, \
                                            .intFlags       = 0, \
                                        }

//                                                    .p_uartIntHandler   = NULL,

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

#define DMA_UART_TX_PRI_CHANNEL         (UDMA_CHANNEL_UART0_TX | UDMA_PRI_SELECT)
#define DMA_UART_TX_PRI_DATA_SIZE       (UDMA_SIZE_8)
#define DMA_UART_TX_PRI_SRC_INC         (UDMA_SRC_INC_8)
#define DMA_UART_TX_PRI_DST_INC         (UDMA_DST_INC_NONE)
#define DMA_UART_TX_PRI_BURST_ARB       (UDMA_ARB_4)
#define DMA_UART_TX_PRI_CONFIG          (DMA_UART_TX_PRI_DATA_SIZE | \
                                         DMA_UART_TX_PRI_SRC_INC | \
                                         DMA_UART_TX_PRI_DST_INC | \
                                         DMA_UART_TX_PRI_BURST_ARB)
// | UDMA_NEXT_USEBURST)
#define DMA_UART_TX_PRI_ATTRIBUTE       (UDMA_CH_ATTR_USEBURST | \
                                         UDMA_CH_ATTR_HIGH_PRIORITY)
#define DMA_UART_TX_PRI_SRC_ADDR        ((void *) &dbg_msg_tx)//.dbg_msg[0])
#define DMA_UART_TX_PRI_DST_ADDR        ((void *) (UART0_BASE + UART_O_DR))
#define DMA_UART_TX_PRI_TRANSFER_MODE   (UDMA_MODE_BASIC)
#define DMA_UART_TX_PRI_TRANSFER_SIZE   (sizeof(dbg_msg_t))
#define DMA_UART_TX_PRI_CTRL_ATTRIBUTE  (DMA_UART_TX_PRI_CONFIG | \
                                         ((DMA_UART_TX_PRI_TRANSFER_SIZE - 1) << UDMA_DMACHCTL_XFERSIZE_S) | \
                                         ((DMA_UART_TX_PRI_TRANSFER_MODE) << UDMA_DMACHCTL_XFERMODE_S))

#define DMA_UART_TX_ALT_CHANNEL         (UDMA_CHANNEL_UART0_TX | UDMA_ALT_SELECT)
#define DMA_UART_TX_ALT_CONFIG          (UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4)// | UDMA_NEXT_USEBURST)
#define DMA_UART_TX_ALT_ATTRIBUTE       (UDMA_CH_ATTR_USEBURST | UDMA_CH_ATTR_ALTSELECT | UDMA_CH_ATTR_HIGH_PRIORITY)
#define DMA_UART_TX_ALT_SRC_ADDR        ((void *) &dbg_msg_tx)//.dbg_msg[1])
#define DMA_UART_TX_ALT_DST_ADDR        ((void *) (UART0_BASE + UART_O_DR))
#define DMA_UART_TX_ALT_TRANSFER_SIZE   (sizeof(dbg_msg_t))

#define DMA_UART_TX_PRI_DEF             { \
                                            .dmaChannel     = DMA_UART_TX_PRI_CHANNEL, \
                                            .dmaCfg         = DMA_UART_TX_PRI_CONFIG, \
                                            .dmaAttribute   = DMA_UART_TX_PRI_ATTRIBUTE, \
                                            .srcAddr        = DMA_UART_TX_PRI_SRC_ADDR, \
                                            .dstAddr        = DMA_UART_TX_PRI_DST_ADDR, \
                                            .transferMode   = DMA_UART_TX_PRI_TRANSFER_MODE, \
                                            .transferSize   = DMA_UART_TX_PRI_TRANSFER_SIZE, \
                                            .dmaChEnable    = true, \
                                        }
#define DMA_UART_TX_ALT_DEF             { \
                                            .dmaChannel     = DMA_UART_TX_ALT_CHANNEL, \
                                            .dmaCfg         = DMA_UART_TX_ALT_CONFIG, \
                                            .srcAddr        = DMA_UART_TX_ALT_SRC_ADDR, \
                                            .dstAddr        = DMA_UART_TX_ALT_DST_ADDR, \
                                            .transferMode   = UDMA_MODE_BASIC, \
                                            .transferSize   = DMA_UART_TX_ALT_TRANSFER_SIZE, \
                                            .dmaChEnable    = false, \
                                        }
//                                                    .dmaIntEnable   = true, \
//                                                    .p_dmaIntHandler= &dma_isr, \

//.dmaIntEnable   = true, \
//.p_dmaIntHandler= &dma_isr, \

/************************************************************************
 * IPC DEFINES
 ************************************************************************/
/** @defgroup Debug pins configuration.
 *  @brief    Configure 2 pins (pin number and mux function) for debug.
 * @{
 */
#define IPC_SYNC_FLAG                   IPC_FLAG31

#define IPC_CM_TO_CPU1_FLAG             IPC_FLAG0

#define IPC_CPU1_TO_CM_DIR              IPC_CM_L_CPU1_R
#define IPC_CPU1_TO_CM_FLAG             IPC_FLAG0
//#define IPC_CPU1_TO_CM_IT                       IPC_INT0

#define IPC_CPU1_TO_CM_DEF              { \
                                            .ipcType        = IPC_CPU1_TO_CM_DIR, \
                                            .p_ipcMsgQ      = NULL, \
                                            .ipcIntMsgQL    = 0, \
                                            .ipcIntMsgQR    = 0, \
                                            .ipcFlag        = IPC_CPU1_TO_CM_FLAG, \
                                            .ipcMsgQEnable  = false, \
                                        }
//                                                    .ipcIntEnable   = true, \
//.ipcInt         = IPC_CPU1_TO_CM_IT, \
//.p_ipcIntHandler= &ipc0_isr,

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
