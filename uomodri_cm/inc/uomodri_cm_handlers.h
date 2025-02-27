#ifndef __UOMODRI_CM_HANDLERS_H__
#define __UOMODRI_CM_HANDLERS_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "driverlib_cm.h"
#include "cm.h"

#include "communication.h"
#include "hal_cm.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_hal_defines.h"

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
 DMA_UART_TX_PRI_DEF,
 DMA_UART_TX_ALT_DEF,
 {.dmaChannel       = UINT32_MAX},
};

static const ipc_cfg_t cm_ipcCfgList[] =
{
 {
  .ipcType          = IPC_CM_L_CPU1_R,
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
{
 .poly              = GCRC_FIXEDPATH_POLY,
 .polysize          = GCRC_FIXEDPATH_POLYSIZE,
 .seed              = UINT32_MAX,
 .endianness        = GCRC_ENDIANNESS_BIG,
 .dataType          = GCRC_DATATYPE_32BIT,
 .dataMask          = GCRC_FIXEDPATH_DATAMASK,
 .bitRev            = GCRC_FIXEDPATH_BITREVERSE,
};

static usb_cfg_t cm_usbCfg =
{
 .p_usbRxBuf        = &usb_RxBuffer,
 .p_usbTxBuf        = &usb_TxBuffer,
 .p_usbCdcDevice    = &usb_CDCDevice,
 .p_usbIntHandler   = &CM_USB0DeviceIntHandler,
// .usbCfgDone        = false,
};

/** @var    HAL     hal
 *  @brief  General configuration structure for the Hardware Abstraction Layer.
 */
static const hal_cm_t   hal_cm =
{
#if (!USB_BUS_ENABLE) && (UART_BUS_ENABLE)
 .p_uartHandle      = &cm_uartCfg,
 .p_usbHandle       = NULL,
#elif (USB_BUS_ENABLE) && (!UART_BUS_ENABLE)
 .p_uartHandle      = NULL,
 .p_usbHandle       = &cm_usbCfg,
#else
 .p_uartHandle      = NULL,
 .p_usbHandle       = NULL,
#endif
 .p_udmaHandle      = cm_udmaCfgList,
 .p_ipcHandle       = cm_ipcCfgList,
 .p_crcHandle       = NULL,//&cm_crcCfg,
};

#endif  /*__UOMODRI_CM_HANDLERS_H__*/
