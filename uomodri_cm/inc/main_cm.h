#ifndef __MAIN_CM_H__
#define __MAIN_CM_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
//#include <stdbool.h>
//#include <stdint.h>
#include <math.h>

#include "driverlib_cm.h"
#include "cm.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_types.h"
#include "usb.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usb_ids.h"
#include "device/usbdevice.h"
#include "device/usbdcdc.h"

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
//__attribute__((interrupt)) void ipc_isr0(void);
//
//#define UART_BUFFER_SIZE        256
//
//extern tUSBBuffer g_sTxBuffer;
//extern tUSBBuffer g_sRxBuffer;
//extern tUSBDCDCDevice g_sCDCDevice;
//
//uint8_t g_pi8USBRxBuffer[UART_BUFFER_SIZE];
//uint8_t g_pui8RxBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
//uint8_t g_pi8USBTxBuffer[UART_BUFFER_SIZE];
//uint8_t g_pui8TxBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
//
//////******************************************************************************
//////
////// CDC device callback function prototypes.
//////
//////*****************************************************************************
////uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
////                   void *pvMsgData);
////uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
////                   void *pvMsgData);
////uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event,
////                        uint32_t ui32MsgValue, void *pvMsgData);
////******************************************************************************
////
//// CDC device callback function prototypes.
////
////*****************************************************************************
//uint32_t RxHandler(void*, uint32_t, uint32_t, void*);
//uint32_t TxHandler(void*, uint32_t, uint32_t, void*);
//uint32_t CtrlHandler(void*, uint32_t, uint32_t, void*);

#endif /*__MAIN_CM_H__*/
