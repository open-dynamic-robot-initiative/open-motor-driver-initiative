//##############################################################################
//
// FILE:   usb_ex1_device_structs.h
//
// TITLE:  Data structures defining this USB CDC device.
//
//##############################################################################
//
//
// 
// C2000Ware v5.04.00.00
//
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//##############################################################################

#ifndef USB_EX1_DEVICE_STRUCTS_H
#define USB_EX1_DEVICE_STRUCTS_H

//
// Included Files
//
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_types.h"
#include "usb.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usb_ids.h"
#include "device/usbdevice.h"
#include "device/usbdcdc.h"

//
// Defines
//
#define UART_BUFFER_SIZE        256

extern tUSBBuffer usb_TxBuffer;
extern tUSBBuffer usb_RxBuffer;
extern tUSBDCDCDevice usb_CDCDevice;

uint8_t g_pi8USBRxBuffer[UART_BUFFER_SIZE];
uint8_t g_pui8RxBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
uint8_t g_pi8USBTxBuffer[UART_BUFFER_SIZE];
uint8_t g_pui8TxBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];

//******************************************************************************
//
// CDC device callback function prototypes.
//
//*****************************************************************************
//uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
//uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
//uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
uint32_t usb_RxHandler(void*, uint32_t, uint32_t, void*);
uint32_t usb_TxHandler(void*, uint32_t, uint32_t, void*);
uint32_t usb_CtrlHandler(void*, uint32_t, uint32_t, void*);


#endif // USB_EX1_DEVICE_STRUCTS_H
