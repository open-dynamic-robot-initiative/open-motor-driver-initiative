//##############################################################################
//
// FILE:   usb_ex1_device_structs.c
//
// TITLE:   Data structures defining this CDC USB device.
// <h1> Data structures defining this CDC USB device. </h1>
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

//
// Included Files
//
#include "uomodri_cm_usb_device_structs.h"

//******************************************************************************
//
// The languages supported by this device.
//
//******************************************************************************
const uint8_t g_pui8LangDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

//******************************************************************************
//
// The manufacturer string.
//
//******************************************************************************
const uint8_t g_pui8ManufacturerString[] =
{
    (17 + 1) * 2,
    USB_DTYPE_STRING,
    'T', 0,
    'e', 0,
    'x', 0,
    'a', 0,
    's', 0,
    ' ', 0,
    'I', 0,
    'n', 0,
    's', 0,
    't', 0,
    'r', 0,
    'u', 0,
    'm', 0,
    'e', 0,
    'n', 0,
    't', 0,
    's', 0,
};

//******************************************************************************
//
// The product string.
//
//******************************************************************************
const uint8_t g_pui8ProdectString[] =
{
    2 + (16 * 2),
    USB_DTYPE_STRING,
    'V', 0,
    'i', 0,
    'r', 0,
    't', 0,
    'u', 0,
    'a', 0,
    'l', 0,
    ' ', 0,
    'C', 0,
    'O', 0,
    'M', 0,
    ' ', 0,
    'P', 0,
    'o', 0,
    'r', 0,
    't', 0
};

//******************************************************************************
//
// The serial number string.
//
//******************************************************************************
const uint8_t g_pui8SerialNumberString[] =
{
    2 + (8 * 2),
    USB_DTYPE_STRING,
    '1', 0,
    '2', 0,
    '3', 0,
    '4', 0,
    '5', 0,
    '6', 0,
    '7', 0,
    '8', 0
};

//******************************************************************************
//
// The control interface description string.
//
//******************************************************************************
const uint8_t g_pui8ControlInterfaceString[] =
{
    2 + (21 * 2),
    USB_DTYPE_STRING,
    'A', 0,
    'C', 0,
    'M', 0,
    ' ', 0,
    'C', 0,
    'o', 0,
    'n', 0,
    't', 0,
    'r', 0,
    'o', 0,
    'l', 0,
    ' ', 0,
    'I', 0,
    'n', 0,
    't', 0,
    'e', 0,
    'r', 0,
    'f', 0,
    'a', 0,
    'c', 0,
    'e', 0
};

//******************************************************************************
//
// The configuration description string.
//
//******************************************************************************
const uint8_t g_pui8ConfigString[] =
{
    2 + (26 * 2),
    USB_DTYPE_STRING,
    'S', 0,
    'e', 0,
    'l', 0,
    'f', 0,
    ' ', 0,
    'P', 0,
    'o', 0,
    'w', 0,
    'e', 0,
    'r', 0,
    'e', 0,
    'd', 0,
    ' ', 0,
    'C', 0,
    'o', 0,
    'n', 0,
    'f', 0,
    'i', 0,
    'g', 0,
    'u', 0,
    'r', 0,
    'a', 0,
    't', 0,
    'i', 0,
    'o', 0,
    'n', 0
};

//******************************************************************************
//
// The descriptor string table.
//
//******************************************************************************
const uint8_t * const g_pui8StringDescriptors[] =
{
    g_pui8LangDescriptor,
    g_pui8ManufacturerString,
    g_pui8ProdectString,
    g_pui8SerialNumberString,
    g_pui8ControlInterfaceString,
    g_pui8ConfigString
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_pui8StringDescriptors) / sizeof(uint8_t *))

//******************************************************************************
//
// The CDC device initialization and customization structures. In this case,
// we are using USBBuffers between the CDC device class driver and the
// application code. The function pointers and callback data values are set
// to insert a buffer in each of the data channels, transmit and receive.
//
// With the buffer in place, the CDC channel callback is set to the relevant
// channel function and the callback data is set to point to the channel
// instance data. The buffer, in turn, has its callback set to the application
// function and the callback data set to our CDC instance structure.
//
//******************************************************************************
tUSBDCDCDevice g_sCDCDevice =
{
    USB_VID_TI_1CBE,
    USB_PID_SERIAL,
    0,
    USB_CONF_ATTR_SELF_PWR,
    CtrlHandler,
    (void *)&g_sCDCDevice,
    USBBufferEventCallback,
    (void *)&g_sRxBuffer,
    USBBufferEventCallback,
    (void *)&g_sTxBuffer,
    g_pui8StringDescriptors,
    NUM_STRING_DESCRIPTORS
};

//******************************************************************************
//
// Receive buffer (from the USB perspective).
//
//******************************************************************************
tUSBBuffer g_sRxBuffer =
{
    false,                          // This is a receive buffer.
    RxHandler,                      // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketRead,              // pfnTransfer
    USBDCDCRxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pi8USBRxBuffer,               // pi8Buffer
    UART_BUFFER_SIZE                 // ui32BufferSize
};

//******************************************************************************
//
// Transmit buffer (from the USB perspective).
//
//******************************************************************************
tUSBBuffer g_sTxBuffer =
{
    true,                           // This is a transmit buffer.
    TxHandler,                      // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketWrite,             // pfnTransfer
    USBDCDCTxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pi8USBTxBuffer,               // pi8Buffer
    UART_BUFFER_SIZE                 // ui32BufferSize
};

//
// End of file
//
