#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "driverlib.h"
#include "device.h"
#include "hw_types.h"
#include "communication.h"
//#include "usblib.h"
//#include "usbcdc.h"
//#include "usb_ids.h"
//#include "device/usbdevice.h"
//#include "device/usbdcdc.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
#pragma DATA_ALIGN(cmd_msg, 8)
#pragma DATA_SECTION(cmd_msg, "ramgs1")
mst2slv_msg_t   cmd_msg;

#pragma DATA_ALIGN(status_msg, 8)
#pragma DATA_SECTION(status_msg, "ramgs1")
slv2mst_msg_t   status_msg;

#pragma DATA_SECTION(led_msg, "ramgs1")
uint16_t        led_msg[LED_MSG_TX_16BIT_LENGTH];

#if (defined DEBUG) && (CM_CORE_ENABLE) && (0)
#pragma DATA_SECTION(cpu2cm_dbg_msg, "MSGRAM_CPU_TO_CM");
com_cpu_2_cm_t  cpu2cm_dbg_msg;
#endif

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
//tUSBBuffer g_sTxBuffer =
//{
//    true,                           // This is a transmit buffer.
//    TxHandler,                      // pfnCallback
//    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
//    USBDCDCPacketWrite,             // pfnTransfer
//    USBDCDCTxPacketAvailable,       // pfnAvailable
//    (void *)&g_sCDCDevice,          // pvHandle
//    g_pi8USBTxBuffer,               // pi8Buffer
//    SCI_BUFFER_SIZE                 // ui32BufferSize
//};
//
//tUSBBuffer g_sRxBuffer =
//{
//    false,                          // This is a receive buffer.
//    RxHandler,                      // pfnCallback
//    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
//    USBDCDCPacketRead,              // pfnTransfer
//    USBDCDCRxPacketAvailable,       // pfnAvailable
//    (void *)&g_sCDCDevice,          // pvHandle
//    g_pi8USBRxBuffer,               // pi8Buffer
//    SCI_BUFFER_SIZE                 // ui32BufferSize
//};

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void adc_isrMotor1(void);
__attribute__((interrupt)) void adc_isrMotor2(void);

#endif /* __MAIN_H__ */
