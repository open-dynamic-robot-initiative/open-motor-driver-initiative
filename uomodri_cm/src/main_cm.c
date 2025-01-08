//
// Included Files
//
//#include <stdint.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <string.h>
//#include <float.h>

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <string.h>
#include "main_cm.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usb_ids.h"
#include "device/usbdevice.h"
#include "device/usbdcdc.h"
#include "ustdlib.h"
//#include "usb_ex1_device_structs.h"
//#include "usb_hal.h"

//#include "foc.h"
#include "communication.h"
#include "uomodri_cm_usb_device_structs.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_hal_config_handlers.h"


/***********************************************************************
 * DEFINES
 ***********************************************************************/
tUSBMode g_eCurrentUSBMode;   // The current USB operating mode - Host, Device or unknown.

//******************************************************************************
//
// Flag indicating whether or not we are currently sending a Break condition.
//
//******************************************************************************
//static bool g_bSendingBreak = false;

//******************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//******************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002
#define COMMAND_RECEIVED        0x00000004

tLineCoding psLineCoding;
volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;
uint32_t dataSize;
uint8_t g_char_tmp[256];

//****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//****************************************************************************
static volatile bool g_bUSBConfigured = false;


/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
inline void IPC_CpuToCmAddrSync(dbg_uart_addr_t*);
__interrupt void CM_USB0DeviceIntHandler(void);

////******************************************************************************
////
//// CDC device callback function prototypes.
////
////*****************************************************************************
//uint32_t RxHandler(void*, uint32_t, uint32_t, void*);
//uint32_t TxHandler(void*, uint32_t, uint32_t, void*);
//uint32_t CtrlHandler(void*, uint32_t, uint32_t, void*);

/************************************************************************
 * LOCAL VARIABLES
 ************************************************************************/
#pragma DATA_ALIGN(ucControlTable, 1024)
UDMA_ControlTable ucControlTable[32];
dbg_uart_msg_t dbg_uart_tx_msg =
{
 .header[0]         = 'a',
 .header[1]         = 'b',
// .header[2]         = 'c',
// .count             = 0U,
 .ia                = 0.0f,
 .ib                = 0.0f,
 .ic                = 0.0f,
// .vbus              = 0.0f,
// .ialpha            = 0.0f,
// .ibeta             = 0.0f,
 .id                = 0.0f,
 .iq                = 0.0f,
 .iqref             = 0.0f,
 .ud                = 0.0f,
 .uq                = 0.0f,
// .ualpha            = 0.0f,
// .ubeta             = 0.0f,
// .ua                = 0.0f,
// .ub                = 0.0f,
// .uc                = 0.0f,
 .pos               = 0.0f,
// .posref            = 0.0f,
 .vel               = 0.0f,
// .velref            = 0.0f,
// .itcnt             = 0U,
 .crc               = 0,
// .padding           = 0,
};

dbg_uart_addr_t dbg_uart_addr =
{
 .p_ia              = NULL,
 .p_ib              = NULL,
 .p_ic              = NULL,
 .p_vbus            = NULL,
 .p_ialpha          = NULL,
 .p_ibeta           = NULL,
 .p_id              = NULL,
 .p_iq              = NULL,
 .p_ud              = NULL,
 .p_uq              = NULL,
 .p_ualpha          = NULL,
 .p_ubeta           = NULL,
 .p_ua              = NULL,
 .p_ub              = NULL,
 .p_uc              = NULL,
 .p_pos             = NULL,
 .p_posref          = NULL,
 .p_vel             = NULL,
 .p_velref          = NULL,
 .p_iqref           = NULL,
// .p_itcnt           = NULL,
};



//#define UART_BUFFER_SIZE        256
//#define NUM_STRING_DESCRIPTORS (sizeof(g_pui8StringDescriptors) / sizeof(uint8_t *))
//
////******************************************************************************
////
//// Receive buffer (from the USB perspective).
////
////******************************************************************************
//tUSBBuffer g_sRxBuffer =
//{
//    false,                          // This is a receive buffer.
//    RxHandler,                      // pfnCallback
//    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
//    USBDCDCPacketRead,              // pfnTransfer
//    USBDCDCRxPacketAvailable,       // pfnAvailable
//    (void *)&g_sCDCDevice,          // pvHandle
//    g_pi8USBRxBuffer,               // pi8Buffer
//    UART_BUFFER_SIZE                 // ui32BufferSize
//};
//
////******************************************************************************
////
//// Transmit buffer (from the USB perspective).
////
////******************************************************************************
//tUSBBuffer g_sTxBuffer =
//{
//    true,                           // This is a transmit buffer.
//    TxHandler,                      // pfnCallback
//    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
//    USBDCDCPacketWrite,             // pfnTransfer
//    USBDCDCTxPacketAvailable,       // pfnAvailable
//    (void *)&g_sCDCDevice,          // pvHandle
//    g_pi8USBTxBuffer,               // pi8Buffer
//    UART_BUFFER_SIZE                 // ui32BufferSize
//};
//
////******************************************************************************
////
//// The CDC device initialization and customization structures. In this case,
//// we are using USBBuffers between the CDC device class driver and the
//// application code. The function pointers and callback data values are set
//// to insert a buffer in each of the data channels, transmit and receive.
////
//// With the buffer in place, the CDC channel callback is set to the relevant
//// channel function and the callback data is set to point to the channel
//// instance data. The buffer, in turn, has its callback set to the application
//// function and the callback data set to our CDC instance structure.
////
////******************************************************************************
//tUSBDCDCDevice g_sCDCDevice =
//{
//    USB_VID_TI_1CBE,
//    USB_PID_SERIAL,
//    0,
//    USB_CONF_ATTR_SELF_PWR,
//    CtrlHandler,
//    (void *)&g_sCDCDevice,
//    USBBufferEventCallback,
//    (void *)&g_sRxBuffer,
//    USBBufferEventCallback,
//    (void *)&g_sTxBuffer,
//    g_pui8StringDescriptors,
//    NUM_STRING_DESCRIPTORS
//};
/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
extern const hal_cm_t hal_cm;

////****************************************************************************
////
//// Get the communication parameters in use on the UART.
////
////****************************************************************************
//static void
//GetLineCoding(tLineCoding *psLineCoding)
//{
//    // Get the current line coding set in the UART.
//    psLineCoding->ui8Databits   = 8U;
//    psLineCoding->ui8Parity     = USB_CDC_PARITY_NONE;
//    psLineCoding->ui8Stop       = USB_CDC_STOP_BITS_1;
//    psLineCoding->ui32Rate      = 115200U;
//}

////****************************************************************************
////
//// Set the communication parameters to use on the UART.
////
////****************************************************************************
//static bool SetLineCoding(tLineCoding *psLineCoding)
//{
//    // Let the caller know if we had a problem or not.
//    return(true);
//}

////****************************************************************************
////
//// Set the state of the RS232 RTS and DTR signals. Handshaking is not
//// supported so this request will be ignored.
////
////****************************************************************************
//static void SetControlLineState(uint16_t ui16State)
//{
//    //
//    // If configured with GPIOs controlling the handshake lines,
//    // set them appropriately depending upon the flags passed in the wValue
//    // field of the request structure passed.
//    //
//}

////****************************************************************************
////
//// This function sets or clears a break condition on the redirected UART RX
//// line.  A break is started when the function is called with \e bSend set to
//// \b true and persists until the function is called again with \e bSend set
//// to \b false.
////
////****************************************************************************
//static void SendBreak(bool bSend)
//{
//    // Are we being asked to start or stop the break condition?
//    g_bSendingBreak = (!bSend) ? (false) : (true);
//}

////******************************************************************************
//// ModeCallback - USB Mode callback
////
//// \param ui32Index is the zero-based index of the USB controller making the
////        callback.
//// \param eMode indicates the new operating mode.
////
//// This function is called by the USB library whenever an OTG mode change
//// occurs and, if a connection has been made, informs us of whether we are to
//// operate as a host or device.
////
//// \return None.
////
////******************************************************************************
//void
//ModeCallback(uint32_t ui32Index, tUSBMode eMode)
//{
//    // Save the new mode.
//    g_eCurrentUSBMode = eMode;
//}

//****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the
// device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//****************************************************************************
uint32_t CtrlHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData)
{
    uint32_t ui32IntsOff;

    // Which event are we being asked to process?
    switch(ui32Event)
    {
        // We are connected to a host and communication is now possible.

        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            // Flush our buffers.
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            // Tell the main loop to update the display.
            ui32IntsOff = Interrupt_disableInProcessor();
            g_pcStatus = "Connected.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            if(!ui32IntsOff)
                Interrupt_enableInProcessor();
            break;
        }

        // The host has disconnected.
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            ui32IntsOff = Interrupt_disableInProcessor();
            g_pcStatus = "Host disconnected.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            if(!ui32IntsOff)
                Interrupt_enableInProcessor();
            break;
        }


        // Return the current serial communication parameters.
        case USBD_CDC_EVENT_GET_LINE_CODING:
        {
            tLineCoding tmp;
            tmp.ui32Rate    = 115200;
            tmp.ui8Databits = 8;
            tmp.ui8Parity   = USB_CDC_PARITY_NONE;
            tmp.ui8Stop     = USB_CDC_STOP_BITS_1;
            pvMsgData       = (tLineCoding *) &tmp;
//            GetLineCoding(pvMsgData);
            break;
        }

        // Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_LINE_CODING:
        {
//            SetLineCoding(pvMsgData);
            break;
        }

        // Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        {
//            SetControlLineState((unsigned short)ui32MsgValue);
            break;
        }

        // Send a break condition on the serial line.
        case USBD_CDC_EVENT_SEND_BREAK:
        {
//            SendBreak(true);
//            g_bSendingBreak = true;
            break;
        }

        // Clear the break condition on the serial line.
        case USBD_CDC_EVENT_CLEAR_BREAK:
        {
//            SendBreak(false);
//            g_bSendingBreak = false;
            break;
        }

        // Ignore SUSPEND and RESUME for now.
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }

        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        default:
        {
            break;
        }
    }

    return(0);
}

//****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//****************************************************************************
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData)
{
    // Which event have we been sent?
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
        {
            // Since we are using the USBBuffer, we don't need to do anything here.
            break;
        }

        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }
    return(0);
}

//****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this
// channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//****************************************************************************
uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData)
{
//    uint32_t ui32Count;

    // Which event are we being sent?
    switch(ui32Event)
    {
        // A new packet has been received.
        case USB_EVENT_RX_AVAILABLE:
        {
            dataSize = USBBufferDataAvailable((tUSBBuffer *) &g_sRxBuffer);
            USBBufferRead((tUSBBuffer *) &g_sRxBuffer, g_char_tmp, dataSize);
            USBBufferWrite((tUSBBuffer *) &g_sTxBuffer, g_char_tmp, dataSize);
//            uint32_t ui32Read;
////            uint32_t ui32Fullness, ui32Space;
//            uint8_t ucChar;
//
//            // If we are currently sending a break condition, don't receive any more data.
//            // We will resume transmission once the break is turned off.
//            if(g_bSendingBreak)
//                return(0);
//
//            while(USBBufferSpaceAvailable(&g_sTxBuffer))
//            {
//                // Get a character from the buffer.
//                ui32Read = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ucChar, 1);
//                if(ui32Read)
//                {
//                    ucChar++;
//                    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,(uint8_t *)&ucChar, 1);
//                }
//                else
//                    break;
//            }
//
//            // How much elements do we have received in the buffer?
//            ui32Fullness = USBBufferDataAvailable(&g_sRxBuffer);
//            // How much space do we have in the buffer?
//            ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);
//
//            while(ui32Fullness && ui32Space)
//            {
//                // Get a character from the buffer.
//                ui32Read = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ucChar, 1);
////                ucChar = (uint8_t)(ui32Read & 0xFF);
//                USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,(uint8_t *)&ucChar, 1);
//                ui32Fullness--;
//                ui32Space--;
//            }
//
            break;
        }

        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the UART is currently idle or 1 if it is
        // in the process of transmitting something. The actual number of
        // bytes in the UART FIFO is not important here, merely whether or
        // not everything previously sent to us has been transmitted.
        case USB_EVENT_DATA_REMAINING:
        {
            // Get the number of bytes in the buffer and add 1 if some data
            // still has to clear the transmitter.
            return(USBBufferSpaceAvailable(&g_sRxBuffer));
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }

    return(0);
}

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief   Main function running on CM processor. MCU init.
 */
void main(void)
{
//    uint32_t i = 0;
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
    HAL_CM_init(&hal_cm);

    // IPC address translation
//    IPC_CpuToCmAddrSync(&dbg_uart_addr);

    // Not configured initially.
    g_bUSBConfigured = false;
    // Register the interrupt handler, for USB.
    Interrupt_registerHandler(INT_USB0, &CM_USB0DeviceIntHandler);
    // Initialize the transmit and receive buffers for first serial device.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);
    // Set the USB stack mode to Device mode with VBUS monitoring.
    USBStackModeSet(0, eUSBModeForceDevice, 0);//eUSBModeDevice
    // Pass our device information to the USB library and place the device on the bus.
    USBDCDCInit(0, (tUSBDCDCDevice *) &g_sCDCDevice);

    // Enable interrupts now that the application is ready to start.
    Interrupt_enable(INT_USB0);
    Interrupt_enableInProcessor();

    while(1)
    {
        // Have we been asked to update the status display?
        if(g_ui32Flags & COMMAND_STATUS_UPDATE)
        {
            // Clear the command flag
            Interrupt_disableInProcessor();
            g_ui32Flags &= ~COMMAND_STATUS_UPDATE;
            Interrupt_enableInProcessor();
        }
    }
}

inline void IPC_CpuToCmAddrSync(dbg_uart_addr_t* p_addr)
{
    uint32_t command;// = NULL;
    uint32_t addr;// = NULL;
    uint32_t data;// = NULL;

    // Get address of ia of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, &addr, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    p_addr->p_ia        = (float_t*)addr;
    p_addr->p_ib        = (float_t*)addr + 1;
    p_addr->p_ic        = (float_t*)addr + 2;
    p_addr->p_vbus      = (float_t*)addr + 3;
    p_addr->p_ialpha    = (float_t*)addr + 4;
    p_addr->p_ibeta     = (float_t*)addr + 5;
    p_addr->p_id        = (float_t*)addr + 6;
    p_addr->p_iq        = (float_t*)addr + 7;
    p_addr->p_ud        = (float_t*)addr + 8;
    p_addr->p_uq        = (float_t*)addr + 9;
    p_addr->p_ualpha    = (float_t*)addr + 10;
    p_addr->p_ubeta     = (float_t*)addr + 11;
    p_addr->p_ua        = (float_t*)addr + 12;
    p_addr->p_ub        = (float_t*)addr + 13;
    p_addr->p_uc        = (float_t*)addr + 14;
    p_addr->p_pos       = (float_t*)addr + 15;
    p_addr->p_posref    = (float_t*)addr + 16;
    p_addr->p_vel       = (float_t*)addr + 17;
    p_addr->p_velref    = (float_t*)addr + 18;
    p_addr->p_iqref     = (float_t*)addr + 19;
#if (0)
    uint32_t command, data;
    // Get address of ia of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ia, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ib of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ib, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ic of MOTOR_2
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ic, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of vbus of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_vbus, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ialpha of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ialpha, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ibeta of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ibeta, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of id of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_id, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of iq of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_iq, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ud of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ud, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of uq of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_uq, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ualpha of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ualpha, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ubeta of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ubeta, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ua of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ua, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ub of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ub, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of uc of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_uc, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of absolute encoder position of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_pos, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of PositionRef of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_posref, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of encoder speed variable of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_vel, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of SpeedRef of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_velref, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of IqRef of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_iqref, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of IT counter of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_itcnt, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
#endif
}

//******************************************************************************
//
//! Device interrupt service routine wrapper to make ISR compatible with
//! C2000 PIE controller.
//
//******************************************************************************
__interrupt void CM_USB0DeviceIntHandler(void)
{
    USB0DeviceIntHandler();
}
//
// End of File
//
