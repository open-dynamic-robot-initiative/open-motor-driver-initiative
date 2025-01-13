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
//#include "main_cm.h"
#include "cm.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usb_ids.h"
#include "device/usbdevice.h"
#include "device/usbdcdc.h"
#include "ustdlib.h"
//#include "usb_hal.h"
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

#define USB_DETECT_NUM          (67U)

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
static volatile bool usb_configDone = false;


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

/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
extern const hal_cm_t hal_cm;

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief   Handles CDC driver notifications related to control and setup of the device.
 *          This function is called by the CDC driver to perform control-related
 *          operations on behalf of the USB host. These functions include setting
 *          and querying the serial communication parameters, setting handshake line
 *          states and sending break conditions.
 * @param   p_CBData is the client-supplied callback pointer for this channel.
 * @param   ui32evt identifies the event we are being notified about.
 * @param   ui32msgVal is an event-specific value.
 * @param   p_msgData is an event-specific pointer.
 * @return  The return value is event-specific.
 */
uint32_t CtrlHandler(void *p_CBData, uint32_t ui32evt, uint32_t ui32msgVal, void *p_msgData)
{
//    uint32_t ui32IntsOff;

    // Which event are we being asked to process?
    switch(ui32evt)
    {
        // We are connected to a host and communication is now possible.

        case USB_EVENT_CONNECTED:
        {
//            Interrupt_enable(INT_USB0);
            usb_configDone = true;
            // Flush our buffers.
            USBBufferFlush(&usb_TxBuffer);
            USBBufferFlush(&usb_RxBuffer);
//            // Tell the main loop to update the display.
//            ui32IntsOff = Interrupt_disableInProcessor();
//            g_pcStatus = "Connected.";
//            g_ui32Flags |= COMMAND_STATUS_UPDATE;
//            if(!ui32IntsOff)
//                Interrupt_enableInProcessor();
            break;
        }

        // The host has disconnected.
        case USB_EVENT_DISCONNECTED:
        {
            usb_configDone = false;
//            ui32IntsOff = Interrupt_disableInProcessor();
//            g_pcStatus = "Host disconnected.";
//            g_ui32Flags |= COMMAND_STATUS_UPDATE;
//            if(!ui32IntsOff)
//                Interrupt_enableInProcessor();
            break;
        }


        // Return the current serial communication parameters.
        case USBD_CDC_EVENT_GET_LINE_CODING:
        {
            psLineCoding.ui32Rate   = 115200;
            psLineCoding.ui8Databits= 8;
            psLineCoding.ui8Parity  = USB_CDC_PARITY_NONE;
            psLineCoding.ui8Stop    = USB_CDC_STOP_BITS_1;
            p_msgData               = (tLineCoding *) &psLineCoding;
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

/**
 * @brief   Handles CDC driver notifications related to the transmit channel (data to the USB host).
 *          This function is called by the CDC driver to notify us of any events related to
 *          operation of the transmit data channel (the IN channel carrying data to the USB host).
 * @param   p_CBData is the client-supplied callback pointer for this channel.
 * @param   ui32evt identifies the event we are being notified about.
 * @param   ui32msgVal is an event-specific value.
 * @param   p_msgData is an event-specific pointer.
 * @return  The return value is event-specific.
 */
uint32_t TxHandler(void *p_CBData, uint32_t ui32evt, uint32_t ui32msgVal, void *p_msgData)
{
    // Which event have we been sent?
    switch(ui32evt)
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

/**
 * @brief   Handles CDC driver notifications related to the receive channel (data from the USB host).
 *          This function is called by the CDC driver to notify us of any events related to
 *          operation of the receive data channel (the OUT channel carrying data from the USB host).
 * @param   p_CBData is the client-supplied callback pointer for this channel.
 * @param   ui32evt identifies the event we are being notified about.
 * @param   ui32msgVal is an event-specific value.
 * @param   p_msgData is an event-specific pointer.
 * @return  The return value is event-specific.
 */
uint32_t RxHandler(void *p_CBData, uint32_t ui32evt, uint32_t ui32msgVal, void *p_msgData)
{
    // Which event are we being sent?
    switch(ui32evt)
    {
        // A new packet has been received.
        case USB_EVENT_RX_AVAILABLE:
        {
//            tUSBDCDCDevice *psDevice;
//            psDevice = (tUSBDCDCDevice *)p_CBData;
//
//            tUSBRingBufObject sTxRing;
//            // Get the current buffer information to allow us to write directly to the
//            // transmit buffer (there is already have enough information from the
//            // parameters to access the receive buffer directly).
//            USBBufferInfoGet(&usb_TxBuffer, &sTxRing);
//
//            uint32_t ui32Space;
//            // How much space is there in the transmit buffer?
//            ui32Space = USBBufferSpaceAvailable(&usb_TxBuffer);
//            dataSize = USBBufferDataAvailable((tUSBBuffer *) &usb_RxBuffer);

//            uint32_t diff_space = (ui32Space <= dataSize) ? (ui32Space) : (dataSize);

            dataSize = USBBufferDataAvailable((tUSBBuffer *) &usb_RxBuffer);
            USBBufferRead((tUSBBuffer *) &usb_RxBuffer, g_char_tmp, dataSize);
            USBBufferWrite((tUSBBuffer *) &usb_TxBuffer, g_char_tmp, dataSize);

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
            return(USBDCDCRxPacketAvailable((tUSBDCDCDevice *) &usb_CDCDevice));
//            return(USBBufferSpaceAvailable(&usb_RxBuffer));
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
//        case USB_EVENT_ERROR:
////        case USB_EVENT_REQUEST_BUFFER:
//        {
//            return(0);
//        }
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

/**
 * @brief   Main function running on CM processor. MCU init.
 */
void main(void)
{

//    uint16_t vbus_detect = 0;
//    bool    usb_ITset   = false;
//    bool    usb_ITclear = true;
//    bool    usb_detect  = false;
//    uint32_t i = 0;
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
//    HAL_CM_init(&hal_cm);

    // IPC address translation
//    IPC_CpuToCmAddrSync(&dbg_uart_addr);

    // Not initially configured.
    usb_configDone = false;
    // Register the interrupt handler, for USB.
    Interrupt_registerHandler(INT_USB0, &CM_USB0DeviceIntHandler);
    // Initialize the transmit and receive buffers for first serial device.
    USBBufferInit(&usb_TxBuffer);
    USBBufferInit(&usb_RxBuffer);
    // Set the USB stack mode to Device mode with VBUS monitoring.
    USBStackModeSet(0, eUSBModeForceDevice, 0);//eUSBModeDevice
    // Pass our device information to the USB library and place the device on the bus.
    USBDCDCInit(0, (tUSBDCDCDevice *) &usb_CDCDevice);

    // Enable interrupts now that the application is ready to start.
//    Interrupt_enable(INT_USB0);
    Interrupt_enableInProcessor();

    while(1)
    {
//        vbus_detect = (vbus_detect << 1) | (GPIO_readPin(USB_DETECT_NUM) & 0x01);
//
//        if(vbus_detect == 0x0F)
//        {
//            usb_ITset   = true;
//            usb_ITclear = false;
//            usb_detect  = true;
//        }
//        else if(vbus_detect == 0xF0)
//        {
//            usb_ITset   = false;
//            usb_ITclear = true;
//            usb_detect  = true;
//        }
//        else
//            usb_detect = false;
//
//        if(usb_detect)
//        {
//            Interrupt_disableInProcessor();
//            if(usb_ITset)
//                Interrupt_enable(INT_USB0);
//            else if(usb_ITclear)
//                Interrupt_disable(INT_USB0);
//            Interrupt_enableInProcessor();
//        }

        // Have we been asked to update the status display?
//        if(g_ui32Flags & COMMAND_STATUS_UPDATE)
//        {
//            // Clear the command flag
//            Interrupt_disableInProcessor();
//            g_ui32Flags &= ~COMMAND_STATUS_UPDATE;
//            Interrupt_enableInProcessor();
//        }
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
