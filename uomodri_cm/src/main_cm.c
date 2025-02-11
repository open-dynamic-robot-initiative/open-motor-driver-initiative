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
#include "hal_cm.h"
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

/***********************************************************************
 * INTERRUPTS DEFINES
 ***********************************************************************/
#define INT_USB_DEVICE                  INT_USB0
#define INT_USB_DEVICE_HANDLER          &CM_USB0DeviceIntHandler

#define INT_USB_DEVICE_DEF              { \
                                            .intNum             = INT_USB_DEVICE, \
                                            .p_intHandler       = INT_USB_DEVICE_HANDLER \
                                        }

//******************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//******************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002
#define COMMAND_RECEIVED        0x00000004

#define USB_DETECT_NUM          (67U)
#define DBG_PIN3_NUM            (77U)

tLineCoding psLineCoding;
volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;
uint32_t dataSize;
uint8_t g_char_tmp[256];

#pragma DATA_ALIGN(dbg_m1 , 256U)
#pragma DATA_SECTION(dbg_m1, "MsgRamCpu1ToCm")
volatile debug_t dbg_m1;

#pragma DATA_ALIGN(dbg_m2 , 256U)
#pragma DATA_SECTION(dbg_m2, "MsgRamCpu1ToCm")
volatile debug_t dbg_m2;

uint32_t it_cnt_prev = 0;

//****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//****************************************************************************
static volatile bool usb_configDone = false;

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
//inline void IPC_CpuToCmAddrSync(dbg_uart_addr_t*);
__attribute__((interrupt)) void CM_USB0DeviceIntHandler(void);
__attribute__((interrupt)) void ipc_isr0(void);

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
UDMA_ControlTable ucControlTable[64];

dbg_uart_msg_t dbg_uart_tx_msg =
{
 .header[0]         = 'a',
 .header[1]         = 'b',
 .count             = 0U,
 .ia                = 0.0f,
 .ib                = 0.0f,
 .ic                = 0.0f,
 .pos               = 0.0f,
 .vel               = 0.0f,
 .crc               = 0,
};

static const int_cfg_t interruptList[] =
{
 INT_USB_DEVICE_DEF,
 {.intNum = UINT32_MAX},        /* END OF ARRAY */
};

/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
//extern const hal_cm_t hal_cm;

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
#if (USB_BUS_ENABLE) && (!UART_BUS_ENABLE)
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
//            return(0);
            break;
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

#endif

/**
 * @brief   Main function running on CM processor. MCU init.
 */
void main(void)
{
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
    HAL_CM_ini(&hal_cm);
    // Interrupts initialization.
    HAL_INT_ini(interruptList);

    // IPC address translation
//    IPC_CpuToCmAddrSync(&dbg_uart_addr);
//#if (USB_BUS_ENABLE) && (!UART_BUS_ENABLE)
//    // Not initially configured.
//    usb_configDone = false;
    // Register the interrupt handler, for USB.
//    Interrupt_registerHandler(INT_USB0, &CM_USB0DeviceIntHandler);
//    // Initialize the transmit and receive buffers for first serial device.
//    USBBufferInit(&usb_TxBuffer);
//    USBBufferInit(&usb_RxBuffer);
//    // Set the USB stack mode to Device mode with VBUS monitoring.
//    USBStackModeSet(0, eUSBModeForceDevice, 0);//eUSBModeDevice
//    // Pass our device information to the USB library and place the device on the bus.
//    USBDCDCInit(0, (tUSBDCDCDevice *) &usb_CDCDevice);

    // Enable interrupts now that the application is ready to start.
//    Interrupt_enable(INT_USB0);
//    Interrupt_enableInProcessor();
//#endif

    while(1)
    {
#if (!USB_BUS_ENABLE) && (UART_BUS_ENABLE)

        // No new message from CLA, skip TX:
        IPC_waitForFlag(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
        // Acknowledge IT associated to input flag.
        IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);


//        if(it_cnt_prev != dbg_m1.itCnt)
//        {
//            continue;
//        }
            // Wait Tx buffer + UART Tx module are empty.
            while(UART_isBusy(UART0_BASE));

            GPIO_writePin(DBG_PIN3_NUM, 1U);
            // New message received, prepare to send via UART:
            COM_msgDbgTx(&dbg_m1, &dbg_uart_tx_msg);
            GPIO_writePin(DBG_PIN3_NUM, 0U);
//            it_cnt_prev = dbg_m1.itCnt;

            // Force purge of Rx buffer.
            //while(UART_readCharNonBlocking(UART0_BASE) != -1);
            // Get the index associated to uDMA UART0_TX configuration
//            while((cm_udmaCfgList[udmaCfgIdx].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIdx++) : (udmaCfgIdx));
//             Enable the uDMA UART0 Tx channel
            if(!UDMA_isChannelEnabled(UDMA_BASE, cm_udmaCfgList[0].dmaChannel))
            {
                ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
                ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
                ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
                UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
            }
//        }
#endif
    }
}


//******************************************************************************
//
//! Device interrupt service routine wrapper to make ISR compatible with
//! C2000 PIE controller.
//
//******************************************************************************
__attribute__((interrupt)) void CM_USB0DeviceIntHandler(void)
{
    USB0DeviceIntHandler();
}

__attribute__((interrupt)) void ipc_isr0(void)
{
    uint32_t    cmd, data, addr;
    uint8_t     ipcCfgIdx = 0;
    //bool        com_tx_msg_done = false;
    // Get index associated to IPC structure configuration
    while(cm_ipcCfgList[(cm_ipcCfgList[ipcCfgIdx].p_ipcIntHandler != &ipc_isr0) ? (ipcCfgIdx++) : (ipcCfgIdx)].p_ipcIntHandler != &ipc_isr0);
    // Get structure address
    IPC_readCommand(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, &cmd, &addr, &data);
    // Create the reply message
    //com_tx_msg_done = com_msgTx((FOC_STRUCT *)addr, &com_tx_msg);
//#ifdef DEBUG
//    uint8_t udmaCfgIndex = 0;
//    // Create debug message
//    com_msgDbgTx((FOC_STRUCT *)addr, dbg_tx_msg);
//    // Wait Tx buffer + UART Tx module are empty.
//    while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY) == 1);
//    // Force purge of Rx buffer.
//    while(UART_readCharNonBlocking(UART0_BASE) != -1);
//    // Get the index associated to uDMA UART0_TX configuration
//    while((cm_udmaCfgList[udmaCfgIndex].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIndex++) : (udmaCfgIndex));
//    // Enable the uDMA UART0 Tx channel
//    if(!UDMA_isChannelEnabled(UDMA_BASE, UDMA_CHANNEL_UART0_TX))
//    {
//        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= cm_udmaCfgList[udmaCfgIndex].transferMode;
//        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[udmaCfgIndex].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
//        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
//    }
//#endif /*DEBUG*/
//    if(com_tx_msg_done == true)
//        // Share the message generated with C28-CPU1
//        IPC_sendCommand(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, MSG_COM_INTER, (uint32_t)(&com_tx_msg), 1);
    // Acknowledge IT associated to input flag.
    IPC_ackFlagRtoL(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlag);
}


//
// End of File
//
