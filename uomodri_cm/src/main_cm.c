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
#include <uomodri_cm_handlers.h>
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
#include "fifo_cm.h"

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
#define INT_USB_DEVICE          INT_USB0
#define INT_USB_DEVICE_HANDLER  &CM_USB0DeviceIntHandler

#define INT_IPC_RX              INT_CPU1TOCMIPC0
#define INT_IPC_RX_HANDLER      ipc0_isr

#define INT_UDMA                INT_UDMA_SW
#define INT_UDMA_HANDLER        udma_isr

#define INT_USB_DEVICE_DEF      { \
                                    .intNum         = INT_USB_DEVICE, \
                                    .p_intHandler   = INT_USB_DEVICE_HANDLER, \
                                    .intEnable      = false, \
                                }
#define INT_IPC_RX_DEF          { \
                                    .intNum         = INT_IPC_RX, \
                                    .p_intHandler   = INT_IPC_RX_HANDLER, \
                                    .intEnable      = true, \
                                }
#define INT_UDMA_DEF            { \
                                    .intNum         = INT_UDMA, \
                                    .p_intHandler   = INT_UDMA_HANDLER, \
                                    .intEnable      = true, \
                                }

//******************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//******************************************************************************
#define FIFO_DBG_MSG_NB_ELEM    (8)


#define USB_DETECT_NUM          (67U)
#define DBG_PIN3_NUM            (77U)

//tLineCoding psLineCoding;
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
__attribute__((interrupt)) void ipc0_isr(void);
__attribute__((interrupt)) void udma_isr(void);

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

dbg_msg_tx_t dbg_msg_tx = {.dbg_msg[0] = {.header = "ab"},// .footer = "\n\r"},
                           .dbg_msg[1] = {.header = "ab"},// .footer = "\n\r"},
                           .dbg_msg_cr = 0U, .dbg_msg_nx = 1U,
                           .dbg_msg_nx_rdy = false};

dbg_msg_t tmp_dbg_msg_tx    = {.header = "ab"};
Fifo_TypeDef fifo_dbg_msg_tx;

//dbg_msg_t fifo_dbg_msg_tx[FIFO_DBG_MSG_NB_ELEM];

static const int_cfg_t interruptList[] =
{
// INT_USB_DEVICE_DEF,
 INT_IPC_RX_DEF,
 INT_UDMA_DEF,
 {.intNum = UINT32_MAX},        /* END OF ARRAY */
};

/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
//extern const hal_cm_t hal_cm;

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/


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
    FIFO_init(&fifo_dbg_msg_tx, sizeof(dbg_msg_t), FIFO_DBG_MSG_NB_ELEM);

//    UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
//    UDMA_requestSoftwareTransfer(UDMA_BASE, UDMA_CHANNEL_UART0_TX);


    while(1);
}

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
//uint32_t CtrlHandler(void *p_CBData, uint32_t ui32evt, uint32_t ui32msgVal, void *p_msgData)
//{
//    // Which event are we being asked to process?
//    switch(ui32evt)
//    {
//        // Connected to host and communication is now possible.
//        case USB_EVENT_CONNECTED:
//        {
////            Interrupt_enable(INT_USB0);
//            usb_configDone = true;
//            // Flush our buffers.
//            USBBufferFlush(&usb_TxBuffer);
//            USBBufferFlush(&usb_RxBuffer);
////            // Tell the main loop to update the display.
////            ui32IntsOff = Interrupt_disableInProcessor();
////            g_pcStatus = "Connected.";
////            g_ui32Flags |= COMMAND_STATUS_UPDATE;
////            if(!ui32IntsOff)
////                Interrupt_enableInProcessor();
//            break;
//        }
//
//        // The host has disconnected.
//        case USB_EVENT_DISCONNECTED:
//        {
//            usb_configDone = false;
////            ui32IntsOff = Interrupt_disableInProcessor();
////            g_pcStatus = "Host disconnected.";
////            g_ui32Flags |= COMMAND_STATUS_UPDATE;
////            if(!ui32IntsOff)
////                Interrupt_enableInProcessor();
//            break;
//        }
//
//
//        // Return the current serial communication parameters.
//        case USBD_CDC_EVENT_GET_LINE_CODING:
//        {
//            psLineCoding.ui32Rate   = 115200;
//            psLineCoding.ui8Databits= 8;
//            psLineCoding.ui8Parity  = USB_CDC_PARITY_NONE;
//            psLineCoding.ui8Stop    = USB_CDC_STOP_BITS_1;
//            p_msgData               = (tLineCoding *) &psLineCoding;
////            GetLineCoding(pvMsgData);
//            break;
//        }
//
//        // Set the current serial communication parameters.
//        case USBD_CDC_EVENT_SET_LINE_CODING:
//        {
////            SetLineCoding(pvMsgData);
//            break;
//        }
//
//        // Set the current serial communication parameters.
//        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
//        {
////            SetControlLineState((unsigned short)ui32MsgValue);
//            break;
//        }
//
//        // Send a break condition on the serial line.
//        case USBD_CDC_EVENT_SEND_BREAK:
//        {
////            SendBreak(true);
////            g_bSendingBreak = true;
//            break;
//        }
//
//        // Clear the break condition on the serial line.
//        case USBD_CDC_EVENT_CLEAR_BREAK:
//        {
////            SendBreak(false);
////            g_bSendingBreak = false;
//            break;
//        }
//
//        // Ignore SUSPEND and RESUME for now.
//        case USB_EVENT_SUSPEND:
//        case USB_EVENT_RESUME:
//        {
//            break;
//        }
//
//        // We don't expect to receive any other events.  Ignore any that show
//        // up in a release build or hang in a debug build.
//        default:
//        {
//            break;
//        }
//    }
//
//    return(0);
//}

///**
// * @brief   Handles CDC driver notifications related to the transmit channel (data to the USB host).
// *          This function is called by the CDC driver to notify us of any events related to
// *          operation of the transmit data channel (the IN channel carrying data to the USB host).
// * @param   p_CBData is the client-supplied callback pointer for this channel.
// * @param   ui32evt identifies the event we are being notified about.
// * @param   ui32msgVal is an event-specific value.
// * @param   p_msgData is an event-specific pointer.
// * @return  The return value is event-specific.
// */
//uint32_t TxHandler(void *p_CBData, uint32_t ui32evt, uint32_t ui32msgVal, void *p_msgData)
//{
//    // Which event have we been sent?
//    switch(ui32evt)
//    {
//        case USB_EVENT_TX_COMPLETE:
//        {
//            // Since we are using the USBBuffer, we don't need to do anything here.
//            break;
//        }
//
//        // We don't expect to receive any other events.  Ignore any that show
//        // up in a release build or hang in a debug build.
//        default:
//#ifdef DEBUG
//            while(1);
//#else
//            break;
//#endif
//    }
//    return(0);
//}
//
///**
// * @brief   Handles CDC driver notifications related to the receive channel (data from the USB host).
// *          This function is called by the CDC driver to notify us of any events related to
// *          operation of the receive data channel (the OUT channel carrying data from the USB host).
// * @param   p_CBData is the client-supplied callback pointer for this channel.
// * @param   ui32evt identifies the event we are being notified about.
// * @param   ui32msgVal is an event-specific value.
// * @param   p_msgData is an event-specific pointer.
// * @return  The return value is event-specific.
// */
//uint32_t RxHandler(void *p_CBData, uint32_t ui32evt, uint32_t ui32msgVal, void *p_msgData)
//{
//    // Which event are we being sent?
//    switch(ui32evt)
//    {
//        // A new packet has been received.
//        case USB_EVENT_RX_AVAILABLE:
//        {
////            tUSBDCDCDevice *psDevice;
////            psDevice = (tUSBDCDCDevice *)p_CBData;
////
////            tUSBRingBufObject sTxRing;
////            // Get the current buffer information to allow us to write directly to the
////            // transmit buffer (there is already have enough information from the
////            // parameters to access the receive buffer directly).
////            USBBufferInfoGet(&usb_TxBuffer, &sTxRing);
////
////            uint32_t ui32Space;
////            // How much space is there in the transmit buffer?
////            ui32Space = USBBufferSpaceAvailable(&usb_TxBuffer);
////            dataSize = USBBufferDataAvailable((tUSBBuffer *) &usb_RxBuffer);
//
////            uint32_t diff_space = (ui32Space <= dataSize) ? (ui32Space) : (dataSize);
//
//            dataSize = USBBufferDataAvailable((tUSBBuffer *) &usb_RxBuffer);
//            USBBufferRead((tUSBBuffer *) &usb_RxBuffer, g_char_tmp, dataSize);
//            USBBufferWrite((tUSBBuffer *) &usb_TxBuffer, g_char_tmp, dataSize);
//
//            break;
//        }
//
//        // We are being asked how much unprocessed data we have still to
//        // process. We return 0 if the UART is currently idle or 1 if it is
//        // in the process of transmitting something. The actual number of
//        // bytes in the UART FIFO is not important here, merely whether or
//        // not everything previously sent to us has been transmitted.
//        case USB_EVENT_DATA_REMAINING:
//        {
//            // Get the number of bytes in the buffer and add 1 if some data
//            // still has to clear the transmitter.
//            return(USBDCDCRxPacketAvailable((tUSBDCDCDevice *) &usb_CDCDevice));
////            return(USBBufferSpaceAvailable(&usb_RxBuffer));
//        }
//
//        //
//        // We are being asked to provide a buffer into which the next packet
//        // can be read. We do not support this mode of receiving data so let
//        // the driver know by returning 0. The CDC driver should not be sending
//        // this message but this is included just for illustration and
//        // completeness.
//        //
////        case USB_EVENT_ERROR:
//////        case USB_EVENT_REQUEST_BUFFER:
////        {
////            return(0);
////        }
//        case USB_EVENT_REQUEST_BUFFER:
//        {
////            return(0);
//            break;
//        }
//
//        //
//        // We don't expect to receive any other events.  Ignore any that show
//        // up in a release build or hang in a debug build.
//        //
//        default:
//#ifdef DEBUG
//            while(1);
//#else
//            break;
//#endif
//    }
//
//    return(0);
//}

#endif
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

__attribute__((interrupt)) void ipc0_isr(void)
{
    uint32_t    cmd, data, addr;
    uint8_t     ipcIndex = 0;
    // Get index associated to IPC structure configuration
//    while(cm_ipcCfgList[ipcIndex].ipcType != IPC_CPU1_TO_CM_DIR)
//        ipcIndex++;
    // Get structure address
    IPC_readCommand(cm_ipcCfgList[ipcIndex].ipcType, \
                    cm_ipcCfgList[ipcIndex].ipcFlag, \
                    IPC_ADDR_CORRECTION_ENABLE, \
                    &cmd, &addr, &data);

    if(addr == (uint32_t) &dbg_m1)
    {
//        COM_msgDbgTx(&dbg_m1, &dbg_msg_tx.dbg_msg[dbg_msg_tx.dbg_msg_nx]);
        COM_msgDbgTx(&dbg_m1, &tmp_dbg_msg_tx);
        FIFO_write(&fifo_dbg_msg_tx, &tmp_dbg_msg_tx);

//        dbg_msg_tx.dbg_msg_nx_rdy = true;
    }

//    // Get the index associated to uDMA UART0_TX configuration
//    //while((cm_udmaCfgList[udmaCfgIdx].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIdx++) : (udmaCfgIdx));
//    // Enable the uDMA UART0 Tx channel
//    if(!UDMA_isChannelEnabled(UDMA_BASE, cm_udmaCfgList[0].dmaChannel))
//    {
//        //ucControlTable[UDMA_CHANNEL_UART0_TX].srcEndAddr     = (uint8_t *) &dbg_msg_tx.dbg_msg[dbg_msg_tx.dbg_msg_cr];
//        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
//        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
//        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
//        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
//    }

//    if(addr == (uint32_t) &dbg_m1)

//    {
//        COM_msgDbgTx(&dbg_m1, &dbg_msg_tx.dbg_msg[dbg_msg_tx.dbg_msg_nx]);
//        dbg_msg_tx.dbg_msg_cr   = !dbg_msg_tx.dbg_msg_cr;
//        dbg_msg_tx.dbg_msg_nx   = !dbg_msg_tx.dbg_msg_cr;
//    }

//    UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);

    // Acknowledge IT associated to input flag.
    IPC_ackFlagRtoL(cm_ipcCfgList[ipcIndex].ipcType, cm_ipcCfgList[ipcIndex].ipcFlag);
}

__attribute__((interrupt)) void udma_isr(void)
{
    if(dbg_msg_tx.dbg_msg_nx_rdy)
    {
        dbg_msg_tx.dbg_msg_cr       = !dbg_msg_tx.dbg_msg_cr;
        dbg_msg_tx.dbg_msg_nx       = !dbg_msg_tx.dbg_msg_cr;
        dbg_msg_tx.dbg_msg_nx_rdy   = false;

//        // Get the index associated to uDMA UART0_TX configuration
//        while((cm_udmaCfgList[udmaCfgIdx].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIdx++) : (udmaCfgIdx));
//        // Enable the uDMA UART0 Tx channel
//        if(!UDMA_isChannelEnabled(UDMA_BASE, cm_udmaCfgList[0].dmaChannel))
//        {
//            //ucControlTable[UDMA_CHANNEL_UART0_TX].srcEndAddr     = (uint8_t *) &dbg_msg_tx.dbg_msg[dbg_msg_tx.dbg_msg_cr];
//            ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
//            ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
//            ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
//            UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
//        }

        if(dbg_msg_tx.dbg_msg_cr)
            UDMA_enableChannel(UDMA_BASE, DMA_UART_TX_ALT_CHANNEL);
        else
            UDMA_enableChannel(UDMA_BASE, DMA_UART_TX_PRI_CHANNEL);
    }

}

//    while(UART_isBusy(UART0_BASE));

//    if(addr == (uint32_t) &dbg_m1)
//    {
//            GPIO_writePin(DBG_PIN3_NUM, 1U);
    // New message received, prepare to send via UART:
//        COM_msgDbgTx(&dbg_m1, &dbg_tx_msg);
//            GPIO_writePin(DBG_PIN3_NUM, 0U);
//        it_cnt_prev = dbg_m1.itCnt;

    // Force purge of Rx buffer.
    //while(UART_readCharNonBlocking(UART0_BASE) != -1);
    // Get the index associated to uDMA UART0_TX configuration
//            while((cm_udmaCfgList[udmaCfgIdx].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIdx++) : (udmaCfgIdx));
//             Enable the uDMA UART0 Tx channel
//        if(!UDMA_isChannelEnabled(UDMA_BASE, cm_udmaCfgList[0].dmaChannel))
//        {
//            //ucControlTable[UDMA_CHANNEL_UART0_TX].srcEndAddr     = (uint8_t *) &dbg_msg_tx.dbg_msg[dbg_msg_tx.dbg_msg_cr];
//            ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
//            ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
//            ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
//            UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
//        }
//    }
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
//    IPC_ackFlagRtoL(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlag);
//}


//
// End of File
//
