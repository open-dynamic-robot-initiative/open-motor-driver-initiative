/*
 * File name: communication_cm.c
 * Description: Source file containing extraction & build/update message functions
 */

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <strings.h>
#include "communication.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_usb_device_structs.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002
#define COMMAND_RECEIVED        0x00000004

#define LSB_32(x)               ((x) & 0x0000FFFF)
#define MSB_32(x)               (((x) >> 16) & 0x0000FFFF)
#define LSB_16(x)               ((x) & 0x00FF)
#define MSB_16(x)               (((x) >> 8) & 0x00FF)
#define MSG_RX_CFG              (0x01)
#define MSG_RX_CMD              (0x02)
#define MSG_TX_CFG              (0x01)
#define MSG_TX_CMD              (0x02)
#define MSG_CMD_LIST_MASK       (0x07FF07FF)
#define PD_KP_MASK              (0x0001)
#define PD_KI_MASK              (0x0002)
#define POS_REF_MASK            (0x0004)
#define VEL_REF_MASK            (0x0008)
#define IQ_FF_MASK              (0x0010)
#define ISAT_MASK               (0x0020)
#define CMD_MASK                (0x0040)
#define PI_KP_ID_MASK           (0x0080)
#define PI_KI_ID_MASK           (0x0100)
#define PI_KP_IQ_MASK           (0x0200)
#define PI_KI_IQ_MASK           (0x0400)
#define PD_KP_POS               (0)
#define PD_KI_POS               (1)
#define POS_REF_POS             (2)
#define VEL_REF_POS             (3)
#define IQ_FF_POS               (4)
#define ISAT_POS                (5)
#define CMD_POS                 (6)
#define PI_KP_ID_POS            (7)
#define PI_KI_ID_POS            (8)
#define PI_KP_IQ_POS            (9)
#define PI_KI_IQ_POS            (10)

#define MSG_TX_HEADER_SIZE      (2)
#define MSG_TX_COUNTER_SIZE     (4)
#define MSG_TX_MSG_TYPE_SIZE    (1)

tLineCoding psLineCoding ={.ui32Rate    = 115200, \
                           .ui8Databits = 8, \
                           .ui8Parity   = USB_CDC_PARITY_NONE, \
                           .ui8Stop     = USB_CDC_STOP_BITS_1};
//extern uint32_t g_ui32Flags;


extern uint32_t dataSize;
extern uint8_t g_char_tmp[256];
volatile uint32_t usb_ui32Flags = 0;
//extern int8_t *g_pcStatus;

uint32_t    usb_updateFlags;
int8_t*     usb_pcStatus;
bool        usb_cfgDone;


/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/

/**
 * @brief   Handles CDC driver notifications related to control and setup of the device.
 *          Function is called by the CDC driver to perform control-related
 *          operations on behalf of the USB host.
 *          Functions include setting and querying the serial communication parameters,
 *          setting handshake line states and sending break conditions.
 * @param   p_CBData is the client-supplied callback pointer for this channel.
 * @param   ui32evt identifies the event we are being notified about.
 * @param   ui32msgVal is an event-specific value.
 * @param   p_msgData is an event-specific pointer.
 * @return  The return value is event-specific.
 */
uint32_t usb_CtrlHandler(void *p_CBData, uint32_t ui32evt, uint32_t ui32msgVal, void *p_msgData)
{
    uint32_t itStatus;
    // Which event are we being asked to process?
    switch(ui32evt)
    {
        // Connected to host computer.
        case USB_EVENT_CONNECTED:
        {
            // Flush our buffers.
            USBBufferFlush(&usb_TxBuffer);
            USBBufferFlush(&usb_RxBuffer);
            // Tell the main loop to update the display.
            usb_cfgDone     = true;
            itStatus        = Interrupt_disableInProcessor();
            usb_pcStatus    = "Connected.";
            usb_updateFlags|= COMMAND_STATUS_UPDATE;
            (!itStatus) ? (Interrupt_enableInProcessor()) : (__nop());
            break;
        }

        // Disconnected from host computer.
        case USB_EVENT_DISCONNECTED:
        {
            usb_cfgDone     = false;
            itStatus        = Interrupt_disableInProcessor();
            usb_pcStatus    = "Host disconnected.";
            usb_updateFlags|= COMMAND_STATUS_UPDATE;
            (!itStatus) ? (Interrupt_enableInProcessor()) : (__nop());
            break;
        }

        // Return the current serial communication parameters.
        case USBD_CDC_EVENT_GET_LINE_CODING:
        {
            p_msgData       = (tLineCoding *) &psLineCoding;
            break;
        }

        // Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_LINE_CODING:
        // Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        // Send a break condition on the serial line.
        case USBD_CDC_EVENT_SEND_BREAK:
        // Clear the break condition on the serial line.
        case USBD_CDC_EVENT_CLEAR_BREAK:
        // Ignore SUSPEND and RESUME for now.
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        default:
            break;
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
uint32_t usb_TxHandler(void *p_CBData, uint32_t ui32evt, uint32_t ui32msgVal, void *p_msgData)
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
uint32_t usb_RxHandler(void *p_CBData, uint32_t ui32evt, uint32_t ui32msgVal, void *p_msgData)
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


/**
 * @brief       Message created/updates for debug
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
//void COM_msgDbgTx(com_dbg_tx_msg_t* p_msg, uint8_t msg_type)
//void COM_msgDbgTx(dbg_uart_addr_t* p_addr, dbg_uart_msg_t* p_msg)
//{
//#if (0)
//    static uint64_t cfg_lst = 0;
//    static uint8_t  i = 0, j = 0;
//
//    switch(p_msg->state)
//    {
//    case COM_DBG_TX_HEADER:
//    default:
//        p_msg->msg[0]   = 'a';
//        p_msg->msg[1]   = 'b';
//        p_msg->msg_size = MSG_TX_HEADER_SIZE;
//        p_msg->state    = COM_DBG_TX_CNT;
//        break;
//
//    case COM_DBG_TX_CNT:
//        for(i = 0; i < sizeof(counter_u); i++)
//            p_msg->msg[MSG_TX_HEADER_SIZE + i]  = p_msg->counter.byte[i];
//        p_msg->counter.all++;
//        p_msg->msg_size                        += sizeof(counter_u);
//        p_msg->state                            = COM_DBG_TX_MSG_TYPE;
//        //(msg_type == MSG_TX_CFG)  ? (COM_DBG_TX_CFG_LIST_4_TX) : (COM_DBG_TX_CMD_MSG);
//        break;
//
//    case COM_DBG_TX_MSG_TYPE:
//        p_msg->msg[p_msg->msg_size++]   = msg_type;
//        p_msg->state                    = (msg_type == MSG_TX_CFG)  ? (COM_DBG_TX_CFG_ACK)  : (COM_DBG_TX_HEADER);
//        p_msg->state                    = (msg_type == MSG_TX_CMD)  ? (COM_DBG_TX_CMD_MSG)  : (p_msg->state);
//        break;
//
//    case COM_DBG_TX_CFG_ACK:
//        for(i = 0; i < sizeof(config_u); i++)
//            p_msg->msg[p_msg->msg_size + i] = p_msg->cfg_lst.byte[i];
//        p_msg->msg_size                        += sizeof(config_u);
//        for(i = 0; i < sizeof(command_u); i++)
//            p_msg->msg[p_msg->msg_size + i] = p_msg->cmd_lst.byte[i];
//        p_msg->msg_size                        += sizeof(command_u);
//        p_msg->state                            = COM_DBG_TX_CRC;
//        break;
//
//    case COM_DBG_TX_CMD_MSG:
//        cfg_lst = p_msg->cfg_lst.all;
//        while(cfg_lst)
//        {
//            j           = ffs(cfg_lst) - 1;
//            cfg_lst    &= ~(1U << j);
//            for(i = 0; i < sizeof(word_32b_u); i++)
//                p_msg->msg[p_msg->msg_size + i] = p_msg->val[j].uint_8b[i];
//            p_msg->msg_size                    += sizeof(word_32b_u);
//        }
//        p_msg->state                            = COM_DBG_TX_CRC;
//        break;
//
//    case COM_DBG_TX_CRC:
//        p_msg->msg[p_msg->msg_size] = COM_crc8Fast(0, &p_msg->msg[0], p_msg->msg_size - 1);
//        p_msg->msg_size++;
//        p_msg->valid = true;
//        p_msg->state = COM_DBG_TX_HEADER;
//        break;
//
//    }
//#endif
//#if (1)
////    static uint32_t cntMsg  = 0;
////    p_msg->count            = cntMsg++;
//    p_msg->ia               = (float_t)(*p_addr->p_ia);
//    p_msg->ib               = (float_t)(*p_addr->p_ib);
//    p_msg->ic               = (float_t)(*p_addr->p_ic);
////    p_msg->vbus             = (float_t)(*p_addr->p_vbus);
////    p_msg->ialpha           = (float_t)(*p_addr->p_ialpha);
////    p_msg->ibeta            = (float_t)(*p_addr->p_ibeta);
//    p_msg->id               = (float_t)(*p_addr->p_id);
//    p_msg->iq               = (float_t)(*p_addr->p_iq);
//    p_msg->iqref            = (float_t)(*p_addr->p_iqref);
//    p_msg->ud               = (float_t)(*p_addr->p_ud);
//    p_msg->uq               = (float_t)(*p_addr->p_uq);
////    p_msg->ualpha           = (float_t)(*p_addr->p_ualpha);
////    p_msg->ubeta            = (float_t)(*p_addr->p_ubeta);
////    p_msg->ua               = (float_t)(*p_addr->p_ua);
////    p_msg->ub               = (float_t)(*p_addr->p_ub);
////    p_msg->uc               = (float_t)(*p_addr->p_uc);
//    p_msg->pos              = (float_t)(*p_addr->p_pos);
////    p_msg->posref           = (float_t)(*p_addr->p_posref);
//    p_msg->vel              = (float_t)(*p_addr->p_vel);
////    p_msg->velref           = (float_t)(*p_addr->p_velref);
////    p_msg->itcnt            = (uint32_t)(*p_addr->p_itcnt);
//    p_msg->crc              = COM_crc8Fast(0, (uint8_t *)p_msg, (sizeof(dbg_uart_msg_t) - 1));
//#endif
//    return;
//}

void COM_msgDbgTx(volatile debug_t* p_addr, dbg_msg_t* p_msg)
{
    p_msg->count= p_addr->itCnt;
    p_msg->ia   = p_addr->ia;
    p_msg->ib   = p_addr->ib;
    p_msg->ic   = p_addr->ic;
    p_msg->pos  = p_addr->enc_thetaAbsolute;
    p_msg->vel  = p_addr->enc_speed_speedMech;
    p_msg->crc  = COM_crc8Fast(0, (uint8_t *)p_msg, DBG_SIZE_MSG);

    return;
}

/**
 * @brief       Message extract for debug (display \& command)
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
//void COM_msgDbgRx(com_dbg_rx_msg_t* p_msg, uint8_t rx_byte)
//{
//    static uint32_t cmd_lst = 0;
//    static uint8_t  i = 0, j = 0;
//
//    switch(p_msg->state)
//    {
//    case COM_DBG_RX_WORD_A:
//    default:
//        i                               = 0;
//        j                               = 0;
//        cmd_lst                         = 0;
//        p_msg->state                    = (rx_byte == 'a')  ? (COM_DBG_RX_WORD_B)               : (COM_DBG_RX_WORD_A);
//        p_msg->crc                      = (rx_byte == 'a')  ? (COM_crc8Fast(0, &rx_byte, 1))    : (0);
//        p_msg->valid                    = false;
//
//        break;
//
//    case COM_DBG_RX_WORD_B:
//        p_msg->state                    = (rx_byte == 'b')  ? (COM_DBG_RX_CNT)                          : (COM_DBG_RX_WORD_A);
//        p_msg->crc                      = (rx_byte == 'b')  ? (COM_crc8Fast(p_msg->crc, &rx_byte, 1))   : (0);
//        break;
//
//    case COM_DBG_RX_CNT:
//        p_msg->counter.byte[i++] = rx_byte;
//        p_msg->state                    = (i < sizeof(counter_u))   ? (COM_DBG_RX_CNT)  : (COM_DBG_RX_MSG_TYPE);
//        i                               = (i < sizeof(counter_u))   ? (i)               : (0);
//        p_msg->crc                      = COM_crc8Fast(p_msg->crc, &rx_byte, 1);
//        break;
//
//     case COM_DBG_RX_MSG_TYPE:
//         i                              = 0;
//         j                              = 0;
//         cmd_lst                        = (rx_byte == MSG_RX_CMD)   ? (p_msg->cmd_lst.all & MSG_CMD_LIST_MASK)  : (0);
//         p_msg->state                   = (rx_byte == MSG_RX_CMD)   ? (COM_DBG_RX_CMD_MSG)                      : (COM_DBG_RX_WORD_A);
//         p_msg->state                   = (rx_byte == MSG_RX_CMD) && (!cmd_lst) ? (COM_DBG_RX_CRC)              : (p_msg->state);
//         p_msg->state                   = (rx_byte == MSG_RX_CFG)   ? (COM_DBG_RX_CFG_LIST_4_TX)                : (p_msg->state);
//         p_msg->crc                     = (rx_byte == MSG_RX_CMD)   ? (COM_crc8Fast(p_msg->crc, &rx_byte, 1))   : (p_msg->crc);
//         p_msg->crc                     = (rx_byte == MSG_RX_CFG)   ? (COM_crc8Fast(p_msg->crc, &rx_byte, 1))   : (0);
//         p_msg->msgType                 = (rx_byte == MSG_RX_CMD)   ? (MSG_RX_CMD)                              : (0);
//         p_msg->msgType                 = (rx_byte == MSG_RX_CFG)   ? (MSG_RX_CFG)                              : (p_msg->msgType);
//         break;
//
//     case COM_DBG_RX_CFG_LIST_4_TX:
//         p_msg->cfg_lst.byte[i++]       = rx_byte;
//         p_msg->state                   = (i < sizeof(config_u))    ? (COM_DBG_RX_CFG_LIST_4_TX)    : (COM_DBG_RX_CMD_LIST_4_RX);
//         i                              = (i < sizeof(config_u))    ? (i)                           : (0);
//         p_msg->crc                     = COM_crc8Fast(p_msg->crc, &rx_byte, 1);
//         break;
//
//     case COM_DBG_RX_CMD_LIST_4_RX:
//         p_msg->cmd_lst.byte[i++]       = rx_byte;
//         p_msg->state                   = (i < sizeof(command_u))   ? (COM_DBG_RX_CMD_LIST_4_RX)    : (COM_DBG_RX_CRC);
//         i                              = (i < sizeof(command_u))   ? (i)                           : (0);
//         p_msg->crc                     = COM_crc8Fast(p_msg->crc, &rx_byte, 1);
//         break;
//
//     case COM_DBG_RX_CMD_MSG:
//         i                              = (i % 4)   ? (i)       : (0);
//         j                              = (i % 4)   ? (j)       : (ffs(cmd_lst) - 1);
//         cmd_lst                       &= (i % 4)   ? (cmd_lst) : (~(1U << j));
//         p_msg->cmd_msg[j].uint_8b[i++] = rx_byte;
//         p_msg->state                   = (cmd_lst || (i % 4))  ? (COM_DBG_RX_CMD_MSG)  : (COM_DBG_RX_CRC);
//         p_msg->crc                     = COM_crc8Fast(p_msg->crc, &rx_byte, 1);
//         break;
//
//     case COM_DBG_RX_CRC:
//         p_msg->state                   = COM_DBG_RX_WORD_A;
////         p_msg->msgType             = (p_msg->crc == rx_byte)   ? (msg_type)    : (p_msg->msgType);
//         p_msg->valid                   = (p_msg->crc == rx_byte)   ? (true)    : (false);
//         break;
//     }
//
//}

uint32_t COM_crc32(uint16_t* p_buf, size_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  while (len--)
    {
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ MSB_16(*p_buf)) & 0xFF];
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ LSB_16(*p_buf)) & 0xFF];
      p_buf++;
    }

  return(crc);
}


uint8_t COM_crc8(uint8_t *p_buf, size_t len)
{
    uint8_t crc = 0, i = 0, j = 0;
    for(i = 0; i < len; i++)
    {
        uint8_t inbyte = p_buf[i];
        for(j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if(mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return(crc);
}

//uint8_t com_crc8_fast(uint8_t crc, void const *p_buf, size_t len)
//{
//    uint8_t const *data = p_buf;
//    if (data == NULL)
//        return 0xff;
//    crc &= 0xff;
//    while (len--)
//        crc = crc8_table[crc ^ *data++];
//    return crc;
//}

uint8_t COM_crc8Fast(uint8_t crc, uint8_t* p_buf, size_t len)
{
//    uint8_t crc = 0;
    size_t i = 0;
    for(i = 0; i < len; i++)
        crc = crc8_table[p_buf[i] ^ crc];
    return crc;
}
