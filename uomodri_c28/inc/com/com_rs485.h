#ifndef __COM_RS485_H__
#define __COM_RS485_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#ifdef CPU1
#include "f2838x_device.h"
#include "../communication.h"
#else
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include "uomodri_cm_user_defines.h"
#endif

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define COM_MSG_EXTRACT_TX_RX_MAX_SIZE          (64)
#define COM_MSG_RS485_TX_RX_MAX_SIZE            (20)
#define COM_MSG_RS485_RX_ADDRESS_1              (0x42)
#define COM_MSG_RS485_RX_ADDRESS_2              (0x00)
#define COM_MSG_RS485_TX_ADDRESS                (0xFF)
#define COM_MSG_RS485_TX_TYPE_STANDARD          (0x00)

#define COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH   (sizeof(header_reg_u) + sizeof(slv2mst_msg_t))
#define COM_MSG_RS485_TX_8BIT_PAYLOAD_LENGTH    (COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH * 2)
#define COM_MSG_RS485_TX_16BIT_FULL_LENGTH      (COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH + sizeof(crc_reg_t))
#define COM_MSG_RS485_TX_8BIT_FULL_LENGTH       (COM_MSG_RS485_TX_16BIT_FULL_LENGTH * 2)

#define COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH   (sizeof(header_reg_u) + sizeof(mst2slv_msg_t))
#define COM_MSG_RS485_RX_8BIT_PAYLOAD_LENGTH    (COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH * 2)
#define COM_MSG_RS485_RX_16BIT_FULL_LENGTH      (COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH + sizeof(crc_reg_t))
#define COM_MSG_RS485_RX_8BIT_FULL_LENGTH       (COM_MSG_RS485_RX_16BIT_FULL_LENGTH * 2)

/***********************************************************************
 * COMMUNICATION FOR RS485
 ***********************************************************************/
/**
 * @union   rs485_tx_msg_u
 * @brief   RS485 reception message structure. Separating header, received values and CRC to be computed.
 */
typedef union __rs485_rx_msg_u__
{
    uint16_t            raw[COM_MSG_RS485_TX_RX_MAX_SIZE];
    struct
    {
        header_reg_u    header;
        mst2slv_msg_t   data;
        crc_reg_t       crc;
    };
} rs485_rx_msg_u;

/**
 * @union   rs485_tx_msg_u
 * @brief   RS485 transmission message structure. Separating header and values to transmit plus computed CRC.
 */
typedef union __rs485_tx_msg_u__
{
    uint16_t            raw[COM_MSG_RS485_TX_RX_MAX_SIZE];
    struct
    {
        header_reg_u    header;
        slv2mst_msg_t   data;
        crc_reg_t       crc;
    };
} rs485_tx_msg_u;

/**
 * @struct  rs485_rx_t
 * @brief   RS485 global reception structure.
 */
typedef struct __rs485_rx_t__
{
    rs485_rx_msg_u      msg;
    uint16_t            index;
    struct
    {
        uint16_t        addr1   : 1;
        uint16_t        addr2   : 1;
        uint16_t        rsv     : 14;
    };
} rs485_rx_t;

/**
 * @struct  rs485_tx_t
 * @brief   RS485 global transmission structure.
 */
typedef struct __rs485_tx_t__
{
    rs485_tx_msg_u      msg[2];
    uint16_t            index;
    uint16_t            msg_cr;
    uint16_t            msg_nx;
} rs485_tx_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
#ifdef CPU1
inline void RS485_msgReceive(uint32_t, rs485_rx_t*);
inline void RS485_msgTransmit(uint32_t, rs485_tx_t*);
#endif

#endif /*__COM_RS485_H__*/
