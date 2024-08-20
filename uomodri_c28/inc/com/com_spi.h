#ifndef __COM_SPI_H__
#define __COM_SPI_H__

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
#define COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH     (sizeof(slv2mst_msg_t))
#define COM_MSG_SPI_TX_16BIT_FULL_LENGTH        (COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH + sizeof(crc_reg_t))

#define COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH     (sizeof(mst2slv_msg_t))
#define COM_MSG_SPI_RX_16BIT_FULL_LENGTH        (COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH + sizeof(crc_reg_t))

/***********************************************************************
 * COMMUNICATION FOR SPI
 ***********************************************************************/
/**
 * @struct  spi_rx_msg_t
 * @brief   SPI reception message structure. Separating received values and CRC to be computed.
 */
typedef struct __spi_rx_msg_t__
{
    mst2slv_msg_t   data;
    crc_reg_t       crc;
} spi_rx_msg_t;

/**
 * @struct  spi_tx_msg_t
 * @brief   SPI transmission message structure. Separating values to transmit and computed CRC.
 */
typedef struct __spi_tx_msg_t__
{
    slv2mst_msg_t   data;
    crc_reg_t       crc;
} spi_tx_msg_t;

/**
 * @struct  spi_rx_t
 * @brief   SPI global reception structure.
 */
typedef struct __spi_rx_t__
{
    spi_rx_msg_t    msg;
} spi_rx_t;

/**
 * @struct  spi_tx_t
 * @brief   SPI global transmission structure.
 */
typedef struct __spi_tx_t__
{
    spi_tx_msg_t    msg[2];
    uint16_t        msg_cr;
    uint16_t        msg_nx;
} spi_tx_t;

#endif /*__COM_SPI_H__*/
