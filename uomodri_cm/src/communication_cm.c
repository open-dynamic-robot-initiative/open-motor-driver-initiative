/*
 * File name: communication_cm.c
 * Description: Source file containing extraction & build/update message functions
 */

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "communication.h"
#include "uomodri_cm_user_defines.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define LSB_32(x)           ((x) & 0x0000FFFF)
#define MSB_32(x)           (((x) >> 16) & 0x0000FFFF)
#define LSB_16(x)           ((x) & 0x00FF)
#define MSB_16(x)           (((x) >> 8) & 0x00FF)

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Message created/updates for debug
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
void com_msgDbgTx(dbg_uart_addr_t* p_addr, dbg_uart_msg_t* p_msg)
{
    static uint32_t cntMsg  = 0;
    p_msg->count            = cntMsg++;
    p_msg->position         = (float)(*p_addr->p_position);
    p_msg->iqRef            = (int16_t)(*p_addr->p_iqRef / IQ_LSB);
    p_msg->iq               = (int16_t)(*p_addr->p_iq / IQ_LSB);
    p_msg->id               = (int16_t)(*p_addr->p_id / IQ_LSB);
    p_msg->uq               = (int16_t)(*p_addr->p_uq / IQ_LSB);
    p_msg->ud               = (int16_t)(*p_addr->p_ud / IQ_LSB);
    p_msg->ia               = (int16_t)(*p_addr->p_ia / IQ_LSB);
    p_msg->ib               = (int16_t)(*p_addr->p_ib / IQ_LSB);
    p_msg->ic               = (int16_t)(*p_addr->p_ic / IQ_LSB);
//    p_msg->speed            = (float32_t)(*p_addr->p_speed);
//    p_msg->speedMech        = (float32_t)(*p_addr->p_speedMech);
    p_msg->crc              = com_crc8_fast(0, (uint8_t *)p_msg, (sizeof(dbg_uart_msg_t) - 1));
    return;
}

uint32_t com_crc32(uint16_t* p_buf, size_t len)
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

uint8_t com_crc8(uint8_t *p_buf, size_t len)
{
    uint8_t crc = 0, i = 0, j = 0;
    for (i = 0; i < len; i++)
    {
        uint8_t inbyte = p_buf[i];
        for (j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return(crc);
}

uint8_t com_crc8_fast(uint8_t crc, void const *p_buf, size_t len)
{
    uint8_t const *data = p_buf;
    if (data == NULL)
        return 0xff;
    crc &= 0xff;
    while (len--)
        crc = crc8_table[crc ^ *data++];
    return crc;
}
