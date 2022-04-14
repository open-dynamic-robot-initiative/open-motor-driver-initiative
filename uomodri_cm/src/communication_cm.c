/*
 * File name: communication_cm.c
 * Description: Source file containing extraction & build/update message functions
 */

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "communication.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define LSB_32(x)                               ((x) & 0x0000FFFF)
#define MSB_32(x)                               (((x) >> 16) & 0x0000FFFF)
#define LSB_16(x)                               ((x) & 0x00FF)
#define MSB_16(x)                               (((x) >> 8) & 0x00FF)

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
    static uint32_t cntMsg              = 0;
    p_msg->count                        = cntMsg++;
    p_msg->position.u16_lsb             = *p_addr->position_msb;
    p_msg->position.u16_msb             = *p_addr->position_lsb;
    p_msg->iqRef                        = (int16_t)(*p_addr->iqRef / IQ_LSB);
    p_msg->iqMeas                       = (int16_t)(*p_addr->iqMeas / IQ_LSB);
    p_msg->idMeas                       = (int16_t)(*p_addr->idMeas / IQ_LSB);
    p_msg->dtc_u                        = (int16_t)(*p_addr->dtc_u / VOLTAGE_LSB);
    p_msg->dtc_v                        = (int16_t)(*p_addr->dtc_v / VOLTAGE_LSB);
    p_msg->dtc_w                        = (int16_t)(*p_addr->dtc_w / VOLTAGE_LSB);

//    p_msg->stateMachine                 = (uint8_t)*p_addr->stateMachine;//[MOTOR_1];
//    p_msg->stateMachine.u8_m2_state     = (uint8_t)*p_addr->stateMachine[MOTOR_2];
    p_msg->crc                          = com_crc8((uint8_t *)p_msg, DBG_TX_MSG_PAYLOAD_8BIT_SIZE);

    return;
}

uint32_t com_crc32(uint16_t* p_buf, uint16_t len)
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

uint8_t com_crc8(uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0, i = 0, j = 0;
    for (i = 0; i < len; i++)
    {
        uint8_t inbyte = addr[i];
        for (j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}
