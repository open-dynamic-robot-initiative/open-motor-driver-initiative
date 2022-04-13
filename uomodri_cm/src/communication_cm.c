/*
 * File name: communication_cm.c
 * Description: Source file containing extraction & build/update message functions
 */

/***********************************************************************.
 * INCLUDE FILES
 ************************************************************************/
#include "communication.h"

#define LSB_16(x)                               ((x) & 0x00FF)
#define MSB_16(x)                               (((x) >> 8) & 0x00FF)

#define LSB_32(x)                               ((x) & 0x0000FFFF)
#define MSB_32(x)                               (((x) >> 16) & 0x0000FFFF)

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Message created/updates for debug
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
void com_msgDbgTx(foc_t* p_foc_m1, foc_t* p_foc_m2, slv2mst_msg_t* p_status_msg, dbg_uart_msg_t* p_msg)
{
    static uint32_t cntMsg              = 0;
    p_msg->header[0]                    = 'a';
    p_msg->header[1]                    = 'b';
    p_msg->header[2]                    = 'c';
    p_msg->header[3]                    = 'd';
    p_msg->timeStamp                    = cntMsg++;
    p_msg->status                       = p_status_msg->status;
    p_msg->position[MOTOR_1].u16_msb    = p_status_msg->position[MOTOR_1].u16_lsb;
    p_msg->position[MOTOR_1].u16_lsb    = p_status_msg->position[MOTOR_1].u16_msb;
    p_msg->position[MOTOR_2].u16_msb    = p_status_msg->position[MOTOR_2].u16_lsb;
    p_msg->position[MOTOR_2].u16_lsb    = p_status_msg->position[MOTOR_2].u16_msb;
    p_msg->current[MOTOR_1]             = p_status_msg->current[MOTOR_1];
    p_msg->current[MOTOR_2]             = p_status_msg->current[MOTOR_2];

    p_msg->stateMachine.bit.M1_CR_STATE = MOTOR_STATE_READY;//p_foc_m1->motor_state[CURRENT];
    p_msg->stateMachine.bit.M1_NX_STATE = MOTOR_STATE_ERROR;//p_foc_m1->motor_state[NEXT];
    p_msg->stateMachine.bit.M2_CR_STATE = MOTOR_STATE_ALIGN_FIX;//p_foc_m2->motor_state[CURRENT];
    p_msg->stateMachine.bit.M2_NX_STATE = MOTOR_STATE_STOP;//p_foc_m2->motor_state[NEXT];

//    p_msg->stateMachine                 = p_foc_m2->motor_state[NEXT];
//    p_msg->stateMachine               <<= 4;
//
//    p_msg->stateMachine                |= p_foc_m2->motor_state[CURRENT];
//    p_msg->stateMachine               <<= 4;
//
//    p_msg->stateMachine                |= p_foc_m1->motor_state[NEXT];
//    p_msg->stateMachine               <<= 4;
//
//    p_msg->stateMachine                |= p_foc_m1->motor_state[CURRENT];

//    tmp = ((p_foc_m1->motor_state[NEXT] != MOTOR_STATE_ERROR) ? (p_foc_m1->motor_state[NEXT]) : (0x000F)) << 4;
//        p_msg->stateMachine                |= tmp;
//        tmp = ((p_foc_m2->motor_state[CURRENT] != MOTOR_STATE_ERROR) ? (p_foc_m2->motor_state[CURRENT]) : (0x000F)) << 8;
//        p_msg->stateMachine                |= tmp;
//        tmp = ((p_foc_m2->motor_state[NEXT] != MOTOR_STATE_ERROR) ? (p_foc_m2->motor_state[NEXT]) : (0x000F)) << 12;
//        p_msg->stateMachine                |= tmp;
//
//
//    tmp = ((p_foc_m1->motor_state[CURRENT] != MOTOR_STATE_ERROR) ? (p_foc_m1->motor_state[CURRENT]) : (0x000F));
//    p_msg->stateMachine                 = tmp;
//    tmp = ((p_foc_m1->motor_state[NEXT] != MOTOR_STATE_ERROR) ? (p_foc_m1->motor_state[NEXT]) : (0x000F)) << 4;
//    p_msg->stateMachine                |= tmp;
//    tmp = ((p_foc_m2->motor_state[CURRENT] != MOTOR_STATE_ERROR) ? (p_foc_m2->motor_state[CURRENT]) : (0x000F)) << 8;
//    p_msg->stateMachine                |= tmp;
//    tmp = ((p_foc_m2->motor_state[NEXT] != MOTOR_STATE_ERROR) ? (p_foc_m2->motor_state[NEXT]) : (0x000F)) << 12;
//    p_msg->stateMachine                |= tmp;

    p_msg->crc = com_crc8((uint8_t *)p_msg, DBG_TX_MSG_PAYLOAD_8BIT_SIZE);

//    uint32_t crc = com_crc32((uint16_t *)p_msg, DBG_TX_MSG_PAYLOAD_16BIT_SIZE);
//    p_msg->crc.u16_msb                  = LSB_32(crc);// required for Solo
//    p_msg->crc.u16_lsb                  = MSB_32(crc);


//    if(((i % 4) == 0) && (i != 0) && (i < DBG_TX_MSG_PAYLOAD_8BIT_SIZE))
//    for(i = 0; i < (DBG_TX_MSG_PAYLOAD_8BIT_SIZE / 4); i++)
//        GCRC_writeData(GCRC_BASE, *p_msg_32b++, GCRC_DATATYPE_32BIT);
//    crc = GCRC_readResult(GCRC_BASE);
//
//    p_msg->crc.u16_lsb = (uint16_t) (crc & 0x0000FFFF);
//    p_msg->crc.u16_msb = (uint16_t) ((crc >> 16) & 0x0000FFFF);

//    for(i = 0; i < DBG_TX_MSG_SIZE; i++)
//    {
//        if(i < DBG_TX_MSG_HEADER_8BIT_SIZE)
//            *p_msg = 'a' + i;
//        else if(i < (DBG_TX_MSG_HEADER_8BIT_SIZE + DBG_TX_MSG_COUNTER_8BIT_SIZE))
//            *p_msg = *p_cntMsg++;
//        else if(i < DBG_TX_MSG_PAYLOAD_8BIT_SIZE)
//            *p_msg = *p_array++;
//        else
//        {
//            if(i == DBG_TX_MSG_PAYLOAD_8BIT_SIZE)

//            *p_msg = *p_crc++;
//        }
//        p_msg++;
//
//        if(((i % 4) == 0) && (i != 0) && (i < DBG_TX_MSG_PAYLOAD_8BIT_SIZE))
//            GCRC_writeData(GCRC_BASE, *p_msg_32b++, GCRC_DATATYPE_32BIT);
//
//    }
//    cntMsg++;

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
    for (i = 0; i < len; i++) {
        uint8_t inbyte = addr[i];
        for (j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}
