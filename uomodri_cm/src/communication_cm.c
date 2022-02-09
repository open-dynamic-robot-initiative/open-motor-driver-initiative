/*
 * File name: communication_cm.c
 * Description: Source file containing extraction & build/update message functions
 */

/***********************************************************************.
 * INCLUDE FILES
 ************************************************************************/
#include "communication_cm.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Message created/updates for debug
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
void com_msgDbgTx(foc_t* p_foc, uint8_t* p_msg)
{
    static uint32_t cntMsg      = 0;
    uint32_t        crc         = 0;
    uint32_t*       p_msg_32b   = (uint32_t *)p_msg;
    float32_t       arrayTmp[4] = {p_foc->motor_acq.ia, p_foc->motor_acq.ib,
                                   p_foc->motor_acq.ic, p_foc->motor_enc.thetaMech[NEW]};
    uint8_t*        p_array     = (uint8_t *)&arrayTmp[0];
    uint8_t*        p_cntMsg    = (uint8_t *)&cntMsg;
    uint8_t*        p_crc       = (uint8_t *)&crc;
    uint8_t         i           = 0;

    for(i = 0; i < DBG_TX_MSG_SIZE; i++)
    {
        if(i < DBG_TX_MSG_HEADER_8BIT_SIZE)
            *p_msg = 'a' + i;
        else if(i < (DBG_TX_MSG_HEADER_8BIT_SIZE + DBG_TX_MSG_COUNTER_8BIT_SIZE))
            *p_msg = *p_cntMsg++;
        else if(i < DBG_TX_MSG_PAYLOAD_8BIT_SIZE)
            *p_msg = *p_array++;
        else
        {
            if(i == DBG_TX_MSG_PAYLOAD_8BIT_SIZE)
                crc = GCRC_readResult(GCRC_BASE);
            *p_msg = *p_crc++;
        }
        p_msg++;

        if(((i % 4) == 0) && (i != 0) && (i < DBG_TX_MSG_PAYLOAD_8BIT_SIZE))
            GCRC_writeData(GCRC_BASE, *p_msg_32b++, GCRC_DATATYPE_32BIT);

    }
    cntMsg++;

    return;
}
