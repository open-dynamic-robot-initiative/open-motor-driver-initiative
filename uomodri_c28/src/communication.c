/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "communication.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Master to slave message extraction to be performed by motor_foc
 * @param[in]   p_msg   Pointer on the message received from Master board
 * @param[out]  p_cmd   Pointer on the command structure (actions to be realized by motor_foc)
 * @return      Is message received from Master board valid or not ?
 */
bool_t com_MsgExtract(mst2slv_msg_t* p_msg, cmd_uomodri_t* p_cmd)
{
    uint32_t crc = com_crc32((uint16_t *) p_msg, DMA_COM_SPI_TX_RX_16BIT_PAYLOAD_LENGTH);
    if((MSB_32(crc) == p_msg->crc.u16_msb) && (LSB_32(crc) == p_msg->crc.u16_lsb))
    {
        U32toU16_reg_t pos_m1       = p_msg->position[MOTOR_1];
        U32toU16_reg_t pos_m2       = p_msg->position[MOTOR_2];
        p_cmd->mode.bit.TIMEOUT     = p_msg->mode.bit.TIMEOUT;
        p_cmd->mode.bit.EI2OC       = p_msg->mode.bit.EI2OC;
        p_cmd->mode.bit.EI1OC       = p_msg->mode.bit.EI1OC;
        p_cmd->mode.bit.EPRE        = p_msg->mode.bit.EPRE;
        p_cmd->mode.bit.EM2         = p_msg->mode.bit.EM2;
        p_cmd->mode.bit.EM1         = p_msg->mode.bit.EM1;
        p_cmd->mode.bit.ES          = p_msg->mode.bit.ES;
        p_cmd->position[MOTOR_1]    = (float32_t)((int32_t)FUS_16_TO_32(pos_m1.u16_msb, pos_m1.u16_lsb) * POSITION_LSB);
        p_cmd->position[MOTOR_2]    = (float32_t)((int32_t)FUS_16_TO_32(pos_m2.u16_msb, pos_m2.u16_lsb) * POSITION_LSB);
        p_cmd->velocity[MOTOR_1]    = (float32_t)(p_msg->velocity[MOTOR_1]                              * VELOCITY_LSB);
        p_cmd->velocity[MOTOR_2]    = (float32_t)(p_msg->velocity[MOTOR_2]                              * VELOCITY_LSB);
        p_cmd->current[MOTOR_1]     = (float32_t)(p_msg->current[MOTOR_1]                               * IQ_LSB);
        p_cmd->current[MOTOR_2]     = (float32_t)(p_msg->current[MOTOR_2]                               * IQ_LSB);
        p_cmd->kpCoeff[MOTOR_1]     = (float32_t)(p_msg->kpCoeff[MOTOR_1]                               * KP_LSB);
        p_cmd->kpCoeff[MOTOR_2]     = (float32_t)(p_msg->kpCoeff[MOTOR_2]                               * KP_LSB);
        p_cmd->kdCoeff[MOTOR_1]     = (float32_t)(p_msg->kdCoeff[MOTOR_1]                               * KD_LSB);
        p_cmd->kdCoeff[MOTOR_2]     = (float32_t)(p_msg->kdCoeff[MOTOR_2]                               * KD_LSB);
        p_cmd->iSat[MOTOR_1]        = (float32_t)(MSB_16(p_msg->iSat)                                   * CURRENT_SAT_LSB);
        p_cmd->iSat[MOTOR_2]        = (float32_t)(LSB_16(p_msg->iSat)                                   * CURRENT_SAT_LSB);
        p_cmd->index                = (uint16_t) p_msg->index;
        return (true);
    }

    return(false);
}

/**
 * @brief       Slave to Master message creation/update using motor_foc data structure.
 * @param[in]   p_foc   Pointer on data structure array used for motor control.
 * @param[in]   p_cmd   Pointer on the command structure (actions to be realized by motor_foc)
 * @param[out]  p_msg   Pointer on the message to transmit tothe Master board
 * @return      Is message fully created/updated using motor_1 \& motor_2 data structure.
 */
bool_t com_MsgCreate(foc_t* p_foc, cmd_uomodri_t* p_cmd, slv2mst_msg_t* p_msg)
{
    int32_t pos                             = (int32_t)(p_foc->motor_enc.thetaAbsolute / POSITION_LSB);
    p_msg->status.bit.SE                    = p_cmd->mode.bit.ES;                                           // System enabled
    p_msg->status.bit.ERROR_CODE            = STATUS_ERROR_NO_ERROR;
    p_msg->lastCmdIndex                     = p_cmd->index;
//    p_msg->timeStamp++;//
    if(p_foc->motor_id == MOTOR_1)
    {
        // Update status field - MOTOR_1
        p_msg->status.bit.M1E               = p_cmd->mode.bit.EM1;                                          // MOTOR_1 enabled
        p_msg->status.bit.M1R               = (p_foc->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_1 ready
        p_msg->status.bit.IDX1T             = p_foc->motor_enc.indexToggle;                                 // MOTOR_1 index toggle
        p_msg->status.bit.IDX1D             = p_foc->motor_enc.indexDetected;                               // MOTOR_1 index detected
        p_msg->position[MOTOR_1].u16_msb    = MSB_32(pos);
        p_msg->position[MOTOR_1].u16_lsb    = LSB_32(pos);
        p_msg->velocity[MOTOR_1]            = (int16_t)(p_foc->motor_enc.speed.speedMech / VELOCITY_LSB);
        p_msg->current[MOTOR_1]             = (int16_t)(p_foc->iq / IQ_LSB);
        p_msg->coilRes[MOTOR_1]             = (uint16_t)(p_foc->resEst / RESISTANCE_LSB);
        p_msg->adcSamples[MOTOR_1]          = (uint16_t)(p_foc->motor_acq.auxAcq / VOLTAGE_LSB);
    }
    else if(p_foc->motor_id == MOTOR_2)
    {
        // Update status field - MOTOR_2
        p_msg->status.bit.M2E               = p_cmd->mode.bit.EM2;                                          // MOTOR_2 enabled
        p_msg->status.bit.M2R               = (p_foc->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_2 ready
        p_msg->status.bit.IDX2T             = p_foc->motor_enc.indexToggle;                                 // MOTOR_2 index toggle
        p_msg->status.bit.IDX2D             = p_foc->motor_enc.indexDetected;                               // MOTOR_2 index detected
        p_msg->position[MOTOR_2].u16_msb    = MSB_32(pos);
        p_msg->position[MOTOR_2].u16_lsb    = LSB_32(pos);
        p_msg->velocity[MOTOR_2]            = (int16_t)(p_foc->motor_enc.speed.speedMech / VELOCITY_LSB);
        p_msg->current[MOTOR_2]             = (int16_t)(p_foc->iq / IQ_LSB);
        p_msg->coilRes[MOTOR_2]             = (uint16_t)(p_foc->resEst / RESISTANCE_LSB);
        p_msg->adcSamples[MOTOR_2]          = (uint16_t)(p_foc->motor_acq.auxAcq / VOLTAGE_LSB);
        // Compute CRC
        uint32_t crc = com_crc32((uint16_t *)p_msg, DMA_COM_SPI_TX_RX_16BIT_PAYLOAD_LENGTH);
        p_msg->crc.u16_msb                  = LSB_32(crc);// required for Solo
        p_msg->crc.u16_lsb                  = MSB_32(crc);
        return (true);
    }

    return(false);
}

/**
 * @brief       Software cyclic redundancy check (CRC)..
 * @param[in]   p_buf   Pointer on the array to be CRCed.
 * @param[in]   len     Length of the array
 * @return
 */
uint32_t com_crc32 (uint16_t* p_buf, uint16_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  while (len--)
    {
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ MSB_16(*p_buf)) & 0xFF];
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ LSB_16(*p_buf)) & 0xFF];
      p_buf++;
    }
  return crc;
}

/**
 * @brief       Message created/updates for debug
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
//void com_msgDbgTx(FOC_STRUCT* p_foc, uint8_t* p_msg)
//{
//    float32_t       arrayTmp[4];
//    static uint32_t CntMsg = 0;
//    //    uint8_t     aMsgCreate[MSG_SIZE] = {'a', 'b', 'c', 'd'};
//    uint8_t         *pspl = NULL, *pCntMsg = NULL, i = 0;
//
//    arrayTmp[0] = p_foc->motor_acq.ia + 1.0;
//    arrayTmp[1] = p_foc->motor_acq.ib + 1.0;
//    arrayTmp[2] = p_foc->motor_acq.ic + 1.0;
//    arrayTmp[3] = p_foc->motor_enc.thetaMech[NEW];
//    //arrayTmp[4] = p_foc->motor_enc.thetaElec;
//    pCntMsg     = (uint8_t *)&CntMsg;
//    pspl        = (uint8_t *)&arrayTmp[0];
//
//    for(i = 0; i < DBG_TX_MSG_SIZE; i++)
//    {
//        if(i < 4)
//            *p_msg = 'a' + i; //aMsgCreate[i] = 'a' + i;
//        else if(i < 8)
//            *p_msg = *pCntMsg++; //aMsgCreate[i] = *pCntMsg++;
//        else if(i < DBG_TX_MSG_SIZE)
//            *p_msg = *pspl++; //aMsgCreate[i] = *pspl++;
//        p_msg++;
//    }
//    CntMsg++;
//}

