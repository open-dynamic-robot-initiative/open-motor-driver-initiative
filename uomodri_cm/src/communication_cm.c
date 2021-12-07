/*
 * File name: communication_cm.c
 * Description: Source file containing extraction & build/update message functions
 */

// Includes
#include "communication_cm.h"

// Function definitions
/**
 * @brief       Extract commands from Master to be performed by motor_foc
 * @param p_msg Pointer on the message received from Master
 * @param p_cmd Pointer on the command structure (actions to be realized by motor_foc)
 * @return      Message received from Master is valid or not ?
 */
bool com_cmdRx(rx_msg_t* p_msg, rx_uomodri_t* p_cmd)
{
    uint32_t addr_offset = (uint32_t)p_msg + sizeof(rx_payload_t);
    bool rx_msg_valid = false;

    while((uint32_t)p_msg < addr_offset)
        GCRC_writeData(GCRC_BASE, *(uint32_t *)p_msg++, GCRC_DATATYPE_32BIT);

//    GCRC_writeData(GCRC_BASE, p_msg->payload.modeReg, GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.positionRef[MOTOR_1], GCRC_DATATYPE_32BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.positionRef[MOTOR_2], GCRC_DATATYPE_32BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.velocityRef[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.velocityRef[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.iqRef[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.iqRef[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.kpCoeff[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.kpCoeff[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.kdCoeff[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.kdCoeff[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.iSat[MOTOR_1], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.iSat[MOTOR_2], GCRC_DATATYPE_16BIT);
//    GCRC_writeData(GCRC_BASE, p_msg->payload.index, GCRC_DATATYPE_16BIT);

    if(GCRC_readResult(GCRC_BASE) == p_msg->crc)
    {
        // run crc check
        p_cmd->modeReg.bit.TIMEOUT  = p_msg->payload.modeReg.bit.TIMEOUT;
        p_cmd->modeReg.bit.EI2OC    = p_msg->payload.modeReg.bit.EI2OC;
        p_cmd->modeReg.bit.EI1OC    = p_msg->payload.modeReg.bit.EI1OC;
        p_cmd->modeReg.bit.EPRE     = p_msg->payload.modeReg.bit.EPRE;
        p_cmd->modeReg.bit.EM2      = p_msg->payload.modeReg.bit.EM2;
        p_cmd->modeReg.bit.EM1      = p_msg->payload.modeReg.bit.EM1;
        p_cmd->modeReg.bit.ES       = p_msg->payload.modeReg.bit.ES;
        p_cmd->positionRef[MOTOR_1] = (float32_t)(p_msg->payload.positionRef[MOTOR_1]   * POSITION_LSB);
        p_cmd->positionRef[MOTOR_2] = (float32_t)(p_msg->payload.positionRef[MOTOR_2]   * POSITION_LSB);
        p_cmd->velocityRef[MOTOR_1] = (float32_t)(p_msg->payload.velocityRef[MOTOR_1]   * VELOCITY_LSB);
        p_cmd->velocityRef[MOTOR_2] = (float32_t)(p_msg->payload.velocityRef[MOTOR_2]   * VELOCITY_LSB);
        p_cmd->iqRef[MOTOR_1]       = (float32_t)(p_msg->payload.iqRef[MOTOR_1]         * IQ_LSB);
        p_cmd->iqRef[MOTOR_2]       = (float32_t)(p_msg->payload.iqRef[MOTOR_2]         * IQ_LSB);
        p_cmd->kpCoeff[MOTOR_1]     = (float32_t)(p_msg->payload.kpCoeff[MOTOR_1]       * KP_LSB);
        p_cmd->kpCoeff[MOTOR_2]     = (float32_t)(p_msg->payload.kpCoeff[MOTOR_2]       * KP_LSB);
        p_cmd->kdCoeff[MOTOR_1]     = (float32_t)(p_msg->payload.kdCoeff[MOTOR_1]       * KD_LSB);
        p_cmd->kdCoeff[MOTOR_2]     = (float32_t)(p_msg->payload.kdCoeff[MOTOR_2]       * KD_LSB);
        p_cmd->iSat[MOTOR_1]        = (float32_t)(p_msg->payload.iSat[MOTOR_1]          * CURRENT_SAT_LSB);
        p_cmd->iSat[MOTOR_1]        = (float32_t)(p_msg->payload.iSat[MOTOR_2]          * CURRENT_SAT_LSB);
        rx_msg_valid                = true;
    }

    return(rx_msg_valid);
}

/**
 * @brief       Create/update the status message returned to Master using motor_foc data structure.
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message to create/update for Master
 * @return      Is message fully created/updated using motor_1 \& motor_2 data structure.
 */
bool com_msgTx(FOC_STRUCT* p_foc, tx_msg_t* p_msg)
{
    uint32_t* p_crc = (uint32_t *)p_msg;
    bool tx_msg_end = false;

    if(p_foc->motor_id == MOTOR_1)
    {
        p_msg->payload.statusReg.bit.SE     = p_foc->system_enable;                                         // System enabled
        // Update status field - MOTOR_1
        p_msg->payload.statusReg.bit.M1E    = p_foc->motor_enable;                                          // MOTOR_1 enabled
        p_msg->payload.statusReg.bit.M1R    = (p_foc->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_1 ready
        p_msg->payload.statusReg.bit.IDX1T  = p_foc->motor_enc.indexToggle;                                 // MOTOR_1 index toggle
        p_msg->payload.statusReg.bit.IDX1D  = p_foc->motor_enc.indexDetected;                               // MOTOR_1 index detected
        // Update time stamp in milliseconds - MOTOR_1
        p_msg->payload.timeStamp            = (uint16_t)(p_msg->time[MOTOR_1] * 1000.0f);
        // Update motor position, velocity, Iq, coil resistance and alternative aquisition - MOTOR_1
        p_msg->payload.position[MOTOR_1]    = (int32_t)(p_foc->motor_enc.thetaAbsolute / POSITION_LSB);
        p_msg->payload.velocity[MOTOR_1]    = (int16_t)(p_foc->motor_enc.speedEst.speedMech / VELOCITY_LSB);
        p_msg->payload.iq[MOTOR_1]          = (int16_t)(p_foc->iq / IQ_LSB);
        p_msg->payload.coilRes[MOTOR_1]     = (uint16_t)(p_foc->resistanceEst / RESISTANCE_LSB);
        p_msg->payload.adcSamples[MOTOR_1]  = (uint16_t)(p_foc->motor_acq.auxAcq / VOLTAGE_LSB);
        tx_msg_end                          = false;
    }
    else if(p_foc->motor_id == MOTOR_2)
    {
        p_msg->payload.statusReg.bit.SE    &= p_foc->system_enable;                                         // System enabled
        // Update status field - MOTOR_2
        p_msg->payload.statusReg.bit.M2E    = p_foc->motor_enable;                                          // MOTOR_2 enabled
        p_msg->payload.statusReg.bit.M2R    = (p_foc->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_2 ready
        p_msg->payload.statusReg.bit.IDX2T  = p_foc->motor_enc.indexToggle;                                 // MOTOR_2 index toggle
        p_msg->payload.statusReg.bit.IDX2D  = p_foc->motor_enc.indexDetected;                               // MOTOR_2 index detected
        // Update time stamp in milliseconds - MOTOR_1
        p_msg->payload.timeStamp            = (uint16_t)(p_msg->time[MOTOR_2] * 1000.0f);
        // Update motor position, velocity, Iq, coil resistance and alternative aquisition - MOTOR_1
        p_msg->payload.position[MOTOR_2]    = (int32_t)(p_foc->motor_enc.thetaAbsolute / POSITION_LSB);
        p_msg->payload.velocity[MOTOR_2]    = (int16_t)(p_foc->motor_enc.speedEst.speedMech / VELOCITY_LSB);
        p_msg->payload.iq[MOTOR_2]          = (int16_t)(p_foc->iq / IQ_LSB);
        p_msg->payload.coilRes[MOTOR_2]     = (uint16_t)(p_foc->resistanceEst / RESISTANCE_LSB);
        p_msg->payload.adcSamples[MOTOR_2]  = (uint16_t)(p_foc->motor_acq.auxAcq / VOLTAGE_LSB);
        tx_msg_end                          = true;
    }
    p_msg->payload.statusReg.bit.ERROR_CODE = 0x00;//p_cmdcomInterface.canError;
    // Update last received command index
    //p_msg->lastCmdIndex             = comInterface.lastCommandIndex;

    while((uint32_t)p_crc < ((uint32_t)p_msg + sizeof(tx_payload_t)))
        GCRC_writeData(GCRC_BASE, *p_crc++, GCRC_DATATYPE_32BIT);
    p_msg->crc = GCRC_readResult(GCRC_BASE);

    return(tx_msg_end);
}

/**
 * @brief       Message created/updates for debug
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
void com_msgDbgTx(FOC_STRUCT* p_foc, uint8_t* p_msg)
{
    float32_t       arrayTmp[4];
    static uint32_t CntMsg = 0;
    //    uint8_t     aMsgCreate[MSG_SIZE] = {'a', 'b', 'c', 'd'};
    uint8_t         *pspl = NULL, *pCntMsg = NULL, i = 0;

    arrayTmp[0] = p_foc->motor_acq.ia + 1.0;
    arrayTmp[1] = p_foc->motor_acq.ib + 1.0;
    arrayTmp[2] = p_foc->motor_acq.ic + 1.0;
    arrayTmp[3] = p_foc->motor_enc.thetaMech[NEW];
    //arrayTmp[4] = p_foc->motor_enc.thetaElec;
    pCntMsg     = (uint8_t *)&CntMsg;
    pspl        = (uint8_t *)&arrayTmp[0];

    for(i = 0; i < DBG_TX_MSG_SIZE; i++)
    {
        if(i < 4)
            *p_msg = 'a' + i; //aMsgCreate[i] = 'a' + i;
        else if(i < 8)
            *p_msg = *pCntMsg++; //aMsgCreate[i] = *pCntMsg++;
        else if(i < DBG_TX_MSG_SIZE)
            *p_msg = *pspl++; //aMsgCreate[i] = *pspl++;
        p_msg++;
    }
    CntMsg++;
}
