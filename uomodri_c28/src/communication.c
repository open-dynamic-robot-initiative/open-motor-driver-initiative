/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "uomodri_user_defines.h"
#include "communication.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Master to slave message extraction to be performed by motor_foc
 * @param[in]   p_msg       Pointer on the message received from Master board
 * @param[out]  p_motor_m1  Pointer on the command structure used for MOTOR_1 control(actions to be realized by motor_foc).
 * @param[out]  p_motor_m2  Pointer on the command structure used for MOTOR_2 control(actions to be realized by motor_foc).
 * @return      Is message received from Master board valid or not ?
 */
bool_t COM_msgExtract(mst2slv_msg_t* p_msg,  cmd_t* p_cmd_m1,  cmd_t* p_cmd_m2)
{
    uint32_t crc                        = COM_crc32((uint16_t *) p_msg, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
    if((MSB_32(crc) == p_msg->crc.u16_msb) && (LSB_32(crc) == p_msg->crc.u16_lsb))
    {

        int32_t pos_s32                 = (int32_t)FUS_16_TO_32(p_msg->position[MOTOR_1].u16_msb, p_msg->position[MOTOR_1].u16_lsb);
        cmd_reg_t* p_cmd_reg            = &p_cmd_m1->enableReg.bit;
        p_cmd_m1->posRef                = (float32_t)(pos_s32                   * POSITION_LSB      * FM_ROUND2RAD);
        p_cmd_m1->velRef                = (float32_t)(p_msg->velocity[MOTOR_1]  * VELOCITY_LSB      * FM_KRPM2RADPS);
        p_cmd_m1->iqRef                 = (float32_t)(p_msg->current[MOTOR_1]   * IQ_LSB);
        p_cmd_m1->kpCoeff               = (float32_t)(p_msg->kpCoeff[MOTOR_1]   * KP_LSB            * FM_RAD2ROUND);
        p_cmd_m1->kdCoeff               = (float32_t)(p_msg->kdCoeff[MOTOR_1]   * KD_LSB            * FM_RADPS2KRPM);
        float32_t isat                  = (float32_t)(MSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
        p_cmd_m1->iSat                  = (isat < MOTOR1_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR1_CURRENT_CMD_SAT_MAX);
        p_cmd_m1->iSat                  = (p_cmd_m1->iSat > 0.0f)               ? (p_cmd_m1->iSat)  : (MOTOR1_CURRENT_CMD_SAT_MAX);
        p_cmd_m1->timeoutRef            = (uint16_t)p_msg->mode.bit.TIMEOUT;
        p_cmd_m1->cptTimeout            = 0U;
        p_cmd_m1->index                 = p_msg->index;
        p_cmd_reg->encOffsetEnable      = (bool_t)p_msg->mode.bit.EI1OC;
        p_cmd_reg->rollOverEnable       = (bool_t)p_msg->mode.bit.EPRE;
        p_cmd_reg->motorEnable          = (bool_t)p_msg->mode.bit.EM1;
        p_cmd_reg->systemEnable         = (bool_t)p_msg->mode.bit.ES;

        pos_s32                         = (int32_t)FUS_16_TO_32(p_msg->position[MOTOR_2].u16_msb, p_msg->position[MOTOR_2].u16_lsb);
        p_cmd_reg                       = &p_cmd_m2->enableReg.bit;
        p_cmd_m2->posRef                = (float32_t)(pos_s32                   * POSITION_LSB      * FM_ROUND2RAD);
        p_cmd_m2->velRef                = (float32_t)(p_msg->velocity[MOTOR_2]  * VELOCITY_LSB      * FM_KRPM2RADPS);
        p_cmd_m2->iqRef                 = (float32_t)(p_msg->current[MOTOR_2]   * IQ_LSB);
        p_cmd_m2->kpCoeff               = (float32_t)(p_msg->kpCoeff[MOTOR_2]   * KP_LSB            * FM_RAD2ROUND);
        p_cmd_m2->kdCoeff               = (float32_t)(p_msg->kdCoeff[MOTOR_2]   * KD_LSB            * FM_RADPS2KRPM);
        isat                            = (float32_t)(LSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
        p_cmd_m2->iSat                  = (isat < MOTOR2_CURRENT_CMD_SAT_MAX)   ? (isat)            : (MOTOR2_CURRENT_CMD_SAT_MAX);
        p_cmd_m2->iSat                  = (p_cmd_m2->iSat > 0.0f)               ? (p_cmd_m2->iSat)  : (MOTOR2_CURRENT_CMD_SAT_MAX);
        p_cmd_m2->timeoutRef            = (uint16_t)p_msg->mode.bit.TIMEOUT;
        p_cmd_m2->cptTimeout            = 0U;
        p_cmd_m2->index                 = p_msg->index;
        p_cmd_reg->encOffsetEnable      = (bool_t)p_msg->mode.bit.EI2OC;
        p_cmd_reg->rollOverEnable       = (bool_t)p_msg->mode.bit.EPRE;
        p_cmd_reg->motorEnable          = (bool_t)p_msg->mode.bit.EM2;
        p_cmd_reg->systemEnable         = (bool_t)p_msg->mode.bit.ES;

        return(true);
    }

    return(false);
}

/**
 * @brief           Slave to Master message creation/update using motor_foc data structure.
 * @param[in]       p_motor_m1  Pointer on data structure used for MOTOR_1 control.
 * @param[in]       p_motor_m2  Pointer on data structure used for MOTOR_1 control.
 * @param[inout]    p_msg       Pointer on the message to transmit to the Master board
 * @return          Is message fully created/updated using motor_1 \& motor_2 data structure.
 */
void COM_msgCreate(motor_t* p_motor_m1, motor_t* p_motor_m2, slv2mst_msg_t* p_msg)
{
    foc_t* p_foc                        = p_motor_m1->p_motorFOC;
    motor_state_e state                 = p_motor_m1->motor_state;
    error_reg_u error[2]                = {p_motor_m1->motor_error, p_motor_m2->motor_error};
    encoder_t* p_enc                    = &p_foc->motor_enc;
    cmd_reg_t* p_cmd_reg                = &p_foc->motor_cmd.enableReg.bit;
    float32_t pos_offset                = p_enc->thetaAbsolute - ((p_cmd_reg->encOffsetEnable) ? (p_enc->thetaIndex) : (0.0f));
    int32_t pos                         = (int32_t)(pos_offset                  / POSITION_LSB      * FM_RAD2ROUND);
    bool_t align_done                   = (state == MOTOR_STATE_READY) || (state == MOTOR_STATE_STOP);
    // Update status field - MOTOR_1
    p_msg->status.bit.M1E               = p_cmd_reg->motorEnable;   // MOTOR_1 enabled
    p_msg->status.bit.M1R               = align_done;               // MOTOR_1 ready
    p_msg->status.bit.IDX1T             = p_enc->indexToggle;       // MOTOR_1 index toggle
    p_msg->status.bit.IDX1D             = p_enc->indexDetect;       // MOTOR_1 index detected
    p_msg->position[MOTOR_1].u16_msb    = MSB_32(pos);
    p_msg->position[MOTOR_1].u16_lsb    = LSB_32(pos);
    p_msg->velocity[MOTOR_1]            = (int16_t)(p_enc->speed.speedMech      / VELOCITY_LSB      * FM_RADPS2KRPM);
    p_msg->current[MOTOR_1]             = (int16_t)(p_foc->iq                   / IQ_LSB);
    p_msg->coilRes[MOTOR_1]             = (uint16_t)(p_foc->resEst              / RESISTANCE_LSB);
    p_msg->adcSamples[MOTOR_1]          = (uint16_t)(p_foc->motor_acq.vExt      / VOLTAGE_LSB);

    p_msg->status.bit.SE                = p_cmd_reg->systemEnable;  // System enabled
    p_msg->status.bit.STATUS_RSV1       = 0;
    p_msg->lastCmdIndex                 = p_foc->motor_cmd.index;
    p_msg->timeStamp                    = (uint16_t)CPUTimer_getTimerCount(CPU_TIMER_0_BASE);

    p_foc                               = p_motor_m2->p_motorFOC;
    state                               = p_motor_m2->motor_state;
    p_enc                               = &p_foc->motor_enc;
    p_cmd_reg                           = &p_foc->motor_cmd.enableReg.bit;
    pos_offset                          = p_enc->thetaAbsolute - ((p_cmd_reg->encOffsetEnable) ? (p_enc->thetaIndex) : (0.0f));
    pos                                 = (int32_t)(pos_offset                  / POSITION_LSB      * FM_RAD2ROUND);
    align_done                          = (state == MOTOR_STATE_READY) || (state == MOTOR_STATE_STOP);
    // Update status field - MOTOR_2
    p_msg->status.bit.M2E               = p_cmd_reg->motorEnable;   // MOTOR_2 enabled
    p_msg->status.bit.M2R               = align_done;               // MOTOR_2 ready
    p_msg->status.bit.IDX2T             = p_enc->indexToggle;       // MOTOR_2 index toggle
    p_msg->status.bit.IDX2D             = p_enc->indexDetect;       // MOTOR_2 index detected
    p_msg->position[MOTOR_2].u16_msb    = MSB_32(pos);
    p_msg->position[MOTOR_2].u16_lsb    = LSB_32(pos);
    p_msg->velocity[MOTOR_2]            = (int16_t)(p_enc->speed.speedMech      / VELOCITY_LSB      * FM_RADPS2KRPM);
    p_msg->current[MOTOR_2]             = (int16_t)(p_foc->iq                   / IQ_LSB);
    p_msg->coilRes[MOTOR_2]             = (uint16_t)(p_foc->resEst              / RESISTANCE_LSB);
    p_msg->adcSamples[MOTOR_2]          = (uint16_t)(p_foc->motor_acq.vExt      / VOLTAGE_LSB);

    if((error[MOTOR_1].all == MOTOR_ERROR_NO_ERROR) && (error[MOTOR_2].all == MOTOR_ERROR_NO_ERROR))
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_NO_ERROR;
    else if((error[MOTOR_1].bit.pos_rollover) || (error[MOTOR_2].bit.pos_rollover))
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_POS_ROLLOVER;
    else if((error[MOTOR_1].bit.drv_fault) || (error[MOTOR_2].bit.drv_fault))
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_DRV_NFAULT;
    else if(error[MOTOR_1].bit.enc_mismatch)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_ENCODER1;
    else if(error[MOTOR_2].bit.enc_mismatch)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_ENCODER2;
    else if((error[MOTOR_1].bit.com_timeout) || (error[MOTOR_2].bit.com_timeout))
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_SPI_RECV_TIMEOUT;
    // Compute CRC
    uint32_t crc = COM_crc32((uint16_t *)p_msg, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
    p_msg->crc.u16_msb                  = LSB_32(crc);              // required for Solo
    p_msg->crc.u16_lsb                  = MSB_32(crc);

    return;
}

/**
 * @brief           Compute cyclic redundancy check (CRC) and add result to the Slave to Master message .
 * @param[inout]    p_msg   Pointer on the message to transmit to the Master board
 */
//void com_MsgCrc32(slv2mst_msg_t* p_msg)
//{
//    // Compute CRC
//    uint32_t crc = com_crc32((uint16_t *)p_msg, DMA_COM_SPI_TX_RX_16BIT_PAYLOAD_LENGTH);
//    p_msg->crc.u16_msb  = LSB_32(crc);// required for Solo
//    p_msg->crc.u16_lsb  = MSB_32(crc);
//
//    return;
//}

/**
 * @brief           Reset the command structure for a motor
 * @param[inout]    p_cmd   Pointer on the command structure (actions to be realized by motor_foc)
 */
void COM_resetCmdStruct(cmd_t* p_cmd)
{
    p_cmd->posRef               = 0.0f;
    p_cmd->velRef               = 0.0f;
    p_cmd->iqRef                = 0.0f;
    p_cmd->kpCoeff              = 0.0f;
    p_cmd->kdCoeff              = 0.0f;
    p_cmd->iSat                 = 0.0f;
    p_cmd->timeoutRef           = 0U;
    p_cmd->cptTimeout           = 0U;
    p_cmd->index                = 0U;
    p_cmd->enableReg.all        = 0U;

    return;
}

/**
 * @brief           Reset the command associated to motor_id
 * @param[in]       p_motor     Pointer on the motor structure to get motor ID.
 * @param[out]      p_cmd       Pointer on the command structure (actions to be realized by motor_foc)
 */
//void COM_resetCmdMotorStruct(motor_t* p_motor, cmd_uomodri_t* p_cmd)
//{
//    if(p_motor->motor_id == MOTOR_1)
//    {
//        p_cmd->mode.bit.EM1     = false;
//        p_cmd->posRef[MOTOR_1]  = 0.0f;
//        p_cmd->velRef[MOTOR_1]  = 0.0f;
//        p_cmd->iqRef[MOTOR_1]   = 0.0f;
//        p_cmd->kpCoeff[MOTOR_1] = 0.0f;
//        p_cmd->kdCoeff[MOTOR_1] = 0.0f;
//        p_cmd->iSat[MOTOR_1]    = MOTOR1_CURRENT_REGULATOR_SAT_MAX;
//    }
//    else
//    {
//        p_cmd->mode.bit.EM2     = false;
//        p_cmd->posRef[MOTOR_2]  = 0.0f;
//        p_cmd->velRef[MOTOR_2]  = 0.0f;
//        p_cmd->iqRef[MOTOR_2]   = 0.0f;
//        p_cmd->kpCoeff[MOTOR_2] = 0.0f;
//        p_cmd->kdCoeff[MOTOR_2] = 0.0f;
//        p_cmd->iSat[MOTOR_2]    = MOTOR2_CURRENT_REGULATOR_SAT_MAX;
//    }
//
//    return;
//}

/**
 * @brief       Software cyclic redundancy check (CRC).
 * @param[in]   p_buf   Pointer on the array to be CRCed.
 * @param[in]   len     Length of the array
 * @return
 */
uint32_t COM_crc32 (uint16_t* p_buf, uint16_t len)
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

//void com_vcrc_ini(uint16_t* p_buf, uint16_t len)
//{
//
//    vcrc.seedValue   = 0xFFFFFFFF;//CRC_INIT_CRC32;
//    vcrc.nMsgBytes   = len;
//    vcrc.nMsgBits    = len * 8;
//    vcrc.parity      = CRC_parity_odd;   //!< the location, in a word, of the first byte of the CRC calculation
//    vcrc.reflected   = 1;
//    vcrc.crcResult   = 0;
//    vcrc.pMsgBuffer  = (uint16_t *)p_buf;            //!< Pointer to the message buffer
//    vcrc.pCrcTable   = (uint16_t *)&crc32_table[0];             //!< Pointer to the CRC lookup table
//    vcrc.polynomial  = POLYNOMIAL_32_P1; //!<user polynomial
//    vcrc.polySize    = 0x1F;//SIZE_32_BITS,     //!<polynomial size
//    vcrc.dataSize    = 0x07;//SIZE_8_BITS,
//    vcrc.init        = (void (*)(void *))CRC_init32Bit;          //!< Function pointer to CRC initialization routine
//    vcrc.run         = (void (*)(void *))CRC_run32BitPoly1Reflected;            //!< Function pointer to CRC computation routine
//
//    // Step 3: Initialize the handle
//    CRC_Handle handleCRC = &vcrc;
//
//    // Step 4: Run the 32-bit table look-up CRC routine and save the first result
//    vcrc.init(handleCRC);
////    vcrc.run(handleCRC);
////    return (crc.crcResult);
//    return;
//}
//
//uint32_t com_vcrc (uint16_t* p_buf, uint16_t len)
//{
//    vcrc.seedValue   = 0xFFFFFFFF;//CRC_INIT_CRC32;
//    vcrc.nMsgBytes   = len;
////    vcrc.nMsgBits    = len * 8;
//    vcrc.parity      = CRC_parity_even;   //!< the location, in a word, of the first byte of the CRC calculation
////    vcrc.reflected   = 0;
//    vcrc.crcResult   = 0;
//    vcrc.pMsgBuffer  = (uint16_t *)p_buf;            //!< Pointer to the message buffer
////    vcrc.pCrcTable   = NULL;//(uint16_t *)&crc32_table[0];             //!< Pointer to the CRC lookup table
////    vcrc.polynomial  = POLYNOMIAL_32_P1; //!<user polynomial
//    vcrc.polySize    = 0x1F;//SIZE_32_BITS,     //!<polynomial size
////    vcrc.dataSize    = 0x07;//SIZE_8_BITS,
//    vcrc.init        = (void (*)(void *))CRC_init32Bit;          //!< Function pointer to CRC initialization routine
//    vcrc.run         = (void (*)(void *))CRC_run32BitPoly1;            //!< Function pointer to CRC computation routine
//
//    // Step 3: Initialize the handle
//    CRC_Handle handleCRC = &vcrc;
//
//    // Step 4: Run the 32-bit table look-up CRC routine and save the first result
//    vcrc.init(handleCRC);
//    vcrc.run(handleCRC);
//    return (vcrc.crcResult);
//}

//void com_bgcrc(void)
//{
//    void bgcrcInit()
//    {
//        // Unlock the BGCRC configuration in case it is locked CFG-1
//        BGCRC_unlockRegister(BGCRC_CPU_BASE, BGCRC_REG_CTRL1 | BGCRC_REG_WD_CFG | BGCRC_REG_INTEN | BGCRC_REG_SEED);
//        // Unlock the BGCRC configuration in case it is locked CFG-2
//        BGCRC_unlockRegister(BGCRC_CPU_BASE, BGCRC_REG_EN | BGCRC_REG_CTRL2 | BGCRC_REG_START_ADDR | BGCRC_REG_GOLDEN | BGCRC_REG_WD_MIN | BGCRC_REG_WD_MAX);
//        // Enable NMI and set the emulation mode as soft.
//        // The CRC module and the watchdog stops immediately on debug suspend
//        BGCRC_setConfig(BGCRC_CPU_BASE, BGCRC_NMI_ENABLE, BGCRC_EMUCTRL_SOFT);
//        // Configure the CRC Watchdog
//        // Watchdog window - Min : DATA_SIZE, Max : DATA_SIZE + 1
//        BGCRC_enableWatchdog(BGCRC_CPU_BASE);
//        BGCRC_setWatchdogWindow(BGCRC_CPU_BASE, DATA_SIZE, DATA_SIZE + 1);
//        // Configure the region
//        BGCRC_setRegion(BGCRC_CPU_BASE, (uint32_t)data, BGCRC_SIZE_KBYTES(1), BGCRC_CRC_MODE);
//        // Set the seed value same as used in SW computation
//        BGCRC_setSeedValue(BGCRC_CPU_BASE, seed);
//        // Initialize the golden value with previously computed CRC using software
//        BGCRC_setGoldenCRCValue(BGCRC_CPU_BASE, crc32);
//        // Enable interrupts
//        // No need to enable the NMIs as all the NMIs are enabled by default
//        BGCRC_enableInterrupt(BGCRC_CPU_BASE, BGCRC_TEST_DONE | BGCRC_CRC_FAIL | BGCRC_UNCORR_ERR | BGCRC_CORR_ERR | BGCRC_WD_UNDERFLOW | BGCRC_WD_OVERFLOW);
//        // Lock Register configuration for CFG1
//        BGCRC_lockRegister(BGCRC_CPU_BASE, BGCRC_REG_CTRL1 | BGCRC_REG_WD_CFG | BGCRC_REG_INTEN | BGCRC_REG_SEED);
//        // Commit Register Lock for CFG1
//        BGCRC_commitRegisterLock(BGCRC_CPU_BASE, BGCRC_REG_CTRL1 | BGCRC_REG_WD_CFG | BGCRC_REG_INTEN | BGCRC_REG_SEED);
//        BGCRC_start(BGCRC_CPU_BASE);
//        // Lock Register configuration for CFG2
//        BGCRC_lockRegister(BGCRC_CPU_BASE, BGCRC_REG_EN | BGCRC_REG_CTRL2 | BGCRC_REG_START_ADDR | BGCRC_REG_GOLDEN | BGCRC_REG_WD_MIN | BGCRC_REG_WD_MAX);
//    }
//}


