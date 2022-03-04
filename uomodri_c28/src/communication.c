/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "communication.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief           Master to slave message extraction to be performed by motor_foc
 * @param[in]       p_msg   Pointer on the message received from Master board
 * @param[out]      p_cmd   Pointer on the command structure (actions to be realized by motor_foc)
 * @return          Is message received from Master board valid or not ?
 */
bool_t COM_msgExtract(mst2slv_msg_t* p_msg, cmd_uomodri_t* p_cmd)
{
//    uint32_t crc = com_crc32((uint16_t *) p_msg, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);

//    CRC_Obj vcrc;
//    uint32_t crc_res;
//    vcrc.seedValue  = 0xFFFFFFFF;//CRC_INIT_CRC32;
//    vcrc.nMsgBytes  = COM_MSG_TX_RX_8BIT_PAYLOAD_LENGTH;
//    vcrc.nMsgBits   = 8 * vcrc.nMsgBytes;
//    vcrc.parity     = CRC_parity_even;   //!< the location, in a word, of the first byte of the CRC calculation
//    vcrc.crcResult  = 0;
//    vcrc.pMsgBuffer = (uint16_t *)p_msg;            //!< Pointer to the message buffer
//    vcrc.pCrcTable  = NULL;//(uint16_t *)&crc32_table[0];
//    vcrc.polySize   = 0;//SIZE_32_BITS,     //!<polynomial size
//    vcrc.dataSize   = 0x07;
//    vcrc.reflected  = 0;
//    vcrc.init       = (void (*)(void *))CRC_init32Bit;          //!< Function pointer to CRC initialization routine
////    vcrc.run        = (void (*)(void *))CRC_run32BitTableLookupC;            //!< Function pointer to CRC computation routine
//    vcrc.run        = (void (*)(void *))CRC_run32BitPoly1;
//    // Step 3: Initialize the handle
//    CRC_Handle handleCRC = &vcrc;
//    // Step 4: Run the 32-bit table look-up CRC routine and save the first result
//    vcrc.init(handleCRC);
//    vcrc.run(handleCRC);
//    crc_res         =  CRC_bitReflect(vcrc.crcResult, 32);

//    vcrc.init(handleCRC);
//    vcrc.run(handleCRC);
//    uint32_t crc3   = vcrc.crcResult;//com_vcrc((uint16_t *) p_msg, COM_MSG_TX_RX_8BIT_PAYLOAD_LENGTH);
    uint32_t crc    = COM_crc32((uint16_t *) p_msg, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
    if((MSB_32(crc) == p_msg->crc.u16_msb) && (LSB_32(crc) == p_msg->crc.u16_lsb))
    {
        U32toU16_reg_t pos_m1_u16   = p_msg->position[MOTOR_1];
        U32toU16_reg_t pos_m2_u16   = p_msg->position[MOTOR_2];
        int32_t pos_m1              = (int32_t)FUS_16_TO_32(pos_m1_u16.u16_msb, pos_m1_u16.u16_lsb);
        int32_t pos_m2              = (int32_t)FUS_16_TO_32(pos_m2_u16.u16_msb, pos_m2_u16.u16_lsb);
        p_cmd->mode.bit.TIMEOUT     = p_msg->mode.bit.TIMEOUT;
        p_cmd->mode.bit.EI2OC       = p_msg->mode.bit.EI2OC;
        p_cmd->mode.bit.EI1OC       = p_msg->mode.bit.EI1OC;
        p_cmd->mode.bit.EPRE        = false;//p_msg->mode.bit.EPRE;
        p_cmd->mode.bit.EM2         = p_msg->mode.bit.EM2;
        p_cmd->mode.bit.EM1         = p_msg->mode.bit.EM1;
        p_cmd->mode.bit.ES          = p_msg->mode.bit.ES;
        p_cmd->position[MOTOR_1]    = (float32_t)(pos_m1                    * POSITION_LSB);
        p_cmd->position[MOTOR_2]    = (float32_t)(pos_m2                    * POSITION_LSB);
        p_cmd->velocity[MOTOR_1]    = (float32_t)(p_msg->velocity[MOTOR_1]  * VELOCITY_LSB);
        p_cmd->velocity[MOTOR_2]    = (float32_t)(p_msg->velocity[MOTOR_2]  * VELOCITY_LSB);
        p_cmd->current[MOTOR_1]     = (float32_t)(p_msg->current[MOTOR_1]   * IQ_LSB);
        p_cmd->current[MOTOR_2]     = (float32_t)(p_msg->current[MOTOR_2]   * IQ_LSB);
        p_cmd->kpCoeff[MOTOR_1]     = 0.0f;//(float32_t)(p_msg->kpCoeff[MOTOR_1]   * KP_LSB);
        p_cmd->kpCoeff[MOTOR_2]     = 0.0f;//(float32_t)(p_msg->kpCoeff[MOTOR_2]   * KP_LSB);
        p_cmd->kdCoeff[MOTOR_1]     = 0.0f;//(float32_t)(p_msg->kdCoeff[MOTOR_1]   * KD_LSB);
        p_cmd->kdCoeff[MOTOR_2]     = 0.0f;//(float32_t)(p_msg->kdCoeff[MOTOR_2]   * KD_LSB);
        p_cmd->iSat[MOTOR_1]        = 2.0f;//(float32_t)(MSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
        p_cmd->iSat[MOTOR_2]        = 2.0f;//(float32_t)(LSB_16(p_msg->iSat)       * CURRENT_SAT_LSB);
        p_cmd->index                = (uint16_t) p_msg->index;
        return (true);
    }

    return(false);
}

/**
 * @brief           Slave to Master message creation/update using motor_foc data structure.
 * @param[in]       p_foc   Pointer on data structure array used for motor control.
 * @param[in]       p_cmd   Pointer on the command structure (actions to be realized by motor_foc)
 * @param[inout]    p_msg   Pointer on the message to transmit to the Master board
 * @return      Is message fully created/updated using motor_1 \& motor_2 data structure.
 */
bool_t COM_msgCreate(foc_t* p_foc, cmd_uomodri_t* p_cmd, slv2mst_msg_t* p_msg)
{
    int32_t pos                             = (int32_t)(p_foc->motor_enc.thetaAbsolute / POSITION_LSB);
    p_msg->status.bit.SE                    = p_cmd->mode.bit.ES;                                           // System enabled

    if(p_msg->status.bit.ERROR_CODE != STATUS_ERROR_NO_ERROR)
        p_msg->status.bit.ERROR_CODE    = p_msg->status.bit.ERROR_CODE;
    else if(p_foc->motor_enc.rollOverFlag)
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_POS_ROLLOVER;
    else
        p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_NO_ERROR;
    p_msg->lastCmdIndex                     = p_cmd->index;
    if(p_foc->motor_id == MOTOR_1)
    {
        // Update status field - MOTOR_1
        p_msg->status.bit.M1E               = p_cmd->mode.bit.EM1;                                          // MOTOR_1 enabled
        p_msg->status.bit.M1R               = (p_foc->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_1 ready
        p_msg->status.bit.IDX1T             = p_foc->motor_enc.indexToggle;                                 // MOTOR_1 index toggle
        p_msg->status.bit.IDX1D             = p_foc->motor_enc.indexDetected;                               // MOTOR_1 index detected
//        p_msg->status.bit.ERROR_CODE        = (p_foc->motor_enc.rollOverFlag) ? (STATUS_ERROR_POS_ROLLOVER) : (STATUS_ERROR_NO_ERROR);
        p_msg->position[MOTOR_1].u16_msb    = MSB_32(pos);
        p_msg->position[MOTOR_1].u16_lsb    = LSB_32(pos);
        p_msg->velocity[MOTOR_1]            = (int16_t)(p_foc->motor_enc.speed.speedMech / 10.0f / FM_RAD2RPM / VELOCITY_LSB);
        p_msg->current[MOTOR_1]             = (int16_t)(p_foc->iq / IQ_LSB);
        p_msg->coilRes[MOTOR_1]             = (uint16_t)(p_foc->resEst / RESISTANCE_LSB);
        p_msg->adcSamples[MOTOR_1]          = (uint16_t)(p_foc->motor_acq.vExt / VOLTAGE_LSB);
    }
    else if(p_foc->motor_id == MOTOR_2)
    {
        // Update status field - MOTOR_2
        p_msg->status.bit.M2E               = p_cmd->mode.bit.EM2;                                          // MOTOR_2 enabled
        p_msg->status.bit.M2R               = (p_foc->motor_align.state == ALIGN_DONE) ? (true) : (false);  // MOTOR_2 ready
        p_msg->status.bit.IDX2T             = p_foc->motor_enc.indexToggle;                                 // MOTOR_2 index toggle
        p_msg->status.bit.IDX2D             = p_foc->motor_enc.indexDetected;                               // MOTOR_2 index detected
//        if(p_msg->status.bit.ERROR_CODE != STATUS_ERROR_NO_ERROR)
//            p_msg->status.bit.ERROR_CODE    = p_msg->status.bit.ERROR_CODE;
//        else if(p_foc->motor_enc.rollOverFlag)
//            p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_POS_ROLLOVER;
//        else
//            p_msg->status.bit.ERROR_CODE    = STATUS_ERROR_NO_ERROR;
//        p_msg->status.bit.ERROR_CODE        = ((p_foc->motor_enc.rollOverFlag) || (p_msg->status.bit.ERROR_CODE != STATUS_ERROR_NO_ERROR))  ? (STATUS_ERROR_POS_ROLLOVER) : (STATUS_ERROR_NO_ERROR);
        p_msg->position[MOTOR_2].u16_msb    = MSB_32(pos);
        p_msg->position[MOTOR_2].u16_lsb    = LSB_32(pos);
        p_msg->velocity[MOTOR_2]            = (int16_t)(p_foc->motor_enc.speed.speedMech / 10.0f / FM_RAD2RPM / VELOCITY_LSB);
        p_msg->current[MOTOR_2]             = (int16_t)(p_foc->iq / IQ_LSB);
        p_msg->coilRes[MOTOR_2]             = (uint16_t)(p_foc->resEst / RESISTANCE_LSB);
        p_msg->adcSamples[MOTOR_2]          = (uint16_t)(p_foc->motor_acq.vExt / VOLTAGE_LSB);
        // Compute CRC
        uint32_t crc = COM_crc32((uint16_t *)p_msg, COM_MSG_TX_RX_16BIT_PAYLOAD_LENGTH);
        p_msg->crc.u16_msb                  = LSB_32(crc);// required for Solo
        p_msg->crc.u16_lsb                  = MSB_32(crc);
        return (true);
    }

    return(false);
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

void COM_resetCmdStruct(cmd_uomodri_t* p_cmd)
{
    p_cmd->mode.all             = 0U;
    p_cmd->position[MOTOR_1]    = 0.0f;
    p_cmd->position[MOTOR_2]    = 0.0f;
    p_cmd->velocity[MOTOR_1]    = 0.0f;
    p_cmd->velocity[MOTOR_2]    = 0.0f;
    p_cmd->current[MOTOR_1]     = 0.0f;
    p_cmd->current[MOTOR_2]     = 0.0f;
    p_cmd->kpCoeff[MOTOR_1]     = 0.0f;
    p_cmd->kpCoeff[MOTOR_2]     = 0.0f;
    p_cmd->kdCoeff[MOTOR_1]     = 0.0f;
    p_cmd->kdCoeff[MOTOR_2]     = 0.0f;
    p_cmd->iSat[MOTOR_1]        = 0.0f;
    p_cmd->iSat[MOTOR_2]        = 0.0f;
    p_cmd->index                = 0U;

    return;
}

void COM_resetCmdMotorStruct(cmd_uomodri_t* p_cmd, motor_id_e motor_id)
{
    (motor_id == MOTOR_1) ? (p_cmd->mode.bit.EM1 = false) : (p_cmd->mode.bit.EM2 = false);
    p_cmd->position[motor_id]   = 0.0f;
    p_cmd->velocity[motor_id]   = 0.0f;
    p_cmd->current[motor_id]    = 0.0f;
    p_cmd->kpCoeff[motor_id]    = 0.0f;
    p_cmd->kdCoeff[motor_id]    = 0.0f;
    p_cmd->iSat[motor_id]       = 1.0f;
}

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
  return crc;
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


