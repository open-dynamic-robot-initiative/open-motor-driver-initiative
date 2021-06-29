#include "encoder.h"

// Reset encoder structure
// This function is defined here because of compilation problems
inline void ENC_reset(ENCODER *encoder)
{
    encoder->mechPos[NEW] = 0.0;
    encoder->mechPos[OLD] = 0.0;
    encoder->elecPos = 0.0f;

    encoder->isIndexCompensated = false;
    encoder->indexAngle = 0.0f;
    encoder->indexedMechAngle = 0.0f;

    encoder->lastMechAngle = 0.0f;
    encoder->absoluteAngle = 0.0f;

    encoder->speedEst.alpha = 0.0f;
    encoder->speedEst.speedRef = 0.0f;
    encoder->speedEst.pos[NEW] = 0.0f;
    encoder->speedEst.pos[OLD] = 0.0f;
    encoder->speedEst.lowSpeed = 0.0f;
    encoder->speedEst.highSpeed = 0.0f;
    encoder->speedEst.mechSpeed = 0.0f;
    encoder->speedEst.elecSpeed = 0.0f;

    encoder->turnNb = 0;
    encoder->indexDetected = false;
    encoder->indexToggle = false;
    encoder->rollOverFlag = false;
}

// Reset encoder and EQEP position counter to zero
// This is used to set the rotor angle to zero during alignment routine
inline void ENC_resetQepPos(ENCODER* p_encoder)
{
    // Reset encoder position
    EQEP_setPosition(p_encoder->p_qepHandle->eqepBase, 0);
    // Reset encoder structure
    ENC_reset(p_encoder);
}

// Read rotor position
inline void encoder_GetPosition(ENCODER* p_encoder)
{
    // Compute mechanical angle normalized between 0 and 1
    p_encoder->mechPos[NEW] = (float32_t)(*p_encoder->p_qepCntPosReg) / p_encoder->mechPosNorm;
    // Compute electrical angle (each pole pair is normalized between 0 and 1)
    p_encoder->elecPos  = (*p_encoder->p_polePairs) * p_encoder->mechPos[NEW];
    p_encoder->elecPos -= (float32_t)((int32_t)p_encoder->elecPos);
    p_encoder->posNew   = true;
}

// Encoder update routine, this is not critical for the control loop to be executed
// so it is executed after the control loop.
// Reference manual par 27.6 contains all the information
// necessary to understand this routine
inline void encoder_GetSpeed(ENCODER* p_encoder)
{

    struct EQEP_REGS*   p_qepRegs   = p_encoder->p_qepRegs;
    SpeedEst            p_speed     = p_encoder->speedEst;
    float32_t           deltaPos    = 0.0;

    //-------------------------------------------------------------------//
    //                      HIGH SPEED ESTIMATION                        //
    //-------------------------------------------------------------------//
    // Check if unit timer overflow event happened for high speed calculation
    if(p_qepRegs->QFLG.bit.UTO)
    {
        p_speed.pos[NEW] = p_qepRegs->QPOSLAT;

        deltaPos = p_speed.pos[NEW] - p_speed.pos[OLD];
        deltaPos += (p_qepRegs->QFLG.bit.PCO) ? (p_encoder->mechPosNorm) : (0.0);
        deltaPos -= (p_qepRegs->QFLG.bit.PCU) ? (p_encoder->mechPosNorm) : (0.0);

        p_speed.highSpeed = deltaPos * p_speed.highSpeedNorm;
        // Save last angle
        p_speed.pos[OLD] = p_speed.pos[NEW];
        // Clear UTO flag
        p_qepRegs->QCLR.all = EQEP_INT_UNIT_TIME_OUT | EQEP_INT_OVERFLOW | EQEP_INT_UNDERFLOW;
//        // Read latched angle normalized between 0 and 1
//        p_speed.pos[NEW] = p_qepRegs->QPOSLAT / p_encoder->mechPosNorm;
//        // Compute angle difference
//        deltaPos = p_speed.pos[NEW] - p_speed.pos[OLD];
//        // Saturate it between -0.5 and 0.5
//        deltaPos += (deltaPos > 0.5) ? (-1.0) : ((deltaPos < -0.5) ? (1.0) : (0.0));
//        // Compute high speed estimation
//        p_speed.highSpeed = deltaPos * p_speed.highSpeedNorm;
//        // Save last angle
//        p_speed.pos[OLD] = p_speed.pos[NEW];
//        // Clear UTO flag
//        p_qepRegs->QCLR.all = EQEP_INT_UNIT_TIME_OUT;
    }

    //-------------------------------------------------------------------//
    //                       LOW SPEED ESTIMATION                        //
    //-------------------------------------------------------------------//
    // Check if unit position event occurred for low speed calculation
    if(p_qepRegs->QEPSTS.bit.UPEVNT)
    {
        // Check if capture value is not valid (direction change and timer overflow)
        if(!(p_qepRegs->QEPSTS.all & (EQEP_STS_CAP_OVRFLW_ERROR | EQEP_STS_CAP_DIR_ERROR)))
            // Compute low speed estimation, set sign according QDF
            p_speed.lowSpeed = ((!(p_qepRegs->QEPSTS.bit.QDF)) ? (-1.0) : (1.0)) * p_speed.lowSpeedNorm / p_qepRegs->QCPRD;
        else
            // Speed is very close to zero because direction changed or the rotor doesn't move fast enough
            p_speed.lowSpeed = 0.0;

        // Clear position event, direction change and timer overflow flags (even if direction and timer overflow are not set)
        p_qepRegs->QEPSTS.all = EQEP_STS_UNIT_POS_EVNT | EQEP_STS_CAP_OVRFLW_ERROR | EQEP_STS_CAP_DIR_ERROR;
    }

    //-------------------------------------------------------------------//
    //                     SPEED ESTIMATION MERGE                        //
    //-------------------------------------------------------------------//
    /*
     *
     *
     *                  Merge coefficient shape:
     * low speed     ________
     *                      |\
     *                        \
     *                      |  \
     *                          \
     *                      |    \________      w1 = 70.8 rad/s
     * high speed                |              w2 = 90.7 rad/s
     *                      w1   w2
     *
     *  High speed estimation is used for this decision because this estimation can only be lower
     */

    // Compute merge coefficient
    p_speed.alpha = __fmin(__fmax(0.05f * (FM_ABS(p_speed.highSpeed) - SPEED_ESTIMATION_THRESHOLD), 0.0f), 1.0f);
    // Compute speed reference // TODO investigate if low speed calculation is useful
    //p_encoder->speedEst.speedRef = (1.0f - p_encoder->speedEst.alpha) * p_encoder->speedEst.lowMechSpeedEst + p_encoder->speedEst.alpha * p_encoder->speedEst.highMechSpeedEst;
    p_speed.speedRef = p_speed.highSpeed;
    // Update speeds
#if (UOMODRI_FLT_FORM == 1)
    LPF_FILTER(p_speed.lpf, p_speed.mechSpeed, p_speed.speedRef);
#elif (UOMODRI_FLT_FORM == 2)
    LPF_FILTER(p_speed.lpf.a, p_speed.mechSpeed, p_speed.speedRef);
#endif

    p_speed.elecSpeed = *(p_encoder->p_polePairs) * p_speed.mechSpeed;

    //-------------------------------------------------------------------//
    //                        ANGLES COMPUTATION                         //
    //-------------------------------------------------------------------//
    // Update turnNb
    deltaPos = p_encoder->mechPos[NEW] - p_encoder->mechPos[OLD];
    // If angle difference is high enough, we overflowed or underflowed
    p_encoder->turnNb += (deltaPos > 0.5f) ? (-1.0f) : ((deltaPos < -0.5f) ? (1.0f) : (0.0f));
    // Saturate turnNb between -128 and 127
    p_encoder->turnNb = (p_encoder->turnNb > 127) ? (-128) : ((p_encoder->turnNb < -128) ? (127) : (p_encoder->turnNb));
    p_encoder->rollOverFlag = ((p_encoder->turnNb > 127) || (p_encoder->turnNb < -128)) ? (true) : (false);
    // Update last angle
    p_encoder->mechPos[OLD] = p_encoder->mechPos[NEW];
    // Compute indexed angle
    p_encoder->indexedMechAngle = p_encoder->mechPos[NEW] - p_encoder->indexAngle;
    p_encoder->indexedMechAngle += (p_encoder->indexedMechAngle < 0.0f) ? (1.0f) : (0.0f);
    // Compute new absolute angle
    p_encoder->absoluteAngle = (p_encoder->isIndexCompensated ? p_encoder->indexedMechAngle : p_encoder->mechPos[NEW]) + (float32_t)p_encoder->turnNb;

    //-------------------------------------------------------------------//
    //                          INDEX HANDLING                           //
    //-------------------------------------------------------------------//

    // Check if index have been detected
    p_encoder->indexDetected = p_qepRegs->QEPSTS.bit.FIMF;
    // Check if index has been detected
    if (p_qepRegs->QFLG.all & EQEP_INT_INDEX_EVNT_LATCH)
    {
        // Save index position
        p_encoder->indexAngle = p_encoder->mechPosNorm * p_qepRegs->QPOSILAT;
        // Toggle index
        p_encoder->indexToggle = !p_encoder->indexToggle;
        // Clear flag
        p_qepRegs->QCLR.all = EQEP_INT_INDEX_EVNT_LATCH;
    }
}
