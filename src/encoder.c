#include "encoder.h"

// Reset encoder structure
// This function is defined here because of compilation problems
inline void encoder_resetStruct(ENCODER *p_encoder)
{
    p_encoder->thetaMech[NEW]       = 0.0;
    p_encoder->thetaMech[OLD]       = 0.0;
    p_encoder->thetaElec            = 0.0;
    p_encoder->thetaDir             = false;
//    p_encoder->thetaNew             = false;

    p_encoder->indexCompensated     = false;
    p_encoder->indexTheta           = 0.0f;
    p_encoder->indexedThetaMech     = 0.0f;

    p_encoder->thetaAbsolute        = 0.0f;

    p_encoder->speedEst.alpha       = 0.0f;
    p_encoder->speedEst.speedRef    = 0.0f;
    p_encoder->speedEst.theta[NEW]  = 0.0f;
    p_encoder->speedEst.theta[OLD]  = 0.0f;
    p_encoder->speedEst.speedLow    = 0.0f;
    p_encoder->speedEst.speedHigh   = 0.0f;
    p_encoder->speedEst.speedMech   = 0.0f;
    p_encoder->speedEst.speedElec   = 0.0f;

    p_encoder->turnNb               = 0;
    p_encoder->indexDetected        = false;
    p_encoder->indexToggle          = false;
    p_encoder->rollOverFlag         = false;
}

// Reset encoder and EQEP position counter to zero
// This is used to set the rotor angle to zero during alignment routine
inline void encoder_setPosition(ENCODER* p_encoder, uint32_t pos)
{
    // Reset encoder position
    EQEP_setPosition(p_encoder->p_qepHandle->eqepBase, pos);
    // Reset encoder structure
    encoder_resetStruct(p_encoder);
}

// Read rotor position
inline void encoder_getPosition(ENCODER* p_encoder)
{
    p_encoder->thetaMech[OLD]   = p_encoder->thetaMech[NEW];
    // Compute mechanical angle normalized between 0 and 1
    p_encoder->thetaMech[NEW]   = (float32_t)(p_encoder->p_qepRegs->QPOSSLAT) / p_encoder->thetaMechScaler;
    //p_encoder->thetaMech    = (float32_t)(*p_encoder->p_qepCntPosReg) / p_encoder->mechScaler;
    // Get direction : 0 = CCW/reverse, 1 = CW/forward
    p_encoder->thetaDir         = p_encoder->p_qepRegs->QEPSTS.bit.QDF;
    // Compute electrical angle (each pole pair is normalized between 0 and 1)
    p_encoder->thetaElec        = __fracf32(p_encoder->polePairs * p_encoder->thetaMech[NEW]);
}

// Encoder update routine, this is not critical for the control loop to be executed
// so it is executed after the control loop.
// Reference manual par 27.6 contains all the information
// necessary to understand this routine
inline void encoder_getSpeed(ENCODER* p_encoder)
{

    struct EQEP_REGS*   p_qepRegs   = p_encoder->p_qepRegs;
    SpeedEst            p_speed     = p_encoder->speedEst;
    float32_t           deltaTheta  = 0.0;

    //-------------------------------------------------------------------//
    //                      HIGH SPEED ESTIMATION                        //
    //-------------------------------------------------------------------//
    // Check if unit timer overflow event happened for high speed calculation
    if(p_qepRegs->QFLG.bit.UTO)
    {
        p_speed.thetaDir    = p_qepRegs->QEPSTS.bit.QDF;
        p_speed.theta[NEW]  = p_qepRegs->QPOSLAT / p_encoder->thetaMechScaler;
#if 0
        if(!p_speed.thetaDir)
            deltaTheta      = p_speed.theta[NEW] - p_speed.theta[OLD] - ((p_speed.theta[NEW] > p_speed.theta[OLD]) ? (1.0) : (0.0));
        else
            deltaTheta      = p_speed.theta[NEW] - p_speed.theta[OLD] + ((p_speed.theta[NEW] < p_speed.theta[OLD]) ? (1.0) : (0.0));
#endif
//        deltaTheta          = p_speed.theta[NEW] - p_speed.theta[OLD];
//        deltaTheta         += (p_qepRegs->QFLG.bit.PCO) ? (1.0) : (0.0);
//        deltaTheta         -= (p_qepRegs->QFLG.bit.PCU) ? (1.0) : (0.0);
        deltaTheta          = p_speed.theta[NEW] - p_speed.theta[OLD] + (1.0 * (p_qepRegs->QFLG.bit.PCO)) - (1.0 * (p_qepRegs->QFLG.bit.PCU));
        // Saturate it between -0.5 and 0.5
        deltaTheta          = __fsat(deltaTheta, 1.0, -1.0) * 0.5f;//FMINMAX(deltaTheta, 1.0, -1.0) * 0.5f;
//        deltaTheta          = (deltaTheta > 1.0) ? (1.0) : ((deltaTheta < -1.0) ? (-1.0) : (deltaTheta));
        // Compute high speed estimation
        p_speed.speedHigh   = deltaTheta * p_speed.speedHighScaler;
        // Save last angle
        p_speed.theta[OLD]  = p_speed.theta[NEW];
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
            p_speed.speedLow    = ((!(p_qepRegs->QEPSTS.bit.QDF)) ? (-1.0) : (1.0)) * p_speed.speedLowScaler / p_qepRegs->QCPRDLAT;
        else
            // Speed is very close to zero because direction changed or the rotor doesn't move fast enough
            p_speed.speedLow    = 0.0;
        // Clear position event, direction change and timer overflow flags (even if direction and timer overflow are not set)
        p_qepRegs->QEPSTS.all   = EQEP_STS_UNIT_POS_EVNT | EQEP_STS_CAP_OVRFLW_ERROR | EQEP_STS_CAP_DIR_ERROR;
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

    // Compute merge coefficient // TODO as no merge is computed alpha term is not necessary
//    p_speed.alpha = FMINMAX((0.05f * (FM_ABS(p_speed.highSpeed) - SPEED_ESTIMATION_THRESHOLD)), 1.0f, 0.0f);//    p_speed.alpha = __fmin(__fmax(0.05f * (FM_ABS(p_speed.highSpeed) - SPEED_ESTIMATION_THRESHOLD), 0.0f), 1.0f);
    // Compute speed reference // TODO merge high & low speed - CSDT
    // see : http://www.diegm.uniud.it/petrella/Azionamenti%20Elettrici%20II/Tesine/Petrella%20et%20al.%20-%20Speed%20Measurement%20Algorithms%20for%20Low-Resolution%20Incremental%20Encoder%20Equipped%20Drives_a%20Comparative%20Analysis.pdf
    p_speed.speedRef = p_speed.speedHigh;//    p_encoder->speedEst.speedRef = (1.0f - p_encoder->speedEst.alpha) * p_encoder->speedEst.lowMechSpeedEst + p_encoder->speedEst.alpha * p_encoder->speedEst.highMechSpeedEst;
    // Update speeds
#if (UOMODRI_SPEED_FLT == 1)
    LPF_FILTER_1(p_speed.lpf, p_speed.speedMech, p_speed.speedRef);
#elif (UOMODRI_SPEED_FLT == 2)
    LPF_FILTER_2(p_speed.lpf, p_speed.speedMech, p_speed.speedRef);
#endif
    p_speed.speedElec = p_encoder->polePairs * p_speed.speedMech;

    //-------------------------------------------------------------------//
    //                        ANGLES COMPUTATION                         //
    //-------------------------------------------------------------------//
    // Update turnNb
    deltaTheta                  = p_encoder->thetaMech[NEW] - p_encoder->thetaMech[OLD];
    // If angle difference is high enough, we overflowed or underflowed
    p_encoder->turnNb          += (deltaTheta > 0.5f) ? (-1.0f) : ((deltaTheta < -0.5f) ? (1.0f) : (0.0f));
    // Saturate turnNb between -128 and 127
    p_encoder->turnNb           = (p_encoder->turnNb > 127) ? (-128) : ((p_encoder->turnNb < -128) ? (127) : (p_encoder->turnNb));
    p_encoder->rollOverFlag     = ((p_encoder->turnNb == 127) || (p_encoder->turnNb == -128)) ? (true) : (false);
    // Compute indexed angle
    p_encoder->indexedThetaMech = p_encoder->thetaMech[NEW] - p_encoder->indexTheta;
    p_encoder->indexedThetaMech+= (p_encoder->indexedThetaMech < 0.0f) ? (1.0f) : (0.0f);
    // Compute new absolute angle
    p_encoder->thetaAbsolute    = ((p_encoder->indexCompensated) ? (p_encoder->indexedThetaMech) : (p_encoder->thetaMech[NEW])) + (float32_t)p_encoder->turnNb;

    //-------------------------------------------------------------------//
    //                          INDEX HANDLING                           //
    //-------------------------------------------------------------------//

    // Check if index have been detected
    p_encoder->indexDetected    = p_qepRegs->QEPSTS.bit.FIMF;
    // Check if index has been detected
    if (p_qepRegs->QFLG.all & EQEP_INT_INDEX_EVNT_LATCH)
    {
        // Save index position
        p_encoder->indexTheta   = p_encoder->thetaMechScaler * p_qepRegs->QPOSILAT;
        // Toggle index
        p_encoder->indexToggle  = !p_encoder->indexToggle;
        // Clear flag
        p_qepRegs->QCLR.all     = EQEP_INT_INDEX_EVNT_LATCH;
    }
}

