#include "encoder.h"

// Reset encoder structure
// This function is defined here because of compilation problems
inline void ENC_reset(EncoderStruct *encoder)
{
    encoder->mechAngle = 0.0f;
    encoder->elecAngle = 0.0f;

    encoder->isIndexCompensated = false;
    encoder->indexAngle = 0.0f;
    encoder->indexedMechAngle = 0.0f;

    encoder->lastMechAngle = 0.0f;
    encoder->absoluteAngle = 0.0f;

    encoder->speedEst.alpha = 0.0f;
    encoder->speedEst.speedRef = 0.0f;
    encoder->speedEst.angle = 0.0f;
    encoder->speedEst.lastAngle = 0.0f;
    encoder->speedEst.lowMechSpeedEst = 0.0f;
    encoder->speedEst.highMechSpeedEst = 0.0f;
    encoder->speedEst.mechSpeed = 0.0f;
    encoder->speedEst.elecSpeed = 0.0f;

    encoder->turnNb = 0;
    encoder->indexDetected = false;
    encoder->indexToggle = false;
    encoder->rollOverFlag = false;
}

// Reset encoder and EQEP position counter to zero
// This is used to set the rotor angle to zero during alignment routine
inline void ENC_resetQepPos(EncoderStruct* p_encoder)
{
    // Reset encoder position
    EQEP_setPosition(p_encoder->p_qepHandle->eqepBase, 0);
    // Reset encoder structure
    ENC_reset(p_encoder);
}

// Read rotor position
inline void ENC_read(EncoderStruct *p_encoder)
{
    // Compute mechanical angle (between 0 and 1)
    p_encoder->mechAngle = EQEP_getPosition(p_encoder->p_qepHandle->eqepBase) / 4.0f / QEP_ENCODER_STEPS;
    // Compute electrical angle
    p_encoder->elecAngle = p_encoder->polePairs * p_encoder->mechAngle;
    p_encoder->elecAngle -= (float32_t)((int32_t)p_encoder->elecAngle);
}

// Encoder update routine, this is not critical for the control loop to be executed
// so it is executed after the control loop.
// Reference manual par 27.6 contains all the information
// necessary to understand this routine
// TODO Maybe do this on CLA
inline void ENC_update(EncoderStruct* p_encoder)
{
    float32_t dAngle;

    //-------------------------------------------------------------------//
    //                      HIGH SPEED ESTIMATION                        //
    //-------------------------------------------------------------------//
    // Check if unit timer overflow event happened for high speed calculation
    if (p_encoder->qepHandle->QFLG.bit.UTO)
    {
        // Read latched angle
        p_encoder->speedEst.angle = p_encoder->qepHandle->QPOSLAT * p_encoder->mechScaler;

        // Compute angle difference
        dAngle = p_encoder->speedEst.angle - p_encoder->speedEst.lastAngle;

        // Saturate it between -0.5 and 0.5
        if (dAngle > 0.5f)
        {
            dAngle -= 1.0f;
        } else if (dAngle < -0.5f)
        {
            dAngle += 1.0f;
        }

        // Compute high speed estimation
        p_encoder->speedEst.highMechSpeedEst = dAngle * p_encoder->speedEst.highSpeedScaler;

        // Save last angle
        p_encoder->speedEst.lastAngle = p_encoder->speedEst.angle;

        // Clear UTO flag
        p_encoder->qepHandle->QCLR.all = EQEP_INT_UNIT_TIME_OUT;
    }

    //-------------------------------------------------------------------//
    //                       LOW SPEED ESTIMATION                        //
    //-------------------------------------------------------------------//
    // Check if unit position event occurred for low speed calculation
    if (p_encoder->qepHandle->QEPSTS.bit.UPEVNT)
    {
        // Check if capture value is not valid (direction change and timer overflow)
        if (p_encoder->qepHandle->QEPSTS.all & 0xC)
        {
            // Speed is very close to zero because direction changed or the rotor doesn't move fast enough
            p_encoder->speedEst.lowMechSpeedEst = 0.0f;

            // Clear flags
            p_encoder->qepHandle->QEPSTS.all = 0xC;
        } else {
            // Check that capture value isn't 0xFFFF (error mode)
            if (p_encoder->qepHandle->QCPRD != 0xFFFF)
            {
                // Compute low speed estimation
                p_encoder->speedEst.lowMechSpeedEst = p_encoder->speedEst.lowSpeedScaler / p_encoder->qepHandle->QCPRD;

                // Check if rotation is counter clock-wise
                if (p_encoder->qepHandle->QEPSTS.bit.QDF == 0)
                {
                    p_encoder->speedEst.lowMechSpeedEst *= -1.0f;
                }
            }
        }

        // Clear position event flag
        p_encoder->qepHandle->QEPSTS.all = EQEP_STS_UNIT_POS_EVNT;
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
    p_encoder->speedEst.alpha = __fmin(__fmax(0.05f * (FM_ABS(p_encoder->speedEst.highMechSpeedEst) - SPEED_ESTIMATION_THRESHOLD), 0.0f), 1.0f);

    // Compute speed reference // TODO investigate if low speed calculation is useful
    //p_encoder->speedEst.speedRef = (1.0f - p_encoder->speedEst.alpha) * p_encoder->speedEst.lowMechSpeedEst + p_encoder->speedEst.alpha * p_encoder->speedEst.highMechSpeedEst;
    p_encoder->speedEst.speedRef = p_encoder->speedEst.highMechSpeedEst;

    // Update speeds
    LPF_FILTER(p_encoder->speedEst.lpf, p_encoder->speedEst.mechSpeed, p_encoder->speedEst.speedRef);
    p_encoder->speedEst.elecSpeed = p_encoder->polePairs * p_encoder->speedEst.mechSpeed;

    //-------------------------------------------------------------------//
    //                        ANGLES COMPUTATION                         //
    //-------------------------------------------------------------------//

    // Update turnNb
    dAngle = p_encoder->mechAngle - p_encoder->lastMechAngle;

    // If angle difference is high enough, we overflowed or underflowed
    if (dAngle > 0.5f)
    {
        p_encoder->turnNb--;
    } else if (dAngle < -0.5f)
    {
        p_encoder->turnNb++;
    }

    // Saturate turnNb between -128 and 127
    if (p_encoder->turnNb > 127)
    {
        p_encoder->turnNb = -128;

        // Raise roll-over flag
        p_encoder->rollOverFlag = true;
    }
    else if (p_encoder->turnNb < -128)
    {
        p_encoder->turnNb = 127;

        // Raise roll-over flag
        p_encoder->rollOverFlag = true;
    }

    // Update last angle
    p_encoder->lastMechAngle = p_encoder->mechAngle;

    // Compute indexed angle
    p_encoder->indexedMechAngle = p_encoder->mechAngle - p_encoder->indexAngle;
    if (p_encoder->indexedMechAngle < 0.0f)
        p_encoder->indexedMechAngle += 1.0f;

    // Compute new absolute angle
    p_encoder->absoluteAngle = (p_encoder->isIndexCompensated ? p_encoder->indexedMechAngle : p_encoder->mechAngle)
            + (float32_t)p_encoder->turnNb;

    //-------------------------------------------------------------------//
    //                          INDEX HANDLING                           //
    //-------------------------------------------------------------------//

    // Check if index have been detected
    p_encoder->indexDetected = p_encoder->qepHandle->QEPSTS.bit.FIMF;

    // Check if index has been detected
    if (p_encoder->qepHandle->QFLG.all & EQEP_INT_INDEX_EVNT_LATCH)
    {
        // Save index position
        p_encoder->indexAngle = p_encoder->mechScaler * p_encoder->qepHandle->QPOSILAT;

        // Toggle index
        p_encoder->indexToggle = !p_encoder->indexToggle;

        // Clear flag
        p_encoder->qepHandle->QCLR.all = EQEP_INT_INDEX_EVNT_LATCH;
    }
}
