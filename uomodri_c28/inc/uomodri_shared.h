#ifndef __UOMODRI_SHARED_H__
#define __UOMODRI_SHARED_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdint.h>
#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
#include <CLAmath.h>
#else
#include <hw_types.h>
#include <math.h>
#endif
#include <f2838x_device.h>

#include "foc.h"
#include "motor.h"
#include "encoder.h"
#include "uomodri_user_defines.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#if (CLA_CORE_ENABLE)
#define MMAX3(x, y, z)                          (__mmaxf32(__mmaxf32((x), (y)), (z)))
#define MMIN3(x, y, z)                          (__mminf32(__mminf32((x), (y)), (z)))
#define MSATF32(x, y, z)                        (__mmaxf32(__mminf32((x), (y)), (z)))
#endif

/***********************************************************************
 * STRUCTURES
 ***********************************************************************/

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
#if (CLA_CORE_ENABLE)
extern foc_t    motor_foc_m1;
extern foc_t    motor_foc_m2;
extern motor_t  motor_m1;
extern motor_t  motor_m2;
#else
//extern static foc_t     motor_foc[];
//extern static motor_t   motor[];
#endif

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
#if (CLA_CORE_ENABLE)
extern float32_t __mminf32(float32_t, float32_t);
extern float32_t __mmaxf32(float32_t, float32_t);
extern float32_t __mfracf32(float32_t);
extern float32_t CLAcos(float32_t);
extern float32_t CLAsin(float32_t);
extern void CLAsincos(float32_t, float32_t*, float32_t*);
#endif

#if (CLA_CORE_ENABLE) || (CLA1_TEST_CORE_ENABLE)
__attribute__((interrupt)) void Cla1Task1();
__attribute__((interrupt)) void Cla1Task2();
__attribute__((interrupt)) void Cla1Task3();
__attribute__((interrupt)) void Cla1Task4();
__attribute__((interrupt)) void Cla1Task5();
__attribute__((interrupt)) void Cla1Task6();
__attribute__((interrupt)) void Cla1Task7();
__attribute__((interrupt)) void Cla1Task8();
#endif

#if (CLA_CORE_ENABLE)
inline float32_t __msatf32(float32_t, float32_t, float32_t);
#endif
inline void ENC_resetStruct(encoder_t*);
inline void ENC_resetPeriph(encoder_t*);
inline void ENC_getPosition(encoder_t*);
inline void ENC_getSpeed(encoder_t*);
inline void ENC_getTheta(encoder_t*);
inline void FOC_resetStructPI(pi_t*);
inline void FOC_resetStructPD(pd_t*);
inline void FOC_resetStructCmd(cmd_t*);
inline void FOC_resetStructMeasures(acq_t*);
inline void FOC_resetStruct(foc_t*);
inline void FOC_getMeasures(acq_t*);
inline float32_t FOC_runPI(pi_t*);
inline float32_t FOC_runPD(pd_t*);
inline void FOC_svpwm1(foc_t*);
inline void FOC_runControl(foc_t*);
inline void MOT_runCommand(motor_t*, float32_t, float32_t, float32_t);
inline void MOT_stopCommand(motor_t*);
inline bool_t MOT_runControl(motor_t*);

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
#if (CLA_CORE_ENABLE)
/**
 * @brief           Saturation function.
 *                  out = a, if sat_min < a < sat_max,
 *                  out = sat_max, if a > sat_max,
 *                  out = sat_min, if a < sat_min
 * @param a         Value to be tested.
 * @param sat_max   Max level for saturation.
 * @param sat_min   Min level for saturation.
 * @return
 */
inline float32_t __msatf32(float32_t a, float32_t sat_max, float32_t sat_min)
{
    return(__mmaxf32(__mminf32(a, sat_max), sat_min));
}
#endif

/**
 * @brief           Reset the ENCODER structure
 * @param[inout]    *p_enc   Pointer on the ENCODER structure to be reset.
 */
inline void ENC_resetStruct(encoder_t* p_enc)
{
    speed_t* p_speed            = &p_enc->speed;

    p_enc->thetaElec            = 0.0f;
    p_enc->thetaMech[NEW]       = 0.0f;
    p_enc->thetaMech[OLD]       = 0.0f;
    p_enc->thetaIndexRelative   = 0.0f;
    p_enc->thetaIndexAbsolute   = 0.0f;
    p_enc->thetaAbsolute        = 0.0f;
    p_enc->turnNb               = 0U;
    p_enc->flags.all            = 0U;

    p_speed->alpha              = 0.0f;
    p_speed->theta[NEW]         = 0.0f;
    p_speed->theta[OLD]         = 0.0f;
    p_speed->speedHigh          = 0.0f;
    p_speed->speedLow           = 0.0f;
    p_speed->speedMech[0]       = 0.0f;
    p_speed->speedMech[1]       = 0.0f;
    p_speed->speedElec          = 0.0f;

    return;
}

/**
 * @brief           Reset the quadrature encoder peripheral (eQEP)
 * @param[inout]    *p_enc   Pointer on the ENCODER structure to reset.
 */
inline void ENC_resetPeriph(encoder_t* p_enc)
{
    uint32_t eqepBase       = p_enc->eqepBaseAddr;
    // Reset encoder position
    EQEP_setPosition(eqepBase, 0);
    /* Reset the time base for edge capture unit */
    HWREGH(eqepBase + EQEP_O_QCPRDLAT) = 0;
    /* Reset the Period count */
    HWREGH(eqepBase + EQEP_O_QCTMRLAT) = 0;
    // Reset interrupts status flags
    EQEP_clearInterruptStatus(eqepBase, (EQEP_INT_GLOBAL | EQEP_INT_POS_CNT_ERROR | EQEP_INT_PHASE_ERROR |
            EQEP_INT_DIR_CHANGE | EQEP_INT_WATCHDOG | EQEP_INT_UNDERFLOW | EQEP_INT_OVERFLOW |
            EQEP_INT_POS_COMP_READY | EQEP_INT_POS_COMP_MATCH | EQEP_INT_STROBE_EVNT_LATCH |
            EQEP_INT_INDEX_EVNT_LATCH | EQEP_INT_UNIT_TIME_OUT | EQEP_INT_QMA_ERROR));
    // Reset general status flags
    EQEP_clearStatus(eqepBase, (EQEP_STS_UNIT_POS_EVNT | EQEP_STS_CAP_OVRFLW_ERROR |
            EQEP_STS_CAP_DIR_ERROR | EQEP_STS_1ST_IDX_FLAG));

    return;
}

/**
 * @brief           Read the position latched on ADCSOC event (generated by PWM SOC), direction \& electric position
 * @param[inout]    *p_enc  Pointer on the encoder structure.
 */
inline void ENC_getPosition(encoder_t* p_enc)
{
    uint32_t eqepBase       = p_enc->eqepBaseAddr;
    p_enc->thetaMech[OLD]   = p_enc->thetaMech[NEW];
    // Compute mechanical angle normalized between 0 and 1
    p_enc->thetaMech[NEW]   = (float32_t)EQEP_getStrobePositionLatch(eqepBase) * p_enc->thetaMechScaler;
    // Get direction : -1 = CCW/reverse, 1 = CW/forward
    p_enc->thetaDir         = EQEP_getDirection(eqepBase);
    // Compute electrical angle (each pole pair is normalized between 0 and 1)
    float32_t thetaElec     = (float32_t)p_enc->polePairs * p_enc->thetaMech[NEW];
    // Compute electrical & mechanical angle (in rad)
#if (defined CPU1) && (CLA_CORE_ENABLE)
    p_enc->thetaElec        = FM_ROUND2RAD * __mfracf32(thetaElec);
    p_enc->thetaMech[NEW]   = FM_ROUND2RAD * p_enc->thetaMech[NEW];
#elif (defined CPU1)
    p_enc->thetaElec        = __mpy2pif32(__fracf32(thetaElec));
    p_enc->thetaMech[NEW]   = __mpy2pif32(p_enc->thetaMech[NEW]);
#else
    p_enc->thetaElec        = __mpy2pif32(p_enc->thetaElec - (float32_t)((int32_t)thetaElec));
    p_enc->thetaMech[NEW]   = __mpy2pif32(p_enc->thetaMech[NEW]);
#endif

    return;
}

/**
 * @brief           Encoder speed estimation
 * @param[inout]    *p_enc   Pointer on the ENCODER structure to use for speed estimate.
 */
inline void ENC_getSpeed(encoder_t* p_enc)
{
    uint32_t eqepBase           = p_enc->eqepBaseAddr;
    speed_t* p_speed            = &p_enc->speed;
//    bool_t newSpeedHigh         = false;
//    bool_t newSpeedLow          = false;
    //-------------------------------------------------------------------
    //     HIGH SPEED ESTIMATION
    //-------------------------------------------------------------------
    if(EQEP_getInterruptStatus(p_enc->eqepBaseAddr) & EQEP_INT_UNIT_TIME_OUT)
    {
        p_speed->theta[OLD]     = p_speed->theta[NEW];
#if (CLA_CORE_ENABLE)
        p_speed->theta[NEW]     = FM_ROUND2RAD * (float32_t)EQEP_getPositionLatch(eqepBase) * p_enc->thetaMechScaler;
#else
        p_speed->theta[NEW]     = __mpy2pif32((float32_t)EQEP_getPositionLatch(eqepBase) * p_enc->thetaMechScaler);
#endif
        EQEP_clearInterruptStatus(eqepBase, (EQEP_INT_UNIT_TIME_OUT));// | EQEP_INT_OVERFLOW | EQEP_INT_UNDERFLOW));
        // Compute angle difference
        float32_t deltaTheta    = p_speed->theta[NEW] - p_speed->theta[OLD];
        // Saturate it between -pi and pi
        deltaTheta              = (deltaTheta > UOMODRI_PI) ? (deltaTheta - FM_2MULTPI) : (deltaTheta);
        deltaTheta              = (deltaTheta < -UOMODRI_PI)? (deltaTheta + FM_2MULTPI) : (deltaTheta);
        // Compute high speed estimation
        p_speed->speedHigh      = deltaTheta * p_speed->speedHighScaler;
//        newSpeedHigh            = true;
    }
    //-------------------------------------------------------------------
    //     LOW SPEED ESTIMATION
    //-------------------------------------------------------------------
    // Check if unit position event occurred for low speed calculation
    if(EQEP_getStatus(eqepBase) & EQEP_STS_UNIT_POS_EVNT)//if(p_qepRegs->QEPSTS.all & EQEP_STS_UNIT_POS_EVNT)
    {
        uint16_t period         = EQEP_getCapturePeriodLatch(eqepBase);
        p_speed->speedLow       = (period != 0xFFFF) ? ((float32_t)(p_enc->thetaDir) * p_speed->speedLowScaler / (float32_t)period) : (p_speed->speedLow);
//        p_speed->speedLow       = (float32_t)(p_enc->thetaDir) * p_speed->speedLowScaler / (float32_t)EQEP_getCapturePeriodLatch(eqepBase);
        // Speed is very close to zero because direction changed or the rotor doesn't move fast enough
        p_speed->speedLow       = (EQEP_getStatus(eqepBase) & (EQEP_STS_CAP_OVRFLW_ERROR | EQEP_STS_CAP_DIR_ERROR)) ? (0.0f) : (p_speed->speedLow);
        // Clear position event, direction change and timer overflow flags (even if direction and timer overflow are not set)
        EQEP_clearStatus(eqepBase, (EQEP_STS_UNIT_POS_EVNT | EQEP_STS_CAP_OVRFLW_ERROR | EQEP_STS_CAP_DIR_ERROR));
//        newSpeedLow             = true;
    }
    //-------------------------------------------------------------------
    //     SPEED ESTIMATION MERGE
    //-------------------------------------------------------------------
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
//    speedEst.alpha = FMINMAX((0.05f * (FM_ABS(speedEst.highSpeed) - SPEED_ESTIMATION_THRESHOLD)), 1.0f, 0.0f);//    speedEst.alpha = __fmin(__fmax(0.05f * (FM_ABS(speedEst.highSpeed) - SPEED_ESTIMATION_THRESHOLD), 0.0f), 1.0f);
// Compute speed reference // TODO merge high & low speed - CSDT
// see : http://www.diegm.uniud.it/petrella/Azionamenti%20Elettrici%20II/Tesine/Petrella%20et%20al.%20-%20Speed%20Measurement%20Algorithms%20for%20Low-Resolution%20Incremental%20Encoder%20Equipped%20Drives_a%20Comparative%20Analysis.pdf
    // Compute merge coefficient
//    p_speed->alpha              = __fsat((float32_t)(0.05f * (fabs(p_speed->speedHigh) - SPEED_ESTIMATION_THRESHOLD)), 1.0f, 0.0f);
//    if(newSpeedHigh)
//    {
    float32_t speedFusion   = p_speed->speedHigh;//    p_enc->speedEst.speedRef = (1.0f - p_enc->speedEst.alpha) * p_enc->speedEst.lowMechSpeedEst + p_enc->speedEst.alpha * p_enc->speedEst.highMechSpeedEst;
    UOMODRI_SPEED_FLT(p_speed->speedFlt[0], p_speed->speedMech[0], speedFusion);
    UOMODRI_SPEED_FLT(p_speed->speedFlt[1], p_speed->speedMech[1], speedFusion);
    //    p_speed->speedMech         = p_speed->speedHigh;//speedFusion;
    p_speed->speedElec      = (float32_t)p_enc->polePairs * p_speed->speedMech[0];
//    }

    return;
}

/**
 * @brief           Encoder multi-turn \& index computation
 * @param[inout]    *p_enc   Pointer on the ENCODER structure to use for turn counting \index detection.
 */
inline void ENC_getTheta(encoder_t* p_enc)
{
    uint32_t eqepBase               = p_enc->eqepBaseAddr;
    //-------------------------------------------------------------------
    //     TURN CONTROL
    //-------------------------------------------------------------------
    float32_t deltaThetaMech        = p_enc->thetaMech[NEW] - p_enc->thetaMech[OLD];
    // Update turnNb
    p_enc->turnNb                   = (deltaThetaMech > UOMODRI_PI) ? (p_enc->turnNb - 1) : (p_enc->turnNb);
    p_enc->turnNb                   = (deltaThetaMech < -UOMODRI_PI)? (p_enc->turnNb + 1) : (p_enc->turnNb);
    // If theta difference is high or low enough, we overflowed or underflowed
    bool_t maxRollOver              = (p_enc->turnNb > MAX_ROLLOVER)? (true) : (false);
    bool_t minRollOver              = (p_enc->turnNb < MIN_ROLLOVER)? (true) : (false);
    p_enc->flags.rollOverError      = (maxRollOver || minRollOver)  ? (true) : (false);
    // Saturate turnNb between -128 and 127
    p_enc->turnNb                   = (maxRollOver) ? (MIN_ROLLOVER) : (p_enc->turnNb);
    p_enc->turnNb                   = (minRollOver) ? (MAX_ROLLOVER) : (p_enc->turnNb);
#if (CLA_CORE_ENABLE)
    float32_t turnNbRad             = FM_ROUND2RAD * (float32_t)p_enc->turnNb;
#else
    float32_t turnNbRad             = __mpy2pif32((float32_t)p_enc->turnNb);
#endif
    //-------------------------------------------------------------------
    //     INDEX HANDLING
    //-------------------------------------------------------------------
    if(EQEP_getInterruptStatus(eqepBase) & EQEP_INT_INDEX_EVNT_LATCH)
    {
        // Compute indexed mechanical angle normalized between 0 and 2 x PI
#if (CLA_CORE_ENABLE)
        float32_t thetaIndex        = FM_ROUND2RAD * (float32_t)EQEP_getIndexPositionLatch(eqepBase) * p_enc->thetaMechScaler;
#else
        float32_t thetaIndex        = __mpy2pif32((float32_t)EQEP_getIndexPositionLatch(eqepBase) * p_enc->thetaMechScaler);
#endif
        // Save first indexed mechanical angle normalized between 0 and 2 x PI
        p_enc->thetaIndexRelative   = (!p_enc->flags.indexDetect) ? (thetaIndex)            : (p_enc->thetaIndexRelative);
        // Save first absolute indexed mechanical angle
        p_enc->thetaIndexAbsolute   = (!p_enc->flags.indexDetect) ? (thetaIndex + turnNbRad): (p_enc->thetaIndexAbsolute);
        // Set index detection flag.
        p_enc->flags.indexDetect    = true;
        // Check if current index capture match the first index capture
#if (CLA_CORE_ENABLE)
        float32_t tmp               = FM_ROUND2RAD * 4.0f / p_enc->thetaMechScaler;
//        p_enc->flags.indexError     = (abs(p_enc->thetaIndexRelative - thetaIndex) < tmp) ? (false) : (true);
#else
        float32_t tmp               = __mpy2pif32(4.0f / p_enc->thetaMechScaler);
//        p_enc->flags.indexError     = (fabsf(p_enc->thetaIndexRelative - thetaIndex) < tmp) ? (false) : (true);
#endif
        p_enc->flags.indexError     = (fabsf(p_enc->thetaIndexRelative - thetaIndex) < tmp) ? (false) : (true);
        // Toggle index
        p_enc->flags.indexToggle    = !p_enc->flags.indexToggle;
        // Clear flag
        EQEP_clearInterruptStatus(eqepBase, EQEP_INT_INDEX_EVNT_LATCH);
    }
    // Compute new absolute angle
    p_enc->thetaAbsolute            = p_enc->thetaMech[NEW] + turnNbRad;

    return;
}

/**
 * @brief           Reset the Proportional Integral (PI) Controller structure
 * @param[inout]    *p_pi   Pointer on the PI control structure.
 */
inline void FOC_resetStructPI(pi_t* p_pi)
{
    p_pi->err       = 0.0;
    p_pi->integral  = 0.0;
    p_pi->ff        = 0.0;
    p_pi->out       = 0.0;

    return;
}

/**
 * @brief           Reset the Proportional Derivative (PD) Controller structure
 * @param[inout]    *p_pd   Pointer on the PD control structure.
 */
inline void FOC_resetStructPD(pd_t* p_pd)
{
    p_pd->errTheta      = 0.0f;
    p_pd->errSpeed      = 0.0f;
    p_pd->derivative    = 0.0f;
    p_pd->out           = 0.0f;

    return;
}

/**
 * @brief           Reset the command structure associated to motor control
 * @param[inout]    p_cmd   Pointer on the local command associated to motor control
 */
inline void FOC_resetStructCmd(cmd_t* p_cmd)
{
    p_cmd->posRef               = 0.0f;
    p_cmd->posRefOffset         = 0.0f;
    p_cmd->velRef               = 0.0f;
    p_cmd->iqff                 = 0.0f;
    p_cmd->kpCoeff              = 0.0f;
    p_cmd->kdCoeff              = 0.0f;
    p_cmd->timeoutRef           = 0;
    p_cmd->cptTimeout           = 0;
    p_cmd->cmdBits.all          = 0;

    return;
}

/**
 * @brief           Reset the acquisition structure - Only I & Vext
 * @param[inout]    *p_acq  Pointer on the motor acquisition structure.
 */
inline void FOC_resetStructMeasures(acq_t* p_acq)
{
    p_acq->ia   = 0.0f;
    p_acq->ib   = 0.0f;
    p_acq->ic   = 0.0f;
    p_acq->vExt = 0.0f;

    return;
}

/**
 * @brief           Reset the FOC structure
 * @param[inout]    *p_foc   Pointer on the FOC structure to reset.
 */
inline void FOC_resetStruct(foc_t* p_foc)
{
    // Reset Id & Iq currents PI controllers
    FOC_resetStructPI(&p_foc->piId);
    FOC_resetStructPI(&p_foc->piIq);
    // Reset position PD controller
    FOC_resetStructPD(&p_foc->pdPosVel);
    // Reset encoder structure
    ENC_resetStruct(&p_foc->enc);
    // Reset analog measures
    FOC_resetStructMeasures(&p_foc->acq);
    // Reset the commands associated to motor control
    FOC_resetStructCmd(&p_foc->cmd);
    // Reset internal elements of FOC
    p_foc->sinTheta         = 0.0f;
    p_foc->cosTheta         = 0.0f;
    p_foc->ialpha           = 0.0f;
    p_foc->ibeta            = 0.0f;
    p_foc->id               = 0.0f;
    p_foc->iq               = 0.0f;
    p_foc->ud               = 0.0f;
    p_foc->uq               = 0.0f;
    p_foc->ualpha           = 0.0f;
    p_foc->ubeta            = 0.0f;
    p_foc->ua               = 0.0f;
    p_foc->ub               = 0.0f;
    p_foc->uc               = 0.0f;
    p_foc->vmax             = 0.0f;
    p_foc->dtc_u            = 0.0f;
    p_foc->dtc_v            = 0.0f;
    p_foc->dtc_w            = 0.0f;
    p_foc->resEst           = 0.0f;

    p_foc->initPos          = 0.0f;
    p_foc->initPosEnc[FW]   = 0.0f;
    p_foc->initPosEnc[REV]  = 0.0f;

    return;
}

/**
 * @brief           Read the phases current and phases \& bus voltages
 * @param[inout]    *p_acq  Pointer on the motor acquisition structure.
 */
inline void FOC_getMeasures(acq_t* p_acq)
{
#if (CLA_CORE_ENABLE)
    // Get the latest current values in Amps
//    p_acq->ia       = (float32_t)(*(int16_t *)p_acq->p_iMeasReg[CHANNEL_1].ptr) * ADC_MOTOR_CURRENT_SCALE;
//    p_acq->ib       = (float32_t)(*(int16_t *)p_acq->p_iMeasReg[CHANNEL_2].ptr) * ADC_MOTOR_CURRENT_SCALE;
//    p_acq->ic       = (float32_t)(*(int16_t *)p_acq->p_iMeasReg[CHANNEL_3].ptr) * ADC_MOTOR_CURRENT_SCALE;

//    p_acq->ia       = (float32_t)(HWREGH(p_acq->p_iMeasReg[CHANNEL_1].addr32b)) * ADC_MOTOR_CURRENT_SCALE;
//    p_acq->ib       = (float32_t)(HWREGH(p_acq->p_iMeasReg[CHANNEL_2].addr32b)) * ADC_MOTOR_CURRENT_SCALE;
//    p_acq->ic       = (float32_t)(HWREGH(p_acq->p_iMeasReg[CHANNEL_3].addr32b)) * ADC_MOTOR_CURRENT_SCALE;

//    p_acq->ia       = (float32_t)(HWREGH(p_acq->iabcMeasAddr[CHANNEL_1])) * ADC_MOTOR_CURRENT_SCALE;
//    p_acq->ib       = (float32_t)(HWREGH(p_acq->iabcMeasAddr[CHANNEL_2])) * ADC_MOTOR_CURRENT_SCALE;
//    p_acq->ic       = (float32_t)(HWREGH(p_acq->iabcMeasAddr[CHANNEL_3])) * ADC_MOTOR_CURRENT_SCALE;

    p_acq->ia       = ((float32_t)ADC_readPPBResult(p_acq->iabcResultBase[CHANNEL_1], p_acq->iabcppbNum[CHANNEL_1])) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ib       = ((float32_t)ADC_readPPBResult(p_acq->iabcResultBase[CHANNEL_2], p_acq->iabcppbNum[CHANNEL_2])) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ic       = ((float32_t)ADC_readPPBResult(p_acq->iabcResultBase[CHANNEL_3], p_acq->iabcppbNum[CHANNEL_3])) * ADC_MOTOR_CURRENT_SCALE;
    // Get the latest Voltage supply value in Volt (minimum is one to avoid dividing by zero later)
//    float32_t vbus  = __mmaxf32(((float32_t)(*(uint16_t *)p_acq->p_vBusMeasReg.ptr) * ADC_VBUS_VOLTAGE_SCALE), 1.0f);
//    UOMODRI_VBUS_FLT(p_acq->vBusFlt, p_acq->vbus, vbus);
//    float32_t vext  = (float32_t)(*(int16_t *)p_acq->p_vExtMeasReg.ptr) * ADC_VEXT12_VOLTAGE_SCALE;
//    UOMODRI_VEXT_FLT(p_acq->vExtFlt, p_acq->vExt, vext);

//    float32_t vbus  = (float32_t)(HWREGH(p_acq->vBusMeasAddr)) * ADC_VBUS_VOLTAGE_SCALE;
    float32_t vbus  = (float32_t)ADC_readPPBResult(p_acq->vBusResultBase, p_acq->vBusppbNum) * ADC_VBUS_VOLTAGE_SCALE;
    vbus            = __mmaxf32(vbus, 1.0f);
    UOMODRI_VBUS_FLT(p_acq->vBusFlt, p_acq->vbus, vbus);

//    float32_t vext  = (float32_t)(HWREGH(p_acq->vExtMeasAddr)) * ADC_VEXT12_VOLTAGE_SCALE;
    float32_t vext  = (float32_t)ADC_readPPBResult(p_acq->vExtResultBase, p_acq->vExtppbNum) * ADC_VEXT12_VOLTAGE_SCALE;
    UOMODRI_VEXT_FLT(p_acq->vExtFlt, p_acq->vExt, vext);
#else
    // Get the latest current values in Amps
    p_acq->ia       = (float32_t)(*p_acq->p_iMeasReg[CHANNEL_1]) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ib       = (float32_t)(*p_acq->p_iMeasReg[CHANNEL_2]) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ic       = (float32_t)(*p_acq->p_iMeasReg[CHANNEL_3]) * ADC_MOTOR_CURRENT_SCALE;
    // Get the latest Voltage supply value in Volt (minimum is one to avoid dividing by zero later)
    float32_t vbus  = __fmax(((float32_t)(*p_acq->p_vBusMeasReg) * ADC_VBUS_VOLTAGE_SCALE), 1.0f);
    UOMODRI_VBUS_FLT(p_acq->vBusFlt, p_acq->vbus, vbus);
    float32_t vext  = (float32_t)(*p_acq->p_vExtMeasReg) * ADC_VEXT12_VOLTAGE_SCALE;
    UOMODRI_VEXT_FLT(p_acq->vExtFlt, p_acq->vExt, vext);
#endif

    return;
}

/**
 * @brief           Run the Proportional Integral Controller (PI)
 * @param[inout]    *p_pi   Pointer on the PI control structure.
 * @return          out     Command of PI controller
 */
inline float32_t FOC_runPI(pi_t* p_pi)
{
#if (CLA_CORE_ENABLE)
    float32_t sat_max   = 0.5f  * (*(float32_t *)p_pi->p_sat.ptr);
    float32_t sat_min   = -1.0f * sat_max;
    p_pi->err           = (*(float32_t *)p_pi->p_set.ptr) - (*(float32_t *)p_pi->p_fb.ptr);
    p_pi->integral     += (p_pi->ki * p_pi->err * (float32_t)PWM_PERIOD);
    p_pi->integral      = __msatf32(p_pi->integral, sat_max, sat_min);
    p_pi->out           = (p_pi->kp * p_pi->err) + p_pi->integral + p_pi->ff;
    p_pi->out           =__msatf32(p_pi->out, sat_max, sat_min);
#else
    float32_t sat_max   = 0.5f  * (*p_pi->p_sat);
    float32_t sat_min   = -1.0f * sat_max;
    p_pi->err           = (*p_pi->p_set) - (*p_pi->p_fb);
    p_pi->integral     += (p_pi->ki * p_pi->err * (float32_t)PWM_PERIOD);
    p_pi->integral      = __fsat(p_pi->integral, sat_max, sat_min);
    p_pi->out           = (p_pi->kp * p_pi->err) + p_pi->integral + p_pi->ff;
    p_pi->out           = __fsat(p_pi->out, sat_max, sat_min);
#endif

    return(p_pi->out);
}

/**
 * @brief           Run the Proportional Derivative (PD) Controller
 * @param[inout]    *p_pd   Pointer on the PD control structure.
 * @return          out     Command of PD controller
 */
inline float32_t FOC_runPD(pd_t* p_pd)
{
#if (CLA_CORE_ENABLE)
    float32_t sat_max   = 1.0f  * (*(float32_t *)p_pd->p_sat.ptr);
    float32_t sat_min   = -1.0f * sat_max;
    p_pd->errTheta      = (*(float32_t *)p_pd->p_setTheta.ptr) - (*(float32_t *)p_pd->p_fbTheta.ptr);
    p_pd->errSpeed      = (*(float32_t *)p_pd->p_setSpeed.ptr) - (*(float32_t *)p_pd->p_fbSpeed.ptr);
    p_pd->derivative    = (*(float32_t *)p_pd->p_kd.ptr) * p_pd->errSpeed;
    // Compute PID and clamp output.
    p_pd->out           = ((*(float32_t *)p_pd->p_kp.ptr) * p_pd->errTheta) + p_pd->derivative + (*(float32_t *)p_pd->p_ff.ptr);
    p_pd->out           = __msatf32(p_pd->out, sat_max, sat_min);
#else
    float32_t sat_max   = 1.0f  * (*p_pd->p_sat);
    float32_t sat_min   = -1.0f * sat_max;
    p_pd->errTheta      = (*p_pd->p_setTheta) - (*p_pd->p_fbTheta);
    p_pd->errSpeed      = (*p_pd->p_setSpeed) - (*p_pd->p_fbSpeed);
    p_pd->derivative    = (*p_pd->p_kd) * p_pd->errSpeed;
    // Compute PID and clamp output.
    p_pd->out           = ((*p_pd->p_kp) * p_pd->errTheta) + p_pd->derivative + (*p_pd->p_ff);
    p_pd->out           = __fsat(p_pd->out, sat_max, sat_min);
#endif

    return(p_pd->out);
}

/**
 * @brief           First implementation of the Space Vector Modulation (SVM)
 * @param[inout]    *p_foc   Pointer on the FOC structure.
 */
inline void FOC_svpwm1(foc_t* p_foc)
{
    // Inverse Clarke transform
    p_foc->ua               = p_foc->ualpha;
    p_foc->ub               = (-0.5f * p_foc->ualpha) + (FM_SQRT3DIV2 * p_foc->ubeta);
    p_foc->uc               = (-0.5f * p_foc->ualpha) - (FM_SQRT3DIV2 * p_foc->ubeta);
    // Compute middle voltage to center PWM duty cycles around it
#if (CLA_CORE_ENABLE)
    float32_t   uOffset     = 0.5f * (MMIN3(p_foc->ua, p_foc->ub, p_foc->uc) + MMAX3(p_foc->ua, p_foc->ub, p_foc->uc));
#else
    float32_t   uOffset     = 0.5f * (FMIN3(p_foc->ua, p_foc->ub, p_foc->uc) + FMAX3(p_foc->ua, p_foc->ub, p_foc->uc));
#endif
    float32_t   uMidPoint   = 0.5f * (p_foc->dtcMax + p_foc->dtcMin);
    // Compute duty cycles
#if (CLA_CORE_ENABLE)
    p_foc->dtc_u            = __msatf32((((p_foc->ua - uOffset) / p_foc->acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_v            = __msatf32((((p_foc->ub - uOffset) / p_foc->acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_w            = __msatf32((((p_foc->uc - uOffset) / p_foc->acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
#else
    p_foc->dtc_u            = __fsat((((p_foc->ua - uOffset) / p_foc->motor_acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_v            = __fsat((((p_foc->ub - uOffset) / p_foc->motor_acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_w            = __fsat((((p_foc->uc - uOffset) / p_foc->motor_acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
#endif

    return;
}

/**
 * @brief           Compute the Field Oriented Control (FOC) algorithm
 * @param[inout]    *p_foc  Pointer on the FOC motor control structure.
 */
inline void FOC_runControl(foc_t* p_foc)
{
    acq_t*          p_acq   = &p_foc->acq;
    encoder_t*      p_enc   = &p_foc->enc;
    /* Compute sine and cosine from the electrical angle */
#if (CLA_CORE_ENABLE)
//    p_foc->sinTheta         = CLAsin(p_enc->thetaElec);
//    p_foc->cosTheta         = CLAcos(p_enc->thetaElec);
    CLAsincos(p_enc->thetaElec, &p_foc->sinTheta, &p_foc->cosTheta);
#else
    p_foc->sinTheta         = __sin(p_enc->thetaElec);
    p_foc->cosTheta         = __cos(p_enc->thetaElec);
#endif
    // Clarke transform
    p_foc->ialpha           = ((2.0f * p_acq->ia) - p_acq->ib - p_acq->ic) * FM_1DIV3;
    p_foc->ibeta            = (p_acq->ib - p_acq->ic) * FM_1DIVSQRT3;
    // Park transform + filtering
    float32_t id            = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
    UOMODRI_IDQ_FLT(p_foc->iParkFlt, p_foc->id, id);
    float32_t iq            = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);
    UOMODRI_IDQ_FLT(p_foc->iParkFlt, p_foc->iq, iq);
    // Compute decoupling feed-forward terms
#if (UOMODRI_FEED_FORWARD_ENABLE)
    const params_t* p_cfg   = &p_foc->motor_cfg;
    speed_t*        p_speed = &p_enc->speed;
    p_foc->piId.ff          = -1.0f * p_speed->speedMech[0] * p_cfg->Ls * p_foc->iq;
    p_foc->piIq.ff          = (p_speed->speedElec * p_cfg->Ls * p_foc->id) + (p_speed->speedMech[0] * p_cfg->ke);
#else
    p_foc->piId.ff          = 0.0f;
    p_foc->piIq.ff          = 0.0f;
#endif
    // Compute Id(Flux Current) and Iq (Torque Current) PI controllers
    p_foc->ud               = FOC_runPI(&p_foc->piId);
    p_foc->uq               = FOC_runPI(&p_foc->piIq);
    p_foc->vmax             = p_acq->vbus * FM_1DIVSQRT3;
//    p_foc->udMax            = p_foc->vmax;
//    p_foc->uqMax            = __sqrt((p_foc->vmax * p_foc->vmax) - (p_foc->ud * p_foc->ud));
//    float32_t v_ref         = __sqrt((p_foc->ud * p_foc->ud) + (p_foc->uq * p_foc->uq));
//    limit_norm(&p_foc->ud, &p_foc->uq, p_foc->vmax);
    // Inverse Park transform
    p_foc->ualpha           = (p_foc->cosTheta * p_foc->ud) - (p_foc->sinTheta * p_foc->uq);
    p_foc->ubeta            = (p_foc->sinTheta * p_foc->ud) + (p_foc->cosTheta * p_foc->uq);

    FOC_svpwm1(p_foc);

    return;
}

/**
 * @brief           Command for the 3 phases of the ePWM
 * @param[out]      p_motor Pointer on the associated motor structure
 * @param[in]       cmd_a   Normalized command on PWM channel A (between 0 and 1)
 * @param[in]       cmd_b   Normalized command on PWM channel B (between 0 and 1)
 * @param[in]       cmd_c   Normalized command on PWM channel C (between 0 and 1)
 */
inline void MOT_runCommand(motor_t* p_motor, float32_t cmd_a, float32_t cmd_b, float32_t cmd_c)
{
#if (CLA_CORE_ENABLE)
#if(0)
    if(p_motor->id == MOTOR_1)
    {
        HWREGH(MOTOR1_PWM1_CMD_ADDR) = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT);
        HWREGH(MOTOR1_PWM2_CMD_ADDR) = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT);
        HWREGH(MOTOR1_PWM3_CMD_ADDR) = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT);
    }
    else
    {
        HWREGH(MOTOR2_PWM1_CMD_ADDR) = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT);
        HWREGH(MOTOR2_PWM2_CMD_ADDR) = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT);
        HWREGH(MOTOR2_PWM3_CMD_ADDR) = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT);
    }
#else
    HWREGH(p_motor->epwmBaseAddr[CHANNEL_1] + MOTOR12_PWMx_CMD_ADDR_OFFSET) = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT);
    HWREGH(p_motor->epwmBaseAddr[CHANNEL_2] + MOTOR12_PWMx_CMD_ADDR_OFFSET) = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT);
    HWREGH(p_motor->epwmBaseAddr[CHANNEL_3] + MOTOR12_PWMx_CMD_ADDR_OFFSET) = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT);
#endif
#else
    *(p_motor->p_cmdReg[CHANNEL_1]) = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT);
    *(p_motor->p_cmdReg[CHANNEL_2]) = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT);
    *(p_motor->p_cmdReg[CHANNEL_3]) = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT);
#endif

    return;
}

/**
 * @brief           Force a hard stop on the 3 phases of the ePWM (low side active)
 * @param[out]      p_motor Pointer on the associated motor structure
 */
inline void MOT_stopCommand(motor_t* p_motor)
{
#if (CLA_CORE_ENABLE)
#if(0)
    if(p_motor->id == MOTOR_1)
    {
        HWREGH(MOTOR1_PWM1_CMD_ADDR) = (uint16_t)PWM_TIMEBASE_CNT;
        HWREGH(MOTOR1_PWM2_CMD_ADDR) = (uint16_t)PWM_TIMEBASE_CNT;
        HWREGH(MOTOR1_PWM3_CMD_ADDR) = (uint16_t)PWM_TIMEBASE_CNT;
    }
    else
    {
        HWREGH(MOTOR2_PWM1_CMD_ADDR) = (uint16_t)PWM_TIMEBASE_CNT;
        HWREGH(MOTOR2_PWM2_CMD_ADDR) = (uint16_t)PWM_TIMEBASE_CNT;
        HWREGH(MOTOR2_PWM3_CMD_ADDR) = (uint16_t)PWM_TIMEBASE_CNT;
    }
#else
    HWREGH(p_motor->epwmBaseAddr[CHANNEL_1] + MOTOR12_PWMx_CMD_ADDR_OFFSET) = (uint16_t)PWM_TIMEBASE_CNT;
    HWREGH(p_motor->epwmBaseAddr[CHANNEL_2] + MOTOR12_PWMx_CMD_ADDR_OFFSET) = (uint16_t)PWM_TIMEBASE_CNT;
    HWREGH(p_motor->epwmBaseAddr[CHANNEL_3] + MOTOR12_PWMx_CMD_ADDR_OFFSET) = (uint16_t)PWM_TIMEBASE_CNT;
#endif
#else
    *(p_motor->p_cmdReg[CHANNEL_1]) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_cmdReg[CHANNEL_2]) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_cmdReg[CHANNEL_3]) = (uint16_t)PWM_TIMEBASE_CNT;
#endif

    return;
}

/**
 * @brief           Control of the motor states
 * @param[inout]    p_motor Pointer on the associated motor structure
 * @param[in]       p_cmd   Pointer on the command structure driving motor
 */
inline bool_t MOT_runControl(motor_t* p_motor)
{
#if (USE_STATE_MACHINE_4_ELECTRONCS_ENGINEERS)
#if (CLA_CORE_ENABLE)
    foc_t*          p_foc       = &p_motor->foc;
#else
    foc_t*          p_foc       = p_motor->p_foc;
#endif
    error_reg_u*    p_err       = &p_motor->error;
    encoder_t*      p_enc       = &p_foc->enc;
    cmd_t*          p_cmd       = &p_foc->cmd;
    float32_t       tmp         = 0.0f;
    float32_t       diff        = 0.0f;
    float32_t       err         = 0.0f;
//    float32_t       err[2]      = {0.0f, 0.0f};
    cmd_reg_u       cmd_reg     = {.all = p_cmd->cmdBits.all};
    uint8_t         motor_id    = p_motor->id;
    bool_t          evt_flag    = false;
    p_enc->flags.indexOffset    = cmd_reg.encOffsetEnable;
    p_cmd->posRefOffset         = p_cmd->posRef + ((cmd_reg.encOffsetEnable) ? (p_enc->thetaIndexAbsolute) : (0.0f));

    // Read currents in 3 phases + Vbus
    FOC_getMeasures(&p_foc->acq);
    // Read encoder position
    ENC_getPosition(p_enc);
    // Estimate speed
    ENC_getSpeed(p_enc);
    // Rollover test & index
    ENC_getTheta(p_enc);

    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->epwmBaseAddr[CHANNEL_1]);
    p_err->drv_fault            = !GPIO_readPin(p_motor->drvFaultNum)                           || p_err->drv_fault;
    p_err->pos_rollover         = (p_enc->flags.rollOverError && cmd_reg.rollOverEnable)        || p_err->pos_rollover;
    p_err->com_timeout          = ((p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef)|| p_err->com_timeout;
    p_err->enc_indexErr         = (p_enc->flags.indexDetect && p_enc->flags.indexError)         || p_err->enc_indexErr;
    p_err->enc_alignErr         = p_enc->flags.alignError                                       || p_err->enc_alignErr;
    p_motor->state              = (p_err->all)              ? (MOTOR_STATE_ERROR)           : (p_motor->state);

    switch(p_motor->state)
    {
    case MOTOR_STATE_START:
    default:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_foc->initPos          = 0.0f;
        p_foc->initPosEnc[FW]   = 0.0f;
        p_foc->initPosEnc[REV]  = 0.0f;
        p_motor->state          = (cmd_reg.motorEnable)     ? (MOTOR_STATE_INIT_FW)         : (MOTOR_STATE_START);
        p_motor->state          = (cmd_reg.systemEnable)    ? (p_motor->state)              : (MOTOR_STATE_START);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_INIT_FW:
        tmp                     = (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_EVT)    : (MOTOR2_POSITION_INIT_EVT);
        evt_flag                = (p_motor->itCnt > tmp)    ? (true)                        : (false);
        p_motor->itCnt          = (evt_flag)                ? (0U)                          : (p_motor->itCnt + 1);
        p_foc->initPosEnc[FW]   = p_enc->thetaAbsolute;
        p_foc->idRef           += (motor_id == MOTOR_1)     ? (MOTOR1_CURRENT_INIT_STEP)    : (MOTOR2_CURRENT_INIT_STEP);
        p_foc->iqRef            = 0.0f;
        p_foc->initPos         -= (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_STEP)   : (MOTOR2_POSITION_INIT_STEP);
        p_enc->thetaElec        = p_foc->initPos;
        p_motor->state          = (evt_flag)                ? (MOTOR_STATE_INIT_REV)        : (MOTOR_STATE_INIT_FW);
        p_motor->state          = (cmd_reg.motorEnable)     ? (p_motor->state)              : (MOTOR_STATE_START);
        p_motor->state          = (cmd_reg.systemEnable)    ? (p_motor->state)              : (MOTOR_STATE_START);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_REV:
        tmp                     = (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_EVT)    : (MOTOR2_POSITION_INIT_EVT);
        evt_flag                = (p_motor->itCnt > tmp)    ? (true)                        : (false);
        p_motor->itCnt          = (evt_flag)                ? (0U)                          : (p_motor->itCnt + 1);
        p_foc->initPosEnc[REV]  = p_enc->thetaAbsolute;
        p_foc->idRef            = (motor_id == MOTOR_1)     ? (MOTOR1_CURRENT_INIT_MAX)     : (MOTOR2_CURRENT_INIT_MAX);
        p_foc->iqRef            = 0.0f;
        p_foc->initPos         += (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_STEP)   : (MOTOR2_POSITION_INIT_STEP);
        p_enc->thetaElec        = p_foc->initPos;
        p_motor->state          = (evt_flag)                ? (MOTOR_STATE_INIT_TEST)       : (MOTOR_STATE_INIT_REV);
        p_motor->state          = (cmd_reg.motorEnable)     ? (p_motor->state)              : (MOTOR_STATE_START);
        p_motor->state          = (cmd_reg.systemEnable)    ? (p_motor->state)              : (MOTOR_STATE_START);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_TEST:
//        diff                    = p_foc->initPosEnc[REV] - p_foc->initPosEnc[FW];
//        err[MIN]                = (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_ERR_MIN): (MOTOR2_POSITION_INIT_ERR_MIN);
//        err[MAX]                = (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_ERR_MAX): (MOTOR2_POSITION_INIT_ERR_MAX);
//        p_enc->flags.alignError = (diff > err[MIN])         ? (false)                       : (true);
//        p_enc->flags.alignError = (diff < err[MAX])         ? (p_enc->flags.alignError)     : (true);
        p_foc->initPosEnc[FW]  -= (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_MAX)    : (MOTOR2_POSITION_INIT_MAX);
        diff                    = fabsf(p_foc->initPosEnc[REV] - p_foc->initPosEnc[FW]);
        err                     = (motor_id == MOTOR_1)     ? (MOTOR1_POSITION_INIT_ERR)    : (MOTOR2_POSITION_INIT_ERR);
        p_enc->flags.alignError = (diff < err)              ? (false)                       : (true);
        p_motor->itCnt          = 0U;
        p_foc->idRef            = (motor_id == MOTOR_1)     ? (MOTOR1_CURRENT_INIT_MAX)     : (MOTOR2_CURRENT_INIT_MAX);
        p_foc->iqRef            = 0.0f;
        p_motor->state          = (cmd_reg.motorEnable)     ? (MOTOR_STATE_INIT_FIX)        : (MOTOR_STATE_START);
        p_motor->state          = (cmd_reg.systemEnable)    ? (p_motor->state)              : (MOTOR_STATE_START);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_FIX:
        tmp                     = (motor_id == MOTOR_1)     ? (MOTOR1_FIX_INIT_EVT)         : (MOTOR2_FIX_INIT_EVT);
        p_motor->itCnt         += 1U;
        p_foc->idRef            = (motor_id == MOTOR_1)     ? (MOTOR1_CURRENT_INIT_MAX)     : (MOTOR2_CURRENT_INIT_MAX);
        p_foc->iqRef            = 0.0f;
        p_motor->state          = (p_motor->itCnt > tmp)    ? (MOTOR_STATE_READY)           : (MOTOR_STATE_INIT_FIX);
        p_motor->state          = (cmd_reg.motorEnable)     ? (p_motor->state)              : (MOTOR_STATE_START);
        p_motor->state          = (cmd_reg.systemEnable)    ? (p_motor->state)              : (MOTOR_STATE_START);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPosVel);
        p_motor->state          = (!cmd_reg.motorEnable)    ? (MOTOR_STATE_STOP)            : (MOTOR_STATE_READY);
        p_motor->state          = (cmd_reg.systemEnable)    ? (p_motor->state)              : (MOTOR_STATE_START);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->state          = (cmd_reg.motorEnable)     ? (MOTOR_STATE_READY)           : (MOTOR_STATE_STOP);
        p_motor->state          = (cmd_reg.systemEnable)    ? (p_motor->state)              : (MOTOR_STATE_START);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ERROR:
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->state          = MOTOR_STATE_ERROR;
        MOT_stopCommand(p_motor);
        break;
    }
#else
#if (CLA_CORE_ENABLE)
    foc_t*          p_foc           = &p_motor->foc;
#else
    foc_t*          p_foc           = p_motor->p_foc;
#endif
    error_reg_u*    p_err           = &p_motor->error;
    encoder_t*      p_enc           = &p_foc->enc;//&p_motor->foc.enc;//
    cmd_t*          p_cmd           = &p_foc->cmd;//&p_motor->foc.cmd;
    float32_t       tmp             = 0.0f;
    float32_t       diff            = 0.0f;
    float32_t       err             = 0.0f;
//    float32_t       err[2]          = {0.0f, 0.0f};
    cmd_reg_u       cmd_reg         = {.all = p_cmd->cmdBits.all};
    p_enc->flags.indexOffset        = cmd_reg.encOffsetEnable;
    p_cmd->posRefOffset             = p_cmd->posRef + ((cmd_reg.encOffsetEnable) ? (p_enc->thetaIndexAbsolute) : (0.0f));

    //  Read currents in 3 phases + Vbus
    FOC_getMeasures(&p_foc->acq);
    // Read encoder position
    ENC_getPosition(p_enc);
    // Estimate speed
    ENC_getSpeed(p_enc);
    // Rollover test & index
    ENC_getTheta(p_enc);

    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb              = EPWM_getTimeBaseCounterValue(p_motor->epwmBaseAddr[CHANNEL_1]);
    p_err->drv_fault                = !GPIO_readPin(p_motor->drvFaultNum)                           || p_err->drv_fault;
    p_err->pos_rollover             = (p_enc->flags.rollOverError && cmd_reg.rollOverEnable)        || p_err->pos_rollover;
    p_err->com_timeout              = ((p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef)|| p_err->com_timeout;
    p_err->enc_indexErr             = (p_enc->flags.indexDetect && p_enc->flags.indexError)         || p_err->enc_indexErr;
    p_err->enc_alignErr             = p_enc->flags.alignError                                       || p_err->enc_alignErr;
    if(p_err->all)
        p_motor->state              = MOTOR_STATE_ERROR;

    switch(p_motor->state)
    {
    case MOTOR_STATE_START:
    default:
        p_motor->itCnt              = 0U;
        p_foc->idRef                = 0.0f;
        p_foc->iqRef                = 0.0f;
        p_foc->initPos              = 0.0f;
        p_foc->initPosEnc[FW]       = 0.0f;
        p_foc->initPosEnc[REV]      = 0.0f;
        if(cmd_reg.motorEnable && cmd_reg.systemEnable)
            p_motor->state          = MOTOR_STATE_INIT_FW;
        else
            p_motor->state          = MOTOR_STATE_START;
//        ENC_resetPeriph(p_enc);
//        FOC_resetStruct(p_foc);
        FOC_runControl(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_INIT_FW:
        if(p_motor->id == MOTOR_1)
        {
            tmp                     = MOTOR1_POSITION_INIT_EVT;
            p_foc->idRef           += MOTOR1_CURRENT_INIT_STEP;
            p_foc->initPos         -= MOTOR1_POSITION_INIT_STEP;
        }
        else
        {
            tmp                     = MOTOR2_POSITION_INIT_EVT;
            p_foc->idRef           += MOTOR2_CURRENT_INIT_STEP;
            p_foc->initPos         -= MOTOR2_POSITION_INIT_STEP;
        }
        p_enc->thetaElec            = p_foc->initPos;
        p_foc->initPosEnc[FW]       = p_enc->thetaAbsolute;
        p_motor->itCnt              = (p_motor->itCnt > tmp) ? (0U) : (p_motor->itCnt + 1);
        if(!(cmd_reg.motorEnable && cmd_reg.systemEnable))
            p_motor->state          = MOTOR_STATE_START;
        else if(p_motor->itCnt > tmp)
            p_motor->state          = MOTOR_STATE_INIT_REV;
        else
            p_motor->state          = MOTOR_STATE_INIT_FW;
//        if(!(cmd_reg.motorEnable && cmd_reg.systemEnable))
//            p_motor->state          = MOTOR_STATE_START;
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_REV:
        if(p_motor->id == MOTOR_1)
        {
            tmp                     = MOTOR1_POSITION_INIT_EVT;
            p_foc->idRef            = MOTOR1_CURRENT_INIT_MAX;
            p_foc->initPos         += MOTOR1_POSITION_INIT_STEP;
        }
        else
        {
            tmp                     = MOTOR2_POSITION_INIT_EVT;
            p_foc->idRef            = MOTOR2_CURRENT_INIT_MAX;
            p_foc->initPos         += MOTOR2_POSITION_INIT_STEP;
        }
        p_enc->thetaElec            = p_foc->initPos;
        p_foc->initPosEnc[REV]      = p_enc->thetaAbsolute;
        p_motor->itCnt              = (p_motor->itCnt > tmp) ? (0U) : (p_motor->itCnt + 1);
        if(!(cmd_reg.motorEnable && cmd_reg.systemEnable))
            p_motor->state          = MOTOR_STATE_START;
        else if(p_motor->itCnt > tmp)
            p_motor->state          = MOTOR_STATE_INIT_TEST;
        else
            p_motor->state          = MOTOR_STATE_INIT_REV;
//        if(!(cmd_reg.motorEnable && cmd_reg.systemEnable))
//            p_motor->state          = MOTOR_STATE_START;
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_TEST:
        if(p_motor->id == MOTOR_1)
        {
            p_foc->initPosEnc[FW]  -= MOTOR1_POSITION_INIT_MAX;
            err                     = MOTOR1_POSITION_INIT_ERR;
            p_foc->idRef            = MOTOR1_CURRENT_INIT_MAX;
        }
        else
        {
            p_foc->initPosEnc[FW]  -= MOTOR2_POSITION_INIT_MAX;
            err                     = MOTOR2_POSITION_INIT_ERR;
            p_foc->idRef            = MOTOR2_CURRENT_INIT_MAX;
        }
        p_motor->itCnt              = 0U;
        p_foc->iqRef                = 0.0f;
        diff                        = fabsf(p_foc->initPosEnc[REV] - p_foc->initPosEnc[FW]);
        p_enc->flags.alignError     = (diff < err) ? (false) : (true);
//        if(p_motor->id == MOTOR_1)
//        {
//            err[MIN]                = MOTOR1_POSITION_INIT_ERR_MIN;
//            err[MAX]                = MOTOR1_POSITION_INIT_ERR_MAX;
//            p_foc->idRef            = MOTOR1_CURRENT_INIT_MAX;
//        }
//        else
//        {
//            err[MIN]                = MOTOR2_POSITION_INIT_ERR_MIN;
//            err[MAX]                = MOTOR2_POSITION_INIT_ERR_MAX;
//            p_foc->idRef            = MOTOR2_CURRENT_INIT_MAX;
//        }
//        p_motor->itCnt              = 0U;
//        p_foc->iqRef                = 0.0f;
//        diff                        = p_foc->initPosEnc[REV] - p_foc->initPosEnc[FW];
//        if((diff < err[MIN]) || (diff > err[MAX]))
//            p_enc->flags.alignError = true;
//        else
//            p_enc->flags.alignError = false;
        if(cmd_reg.motorEnable && cmd_reg.systemEnable)
            p_motor->state          = MOTOR_STATE_INIT_FIX;
        else
            p_motor->state          = MOTOR_STATE_START;
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_INIT_FIX:
        if(p_motor->id == MOTOR_1)
        {
            tmp                     = MOTOR1_FIX_INIT_EVT;
            p_foc->idRef            = MOTOR1_CURRENT_INIT_MAX;
        }
        else
        {
            tmp                     = MOTOR2_FIX_INIT_EVT;
            p_foc->idRef            = MOTOR2_CURRENT_INIT_MAX;
        }
        p_motor->itCnt++;
        p_foc->iqRef                = 0.0f;
        if(!(cmd_reg.motorEnable && cmd_reg.systemEnable))
            p_motor->state          = MOTOR_STATE_START;
        else if(p_motor->itCnt > tmp)
            p_motor->state          = MOTOR_STATE_READY;
        else
            p_motor->state          = MOTOR_STATE_INIT_FIX;
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt++;
        p_foc->idRef                = 0.0f;
        p_foc->iqRef                = FOC_runPD(&p_foc->pdPosVel);
        if(!cmd_reg.systemEnable)
            p_motor->state          = MOTOR_STATE_START;
        else if(!cmd_reg.motorEnable)
            p_motor->state          = MOTOR_STATE_STOP;
        else
            p_motor->state          = MOTOR_STATE_READY;
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt++;
        p_foc->idRef                = 0.0f;
        p_foc->iqRef                = 0.0f;
        if(!cmd_reg.systemEnable)
            p_motor->state          = MOTOR_STATE_START;
        else if(!cmd_reg.motorEnable)
            p_motor->state          = MOTOR_STATE_STOP;
        else
            p_motor->state          = MOTOR_STATE_READY;
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ERROR:
        p_foc->idRef                = 0.0f;
        p_foc->iqRef                = 0.0f;
        p_motor->state              = MOTOR_STATE_ERROR;
        MOT_stopCommand(p_motor);
        break;
    }
#endif

    return(true);
}

#endif /* __UOMODRI_SHARED_H__ */

