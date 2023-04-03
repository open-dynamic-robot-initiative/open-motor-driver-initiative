#ifndef __CLA_SHARED_H__
#define __CLA_SHARED_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
//#include "hal.h"
#include <CLAmath.h>

#define run_loop1_cla (1)
#if (run_loop1_cla)
#include <CLAmath.h>
//#include "DCLCLA.h"
#else
#include "DCLF32.h"
#endif
#include "foc.h"
#include "motor.h"
#include "encoder.h"
#include "uomodri_user_defines.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define MMAX3(x, y, z)                          (__mmaxf32(__mmaxf32((x), (y)), (z)))
#define MMIN3(x, y, z)                          (__mminf32(__mminf32((x), (y)), (z)))

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
extern float32_t __mminf32(float32_t, float32_t);
extern float32_t __mmaxf32(float32_t, float32_t);
extern float32_t __mfracf32(float32_t);

#if (CLA_CORE_ENABLE)
extern foc_t motor_foc[];
extern motor_t motor[];
#endif

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
//
// Function Prototypes
//
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.
//
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();
//void initCLA(void);

inline float32_t __msatf32(float32_t, float32_t, float32_t);
inline void ENC_resetStruct(encoder_t*);
inline void FOC_getMeasures(acq_t*);
inline void FOC_resetMeasuresStruct(acq_t*);
inline void FOC_resetPI(pi_t*);
inline float32_t FOC_runPI(pi_t*);
inline void FOC_resetPD(pd_t*);
inline float32_t FOC_runPD(pd_t*);
inline void FOC_resetCmdStruct(cmd_t*);
inline void FOC_resetStruct(foc_t*);
inline void FOC_svpwm1(foc_t*);
inline void MOT_runCommand(motor_t*, float32_t, float32_t, float32_t);
inline void MOT_stopCommand(motor_t*);
inline void FOC_runControl(foc_t*);
inline void ENC_resetPeriph(encoder_t*);
inline void ENC_getPosition(encoder_t*);
inline void ENC_getSpeed(encoder_t*);
inline void ENC_getTheta(encoder_t*);
inline bool_t MOT_runControl(motor_t*);

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
inline float32_t __msatf32(float32_t a, float32_t sat_max, float32_t sat_min)
{
    return(__mmaxf32(__mminf32(a, sat_max), sat_min));
}

/**
 * @brief           Reset the ENCODER structure
 * @param[inout]    *p_enc   Pointer on the ENCODER structure to reset.
 */
inline void ENC_resetStruct(encoder_t* p_enc)
{
    speed_t* p_speed        = &p_enc->speed;

    p_enc->thetaMech[NEW]   = 0.0f;
    p_enc->thetaMech[OLD]   = 0.0f;
    p_enc->thetaElec        = 0.0f;
    p_enc->thetaIndex       = 0.0f;
    p_enc->thetaAbsolute    = 0.0f;
    p_enc->turnNb           = 0U;
    p_enc->indexDetect      = false;
    p_enc->indexOffset      = false;
    p_enc->indexToggle      = false;
    p_enc->indexError       = false;
    p_enc->rollOverError    = false;

    p_speed->alpha          = 0.0f;
    p_speed->theta[NEW]     = 0.0f;
    p_speed->theta[OLD]     = 0.0f;
    p_speed->speedHigh      = 0.0f;
    p_speed->speedLow       = 0.0f;
    p_speed->speedMech[0]   = 0.0f;
    p_speed->speedMech[1]   = 0.0f;
    p_speed->speedElec      = 0.0f;

    return;
}

/**
 * @brief           Read the phases current and phases \& bus voltages
 * @param[inout]    *p_acq  Pointer on the motor acquisition structure.
 */
inline void FOC_getMeasures(acq_t* p_acq)
{
    // Get the latest current values in Amps
    p_acq->ia       = (float32_t)(*p_acq->p_iMeasReg[CHANNEL_1].ptr_i16) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ib       = (float32_t)(*p_acq->p_iMeasReg[CHANNEL_2].ptr_i16) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ic       = (float32_t)(*p_acq->p_iMeasReg[CHANNEL_3].ptr_i16) * ADC_MOTOR_CURRENT_SCALE;
    // Get the latest Voltage supply value in Volt (minimum is one to avoid dividing by zero later)
    float32_t vbus  = __mmaxf32(((float32_t)(*p_acq->p_vBusMeasReg.ptr_u16) * ADC_VBUS_VOLTAGE_SCALE), 1.0f);
    UOMODRI_VBUS_FLT(p_acq->vBusFlt, p_acq->vbus, vbus);
    float32_t vext  = (float32_t)(*p_acq->p_vExtMeasReg.ptr_u16) * ADC_VEXT12_VOLTAGE_SCALE;
    UOMODRI_VEXT_FLT(p_acq->vExtFlt, p_acq->vExt, vext);

    return;
}

/**
 * @brief           Reset the acquisition structure - Only I & Vext
 * @param[inout]    *p_acq  Pointer on the motor acquisition structure.
 */
inline void FOC_resetMeasuresStruct(acq_t* p_acq)
{
    // Get the latest current values in Amps
    p_acq->ia   = 0.0f;
    p_acq->ib   = 0.0f;
    p_acq->ic   = 0.0f;
    p_acq->vExt = 0.0f;

    return;
}

/**
 * @brief           Reset the Proportional Integral (PI) Controller structure
 * @param[inout]    *p_pi   Pointer on the PI control structure.
 */
inline void FOC_resetPI(pi_t* p_pi)
{
    p_pi->err       = 0.0;
    p_pi->integral  = 0.0;
    p_pi->ff        = 0.0;
    p_pi->out       = 0.0;

    return;
}

/**
 * @brief           Run the Proportional Integral Controller (PI)
 * @param[inout]    *p_pi   Pointer on the PI control structure.
 * @return          out     Command of PI controller
 */
inline float32_t FOC_runPI(pi_t* p_pi)
{
    float32_t sat_max   = 0.5f  * (*p_pi->p_sat);
    float32_t sat_min   = -1.0f * sat_max;
    p_pi->err           = (*p_pi->p_set) - (*p_pi->p_fb);
    p_pi->integral     += (p_pi->ki * p_pi->err * (float32_t)PWM_PERIOD);
    p_pi->integral      = __msatf32(p_pi->integral, sat_max, sat_min);//__fsat(p_pi->integral, sat_max, sat_min);
    p_pi->out           = (p_pi->kp * p_pi->err) + p_pi->integral + p_pi->ff;
    p_pi->out           = __msatf32(p_pi->out, sat_max, sat_min);//__fsat(p_pi->out, sat_max, sat_min);

    return(p_pi->out);
}

/**
 * @brief           Reset the Proportional Derivative (PD) Controller structure
 * @param[inout]    *p_pd   Pointer on the PD control structure.
 */
inline void FOC_resetPD(pd_t* p_pd)
{
    p_pd->errTheta      = 0.0f;
    p_pd->errSpeed      = 0.0f;
    p_pd->derivative    = 0.0f;
    p_pd->out           = 0.0f;

    return;
}

/**
 * @brief           Run the Proportional Derivative (PD) Controller
 * @param[inout]    *p_pd   Pointer on the PD control structure.
 * @return          out     Command of PD controller
 */
inline float32_t FOC_runPD(pd_t* p_pd)
{
    float32_t sat_max   = 1.0f  * (*p_pd->p_sat);
    float32_t sat_min   = -1.0f * sat_max;
    p_pd->errTheta      = (*p_pd->p_setTheta) - (*p_pd->p_fbTheta);
    p_pd->errSpeed      = (*p_pd->p_setSpeed) - (*p_pd->p_fbSpeed);
    p_pd->derivative    = (*p_pd->p_kd) * p_pd->errSpeed;
    // Compute PID and clamp output.
    p_pd->out           = ((*p_pd->p_kp) * p_pd->errTheta) + p_pd->derivative + (*p_pd->p_ff);
    p_pd->out           = __msatf32(p_pd->out, sat_min, sat_max);//__fsat(p_pd->out, sat_max, sat_min);

    return(p_pd->out);
}

/**
 * @brief           Reset the command structure associated to motor control
 * @param[inout]    p_cmd   Pointer on the local command associated to motor control
 */
inline void FOC_resetCmdStruct(cmd_t* p_cmd)
{
    p_cmd->posRef               = 0.0f;
    p_cmd->velRef               = 0.0f;
    p_cmd->iqff                 = 0.0f;
    p_cmd->kpCoeff              = 0.0f;
    p_cmd->kdCoeff              = 0.0f;
    p_cmd->iSat                 = 0.0f;
    p_cmd->timeoutRef           = 0;
    p_cmd->cptTimeout           = 0;
    p_cmd->cmdArray.all         = 0;

    return;
}

/**
 * @brief           Reset the FOC structure
 * @param[inout]    *p_foc   Pointer on the FOC structure to reset.
 */
inline void FOC_resetStruct(foc_t* p_foc)
{
    // Reset Id & Iq currents PI controllers
    FOC_resetPI(&p_foc->piId);
    FOC_resetPI(&p_foc->piIq);
    // Reset position PD controller
    FOC_resetPD(&p_foc->pdPosVel);
    // Reset encoder structure
    ENC_resetStruct(&p_foc->motor_enc);
    // Reset analog measures
    FOC_resetMeasuresStruct(&p_foc->motor_acq);
    // Reset the commands associated to motor control
    FOC_resetCmdStruct(&p_foc->motor_cmd);
    // Reset currents and voltages
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

    return;
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
    float32_t   uOffset     = 0.5f * (MMIN3(p_foc->ua, p_foc->ub, p_foc->uc) + MMAX3(p_foc->ua, p_foc->ub, p_foc->uc));
    float32_t   uMidPoint   = 0.5f * (p_foc->dtcMax + p_foc->dtcMin);
    // Compute duty cycles
    p_foc->dtc_u            = __msatf32((((p_foc->ua - uOffset) / p_foc->motor_acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_v            = __msatf32((((p_foc->ub - uOffset) / p_foc->motor_acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_w            = __msatf32((((p_foc->uc - uOffset) / p_foc->motor_acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
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
    *(p_motor->p_motorCmdReg[CHANNEL_1].ptr_u16)    = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT);
    *(p_motor->p_motorCmdReg[CHANNEL_2].ptr_u16)    = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT);
    *(p_motor->p_motorCmdReg[CHANNEL_3].ptr_u16)    = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT);

    return;
}

/**
 * @brief           Force a hard stop on the 3 phases of the ePWM (low side active)
 * @param[out]      p_motor Pointer on the associated motor structure
 */
inline void MOT_stopCommand(motor_t* p_motor)
{
    *(p_motor->p_motorCmdReg[CHANNEL_1].ptr_u16)    = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorCmdReg[CHANNEL_2].ptr_u16)    = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorCmdReg[CHANNEL_3].ptr_u16)    = (uint16_t)PWM_TIMEBASE_CNT;

    return;
}

/**
 * @brief           Compute the Field Oriented Control (FOC) algorithm
 * @param[inout]    *p_foc  Pointer on the FOC motor control structure.
 */
inline void FOC_runControl(foc_t* p_foc)
{
    acq_t*          p_acq   = &p_foc->motor_acq;
    encoder_t*      p_enc   = &p_foc->motor_enc;
    /* Compute sine and cosine from the electrical angle */
    CLAsincos(p_enc->thetaElec, &p_foc->sinTheta, &p_foc->cosTheta);
//    p_foc->sinTheta         = CLAsin(p_enc->thetaElec);
//    p_foc->cosTheta         = CLAcos(p_enc->thetaElec);
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
    p_enc->thetaElec        = FM_ROUND2RAD * __mfracf32(thetaElec);
    p_enc->thetaMech[NEW]   = FM_ROUND2RAD * p_enc->thetaMech[NEW];

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
        p_speed->theta[NEW]     = FM_ROUND2RAD * (float32_t)EQEP_getPositionLatch(eqepBase) * p_enc->thetaMechScaler;
        EQEP_clearInterruptStatus(eqepBase, (EQEP_INT_UNIT_TIME_OUT));// | EQEP_INT_OVERFLOW | EQEP_INT_UNDERFLOW));
        // Compute angle difference
        float32_t deltaTheta    = p_speed->theta[NEW] - p_speed->theta[OLD];
        // Saturate it between -pi and pi
        deltaTheta              = (deltaTheta > M_PI)   ? (deltaTheta - (float32_t)(2.0f * M_PI)) : (deltaTheta);
        deltaTheta              = (deltaTheta < -M_PI)  ? (deltaTheta + (float32_t)(2.0f * M_PI)) : (deltaTheta);
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
    uint32_t eqepBase           = p_enc->eqepBaseAddr;
    //-------------------------------------------------------------------
    //     TURN CONTROL
    //-------------------------------------------------------------------
    float32_t deltaThetaMech    = p_enc->thetaMech[NEW] - p_enc->thetaMech[OLD];
    // Update turnNb
    p_enc->turnNb               = (deltaThetaMech > M_PI)   ? (p_enc->turnNb - 1) : (p_enc->turnNb);
    p_enc->turnNb               = (deltaThetaMech < -M_PI)  ? (p_enc->turnNb + 1) : (p_enc->turnNb);
    // If theta difference is high or low enough, we overflowed or underflowed
    bool_t maxRollOver          = (p_enc->turnNb > MAX_ROLLOVER)? (true) : (false);
    bool_t minRollOver          = (p_enc->turnNb < MIN_ROLLOVER)? (true) : (false);
    p_enc->rollOverError        = (maxRollOver || minRollOver)  ? (true) : (false);
    // Saturate turnNb between -128 and 127
    p_enc->turnNb               = (maxRollOver) ? (MIN_ROLLOVER) : (p_enc->turnNb);
    p_enc->turnNb               = (minRollOver) ? (MAX_ROLLOVER) : (p_enc->turnNb);
    //-------------------------------------------------------------------
    //     INDEX HANDLING
    //-------------------------------------------------------------------
    if(EQEP_getInterruptStatus(eqepBase) & EQEP_INT_INDEX_EVNT_LATCH)
    {
        // Compute indexed mechanical angle normalized between 0 and 2 x PI
        float32_t thetaIndex    = FM_ROUND2RAD * (float32_t)EQEP_getIndexPositionLatch(eqepBase) * p_enc->thetaMechScaler;
        // Save first indexed mechanical angle normalized between 0 and 2 x PI
        p_enc->thetaIndex       = (!p_enc->indexDetect) ? (thetaIndex) : (p_enc->thetaIndex);
        // Set index detection flag.
        p_enc->indexDetect      = true;
        // Check if current index capture match the first index capture
        p_enc->indexError       = (fabsf(p_enc->thetaIndex - thetaIndex) < (FM_ROUND2RAD * 4.0f / p_enc->thetaMechScaler)) ? (false) : (true);
        // Toggle index
        p_enc->indexToggle      = !p_enc->indexToggle;
        // Clear flag
        EQEP_clearInterruptStatus(eqepBase, EQEP_INT_INDEX_EVNT_LATCH);
    }
    // Compute new absolute angle
    p_enc->thetaAbsolute        = p_enc->thetaMech[NEW] + FM_ROUND2RAD * (float32_t)p_enc->turnNb;

    return;
}

/**
 * @brief           Control of the motor states
 * @param[inout]    p_motor Pointer on the associated motor structure
 * @param[in]       p_cmd   Pointer on the command structure driving motor
 */
inline bool_t MOT_runControl(motor_t* p_motor)
{
    uint8_t         id          = p_motor->motor_id;
    foc_t*          p_foc       = p_motor->p_motorFOC;
    error_reg_u*    p_err       = &p_motor->motor_error;
    encoder_t*      p_enc       = &p_foc->motor_enc;
    cmd_t*          p_cmd       = &p_foc->motor_cmd;
    cmd_reg_u       cmd_reg     = {.all = p_cmd->cmdArray.all};
    p_enc->indexOffset          = cmd_reg.encOffsetEnable;

    FOC_getMeasures(&p_foc->motor_acq);
    // Read encoder position
    ENC_getPosition(p_enc);
    // Estimate speed
    ENC_getSpeed(p_enc);
    // Rollover test & index
    ENC_getTheta(p_enc);

    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->epwmBaseAddr[CHANNEL_1]);
    p_err->drv_fault            = !GPIO_readPin(p_motor->drvFaultNum)                           || p_err->drv_fault;
    p_err->pos_rollover         = (p_enc->rollOverError && cmd_reg.rollOverEnable)              || p_err->pos_rollover;
    p_err->com_timeout          = ((p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef)|| p_err->com_timeout;
    p_err->enc_mismatch         = (p_enc->indexDetect && p_enc->indexError)                     || p_err->enc_mismatch;
    p_motor->motor_state        = (p_err->all)                      ? (MOTOR_STATE_ERROR)       : (p_motor->motor_state);

    switch(p_motor->motor_state)
    {
    case MOTOR_STATE_INIT:
    default:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (cmd_reg.motorEnable)             ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (cmd_reg.systemEnable)            ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ALIGN_UP:
        p_motor->itCnt          = 0U;
        p_foc->idRef           += (id == MOTOR_1)                   ? (MOTOR1_CURRENT_ALIGN_INC): (MOTOR2_CURRENT_ALIGN_INC); // Increment for current alignment procedure
        p_foc->iqRef            = 0.0f;
        p_foc->thetaAlign      -= (id == MOTOR_1)                   ? (MOTOR1_THETA_ALIGN_DEC)  : (MOTOR2_THETA_ALIGN_DEC); // Decrement for position alignment procedure
        p_motor->motor_state    = (p_foc->idRef < p_foc->iAlignMax) ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_ALIGN_FIX);
        p_motor->motor_state    = (cmd_reg.motorEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (cmd_reg.systemEnable)            ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        (p_motor->motor_state  != MOTOR_STATE_ALIGN_UP)             ? (ENC_resetStruct(p_enc))  : (p_enc->thetaElec = p_foc->thetaAlign);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ALIGN_FIX:
        p_motor->itCnt         += 1U; // wait 2s
        p_foc->idRef            = p_foc->iAlignMax; // Maintain alignment current
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (p_motor->itCnt < (2 * PWM_FREQ)) ? (MOTOR_STATE_ALIGN_FIX)   : (MOTOR_STATE_READY);
        p_motor->motor_state    = (cmd_reg.motorEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (cmd_reg.systemEnable)            ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPosVel);
        p_motor->motor_state    = (cmd_reg.motorEnable)             ? (MOTOR_STATE_READY)       : (MOTOR_STATE_STOP);
        p_motor->motor_state    = (cmd_reg.systemEnable)            ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (!cmd_reg.motorEnable)            ? (MOTOR_STATE_STOP)        : (MOTOR_STATE_READY);
        p_motor->motor_state    = (cmd_reg.systemEnable)            ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ERROR:
        p_motor->itCnt          = p_motor->itCnt;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = MOTOR_STATE_ERROR;
//        ENC_resetPeriph(p_enc);
//        FOC_resetStruct(p_foc);
//        FOC_runControl(p_foc);
        MOT_stopCommand(p_motor);
        break;
    }

    return(true);
}

#ifdef __cplusplus
}
#endif // extern "C"
#endif /* __CLA_SHARED_H__ */

