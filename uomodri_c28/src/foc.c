/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "foc.h"
#include "motor.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief           Read the phases current and phases \& bus voltages
 * @param[inout]    *p_acq  Pointer on the motor acquisition structure.
 */
inline void FOC_getMeasures(acq_t* p_acq)
{
    // Get the latest current values in Amps
    p_acq->ia       = (float32_t)(*p_acq->p_iaMeasReg) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ib       = (float32_t)(*p_acq->p_ibMeasReg) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ic       = (float32_t)(*p_acq->p_icMeasReg) * ADC_MOTOR_CURRENT_SCALE;
    // Get the latest phase voltage values in Volt
    p_acq->va       = (float32_t)(*p_acq->p_vaMeasReg) * ADC_MOTOR_VOLTAGE_SCALE;
    p_acq->vb       = (float32_t)(*p_acq->p_vbMeasReg) * ADC_MOTOR_VOLTAGE_SCALE;
    p_acq->vc       = (float32_t)(*p_acq->p_vcMeasReg) * ADC_MOTOR_VOLTAGE_SCALE;
    // Get the latest Voltage supply value in Volt (minimum is one to avoid dividing by zero later)
    float32_t vbus  = __fmax(((float32_t)(*p_acq->p_vBusMeasReg) * ADC_VBUS_VOLTAGE_SCALE), 1.0f);
    UOMODRI_VBUS_FLT(p_acq->vBusFlt, p_acq->vbus, vbus);
    float32_t vext  = (float32_t)(*p_acq->p_vExtMeasReg) * ADC_EXTERN_VOLTAGE_SCALE;
    UOMODRI_VEXT_FLT(p_acq->vExtFlt, p_acq->vExt, vext);

    return;
}

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
 * @brief       Compute sine and cosine from the electrical angle
 * @param[in]   *p_foc  Pointer on the FOC motor control structure.
 */
// Q-axis resistance estimation
// TODO Use VSEN results instead of Iq controller output
inline void FOC_resistanceEstimation(foc_t* p_foc)
{
    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(p_foc->iq) > FOC_RE_CURRENT_THRESHOLD)
        UOMODRI_RES_ESTIM_FLT(p_foc->resEstFlt, p_foc->resEst, (p_foc->uq - p_foc->motor_enc.speed.speedElec * p_foc->motor_cfg.ke) / p_foc->iq);
}

/**
 * @brief           Compute the Field Oriented Control (FOC) algorithm
 * @param[inout]    *p_foc  Pointer on the FOC motor control structure.
 */
inline void FOC_runControl(foc_t* p_foc)
{
    const params_t* p_cfg   = &p_foc->motor_cfg;
    acq_t*          p_acq   = &p_foc->motor_acq;
    encoder_t*      p_enc   = &p_foc->motor_enc;
    speed_t*        p_speed = &p_enc->speed;
    /* Compute sine and cosine from the electrical angle */
    p_foc->sinTheta         = __sinpuf32(p_enc->thetaElec);
    p_foc->cosTheta         = __cospuf32(p_enc->thetaElec);
    // Clarke transform
    p_foc->ialpha           = ((2.0f * p_acq->ia) - p_acq->ib - p_acq->ic) * FM_1DIV3;
    p_foc->ibeta            = (p_acq->ib - p_acq->ic) * FM_1DIVSQRT3;
    // Park transform + filtering
    float32_t id            = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
    UOMODRI_IDQ_FLT(p_foc->iParkFlt, p_foc->id, id);
    float32_t iq            = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);
    UOMODRI_IDQ_FLT(p_foc->iParkFlt, p_foc->iq, iq);
    // Compute decoupling feed-forward terms
    p_foc->piId.ff          = UOMODRI_FEED_FORWARD_ENABLE ? (-1.0f * p_speed->speedElec * p_cfg->Lq * p_foc->iq)        : (0.0);
    p_foc->piIq.ff          = UOMODRI_FEED_FORWARD_ENABLE ? (p_cfg->Rs * p_foc->iqRef + p_speed->speedElec * p_cfg->ke) : (0.0);
    // Compute Id(Flux Current) and Iq (Torque Current) PI controllers
    p_foc->ud               = FOC_runPI(&p_foc->piId);
    p_foc->uq               = FOC_runPI(&p_foc->piIq);
//    TODO : check if uq_max and v_vref are necessaries
//    float32_t uq_max    = __sqrt((p_foc->motor_acq.vmax * p_foc->motor_acq.vmax) - (p_foc->ud * p_foc->ud));
//    float32_t v_ref     = __sqrt((p_foc->ud * p_foc->ud) + (p_foc->uq * p_foc->uq));
//    limit_norm(&p_foc->ud, &p_foc->uq, p_foc->motor_acq.vmax);
    // Inverse Park transform
    p_foc->ualpha           = (p_foc->cosTheta * p_foc->ud) - (p_foc->sinTheta * p_foc->uq);
    p_foc->ubeta            = (p_foc->sinTheta * p_foc->ud) + (p_foc->cosTheta * p_foc->uq);
    // Inverse Clarke transform
    p_foc->ua               = p_foc->ualpha;
    p_foc->ub               =  0.5f * ((FM_SQRT3 * p_foc->ubeta) - p_foc->ualpha);
    p_foc->uc               = -0.5f * ((FM_SQRT3 * p_foc->ubeta) + p_foc->ualpha);
    // Compute middle voltage to center PWM duty cycles around it
    float32_t uOffset       = 0.5f * (FMIN3(p_foc->ua, p_foc->ub, p_foc->uc) + FMAX3(p_foc->ua, p_foc->ub, p_foc->uc));
    float32_t uMidPoint     = 0.5f * (p_foc->dtcMax + p_foc->dtcMin);
    // Compute duty cycles
    p_foc->dtc_u            = __fsat(((0.5f * (p_foc->ua - uOffset) / p_acq->vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_v            = __fsat(((0.5f * (p_foc->ub - uOffset) / p_acq->vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_w            = __fsat(((0.5f * (p_foc->uc - uOffset) / p_acq->vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);

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
    float32_t sat_min   = -0.5f * (*p_pi->p_sat);
    p_pi->err           = (*p_pi->p_set) - (*p_pi->p_fb);
    p_pi->integral     += p_pi->ki * p_pi->err;
    p_pi->integral      = __fsat(p_pi->integral, sat_max, sat_min);
    p_pi->out           = (p_pi->kp * p_pi->err) + p_pi->integral + p_pi->ff;
    p_pi->out           = __fsat(p_pi->out, sat_max, sat_min);

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
    float32_t sat_min   = -1.0f * (*p_pd->p_sat);
    p_pd->errTheta      = __mpy2pif32((*p_pd->p_setTheta) - (*p_pd->p_fbTheta));
    p_pd->errSpeed      = (*p_pd->p_setSpeed) - (*p_pd->p_fbSpeed);
    p_pd->derivative    = (*p_pd->p_kd) * p_pd->errSpeed;
    // Compute PID and clamp output.
    p_pd->out           = ((*p_pd->p_kp) * p_pd->errTheta) + p_pd->derivative + (*p_pd->p_ff);
    p_pd->out           = __fsat(p_pd->out, sat_max, sat_min);

    return(p_pd->out);
}

/**
 * @brief           Reset the FOC structure
 * @param[inout]    *p_foc   Pointer on the FOC structure to reset.
 */
void FOC_resetStruct(foc_t* p_foc)
{
    // Reset Id & Iq currents PI controllers
    FOC_resetPI(&p_foc->piId);
    FOC_resetPI(&p_foc->piIq);
    // Reset position PD controller
    FOC_resetPD(&p_foc->pdPos);
    // Reset encoder structure
    ENC_resetStruct(&p_foc->motor_enc);
    // Reset align structure
//    MOT_resetAlignStruct(&p_foc->motor_align);
    // Reset analog measures
    FOC_resetMeasuresStruct(&p_foc->motor_acq);
    // Reset currents and voltages
//    p_foc->motor_state[NEW] = (p_foc->motor_state[NEW] != MOTOR_ERROR) ? (MOTOR_STOP) : (MOTOR_ERROR);
//    p_foc->motor_state[OLD] = (p_foc->motor_state[NEW] != MOTOR_ERROR) ? (MOTOR_STOP) : (MOTOR_ERROR);
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
    p_foc->dtc_u            = 0.0f;
    p_foc->dtc_v            = 0.0f;
    p_foc->dtc_w            = 0.0f;
//    p_foc->idRef            = 0.0f;
//    p_foc->iqRef            = 0.0f;
    p_foc->resEst           = 0.0f;
    return;
}

void limit_norm(float32_t *x, float32_t *y, float32_t limit)
{
    /// Scales the length of vector (x, y) to be <= limit ///
    float32_t norm = __sqrtf((*x) * (*x) + (*y) * (*y));
    if(norm > limit)
    {
        *x *= limit / norm;
        *y *= limit / norm;
    }
}


//static inline void FOC_PID(PID_CONTROLLER* p_pid)
//{
//    p_pid->integral    += p_pid->ki * p_pid->err;
//    p_pid->derivative   = p_pid->kd * (p_pid->err - p_pid->errPrev) / p_pid->deltaT;
//    // Compute PID and clamp output.
//    p_pid->out          = (p_pid->kp * p_pid->err) + p_pid->integral + p_pid->derivative + p_pid->ff;
//    p_pid->out          = fminf(fmaxf(p_pid->out, -p_pid->sat), p_pid->sat);
//    // Save current error
//    p_pid->errPrev      = p_pid->err;
//}

