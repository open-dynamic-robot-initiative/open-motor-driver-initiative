#include "foc.h"
#include "math_defines.h"


//extern ComInterface comInterface;
//extern MOTOR_STRUCT motor[2];
//#include "f28x_project.h"

// Read current measures
//inline void FOC_getPosition(FOC* p_foc, MOTOR_HAL_CFG* p_motor)
//{
//    // Get latest position of the Quadrature Encoder
//    p_motor[p_motor->motorID].encoderPos = HWREG(p_motor->p_qep->eqepBase + EQEP_O_QPOSCNT);
//}

// Function definitions

/**
 * @brief           Read the phases phases current and the phases and bus voltages
 * @param[inout]    *p_acq  Pointer on the motor acquisition structure.
 */
inline void FOC_getMeasures(ACQ_STRUCT* p_acq)
{
    // Get the latest current values in Amps
    p_acq->ia           = (float32_t)(*p_acq->iaMeasReg) * ADC_CURRENT_SCALE;
    p_acq->ib           = (float32_t)(*p_acq->ibMeasReg) * ADC_CURRENT_SCALE;
    p_acq->ic           = (float32_t)(*p_acq->icMeasReg) * ADC_CURRENT_SCALE;
    // Get the latest phase voltage values in Volt
    p_acq->va           = (float32_t)(*p_acq->vaMeasReg) * ADC_VOLTAGE_SCALE;
    p_acq->vb           = (float32_t)(*p_acq->vbMeasReg) * ADC_VOLTAGE_SCALE;
    p_acq->vc           = (float32_t)(*p_acq->vcMeasReg) * ADC_VOLTAGE_SCALE;
    // Get the latest Voltage supply value in Volt (minimum is one to avoid dividing by zero later)
    UOMODRI_VBUS_FLT(p_acq->vBusLpf, p_acq->vbus, __fmax(((float32_t)(*p_acq->vBusMeasReg) * ADC_VOLTAGE_SCALE), 1.0f));
    p_acq->measNew      = true;
    return;
}

/**
 * @brief       Compute sine and cosine from the electrical angle
 * @param[in]   *p_foc  Pointer on the FOC motor control structure.
 */
// Q-axis resistance estimation
// TODO Use VSEN results instead of Iq controller output
inline void FOC_resistanceEstimation(FOC_STRUCT* p_foc)
{
    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(p_foc->iq) > FOC_RE_CURRENT_THRESHOLD)
        UOMODRI_RES_ESTIM_FLT(p_foc->resEstLpf, p_foc->resistanceEst, (p_foc->uq - p_foc->motor_enc.speedEst.speedElec * p_foc->motor_cfg.ke) / p_foc->iq);
}

/**
 * @brief           Compute the Field Oriented Control (FOC) algorithm
 * @param[inout]    *p_foc  Pointer on the FOC motor control structure.
 */
inline void FOC_runControl(FOC_STRUCT* p_foc)
{
    const PARAMS_STRUCT*    p_cfg   = &p_foc->motor_cfg;
    ACQ_STRUCT*             p_acq   = &p_foc->motor_acq;
    ENC_STRUCT*             p_enc   = &p_foc->motor_enc;
    SPEED_STRUCT*           p_speed = &p_enc->speedEst;

//    p_foc->vmax         = MOTOR12_OVERMODULATION * p_acq.vbus * (MOTOR1_DTC_MAX - MOTOR1_DTC_MIN) * FM_1DIVSQRT3;
//    p_foc->imax         = MOTOR12_IMAX;
    // Compute sine and cosine from the electrical angle
    p_foc->sinTheta     = __sinpuf32(p_enc->thetaElec);
    p_foc->cosTheta     = __cospuf32(p_enc->thetaElec);
    // Clarke transform
    p_foc->ialpha       = ((2.0f * p_acq->ia) - p_acq->ib - p_acq->ic) * FM_1DIV3;
    p_foc->ibeta        = (p_acq->ib - p_acq->ic) * FM_1DIVSQRT3;
    // Park transform + filtering
    UOMODRI_IDQ_FLT(p_foc->iParkLpf, p_foc->id, ((p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha)));
    UOMODRI_IDQ_FLT(p_foc->iParkLpf, p_foc->iq, ((p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha)));
    // Compute decoupling feed-forward terms
    p_foc->piId.ff      = UOMODRI_FEED_FORWARD_EN ? (-1.0f * p_speed->speedElec * p_cfg->Lq * p_foc->iq)        : (0.0);//* UOMODRI_FOC_FEED_FORWARD_EN;
    p_foc->piIq.ff      = UOMODRI_FEED_FORWARD_EN ? (p_cfg->Rs * p_foc->iqRef + p_speed->speedElec * p_cfg->ke) : (0.0);// * UOMODRI_FOC_FEED_FORWARD_EN;
//    p_foc->piIq.ff  = p_foc->motor_enc.speedEst.elecSpeed * (p_foc->motorParams.Ld * p_foc->id + p_foc->motorParams.ke);
//    limit_norm(&p_foc->idRef, &p_foc->iqRef, p_foc->motor_acq.imax);
    // Compute error for PI controllers
    // PI controller - Id(Flux Current Controller) & Iq (Torque Current Controller)
    p_foc->ud           = FOC_runPI(&p_foc->piId);
    p_foc->uq           = FOC_runPI(&p_foc->piIq);
//    float32_t uq_max    = __sqrt((p_foc->motor_acq.vmax * p_foc->motor_acq.vmax) - (p_foc->ud * p_foc->ud));
//    float32_t v_ref     = __sqrt((p_foc->ud * p_foc->ud) + (p_foc->uq * p_foc->uq));
//    limit_norm(&p_foc->ud, &p_foc->uq, p_foc->motor_acq.vmax);
    // Inverse Park transform
    p_foc->ualpha       = (p_foc->cosTheta * p_foc->ud) - (p_foc->sinTheta * p_foc->uq);
    p_foc->ubeta        = (p_foc->sinTheta * p_foc->ud) + (p_foc->cosTheta * p_foc->uq);
    // Inverse Clarke transform
    p_foc->ua           = p_foc->ualpha;
    p_foc->ub           =  0.5f * ((FM_SQRT3 * p_foc->ubeta) - p_foc->ualpha);
    p_foc->uc           = -0.5f * ((FM_SQRT3 * p_foc->ubeta) + p_foc->ualpha);
    // Compute middle voltage to center PWM duty cycles around it
    float32_t uOffset   = 0.5f * (FMIN3(p_foc->ua, p_foc->ub, p_foc->uc) + FMAX3(p_foc->ua, p_foc->ub, p_foc->uc));
    float32_t uMidPoint = 0.5f * (p_foc->dtcMax + p_foc->dtcMin);
    // Compute duty cycles
    p_foc->dtc_u        = __fsat(((0.5f * (p_foc->ua - uOffset) / p_acq->vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_v        = __fsat(((0.5f * (p_foc->ub - uOffset) / p_acq->vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_w        = __fsat(((0.5f * (p_foc->uc - uOffset) / p_acq->vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);

    return;
}

/**
 * @brief           Reset the Proportional Integral (PI) Controller structure
 * @param[inout]    *p_pi   Pointer on the PI control structure.
 */
inline void FOC_resetPI(PI_STRUCT* p_pi)
{
    p_pi->err           = 0.0;
    p_pi->integral      = 0.0;
    p_pi->ff            = 0.0;
    p_pi->out           = 0.0;

    return;
}

/**
 * @brief           Run the Proportional Integral Controller (PI)
 * @param[inout]    *p_pi   Pointer on the PI control structure.
 * @return          out     Command of PI controller
 */
inline float32_t FOC_runPI(PI_STRUCT* p_pi)
{
    float32_t   sat_max = 0.5f  * (*p_pi->sat);
    float32_t   sat_min = -0.5f * (*p_pi->sat);

    p_pi->err           = *p_pi->set - *p_pi->in;
    p_pi->integral     += p_pi->ki * p_pi->err;
    //p_pid->integral     = FMINMAX(p_pid->integral, *p_pid->sat, -(*p_pid->sat)); //fmaxf(fminf(p_pid->integral, *p_pid->sat), -(*p_pid->sat));
    // Compute PID and clamp output.
    p_pi->out           = (p_pi->kp * p_pi->err) + p_pi->integral + p_pi->ff;
    p_pi->out           = __fsat(p_pi->out, sat_max, sat_min);//FMINMAX(p_pi->out, (0.4f * (*p_pi->sat)), (-0.4f * (*p_pi->sat))); //fmaxf(fminf(p_pid->out, *p_pid->sat), -(*p_pid->sat));

    return(p_pi->out);
}

/**
 * @brief           Reset the Proportional Derivative (PD) Controller structure
 * @param[inout]    *p_pd   Pointer on the PD control structure.
 */
inline void FOC_resetPD(PD_STRUCT* p_pd)
{
    p_pd->errTheta      = 0.0;
    p_pd->errSpeed      = 0.0;
    p_pd->derivative    = 0.0;
    p_pd->ff            = 0.0;
    p_pd->out           = 0.0;

    return;
}

/**
 * @brief           Run the Proportional Derivative (PD) Controller
 * @param[inout]    *p_pd   Pointer on the PD control structure.
 * @return          out     Command of PD controller
 */
inline float32_t FOC_runPD(PD_STRUCT* p_pd)
{
    float32_t   sat_max = 1.0f  * (*p_pd->sat);
    float32_t   sat_min = -1.0f * (*p_pd->sat);

    p_pd->errTheta      = __mpy2pif32(*p_pd->setTheta - *p_pd->inTheta);
    p_pd->errSpeed      = *p_pd->setSpeed - *p_pd->inSpeed;
    p_pd->derivative    = p_pd->kd * p_pd->errSpeed;//(p_pid->err - p_pid->errPrev) / p_pid->deltaT;
    // Compute PID and clamp output.
    p_pd->out           = (p_pd->kp * p_pd->errTheta) + p_pd->derivative + p_pd->ff;
    p_pd->out           = __fsat(p_pd->out, sat_max, sat_min);//FMINMAX(p_pd->out, *p_pd->sat, -(*p_pd->sat)); //fmaxf(fminf(p_pid->out, *p_pid->sat), -(*p_pid->sat));
    // Save current error
    //p_pid->errPrev      = p_pid->err;
    return(p_pd->out);
}

/**
 * @brief           Reset the FOC structure
 * @param[inout]    *p_foc   Pointer on the FOC structure to reset.
 */
void FOC_resetStruct(FOC_STRUCT* p_foc)
{
    // Reset Id & Iq currents PI controllers
    FOC_resetPI(&p_foc->piId);
    FOC_resetPI(&p_foc->piIq);
    // Reset position PD controller
    FOC_resetPD(&p_foc->pdPos);
    // Reset encoder structure
    ENC_resetStruct(&p_foc->motor_enc);

    // Reset currents and voltages
    p_foc->sinTheta     = 0.0;
    p_foc->cosTheta     = 0.0;

    p_foc->ialpha       = 0.0;
    p_foc->ibeta        = 0.0;

    p_foc->id           = 0.0;
    p_foc->iq           = 0.0;

    p_foc->ud           = 0.0;
    p_foc->uq           = 0.0;

    p_foc->ualpha       = 0.0;
    p_foc->ubeta        = 0.0;

    p_foc->ua           = 0.0;
    p_foc->ub           = 0.0;
    p_foc->uc           = 0.0;

    p_foc->dtc_u        = 0.0;
    p_foc->dtc_v        = 0.0;
    p_foc->dtc_w        = 0.0;

    // Reset resistance estimation
    p_foc->resistanceEst= 0.0;

    p_foc->enabled      = false;

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

