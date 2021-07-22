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
 * @brief       Read the phases voltage, phases current and bus voltage
 * @param[in]   *p_acq  Pointer on the motor acquisition structure.
 */
inline void FOC_getMeasures(MOTOR_ACQ* p_acq)
{
    // Get the latest current values in Amps
    p_acq->ia       = (float32_t)(*p_acq->iaMeasReg) * ADC_CURRENT_SCALE;
    p_acq->ib       = (float32_t)(*p_acq->ibMeasReg) * ADC_CURRENT_SCALE;
    p_acq->ic       = (float32_t)(*p_acq->icMeasReg) * ADC_CURRENT_SCALE;
    // Get the latest phase voltage values in Volt
    p_acq->va       = (float32_t)(*p_acq->vaMeasReg) * ADC_VOLTAGE_SCALE;
    p_acq->vb       = (float32_t)(*p_acq->vbMeasReg) * ADC_VOLTAGE_SCALE;
    p_acq->vc       = (float32_t)(*p_acq->vcMeasReg) * ADC_VOLTAGE_SCALE;
    // Get the latest Voltage supply value in Volt (minimum is one to avoid dividing by zero later)
    float32_t vbus  = __fmax(((float32_t)(*p_acq->vBusMeasReg) * ADC_VOLTAGE_SCALE), 1.0f);
#if (UOMODRI_VBUS_FLT == 1)
        LPF_FILTER_1(p_acq->vBusLpf, p_acq->vbus, vbus);
#elif (UOMODRI_VBUS_FLT == 2)
        LPF_FILTER_2(p_acq->vBusLpf, p_acq->vbus, vbus);
#else
        p_acq->vbus = vbus;
#endif
        p_acq->vmax = MOTOR12_OVERMODULATION * p_acq->vbus * (MOTOR1_DUTY_CYCLE_MAX - MOTOR1_DUTY_CYCLE_MIN) * FM_1DIVSQRT3;
//    p_acq->measNew  = true;
}

/**
 * @brief       Compute sine and cosine from the electrical angle
 * @param[in]   *p_foc  Pointer on the FOC motor control structure.
 */
//inline void FOC_sineCosine(FOC* p_foc)
//{
//    p_foc->sinTheta = __sinpuf32(p_foc->motor_enc.elecPos);
//    p_foc->cosTheta = __cospuf32(p_foc->motor_enc.elecPos);
//}

// Q-axis resistance estimation
// TODO Use VSEN results instead of Iq controller output
inline void FOC_resistanceEstimation(FOC* p_foc)
{
    // Read measures
    //p_foc->phaseAVolt = (*p_foc->vsenaResult) * p_foc->voltageScalerReg;
    //p_foc->phaseBVolt = (*p_foc->vsenbResult) * p_foc->voltageScalerReg;
    //p_foc->phaseCVolt = (*p_foc->vsencResult) * p_foc->voltageScalerReg;

    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(p_foc->iq) > FOC_RE_CURRENT_THRESHOLD)
    {
#if (UOMODRI_RES_ESTIM_FLT == 1)
        LPF_FILTER_1(p_foc->resEstLpf, p_foc->resistanceEst, (p_foc->uq - p_foc->motor_enc.speedEst.speedElec * p_foc->motor_cfg.ke) / p_foc->iq);
#elif (UOMODRI_RES_ESTIM_FLT == 2)
        LPF_FILTER_2(p_foc->resEstLpf, p_foc->resistanceEst, (p_foc->uq - p_foc->motor_enc.speedEst.speedElec * p_foc->motor_cfg.ke) / p_foc->iq);
#endif
    }
}

/**
 * @brief       Compute the Field Oriented Control (FOC) algorithm
 * @param[in]   *p_foc  Pointer on the FOC motor control structure.
 */
inline void FOC_control(FOC* p_foc)
{
    // Clarke transform
    p_foc->ialpha       = ((2.0f * p_foc->motor_acq.ia) - p_foc->motor_acq.ib - p_foc->motor_acq.ic) * FM_1DIV3;
    p_foc->ibeta        = (p_foc->motor_acq.ib - p_foc->motor_acq.ic) * FM_1DIVSQRT3;
    // Compute sine and cosine from the electrical angle
    p_foc->sinTheta     = __sinpuf32(p_foc->motor_enc.thetaElec);
    p_foc->cosTheta     = __cospuf32(p_foc->motor_enc.thetaElec);
#if (UOMODRI_IDQ_FLT == 1)
    // Park transform
    LPF_FILTER_1(p_foc->iParkLpf, p_foc->id, ((p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha)));
    LPF_FILTER_1(p_foc->iParkLpf, p_foc->iq, ((p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha)));
#elif (UOMODRI_IDQ_FLT == 2)
    // Park transform
    LPF_FILTER_2(p_foc->iParkLpf, p_foc->id, ((p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha)));
    LPF_FILTER_2(p_foc->iParkLpf, p_foc->iq, ((p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha)));
#else
    // Park transform
    p_foc->id       = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
    p_foc->iq       = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);
#endif
#if (UOMODRI_FOC_FEED_FORWARD == 1)
    // Compute decoupling feed-forward terms
    p_foc->piId.ff      = -1.0 * p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.Lq * p_foc->iq;
    p_foc->piIq.ff      = p_foc->motorParams.Rs * p_foc->iqRef + p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.ke;
    //    p_foc->piIq.ff  = p_foc->motor_enc.speedEst.elecSpeed * (p_foc->motorParams.Ld * p_foc->id + p_foc->motorParams.ke);
#else
    p_foc->piId.ff  = 0.0;
    p_foc->piIq.ff  = 0.0;
#endif
    //
//    limit_norm(&p_foc->idRef, &p_foc->iqRef, p_foc->motor_acq.imax);
    // Compute error for PI controllers
//    p_foc->piId.err     = p_foc->idRef - p_foc->id;
//    p_foc->piIq.err     = p_foc->iqRef - p_foc->iq;
    // PI controller - Id(Flux Current Controller) & Iq (Torque Current Controller)
    p_foc->ud           = FOC_pi(&p_foc->piId);
    p_foc->uq           = FOC_pi(&p_foc->piIq);
//    float32_t uq_max    = sqrtf((p_foc->motor_acq.vmax * p_foc->motor_acq.vmax) - (p_foc->ud * p_foc->ud));
//    float32_t v_ref     = sqrtf((p_foc->ud * p_foc->ud) + (p_foc->uq * p_foc->uq));
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
//    float32_t uMidPoint = 0.5f * (MOTOR1_DUTY_CYCLE_MAX - MOTOR1_DUTY_CYCLE_MIN);
    // Compute duty cycles
    p_foc->dtc_u        = __fsat((((p_foc->ua - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);//FMINMAX(((p_foc->ua - uOffset) / p_foc->motor_acq.vbus), p_foc->dtcMax, p_foc->dtcMin) + 0.5f;
    p_foc->dtc_v        = __fsat((((p_foc->ub - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_w        = __fsat((((p_foc->uc - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);

//    p_foc->dtcPhaseA = __fmin(__fmax((p_foc->ua - uOffset) / p_foc->motor_acq.vbus, p_foc->dtcMin), p_foc->dtcMax);
//    p_foc->dtcPhaseB = __fmin(__fmax((p_foc->ub - uOffset) / p_foc->motor_acq.vbus, p_foc->dtcMin), p_foc->dtcMax);
//    p_foc->dtcPhaseC = __fmin(__fmax((p_foc->uc - uOffset) / p_foc->motor_acq.vbus, p_foc->dtcMin), p_foc->dtcMax);

//    p_foc->dtcPhaseA    = FMINMAX((((p_foc->ua - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);
////    p_foc->ducya = fminf(fmaxf((p_foc->ua - uOffset) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//    p_foc->dtcPhaseB    = FMINMAX((((p_foc->ub - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);
////    p_foc->ducyb = fminf(fmaxf((p_foc->ub - uOffset) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//    p_foc->dtcPhaseC    = FMINMAX((((p_foc->uc - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);
//    p_foc->ducyc = fminf(fmaxf((p_foc->uc - uOffset) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    // Compute compare registers:
//    *(p_foc->pwmCompA)  = p_foc->dtcPhaseA * PWM_HALF_TBPRD;
//    *(p_foc->pwmCompB)  = p_foc->dtcPhaseB * PWM_HALF_TBPRD;
//    *(p_foc->pwmCompC)  = p_foc->dtcPhaseC * PWM_HALF_TBPRD;

//    *(p_foc->pwmCompA) = fminf(fmaxf((.5f * (p_foc->ua - uOffset) * MOTOR12_OVERMODULATION / p_foc->motor_acq.vmax + uMidPoint), MOTOR1_DUTY_CYCLE_MIN), MOTOR1_DUTY_CYCLE_MAX);
//    *(p_foc->pwmCompB) = fminf(fmaxf((.5f * (p_foc->ub - uOffset) * MOTOR12_OVERMODULATION / p_foc->motor_acq.vmax + uMidPoint), MOTOR1_DUTY_CYCLE_MIN), MOTOR1_DUTY_CYCLE_MAX);
//    *(p_foc->pwmCompC) = fminf(fmaxf((.5f * (p_foc->uc - uOffset) * MOTOR12_OVERMODULATION / p_foc->motor_acq.vmax + uMidPoint), MOTOR1_DUTY_CYCLE_MIN), MOTOR1_DUTY_CYCLE_MAX);

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

inline float32_t FOC_pi(PI_CONTROLLER* p_pi)
{
    float32_t sat_max   = 0.5f  * (*p_pi->sat);
    float32_t sat_min   = -0.5f * (*p_pi->sat);

    p_pi->err           = p_pi->set - p_pi->in;
    p_pi->integral     += p_pi->ki * p_pi->err;
    //p_pid->integral     = FMINMAX(p_pid->integral, *p_pid->sat, -(*p_pid->sat)); //fmaxf(fminf(p_pid->integral, *p_pid->sat), -(*p_pid->sat));
    // Compute PID and clamp output.
    p_pi->out           = (p_pi->kp * p_pi->err) + p_pi->integral + p_pi->ff;
    p_pi->out           = __fsat(p_pi->out, sat_max, sat_min);//FMINMAX(p_pi->out, (0.4f * (*p_pi->sat)), (-0.4f * (*p_pi->sat))); //fmaxf(fminf(p_pid->out, *p_pid->sat), -(*p_pid->sat));
    return(p_pi->out);
}

inline float32_t FOC_pd(PD_CONTROLLER* p_pd)
{
    float32_t sat_max   = 1.0f  * (*p_pd->sat);
    float32_t sat_min   = -1.0f * (*p_pd->sat);

    p_pd->errTheta      = __mpy2pif32(p_pd->setTheta - p_pd->inTheta);
    p_pd->errSpeed      = p_pd->setSpeed - p_pd->inSpeed;
    p_pd->derivative    = p_pd->kd * p_pd->errSpeed;//(p_pid->err - p_pid->errPrev) / p_pid->deltaT;
    // Compute PID and clamp output.
    p_pd->out           = (p_pd->kp * p_pd->errTheta) + p_pd->derivative + p_pd->ff;
    p_pd->out           = __fsat(p_pd->out, sat_max, sat_min);//FMINMAX(p_pd->out, *p_pd->sat, -(*p_pd->sat)); //fmaxf(fminf(p_pid->out, *p_pid->sat), -(*p_pid->sat));
    // Save current error
    //p_pid->errPrev      = p_pid->err;
    return(p_pd->out);
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

void FOC_resetStruct(FOC* p_foc)
{
    // Reset controllers
    p_foc->piId.err                         = 0.0;
//    p_foc->piId.kp              = MOTOR1_ID_REGULATOR_KP_COEF;
//    p_foc->piId.ki              = MOTOR1_ID_REGULATOR_KI_COEF;
//    p_foc->piId.kd              = 0.0f
    p_foc->piId.integral                    = 0.0;
    p_foc->piId.ff                          = 0.0;
    p_foc->piId.out                         = 0.0;
//    p_foc->piId.sat             = ((float32_t *)(&p_foc->motor_acq.vmax));

    p_foc->piIq.err                         = 0.0;
//    p_foc->piIq.kp              = MOTOR1_IQ_REGULATOR_KP_COEF;
//    p_foc->piIq.ki              = MOTOR1_IQ_REGULATOR_KI_COEF;
//    p_foc->piIq.kd              = 0.0f
    p_foc->piIq.integral                    = 0.0;
    p_foc->piIq.ff                          = 0.0;
    p_foc->piIq.out                         = 0.0;
//    p_foc->piIq.sat             = ((float32_t *)(&p_foc->motor_acq.vmax));

    // Reset position PD controller
    p_foc->pdPos.errTheta                   = 0.0;
    p_foc->pdPos.errSpeed                   = 0.0;
//    p_foc->pdPos.kp             = MOTOR1_SPEED_REGULATOR_KP_COEF;
//    p_foc->pdPos.ki             = 0.0;
//    p_foc->pdPos.kd             = MOTOR1_SPEED_REGULATOR_KD_COEF;
//    p_foc->pdPos.integral       = 0.0;
    p_foc->pdPos.derivative                 = 0.0;
    p_foc->pdPos.ff                         = 0.0;
    p_foc->pdPos.out                        = 0.0;

    // Reset encoder
    p_foc->motor_enc.thetaMech[NEW]         = 0.0;
    p_foc->motor_enc.thetaMech[OLD]         = 0.0;
    p_foc->motor_enc.thetaElec              = 0.0;
    p_foc->motor_enc.thetaDir               = false;
//    p_foc->motor_enc.thetaNew               = false;

    p_foc->motor_enc.indexCompensated       = false;
    p_foc->motor_enc.indexTheta             = 0.0;
    p_foc->motor_enc.indexedThetaMech       = 0.0;

    p_foc->motor_enc.thetaAbsolute          = 0.0;

    p_foc->motor_enc.speedEst.alpha         = 0.0;
    p_foc->motor_enc.speedEst.speedRef      = 0.0;
    p_foc->motor_enc.speedEst.theta[NEW]    = 0.0;
    p_foc->motor_enc.speedEst.theta[OLD]    = 0.0;
    p_foc->motor_enc.speedEst.speedLow      = 0.0;
    p_foc->motor_enc.speedEst.speedHigh     = 0.0;
    p_foc->motor_enc.speedEst.speedMech     = 0.0;
    p_foc->motor_enc.speedEst.speedElec     = 0.0;

    p_foc->motor_enc.turnNb                 = 0;
    p_foc->motor_enc.indexDetected          = false;
    p_foc->motor_enc.indexToggle            = false;
    p_foc->motor_enc.rollOverFlag           = false;

    // Reset currents and voltages
    p_foc->ialpha                           = 0.0;
    p_foc->ibeta                            = 0.0;

    p_foc->id                               = 0.0;
    p_foc->iq                               = 0.0;

    p_foc->ud                               = 0.0;
    p_foc->uq                               = 0.0;

    p_foc->ualpha                           = 0.0;
    p_foc->ubeta                            = 0.0;

    p_foc->ua                               = 0.0;
    p_foc->ub                               = 0.0;
    p_foc->uc                               = 0.0;

    // Reset resistance estimation
//    p_foc->phaseAVolt                       = 0.0;
//    p_foc->phaseBVolt                       = 0.0;
//    p_foc->phaseCVolt                       = 0.0;

    p_foc->resistanceEst                    = 0.0;
}

inline FOC_sineCosine(FOC* p_foc)
{
    // Compute sine and cosine from the electrical angle
    p_foc->sinTheta     = __sinpuf32(p_foc->motor_enc.thetaElec);
    p_foc->cosTheta     = __cospuf32(p_foc->motor_enc.thetaElec);

    return;
}

inline void FOC_clarke(FOC* p_foc)
{
    MOTOR_ACQ* p_acq    = &p_foc->motor_acq;
    // Clarke transform
    p_foc->ialpha       = ((2.0f * p_acq->ia) - p_acq->ib - p_acq->ic) * FM_1DIV3; //((2.0f * p_foc->motor_acq.ia) - p_foc->motor_acq.ib - p_foc->motor_acq.ic) * FM_1DIV3;
    p_foc->ibeta        = (p_acq->ib - p_acq->ic) * FM_1DIVSQRT3; //(p_foc->motor_acq.ib - p_foc->motor_acq.ic) * FM_1DIVSQRT3;

    return;
}

inline FOC_park(FOC* p_foc)
{
    float32_t id        = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
    float32_t iq        = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);

#if (UOMODRI_IDQ_FLT == 1)
    // Park transform
    LPF_FILTER_1(p_foc->iParkLpf, p_foc->id, id);
    LPF_FILTER_1(p_foc->iParkLpf, p_foc->iq, iq);
#elif (UOMODRI_IDQ_FLT == 2)
    // Park transform
    LPF_FILTER_2(p_foc->iParkLpf, p_foc->id, id);
    LPF_FILTER_2(p_foc->iParkLpf, p_foc->iq, iq);
#else
    // Park transform
    p_foc->id           = id;
    p_foc->iq           = iq;
#endif

    return;
}

inline FOC_feedForward(FOC* p_foc)
{
#if (UOMODRI_FOC_FEED_FORWARD == 1)
    // Compute decoupling feed-forward terms
    p_foc->piId.ff      = -1.0 * p_foc->motor_enc.speedEst.speedElec * p_foc->motorParams.Lq * p_foc->iq;
    p_foc->piIq.ff      = p_foc->motorParams.Rs * p_foc->iqRef + p_foc->motor_enc.speedEst.speedElec * p_foc->motorParams.ke;
    //    p_foc->piIq.ff  = p_foc->motor_enc.speedEst.elecSpeed * (p_foc->motorParams.Ld * p_foc->id + p_foc->motorParams.ke);
#else
    p_foc->piId.ff  = 0.0;
    p_foc->piIq.ff  = 0.0;
#endif
}
    //
//    limit_norm(&p_foc->idRef, &p_foc->iqRef, p_foc->motor_acq.imax);
    // Compute error for PI controllers

//    p_foc->piId.err     = p_foc->idRef - p_foc->id;
//    p_foc->piIq.err     = p_foc->iqRef - p_foc->iq;

    // PI controller - Id(Flux Current Controller) & Iq (Torque Current Controller)

//    p_foc->ud           = FOC_pi(&p_foc->piId);
//    p_foc->uq           = FOC_pi(&p_foc->piIq);

//    float32_t uq_max    = sqrtf((p_foc->motor_acq.vmax * p_foc->motor_acq.vmax) - (p_foc->ud * p_foc->ud));
//    float32_t v_ref     = sqrtf((p_foc->ud * p_foc->ud) + (p_foc->uq * p_foc->uq));
//    limit_norm(&p_foc->ud, &p_foc->uq, p_foc->motor_acq.vmax);

inline FOC_parkInv(FOC* p_foc)
{
    // Inverse Park transform
    p_foc->ualpha       = (p_foc->cosTheta * p_foc->ud) - (p_foc->sinTheta * p_foc->uq);
    p_foc->ubeta        = (p_foc->sinTheta * p_foc->ud) + (p_foc->cosTheta * p_foc->uq);

    return;
}

inline FOC_clarkeInv(FOC* p_foc)
{
    // Inverse Clarke transform
    p_foc->ua           = p_foc->ualpha;
    p_foc->ub           =  0.5f * ((FM_SQRT3 * p_foc->ubeta) - p_foc->ualpha);
    p_foc->uc           = -0.5f * ((FM_SQRT3 * p_foc->ubeta) + p_foc->ualpha);

    return;
}

inline FOC_clarke2pwm(FOC* p_foc)
{
    // Compute middle voltage to center PWM duty cycles around it
    float32_t uOffset   = 0.5f * (FMIN3(p_foc->ua, p_foc->ub, p_foc->uc) + FMAX3(p_foc->ua, p_foc->ub, p_foc->uc));
    //    float32_t uMidPoint = 0.5f * (MOTOR1_DUTY_CYCLE_MAX - MOTOR1_DUTY_CYCLE_MIN);
    // Compute duty cycles
    p_foc->dtc_u        = __fsat((((p_foc->ua - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_v        = __fsat((((p_foc->ub - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_w        = __fsat((((p_foc->uc - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);

    return;
}

//    p_foc->dtcPhaseA = __fmin(__fmax((p_foc->ua - uOffset) / p_foc->motor_acq.vbus, p_foc->dtcMin), p_foc->dtcMax);
//    p_foc->dtcPhaseB = __fmin(__fmax((p_foc->ub - uOffset) / p_foc->motor_acq.vbus, p_foc->dtcMin), p_foc->dtcMax);
//    p_foc->dtcPhaseC = __fmin(__fmax((p_foc->uc - uOffset) / p_foc->motor_acq.vbus, p_foc->dtcMin), p_foc->dtcMax);

//    p_foc->dtcPhaseA    = FMINMAX((((p_foc->ua - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);
////    p_foc->ducya = fminf(fmaxf((p_foc->ua - uOffset) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//    p_foc->dtcPhaseB    = FMINMAX((((p_foc->ub - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);
////    p_foc->ducyb = fminf(fmaxf((p_foc->ub - uOffset) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//    p_foc->dtcPhaseC    = FMINMAX((((p_foc->uc - uOffset) / p_foc->motor_acq.vbus) + 0.5f), p_foc->dtcMax, p_foc->dtcMin);
//    p_foc->ducyc = fminf(fmaxf((p_foc->uc - uOffset) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    // Compute compare registers:
//    *(p_foc->pwmCompA)  = p_foc->dtcPhaseA * PWM_HALF_TBPRD;
//    *(p_foc->pwmCompB)  = p_foc->dtcPhaseB * PWM_HALF_TBPRD;
//    *(p_foc->pwmCompC)  = p_foc->dtcPhaseC * PWM_HALF_TBPRD;

//    *(p_foc->pwmCompA) = fminf(fmaxf((.5f * (p_foc->ua - uOffset) * MOTOR12_OVERMODULATION / p_foc->motor_acq.vmax + uMidPoint), MOTOR1_DUTY_CYCLE_MIN), MOTOR1_DUTY_CYCLE_MAX);
//    *(p_foc->pwmCompB) = fminf(fmaxf((.5f * (p_foc->ub - uOffset) * MOTOR12_OVERMODULATION / p_foc->motor_acq.vmax + uMidPoint), MOTOR1_DUTY_CYCLE_MIN), MOTOR1_DUTY_CYCLE_MAX);
//    *(p_foc->pwmCompC) = fminf(fmaxf((.5f * (p_foc->uc - uOffset) * MOTOR12_OVERMODULATION / p_foc->motor_acq.vmax + uMidPoint), MOTOR1_DUTY_CYCLE_MIN), MOTOR1_DUTY_CYCLE_MAX);


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

