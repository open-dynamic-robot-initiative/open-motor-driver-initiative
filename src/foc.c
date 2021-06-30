#include "foc.h"
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
 * @param[in]   vbus_en Enable or disable the voltage bus acquisition.
 */
inline void FOC_getMeasures(MOTOR_ACQ* p_acq, bool vbus_en)
{
    // Get the latest current values in Amps
    p_acq->ia       = (float32_t)(*p_acq->iaMeasReg)    * ADC_CURRENT_SCALE;
    p_acq->ib       = (float32_t)(*p_acq->ibMeasReg)    * ADC_CURRENT_SCALE;
    p_acq->ic       = (float32_t)(*p_acq->icMeasReg)    * ADC_CURRENT_SCALE;
    // Get the latest phase voltage values in Volt
    p_acq->va       = (float32_t)(*p_acq->vaMeasReg)    * ADC_VOLTAGE_SCALE;
    p_acq->vb       = (float32_t)(*p_acq->vbMeasReg)    * ADC_VOLTAGE_SCALE;
    p_acq->vc       = (float32_t)(*p_acq->vcMeasReg)    * ADC_VOLTAGE_SCALE;
    // Get the latest Voltage supply value in Volt (minimum is one to avoid dividing by zero later)
    p_acq->vbus     = (vbus_en) ? (fmaxf(((float32_t)(*p_acq->vBusMeasReg) * ADC_VOLTAGE_SCALE), 1.0f)) : (p_acq->vbus);
    p_acq->measNew  = true;
}

/**
 * @brief       Compute sine and cosine from the electrical angle
 * @param[in]   *p_foc  Pointer on the FOC motor control structure.
 */
inline void FOC_sineCosine(FOC* p_foc)
{
    p_foc->sinTheta = __sinpuf32(p_foc->motor_enc.elecPos);
    p_foc->cosTheta = __cospuf32(p_foc->motor_enc.elecPos);
}

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
#if (UOMODRI_FLT_FORM == 1)
        LPF_FILTER(p_foc->resEstLpf, p_foc->resistanceEst, (p_foc->uq - p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.ke) / p_foc->iq);
#elif (UOMODRI_FLT_FORM == 2)
        LPF_FILTER(p_foc->resEstLpf.a, p_foc->resistanceEst, (p_foc->uq - p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.ke) / p_foc->iq);
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
    p_foc->ialpha   = ((2.0f * p_foc->motor_acq.ia) - (p_foc->motor_acq.ib + p_foc->motor_acq.ic)) * FM_1DIV3;
    p_foc->ibeta    = (p_foc->motor_acq.ib - p_foc->motor_acq.ic) * FM_1DIVSQRT3;
#if (UOMODRI_FLT_FORM == 1)
    // Park transform
    float32_t id    = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
    float32_t iq    = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);
    LPF_FILTER(p_foc->iParkLpf, p_foc->id, id);
    LPF_FILTER(p_foc->iParkLpf, p_foc->iq, iq);
#elif (UOMODRI_FLT_FORM == 2)
    // Park transform
    float32_t id    = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
    float32_t iq    = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);
    LPF_FILTER(p_foc->iParkLpf.a, p_foc->id, id);
    LPF_FILTER(p_foc->iParkLpf.a, p_foc->iq, iq);
#elif (UOMODRI_FLT_FORM == 3)
    // Park transform
    p_foc->id       = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
    p_foc->iq       = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);
#endif
#if (UOMODRI_FOC_FEED_FORWARD == 1)
    // Compute decoupling feed-forward terms
    p_foc->piId.ff  = -1.0 * p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.Lq * p_foc->iq;
    p_foc->piIq.ff  = p_foc->motorParams.Rs * p_foc->iqRef + p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.ke; //    p_foc->piIq.ff  = p_foc->motor_enc.speedEst.elecSpeed * (p_foc->motorParams.Ld * p_foc->id + p_foc->motorParams.ke);
#else
    p_foc->piId.ff  = 0.0;
    p_foc->piIq.ff  = 0.0;
#endif
    // Compute error for PI controllers
    p_foc->piId.err = p_foc->idRef - p_foc->id;
    p_foc->piIq.err = p_foc->iqRef - p_foc->iq;
    // PI controller - Id(Flux Current Controller) & Iq (Torque Current Controller)
    FOC_PI(&p_foc->piId);
    FOC_PI(&p_foc->piIq);
    // Copy output variables
    p_foc->ud = p_foc->piId.out;
    p_foc->uq = p_foc->piIq.out;
    // Inverse Park transform
    p_foc->ualpha = p_foc->cosTheta * p_foc->ud - p_foc->sinTheta * p_foc->uq;
    p_foc->ubeta  = p_foc->sinTheta * p_foc->ud + p_foc->cosTheta * p_foc->uq;
    // Inverse Clarke transform
    p_foc->ua = p_foc->ualpha;
    p_foc->ub =  0.5f * (FM_SQRT3 * p_foc->ubeta - p_foc->ualpha);
    p_foc->uc = -0.5f * (FM_SQRT3 * p_foc->ubeta + p_foc->ualpha);
    // Compute middle voltage to center PWM duty cycles around it
    float32_t uMid = 0.5f * (fminf(fminf(p_foc->ua, p_foc->ub), p_foc->uc) + fmaxf(fmaxf(p_foc->ua, p_foc->ub), p_foc->uc));
    // Compute duty cycles
    p_foc->ducya = fminf(fmaxf((p_foc->ua - uMid) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    p_foc->ducyb = fminf(fmaxf((p_foc->ub - uMid) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    p_foc->ducyc = fminf(fmaxf((p_foc->uc - uMid) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    // Compute compare registers:
    *(p_foc->pwmCompA) = (p_foc->ducya + 0.5f) * PWM_HALF_TBPRD * 0x10000L;
    *(p_foc->pwmCompB) = (p_foc->ducyb + 0.5f) * PWM_HALF_TBPRD * 0x10000L;
    *(p_foc->pwmCompC) = (p_foc->ducyc + 0.5f) * PWM_HALF_TBPRD * 0x10000L;
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

static inline void FOC_PI(PID_CONTROLLER* p_pid)
{
    p_pid->integral    += p_pid->ki * p_pid->err;
    p_pid->integral     = fmaxf(fminf(p_pid->integral, p_pid->sat), -p_pid->sat);
    // Compute PID and clamp output.
    p_pid->out          = (p_pid->kp * p_pid->err) + p_pid->integral + p_pid->ff;
    p_pid->out          = fmaxf(fminf(p_pid->out, p_pid->sat), -p_pid->sat);//fminf(fmaxf(p_pid->out, -p_pid->sat), p_pid->sat);
}

static inline void FOC_PD(PID_CONTROLLER* p_pid)
{
    p_pid->derivative   = p_pid->kd * (p_pid->err - p_pid->errPrev) / p_pid->deltaT;
    // Compute PID and clamp output.
    p_pid->out          = (p_pid->kp * p_pid->err) + p_pid->derivative + p_pid->ff;
    p_pid->out          = fminf(fmaxf(p_pid->out, -p_pid->sat), p_pid->sat);
    // Save current error
    p_pid->errPrev      = p_pid->err;
}

//inline void FOC_getPhaseCurrent(MOTOR_ACQ* p_acq)
//{
//    // Get the latest current values in Amps
//    p_acq->ia       = (float32_t)(*p_acq->iaMeasReg)    * ADC_CURRENT_SCALE;
//    p_acq->ib       = (float32_t)(*p_acq->ibMeasReg)    * ADC_CURRENT_SCALE;
//    p_acq->ic       = (float32_t)(*p_acq->icMeasReg)    * ADC_CURRENT_SCALE;
//    p_acq->measNew  = true;
//}

//inline void FOC_getPhaseVoltage(MOTOR_ACQ* p_acq)
//{
//    // Get the latest phase voltage values in Volt
//    p_acq->va       = (float32_t)(*p_acq->vaMeasReg)   * ADC_VOLTAGE_SCALE;
//    p_acq->vb       = (float32_t)(*p_acq->vbMeasReg)   * ADC_VOLTAGE_SCALE;
//    p_acq->vc       = (float32_t)(*p_acq->vcMeasReg)   * ADC_VOLTAGE_SCALE;
//}

//inline void FOC_getBusVoltage(MOTOR_ACQ* p_acq)
//{
//    // Get the latest Voltage supply value in Volt
//    p_acq->vbus = (float32_t)(*p_acq->vBusMeasReg) * ADC_VOLTAGE_SCALE;
//    // Get bus voltage (minimum is one to avoid dividing by zero later)
//    p_acq->vbus = fmaxf(p_acq->vbus, 1.0f);
//}

// Clarke Park transform wikipedia link:
// https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation
//inline void FOC_parkClarke(FOC* p_foc)
//{
//    // Clarke transform
//    p_foc->ialpha   = ((2.0f * p_foc->motor_acq.ia) - (p_foc->motor_acq.ib + p_foc->motor_acq.ic)) * FM_1DIV3;
//    p_foc->ibeta    = (p_foc->motor_acq.ib - p_foc->motor_acq.ic) * FM_1DIVSQRT3;
//
//    // Park transform
//    float32_t id    = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
//    float32_t iq    = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);
//
//    // Park transform (low-pass filtering)
//    LPF_FILTER(p_foc->iParkLpf, p_foc->id, id);
//    LPF_FILTER(p_foc->iParkLpf, p_foc->iq, iq);
//
//    float32_t coef =  (p_foc->motorID == MOTOR_1) ? (MOTOR1_CURRENT_LPF_ALPHA) : (MOTOR2_CURRENT_LPF_ALPHA);
//
//    LPF_FILTER2(coef, p_foc->id_tmp, id);
//    LPF_FILTER2(coef, p_foc->iq_tmp, iq);
//}

// Compute errors, feed-forwards and execute PI current controllers
//inline void FOC_runPi(FOC* p_foc)
//{
//    // Run PI Id controller
//    p_foc->piId.err = p_foc->idRef - p_foc->id;
//    // Compute feed-forward term. Don't use the U=RI part because idRef = 0
//    p_foc->piId.ff = -1.0 * p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.Lq * p_foc->iq;
//    PI_run(&p_foc->piId);
//
//    // Run PI Iq controller
//    p_foc->piIq.err = p_foc->iqRef - p_foc->iq;
//    // Compute feed-forward term. The coupled id term decreases performances so we don't use it
//    p_foc->piIq.ff = p_foc->motorParams.Rs * p_foc->iqRef + p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.ke;
//    PI_run(&p_foc->piIq);
//
//    // Copy output variables
//    p_foc->vd = p_foc->piId.out;
//    p_foc->vq = p_foc->piIq.out;
//}

// Compute phase voltages from controller outputs
//inline void FOC_invParkClarke(FOC* p_foc)
//{
//    // Inverse Park transform
//    p_foc->valpha = p_foc->cosTheta * p_foc->vd - p_foc->sinTheta * p_foc->vq;
//    p_foc->vbeta  = p_foc->sinTheta * p_foc->vd + p_foc->cosTheta * p_foc->vq;
//
//    // Inverse Clarke transform
//    *p_foc->va = p_foc->valpha;
//    *p_foc->vb =  0.5f * (-p_foc->valpha + FM_SQRT3 * p_foc->vbeta);
//    *p_foc->vc = -0.5f * ( p_foc->valpha + FM_SQRT3 * p_foc->vbeta);
//}

// State vector modulation, compute PWM compare register values from phase voltages
//inline void FOC_svm(FOC* p_foc)
//{
//    // Compute middle voltage to center PWM duty cycles around it
//    float32_t uMid = 0.5f * (fminf(fminf(p_foc->va, p_foc->vb), p_foc->vc) + fmaxf(fmaxf(p_foc->va, p_foc->vb), p_foc->vc));
//
//    // Compute duty cycles
//    p_foc->ducya = fminf(fmaxf((p_foc->va - uMid) / p_foc->vBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//    p_foc->ducyb = fminf(fmaxf((p_foc->vb - uMid) / p_foc->vBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//    p_foc->ducyc = fminf(fmaxf((p_foc->vc - uMid) / p_foc->vBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//
//    // Compute compare registers:
//    *(p_foc->pwmCompA) = (p_foc->ducya + 0.5f) * p_foc->pwmCompScaler;
//    *(p_foc->pwmCompB) = (p_foc->ducyb + 0.5f) * p_foc->pwmCompScaler;
//    *(p_foc->pwmCompC) = (p_foc->ducyc + 0.5f) * p_foc->pwmCompScaler;
//}

