#include "foc.h"
//#include "f28x_project.h"

// Read current measures
//inline void FOC_getPosition(FOC* p_foc, MOTOR_HAL_CFG* p_motor)
//{
//    // Get latest position of the Quadrature Encoder
//    p_motor[p_motor->motorID].encoderPos = HWREG(p_motor->p_qep->eqepBase + EQEP_O_QPOSCNT);
//}

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
    // Get the latest Voltage supply value in Volt
    p_acq->vbus     = (float32_t)(*p_acq->vBusMeasReg)  * ADC_VOLTAGE_SCALE;
    // Get bus voltage (minimum is one to avoid dividing by zero later)
    p_acq->vbus     = (vbus_en) ? (__fmax(p_acq->vbus, 1.0f)) : (p_acq->vbus);
    p_acq->measNew  = true;
}

//inline void FOC_getPhaseCurrent(MOTOR_ACQ* p_acq)
//{
//    // Get the latest current values in Amps
//    p_acq->ia       = (float32_t)(*p_acq->iaMeasReg)    * ADC_CURRENT_SCALE;
//    p_acq->ib       = (float32_t)(*p_acq->ibMeasReg)    * ADC_CURRENT_SCALE;
//    p_acq->ic       = (float32_t)(*p_acq->icMeasReg)    * ADC_CURRENT_SCALE;
//    p_acq->measNew  = true;
//}
//
//inline void FOC_getPhaseVoltage(MOTOR_ACQ* p_acq)
//{
//    // Get the latest phase voltage values in Volt
//    p_acq->va       = (float32_t)(*p_acq->vaMeasReg)   * ADC_VOLTAGE_SCALE;
//    p_acq->vb       = (float32_t)(*p_acq->vbMeasReg)   * ADC_VOLTAGE_SCALE;
//    p_acq->vc       = (float32_t)(*p_acq->vcMeasReg)   * ADC_VOLTAGE_SCALE;
//}
//
//inline void FOC_getBusVoltage(MOTOR_ACQ* p_acq)
//{
//    // Get the latest Voltage supply value in Volt
//    p_acq->vbus = (float32_t)(*p_acq->vBusMeasReg) * ADC_VOLTAGE_SCALE;
//    // Get bus voltage (minimum is one to avoid dividing by zero later)
//    p_acq->vbus = __fmax(p_acq->vbus, 1.0f);
//}

// Compute electrical angle sine and cosine
inline void FOC_sineCosine(FOC* p_foc)
{
    // Sine Cosine computation
    p_foc->sinTheta = __sinpuf32(p_foc->motor_enc.elecPos);
    p_foc->cosTheta = __cospuf32(p_foc->motor_enc.elecPos);
}

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
////    *p_foc->va = p_foc->valpha;
////    *p_foc->vb =  0.5f * (-p_foc->valpha + FM_SQRT3 * p_foc->vbeta);
////    *p_foc->vc = -0.5f * ( p_foc->valpha + FM_SQRT3 * p_foc->vbeta);
//}

// State vector modulation, compute PWM compare register values from phase voltages
//inline void FOC_svm(FOC* p_foc)
//{
//    // Compute middle voltage to center PWM duty cycles around it
//    float32_t uMid = 0.5f * (__fmin(__fmin(p_foc->va, p_foc->vb), p_foc->vc) + __fmax(__fmax(p_foc->va, p_foc->vb), p_foc->vc));
//
//    // Compute duty cycles
//    p_foc->ducya = __fmin(__fmax((p_foc->va - uMid) / p_foc->vBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//    p_foc->ducyb = __fmin(__fmax((p_foc->vb - uMid) / p_foc->vBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//    p_foc->ducyc = __fmin(__fmax((p_foc->vc - uMid) / p_foc->vBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
//
//    // Compute compare registers:
//    *(p_foc->pwmCompA) = (p_foc->ducya + 0.5f) * p_foc->pwmCompScaler;
//    *(p_foc->pwmCompB) = (p_foc->ducyb + 0.5f) * p_foc->pwmCompScaler;
//    *(p_foc->pwmCompC) = (p_foc->ducyc + 0.5f) * p_foc->pwmCompScaler;
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
        LPF_FILTER(p_foc->resEstLpf, p_foc->resistanceEst, (p_foc->uq - p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.ke) / p_foc->iq);
    }
}


inline void FOC_control(FOC* p_foc)
{
    // Clarke transform
    p_foc->ialpha   = ((2.0f * p_foc->motor_acq.ia) - (p_foc->motor_acq.ib + p_foc->motor_acq.ic)) * FM_1DIV3;
    p_foc->ibeta    = (p_foc->motor_acq.ib - p_foc->motor_acq.ic) * FM_1DIVSQRT3;

    // Park transform
    float32_t id    = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
    float32_t iq    = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);

#if (UOMODRI_FLT_FORM == 1)
    // Park transform (low-pass filtering)
    LPF_FILTER(p_foc->iParkLpf, p_foc->id, id);
    LPF_FILTER(p_foc->iParkLpf, p_foc->iq, iq);
#elif (UOMODRI_FLT_FORM == 2)
    float32_t coef =  (p_foc->motorID == MOTOR_1) ? (MOTOR1_CURRENT_LPF_ALPHA) : (MOTOR2_CURRENT_LPF_ALPHA);

    LPF_FILTER2(coef, p_foc->id, id);
    LPF_FILTER2(coef, p_foc->iq, iq);
#endif

    // Compute feed-forward term. Don't use the U=RI part because idRef = 0
    p_foc->piId.ff = -1.0 * p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.Lq * p_foc->iq;
    // Run PI controller for Id
    FOC_PID(&p_foc->piId, p_foc->idRef, p_foc->id);
    // Compute feed-forward term. The coupled id term decreases performances so we don't use it
    p_foc->piIq.ff = p_foc->motorParams.Rs * p_foc->iqRef + p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.ke;
    // Run PI controller for Id
    FOC_PID(&p_foc->piId, p_foc->idRef, p_foc->iq);

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
    float32_t uMid = 0.5f * (__fmin(__fmin(p_foc->ua, p_foc->ub), p_foc->uc) + __fmax(__fmax(p_foc->ua, p_foc->ub), p_foc->uc));

    // Compute duty cycles
    p_foc->ducya = __fmin(__fmax((p_foc->ua - uMid) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    p_foc->ducyb = __fmin(__fmax((p_foc->ub - uMid) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    p_foc->ducyc = __fmin(__fmax((p_foc->uc - uMid) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);

    // Compute compare registers:
    *(p_foc->pwmCompA) = (p_foc->ducya + 0.5f) * p_foc->pwmCompScaler;
    *(p_foc->pwmCompB) = (p_foc->ducyb + 0.5f) * p_foc->pwmCompScaler;
    *(p_foc->pwmCompC) = (p_foc->ducyc + 0.5f) * p_foc->pwmCompScaler;}

static inline void FOC_PID(PID_CONTROLLER* p_pid, float32_t setPoint, float32_t processVar)
{
    // compute error
    p_pid->err          = setPoint - processVar;
    // integral term computation
    p_pid->integral    += p_pid->ki * p_pid->err;
    // derivative term computation
    p_pid->derivative   = p_pid->kd * (p_pid->err - p_pid->errPrev);
    // PID computation
    p_pid->out          = (p_pid->kp * p_pid->err) + p_pid->integral + p_pid->derivative + p_pid->ff;
    // anti-windup
    p_pid->out          = __fmin(__fmax(p_pid->out, -p_pid->sat), p_pid->sat);
    // Save current error
    p_pid->errPrev      = p_pid->err;
}

