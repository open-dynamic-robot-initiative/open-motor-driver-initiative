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
    p_acq->ia       = (float32_t)(*p_acq->iaMeasReg)    * ADC_CURRENT_SCALE;
    p_acq->ib       = (float32_t)(*p_acq->ibMeasReg)    * ADC_CURRENT_SCALE;
    p_acq->ic       = (float32_t)(*p_acq->icMeasReg)    * ADC_CURRENT_SCALE;
    // Get the latest phase voltage values in Volt
    p_acq->va       = (float32_t)(*p_acq->vaMeasReg)    * ADC_VOLTAGE_SCALE;
    p_acq->vb       = (float32_t)(*p_acq->vbMeasReg)    * ADC_VOLTAGE_SCALE;
    p_acq->vc       = (float32_t)(*p_acq->vcMeasReg)    * ADC_VOLTAGE_SCALE;
    // Get the latest Voltage supply value in Volt (minimum is one to avoid dividing by zero later)
    float32_t vbus  = fmaxf(((float32_t)(*p_acq->vBusMeasReg) * ADC_VOLTAGE_SCALE), 1.0f);
#if (UOMODRI_VBUS_FLT == 1)
        LPF_FILTER_1(p_acq->vBusLpf, p_acq->vbus, vbus);
#elif (UOMODRI_VBUS_FLT == 2)
        LPF_FILTER_2(p_acq->vBusLpf, p_acq->vbus, vbus);
#else
        p_acq->vbus = vbus;
#endif
        p_acq->vmax = MOTOR12_OVERMODULATION * p_acq->vbus * (MOTOR1_DUTY_CYCLE_MAX - MOTOR1_DUTY_CYCLE_MIN) * FM_1DIVSQRT3;
    p_acq->measNew  = true;
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
        LPF_FILTER_1(p_foc->resEstLpf, p_foc->resistanceEst, (p_foc->uq - p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.ke) / p_foc->iq);
#elif (UOMODRI_RES_ESTIM_FLT == 2)
        LPF_FILTER_2(p_foc->resEstLpf, p_foc->resistanceEst, (p_foc->uq - p_foc->motor_enc.speedEst.elecSpeed * p_foc->motorParams.ke) / p_foc->iq);
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
    p_foc->ialpha       = ((2.0f * p_foc->motor_acq.ia) - (p_foc->motor_acq.ib + p_foc->motor_acq.ic)) * FM_1DIV3;
    p_foc->ibeta        = (p_foc->motor_acq.ib - p_foc->motor_acq.ic) * FM_1DIVSQRT3;
    // Compute sine and cosine from the electrical angle
    p_foc->sinTheta     = __sinpuf32(p_foc->motor_enc.elecPos);
    p_foc->cosTheta     = __cospuf32(p_foc->motor_enc.elecPos);
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
    p_foc->piId.err     = p_foc->idRef - p_foc->id;
    p_foc->piIq.err     = p_foc->iqRef - p_foc->iq;
    // PI controller - Id(Flux Current Controller) & Iq (Torque Current Controller)
    p_foc->ud           = FOC_PI(&p_foc->piId);
    p_foc->uq           = FOC_PI(&p_foc->piIq);
//    float32_t uq_max    = sqrtf((p_foc->motor_acq.vmax * p_foc->motor_acq.vmax) - (p_foc->ud * p_foc->ud));
//    float32_t v_ref     = sqrtf((p_foc->ud * p_foc->ud) + (p_foc->uq * p_foc->uq));
//    limit_norm(&p_foc->ud, &p_foc->uq, p_foc->motor_acq.vmax);
    // Inverse Park transform
    p_foc->ualpha = p_foc->cosTheta * p_foc->ud - p_foc->sinTheta * p_foc->uq;
    p_foc->ubeta  = p_foc->sinTheta * p_foc->ud + p_foc->cosTheta * p_foc->uq;
    // Inverse Clarke transform
    p_foc->ua = p_foc->ualpha;
    p_foc->ub =  0.5f * ((FM_SQRT3 * p_foc->ubeta) - p_foc->ualpha);
    p_foc->uc = -0.5f * ((FM_SQRT3 * p_foc->ubeta) + p_foc->ualpha);
    // Compute middle voltage to center PWM duty cycles around it
    float32_t uOffset   = 0.5f * (FMIN3(p_foc->ua, p_foc->ub, p_foc->uc) + FMAX3(p_foc->ua, p_foc->ub, p_foc->uc));
    //float32_t uMid = 0.5f * (fminf(fminf(p_foc->ua, p_foc->ub), p_foc->uc) + fmaxf(fmaxf(p_foc->ua, p_foc->ub), p_foc->uc));
//    float32_t uMidPoint = 0.5f * (MOTOR1_DUTY_CYCLE_MAX - MOTOR1_DUTY_CYCLE_MIN);
    // Compute duty cycles
    p_foc->ducya = FMINMAX((((p_foc->ua - uOffset) / *p_foc->uBus) + 0.5f), p_foc->dutyCycleMax, p_foc->dutyCycleMin);
//    p_foc->ducya = fminf(fmaxf((p_foc->ua - uOffset) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    p_foc->ducyb = FMINMAX((((p_foc->ub - uOffset) / *p_foc->uBus) + 0.5f), p_foc->dutyCycleMax, p_foc->dutyCycleMin);
//    p_foc->ducyb = fminf(fmaxf((p_foc->ub - uOffset) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    p_foc->ducyc = FMINMAX((((p_foc->uc - uOffset) / *p_foc->uBus) + 0.5f), p_foc->dutyCycleMax, p_foc->dutyCycleMin);
//    p_foc->ducyc = fminf(fmaxf((p_foc->uc - uOffset) / *p_foc->uBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    // Compute compare registers:
    *(p_foc->pwmCompA) = p_foc->ducya * PWM_HALF_TBPRD;
    *(p_foc->pwmCompB) = p_foc->ducyb * PWM_HALF_TBPRD;
    *(p_foc->pwmCompC) = p_foc->ducyc * PWM_HALF_TBPRD;

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

static inline float32_t FOC_PI(PID_CONTROLLER* p_pid)
{
    p_pid->integral    += p_pid->ki * p_pid->err;
    p_pid->integral     = FMINMAX(p_pid->integral, *p_pid->sat, -(*p_pid->sat)); //fmaxf(fminf(p_pid->integral, *p_pid->sat), -(*p_pid->sat));
    // Compute PID and clamp output.
    p_pid->out          = (p_pid->kp * p_pid->err) + p_pid->integral + p_pid->ff;
    p_pid->out          = FMINMAX(p_pid->out, *p_pid->sat, -(*p_pid->sat)); //fmaxf(fminf(p_pid->out, *p_pid->sat), -(*p_pid->sat));
    return(p_pid->out);
}

static inline float32_t FOC_PD(PID_CONTROLLER* p_pid)
{
    p_pid->derivative   = p_pid->kd * p_pid->vErr;//(p_pid->err - p_pid->errPrev) / p_pid->deltaT;
    // Compute PID and clamp output.
    p_pid->out          = (p_pid->kp * p_pid->err) + p_pid->derivative + p_pid->ff;
    p_pid->out          = FMINMAX(p_pid->out, *p_pid->sat, -(*p_pid->sat)); //fmaxf(fminf(p_pid->out, *p_pid->sat), -(*p_pid->sat));
    // Save current error
    //p_pid->errPrev      = p_pid->err;
    return(p_pid->out);
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
    p_foc->piId.err             = 0.0f;
    p_foc->piId.kp              = MOTOR1_ID_REGULATOR_KP_COEF;
    p_foc->piId.ki              = MOTOR1_ID_REGULATOR_KI_COEF;
    p_foc->piId.integral        = 0.0f;
    p_foc->piId.ff              = 0.0f;
    p_foc->piId.out             = 0.0f;
    p_foc->piId.sat             = ((float32_t *)(&p_foc->motor_acq.vmax));

    p_foc->piIq.err             = 0.0f;
    p_foc->piIq.kp              = MOTOR1_IQ_REGULATOR_KP_COEF;
    p_foc->piIq.ki              = MOTOR1_IQ_REGULATOR_KI_COEF;
    p_foc->piIq.integral        = 0.0f;
    p_foc->piIq.ff              = 0.0f;
    p_foc->piIq.out             = 0.0f;
    p_foc->piIq.sat             = ((float32_t *)(&p_foc->motor_acq.vmax));

    // Reset alignment routine
    //ALIGN_reset(&foc->alignRoutine);

    // Reset position PD controller
    p_foc->pdPos.err            = 0.0f;
    p_foc->pdPos.vErr           = 0.0f;
    p_foc->pdPos.kp             = MOTOR1_IQ_REGULATOR_KP_COEF;
    p_foc->pdPos.ki             = MOTOR1_IQ_REGULATOR_KI_COEF;
    p_foc->pdPos.kd             = 0.0f;
    p_foc->pdPos.integral       = 0.0f;
    p_foc->pdPos.derivative     = 0.0f;
    p_foc->pdPos.ff             = 0.0f;
    p_foc->pdPos.out            = 0.0f;
    p_foc->pdPos.sat            = NULL;

    // Reset encoder
    p_foc->motor_enc.mechPos[NEW] = 0.0;
    p_foc->motor_enc.mechPos[OLD] = 0.0;
    p_foc->motor_enc.elecPos = 0.0f;

    p_foc->motor_enc.isIndexCompensated = false;
    p_foc->motor_enc.indexAngle = 0.0f;
    p_foc->motor_enc.indexedMechAngle = 0.0f;

    p_foc->motor_enc.lastMechAngle = 0.0f;
    p_foc->motor_enc.absoluteAngle = 0.0f;

    p_foc->motor_enc.speedEst.alpha = 0.0f;
    p_foc->motor_enc.speedEst.speedRef = 0.0f;
    p_foc->motor_enc.speedEst.pos[NEW] = 0.0f;
    p_foc->motor_enc.speedEst.pos[OLD] = 0.0f;
    p_foc->motor_enc.speedEst.lowSpeed = 0.0f;
    p_foc->motor_enc.speedEst.highSpeed = 0.0f;
    p_foc->motor_enc.speedEst.mechSpeed = 0.0f;
    p_foc->motor_enc.speedEst.elecSpeed = 0.0f;

    p_foc->motor_enc.turnNb = 0;
    p_foc->motor_enc.indexDetected = false;
    p_foc->motor_enc.indexToggle = false;
    p_foc->motor_enc.rollOverFlag = false;

    // Reset currents and voltages
//    p_foc->ia = 0.0f;
//    foc->ib = 0.0f;
//    foc->ic = 0.0f;

    p_foc->ialpha = 0.0f;
    p_foc->ibeta = 0.0f;

    p_foc->id = 0.0f;
    p_foc->iq = 0.0f;

    p_foc->ud = 0.0f;
    p_foc->uq = 0.0f;

    p_foc->ualpha = 0.0f;
    p_foc->ubeta = 0.0f;

    p_foc->ua = 0.0f;
    p_foc->ub = 0.0f;
    p_foc->uc = 0.0f;

    // Reset potentiometer voltage
    p_foc->auxAdc = 0.0f;

    // Reset resistance estimation
    p_foc->phaseAVolt = 0.0f;
    p_foc->phaseBVolt = 0.0f;
    p_foc->phaseCVolt = 0.0f;

    p_foc->resistanceEst = 0.0f;
}


// No current is generated on motor phases
//static void FOC_noVoltage(const uint16_t motorIndex)
//{
//    EPWM_setCounterCompareValue(hal_motors[motorIndex].pwmHandle[0], EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(hal_motors[motorIndex].pwmHandle[1], EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(hal_motors[motorIndex].pwmHandle[2], EPWM_COUNTER_COMPARE_A, 0);
//}

//// Set the motor phases in security mode
//void FOC_emergencyStop()
//{
//    // Turn off control loop
//    comInterface.systemEnabled = false;
//    motor[MOTOR_1].motorFOC->enabled = false;
//    motor[MOTOR_2].motorFOC->enabled = false;
//
//    // Disable ePWM interrupts
//    Interrupt_disableInCPU(INTERRUPT_CPU_INT3);
//
//    // Tie phases to ground
//    MOTOR_SetCommand(&motor[MOTOR_1], 0.0, 0.0, 0.0);
//    MOTOR_SetCommand(&motor[MOTOR_2], 0.0, 0.0, 0.0);
//
//    // Reset foc structures
//    FOC_resetStruct(motor_foc[MOTOR_1]);
//    FOC_resetStruct(motor_foc[MOTOR_2]);
//
//}

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

