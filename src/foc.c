#include "foc.h"
//#include "f28x_project.h"

// Read current measures
inline void FOC_getPosition(FOC* p_foc, MOTOR_HAL_CFG* p_motor)
{
    // Get latest position of the Quadrature Encoder
    p_motor[p_motor->motorID].encoderPos = HWREG(p_motor->p_qep->eqepBase + EQEP_O_QPOSCNT);
}

inline void FOC_getPhaseCurrent(FOC* p_foc, MOTOR_HAL_CFG* p_motor)
{
    // Get the latest current values in Amps
    p_foc->ia = (float32_t)HWSREGH(p_motor->p_iConv[0]->adcPPBResultBase) * ADC_CURRENT_SCALE;
    p_foc->ib = (float32_t)HWSREGH(p_motor->p_iConv[1]->adcPPBResultBase) * ADC_CURRENT_SCALE;
    p_foc->ic = (float32_t)HWSREGH(p_motor->p_iConv[2]->adcPPBResultBase) * ADC_CURRENT_SCALE;
}

inline void FOC_getPhaseVoltage(FOC* p_foc, MOTOR_HAL_CFG* p_motor)
{
    // Get the latest phase voltage values in Volt
    p_foc->va = (float32_t)HWREGH(p_motor->p_vConv[0]->adcPPBResultBase) * ADC_VOLTAGE_SCALE;
    p_foc->vb = (float32_t)HWREGH(p_motor->p_vConv[1]->adcPPBResultBase) * ADC_VOLTAGE_SCALE;
    p_foc->vc = (float32_t)HWREGH(p_motor->p_vConv[2]->adcPPBResultBase) * ADC_VOLTAGE_SCALE;
}

inline void FOC_getBusVoltage(FOC* p_foc, MOTOR_HAL_CFG* p_motor)
{
    // Get the latest Voltage supply value in Volt
    p_foc->vbus = (float32_t)HWREGH(p_motor->p_vbusConv->adcPPBResultBase) * ADC_VOLTAGE_SCALE;
}


// Read current measures
inline void FOC_getMeasures(FOC* p_foc, MOTOR_HAL_CFG* p_motor)
{
    // Get the latest position of the Quadrature Encoder
//    p_motor[p_motor->motorID].encoderPos = HWREG(p_motor->p_qep->eqepBase + EQEP_O_QPOSCNT);
//    // Get the latest ADC results to the buffer
//    p_motor[p_motor->motorID].ia      = HWREGH(p_motor->p_iConv->adcResultBase + ADC_SOC_NUMBER0);
//    p_motor[p_motor->motorID].ib      = HWREGH(ADCBRESULT_BASE + ADC_SOC_NUMBER0);
//    p_motor[p_motor->motorID].ic      = HWREGH(ADCDRESULT_BASE + ADC_SOC_NUMBER0);
//    p_motor[p_motor->motorID].vbus    = HWREGH(ADCCRESULT_BASE + ADC_SOC_NUMBER0);


    // Convert currents from registers
    p_foc->ia = (*p_foc->isenAResult) * p_foc->currentScalerReg;
    p_foc->ib = (*p_foc->isenBResult) * p_foc->currentScalerReg;
    p_foc->ic = (*p_foc->isenCResult) * p_foc->currentScalerReg;

    // Get bus voltage (minimum is one to avoid dividing by zero later)
    p_foc->vBus = __fmax(*(p_foc->vBusSenResult) * p_foc->voltageScalerReg, 1.0f);

    // Get potentiometer voltage
    p_foc->auxAdc = (*p_foc->potResult) * p_foc->adcScalerReg;
}

// Compute electrical angle sine and cosine
inline void FOC_sineCosine(FOC* p_foc, ENCODER* p_encoder)
{
//    FOC_struct*  p_foc       = p_motor->motorFOC;
//    ENCODER*    p_encoder   = p_motor->motorEncoder;
    // Sine Cosine computation
    p_foc->sinTheta = __sinpuf32(p_encoder->elecAngle);
    p_foc->cosTheta = __cospuf32(p_encoder->elecAngle);
}

// Clarke Park transform wikipedia link:
// https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation
inline void FOC_parkClarke(FOC* p_foc)
{
    // Clarke transform
    p_foc->ialpha =  (2.0f * p_foc->ia - p_foc->ib - p_foc->ic) * 0.333333333f;
    p_foc->ibeta = (p_foc->ib - p_foc->ic) * FM_ISQRT3;

    // Park transform (low-pass filtered)
    LPF_FILTER(p_foc->controlLoopLpf, p_foc->id,  p_foc->cosTheta * p_foc->ialpha + p_foc->sinTheta * p_foc->ibeta);
    LPF_FILTER(p_foc->controlLoopLpf, p_foc->iq, -p_foc->sinTheta * p_foc->ialpha + p_foc->cosTheta * p_foc->ibeta);
}

// Compute errors, feed-forwards and execute PI current controllers
//inline void FOC_runPi(FOC *foc)
//{
//    // Run PI Id controller
//    p_foc->piId.err = p_foc->idRef - p_foc->id;
//    // Compute feed-forward term. Don't use the U=RI part because idRef = 0
//    p_foc->piId.ff = -p_foc->encoder.speedEst.elecSpeed * p_foc->Lq * p_foc->iq;
//    //PI_run(&p_foc->piId);
//
//    // Run PI Iq controller
//    p_foc->piIq.err = p_foc->iqRef - p_foc->iq;
//    // Compute feed-forward term. The coupled id term decreases performances so we don't use it
//    p_foc->piIq.ff = p_foc->Rs * p_foc->iqRef + p_foc->encoder.speedEst.elecSpeed * p_foc->ke;
//    //PI_run(&p_foc->piIq);
//
//    // Copy output variables
//    p_foc->vd = p_foc->piId.out;
//    p_foc->vq = p_foc->piIq.out;
//}

// Compute phase voltages from controller outputs
inline void FOC_invParkClarke(FOC* p_foc)
{
    // Inverse Park transform
    p_foc->valpha = p_foc->cosTheta * p_foc->vd - p_foc->sinTheta * p_foc->vq;
    p_foc->vbeta  = p_foc->sinTheta * p_foc->vd + p_foc->cosTheta * p_foc->vq;

    // Inverse Clarke transform
    p_foc->va = p_foc->valpha;
    p_foc->vb =  0.5f * (-p_foc->valpha + FM_SQRT3 * p_foc->vbeta);
    p_foc->vc = -0.5f * ( p_foc->valpha + FM_SQRT3 * p_foc->vbeta);
}

// State vector modulation, compute PWM compare register values from phase voltages
inline void FOC_svm(FOC* p_foc)
{
    // Compute middle voltage to center PWM duty cycles around it
    float32_t uMid = 0.5f * (__fmin(__fmin(p_foc->va, p_foc->vb), p_foc->vc) + __fmax(__fmax(p_foc->va, p_foc->vb), p_foc->vc));

    // Compute duty cycles
    p_foc->ducya = __fmin(__fmax((p_foc->va - uMid) / p_foc->vBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    p_foc->ducyb = __fmin(__fmax((p_foc->vb - uMid) / p_foc->vBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);
    p_foc->ducyc = __fmin(__fmax((p_foc->vc - uMid) / p_foc->vBus, -p_foc->dutyCycleSat), p_foc->dutyCycleSat);

    // Compute compare registers:
    *(p_foc->pwmCompA) = (p_foc->ducya + 0.5f) * p_foc->pwmCompScaler;
    *(p_foc->pwmCompB) = (p_foc->ducyb + 0.5f) * p_foc->pwmCompScaler;
    *(p_foc->pwmCompC) = (p_foc->ducyc + 0.5f) * p_foc->pwmCompScaler;
}

// Q-axis resistance estimation
// TODO Use VSEN results instead of Iq controller output
inline void FOC_resistanceEstimation(FOC* p_foc)
{
    // Read measures
    p_foc->phaseAVolt = (*p_foc->vsenaResult) * p_foc->voltageScalerReg;
    p_foc->phaseBVolt = (*p_foc->vsenbResult) * p_foc->voltageScalerReg;
    p_foc->phaseCVolt = (*p_foc->vsencResult) * p_foc->voltageScalerReg;

    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(p_foc->iq) > FOC_RE_CURRENT_THRESHOLD)
    {
        LPF_FILTER(p_foc->resEstLpf, p_foc->resistanceEst, (p_foc->vq - p_foc->encoder.speedEst.elecSpeed * p_foc->ke) / p_foc->iq);
    }
}


