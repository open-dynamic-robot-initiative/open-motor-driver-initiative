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
inline void FOC_getMeasures(FOC *foc, MOTOR_HAL_CFG* p_motor)
{
    // Get the latest position of the Quadrature Encoder
//    p_motor[p_motor->motorID].encoderPos = HWREG(p_motor->p_qep->eqepBase + EQEP_O_QPOSCNT);
//    // Get the latest ADC results to the buffer
//    p_motor[p_motor->motorID].ia      = HWREGH(p_motor->p_iConv->adcResultBase + ADC_SOC_NUMBER0);
//    p_motor[p_motor->motorID].ib      = HWREGH(ADCBRESULT_BASE + ADC_SOC_NUMBER0);
//    p_motor[p_motor->motorID].ic      = HWREGH(ADCDRESULT_BASE + ADC_SOC_NUMBER0);
//    p_motor[p_motor->motorID].vbus    = HWREGH(ADCCRESULT_BASE + ADC_SOC_NUMBER0);


    // Convert currents from registers
    foc->ia = (*foc->isenAResult) * foc->currentScalerReg;
    foc->ib = (*foc->isenBResult) * foc->currentScalerReg;
    foc->ic = (*foc->isenCResult) * foc->currentScalerReg;

    // Get bus voltage (minimum is one to avoid dividing by zero later)
    foc->vBus = __fmax(*(foc->vBusSenResult) * foc->voltageScalerReg, 1.0f);

    // Get potentiometer voltage
    foc->auxAdc = (*foc->potResult) * foc->adcScalerReg;
}

// Compute electrical angle sine and cosine
inline void FOC_sineCosine(FOC *foc)
{
    // Sine Cosine computation
    foc->sinTheta = __sinpuf32(foc->encoder.elecAngle);
    foc->cosTheta = __cospuf32(foc->encoder.elecAngle);
}

// Clarke Park transform wikipedia link:
// https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation
inline void FOC_parkClarke(FOC *foc)
{
    // Clarke transform
    foc->ialpha =  (2.0f * foc->ia - foc->ib - foc->ic) * 0.333333333f;
    foc->ibeta = (foc->ib - foc->ic) * FM_ISQRT3;

    // Park transform (low-pass filtered)
    LPF_FILTER(foc->controlLoopLpf, foc->id,  foc->cosTheta * foc->ialpha + foc->sinTheta * foc->ibeta);
    LPF_FILTER(foc->controlLoopLpf, foc->iq, -foc->sinTheta * foc->ialpha + foc->cosTheta * foc->ibeta);
}

// Compute errors, feed-forwards and execute PI current controllers
//inline void FOC_runPi(FOC *foc)
//{
//    // Run PI Id controller
//    foc->piId.err = foc->idRef - foc->id;
//    // Compute feed-forward term. Don't use the U=RI part because idRef = 0
//    foc->piId.ff = -foc->encoder.speedEst.elecSpeed * foc->Lq * foc->iq;
//    //PI_run(&foc->piId);
//
//    // Run PI Iq controller
//    foc->piIq.err = foc->iqRef - foc->iq;
//    // Compute feed-forward term. The coupled id term decreases performances so we don't use it
//    foc->piIq.ff = foc->Rs * foc->iqRef + foc->encoder.speedEst.elecSpeed * foc->ke;
//    //PI_run(&foc->piIq);
//
//    // Copy output variables
//    foc->vd = foc->piId.out;
//    foc->vq = foc->piIq.out;
//}

// Compute phase voltages from controller outputs
inline void FOC_invParkClarke(FOC *foc)
{
    // Inverse Park transform
    foc->valpha = foc->cosTheta * foc->vd - foc->sinTheta * foc->vq;
    foc->vbeta  = foc->sinTheta * foc->vd + foc->cosTheta * foc->vq;

    // Inverse Clarke transform
    foc->va = foc->valpha;
    foc->vb =  0.5f * (-foc->valpha + FM_SQRT3 * foc->vbeta);
    foc->vc = -0.5f * ( foc->valpha + FM_SQRT3 * foc->vbeta);
}

// State vector modulation, compute PWM compare register values from phase voltages
inline void FOC_svm(FOC *foc)
{
    // Compute middle voltage to center PWM duty cycles around it
    float32_t uMid = 0.5f * (__fmin(__fmin(foc->va, foc->vb), foc->vc) + __fmax(__fmax(foc->va, foc->vb), foc->vc));

    // Compute duty cycles
    foc->ducya = __fmin(__fmax((foc->va - uMid) / foc->vBus, -foc->dutyCycleSat), foc->dutyCycleSat);
    foc->ducyb = __fmin(__fmax((foc->vb - uMid) / foc->vBus, -foc->dutyCycleSat), foc->dutyCycleSat);
    foc->ducyc = __fmin(__fmax((foc->vc - uMid) / foc->vBus, -foc->dutyCycleSat), foc->dutyCycleSat);

    // Compute compare registers:
    *(foc->pwmCompA) = (foc->ducya + 0.5f) * foc->pwmCompScaler;
    *(foc->pwmCompB) = (foc->ducyb + 0.5f) * foc->pwmCompScaler;
    *(foc->pwmCompC) = (foc->ducyc + 0.5f) * foc->pwmCompScaler;
}

// Q-axis resistance estimation
// TODO Use VSEN results instead of Iq controller output
inline void FOC_resistanceEstimation(FOC *foc)
{
    // Read measures
    foc->phaseAVolt = (*foc->vsenaResult) * foc->voltageScalerReg;
    foc->phaseBVolt = (*foc->vsenbResult) * foc->voltageScalerReg;
    foc->phaseCVolt = (*foc->vsencResult) * foc->voltageScalerReg;

    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(foc->iq) > FOC_RE_CURRENT_THRESHOLD)
    {
        LPF_FILTER(foc->resEstLpf, foc->resistanceEst, (foc->vq - foc->encoder.speedEst.elecSpeed * foc->ke) / foc->iq);
    }
}


