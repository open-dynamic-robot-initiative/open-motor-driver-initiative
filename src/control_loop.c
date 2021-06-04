#include "control_loop.h"
#include "uomodri_user_defines.h"

// Voltage offset computation functions

// Compute ADC voltage offset for motor 1 and save them in Post Processing Block offset register
// Only ISEN and VSEN offsets are computed currently
void MTR1_offsetComputation(MotorDebug* motorDebug)
{
    uint16_t calibCounter;
    float32_t isenOffset[3];
    float32_t vsenOffset[3];

    EPWM_clearEventTriggerInterruptFlag(MOTOR1_PWM1_BASE);

    // Wait for interrupt to be sure low side transistor is closed
    while(EPWM_getEventTriggerInterruptStatus(MOTOR1_PWM1_BASE) == false);

    isenOffset[0] = ADC_readResult(M1_IA_ADC_RESULT_ADDR, M1_IA_SOC_NUM);
    isenOffset[1] = ADC_readResult(M1_IB_ADC_RESULT_ADDR, M1_IB_SOC_NUM);
    isenOffset[2] = ADC_readResult(M1_IC_ADC_RESULT_ADDR, M1_IC_SOC_NUM);
    vsenOffset[0] = ADC_readResult(M1_VA_ADC_RESULT_ADDR, M1_VA_SOC_NUM);
    vsenOffset[1] = ADC_readResult(M1_VB_ADC_RESULT_ADDR, M1_VB_SOC_NUM);
    vsenOffset[2] = ADC_readResult(M1_VC_ADC_RESULT_ADDR, M1_VC_SOC_NUM);

    // Motor 1
    for (calibCounter = 0; calibCounter < 10000; calibCounter++)
    {
        EPWM_clearEventTriggerInterruptFlag(MOTOR1_PWM1_BASE);

        // Wait for interrupt to be sure low side transistor is closed
        while(EPWM_getEventTriggerInterruptStatus(MOTOR1_PWM1_BASE) == false);

        if (calibCounter > 1000)
        {
            // Update offset
            isenOffset[0] = 0.998f * isenOffset[0] +
                    0.002f * ADC_readResult(M1_IA_ADC_RESULT_ADDR, M1_IA_SOC_NUM);
            isenOffset[1] = 0.998f * isenOffset[1] +
                    0.002f * ADC_readResult(M1_IB_ADC_RESULT_ADDR, M1_IB_SOC_NUM);
            isenOffset[2] = 0.998f * isenOffset[2] +
                    0.002f * ADC_readResult(M1_IC_ADC_RESULT_ADDR, M1_IC_SOC_NUM);
            vsenOffset[0] = 0.998f * vsenOffset[0] +
                    0.002f * ADC_readResult(M1_VA_ADC_RESULT_ADDR, M1_VA_SOC_NUM);
            vsenOffset[1] = 0.998f * vsenOffset[1] +
                    0.002f * ADC_readResult(M1_VB_ADC_RESULT_ADDR, M1_VB_SOC_NUM);
            vsenOffset[2] = 0.998f * vsenOffset[2] +
                    0.002f * ADC_readResult(M1_VC_ADC_RESULT_ADDR, M1_VC_SOC_NUM);
        }
    }

    // Set offsets in PPBs
    ADC_setPPBReferenceOffset(M1_IA_ADC_RESULT_ADDR, M1_IA_PPB_NUM, (uint16_t)isenOffset[0]);
    ADC_setPPBReferenceOffset(M1_IB_ADC_RESULT_ADDR, M1_IB_PPB_NUM, (uint16_t)isenOffset[1]);
    ADC_setPPBReferenceOffset(M1_IC_ADC_RESULT_ADDR, M1_IC_PPB_NUM, (uint16_t)isenOffset[2]);
    ADC_setPPBReferenceOffset(M1_VA_ADC_RESULT_ADDR, M1_VA_PPB_NUM, (uint16_t)vsenOffset[0]);
    ADC_setPPBReferenceOffset(M1_VB_ADC_RESULT_ADDR, M1_VB_PPB_NUM, (uint16_t)vsenOffset[1]);
    ADC_setPPBReferenceOffset(M1_VC_ADC_RESULT_ADDR, M1_VC_PPB_NUM, (uint16_t)vsenOffset[2]);
}

// Compute ADC voltage offset for motor 2 and save them in Post Processing Block offset register
// Only ISEN and VSEN offsets are computed currently
void MTR2_offsetComputation(MotorDebug* motorDebug)
{
    uint16_t calibCounter;
    float32_t isenOffset[3];
    float32_t vsenOffset[3];

    EPWM_clearEventTriggerInterruptFlag(MOTOR2_PWM1_BASE);

    // Wait for interrupt to be sure low side transistor is closed
    while(EPWM_getEventTriggerInterruptStatus(MOTOR2_PWM1_BASE) == false);

    isenOffset[0] = ADC_readResult(M2_IA_ADC_RESULT_ADDR, M2_IA_SOC_NUM);
    isenOffset[1] = ADC_readResult(M2_IB_ADC_RESULT_ADDR, M2_IB_SOC_NUM);
    isenOffset[2] = ADC_readResult(M2_IC_ADC_RESULT_ADDR, M2_IC_SOC_NUM);
    vsenOffset[0] = ADC_readResult(M2_VA_ADC_RESULT_ADDR, M2_VA_SOC_NUM);
    vsenOffset[1] = ADC_readResult(M2_VB_ADC_RESULT_ADDR, M2_VB_SOC_NUM);
    vsenOffset[2] = ADC_readResult(M2_VC_ADC_RESULT_ADDR, M2_VC_SOC_NUM);

    // Motor 1
    for (calibCounter = 0; calibCounter < 10000; calibCounter++)
    {
        EPWM_clearEventTriggerInterruptFlag(MOTOR2_PWM1_BASE);

        // Wait for interrupt to be sure low side transistor is closed
        while(EPWM_getEventTriggerInterruptStatus(MOTOR2_PWM1_BASE) == false);

        if (calibCounter > 1000)
        {
            // Update offset
            isenOffset[0] = 0.998f * isenOffset[0] +
                    0.002f * ADC_readResult(M2_IA_ADC_RESULT_ADDR, M2_IA_SOC_NUM);
            isenOffset[1] = 0.998f * isenOffset[1] +
                    0.002f * ADC_readResult(M2_IB_ADC_RESULT_ADDR, M2_IB_SOC_NUM);
            isenOffset[2] = 0.998f * isenOffset[2] +
                    0.002f * ADC_readResult(M2_IC_ADC_RESULT_ADDR, M2_IC_SOC_NUM);
            vsenOffset[0] = 0.998f * vsenOffset[0] +
                    0.002f * ADC_readResult(M2_VA_ADC_RESULT_ADDR, M2_VA_SOC_NUM);
            vsenOffset[1] = 0.998f * vsenOffset[1] +
                    0.002f * ADC_readResult(M2_VB_ADC_RESULT_ADDR, M2_VB_SOC_NUM);
            vsenOffset[2] = 0.998f * vsenOffset[2] +
                    0.002f * ADC_readResult(M2_VC_ADC_RESULT_ADDR, M2_VC_SOC_NUM);
        }
    }

    // Set offsets in PPBs
    ADC_setPPBReferenceOffset(M2_IA_ADC_RESULT_ADDR, M2_IA_PPB_NUM, (uint16_t)isenOffset[0]);
    ADC_setPPBReferenceOffset(M2_IB_ADC_RESULT_ADDR, M2_IB_PPB_NUM, (uint16_t)isenOffset[1]);
    ADC_setPPBReferenceOffset(M2_IC_ADC_RESULT_ADDR, M2_IC_PPB_NUM, (uint16_t)isenOffset[2]);
    ADC_setPPBReferenceOffset(M2_VA_ADC_RESULT_ADDR, M2_VA_PPB_NUM, (uint16_t)vsenOffset[0]);
    ADC_setPPBReferenceOffset(M2_VB_ADC_RESULT_ADDR, M2_VB_PPB_NUM, (uint16_t)vsenOffset[1]);
    ADC_setPPBReferenceOffset(M2_VC_ADC_RESULT_ADDR, M2_VC_PPB_NUM, (uint16_t)vsenOffset[2]);
}

// Control loop initialization function
void CL_initFoc(FOC* p_foc, const uint16_t motorIndex)
{
    // Initialize scalers
    p_foc->currentScaler = ADC_CURRENT_SCALE;
    p_foc->currentScalerReg = -p_foc->currentScaler / 0x8000L; // from register value to amps
                                                           // signed 15 bits resolution
    p_foc->voltageScaler = ADC_VOLTAGE_SCALE;
    p_foc->voltageScalerReg = p_foc->voltageScaler / 0x10000L; // unsigned 16 bits resolution

//    p_foc->adcScaler = ADC_POT_VOLTAGE_SCALE;
//    p_foc->adcScalerReg = p_foc->adcScaler / 0x10000L; // unsigned 16 bits resolution

    // Initialize duty cycle saturation
    p_foc->dutyCycleSat = 0.4;

    // Initialize control loop low-pass filter
    LPF_setBandwidth(&p_foc->controlLoopLpf, PWM_FREQUENCY, M1_CUR_LOOP_BANDWIDTH);


    // Initialize motor constants
    if (motorIndex == 0)
    {
        p_foc->Rs = M1_RS;
        p_foc->Ld = M1_LD;
        p_foc->Lq = M1_LQ;
        p_foc->ke = M1_KE;
    } else {
        p_foc->Rs = M2_RS;
        p_foc->Ld = M2_LD;
        p_foc->Lq = M2_LQ;
        p_foc->ke = M2_KE;
    }

    // Initialize PWM compare register scaler
    p_foc->pwmCompScaler = PWM_HALF_PERIOD * 0x10000L; // 32 bits registers in HRPWM device function compare register

    // Initialize align routine
    ALIGN_reset(&p_foc->alignRoutine);
    p_foc->alignRoutine.max = PWM_FREQUENCY; // Stay at max Id for a second
    p_foc->alignRoutine.idAlignRef = 0.4;    // 400mA alignment current

    // Initialize encoder
    ENC_reset(&p_foc->encoder);

    if (motorIndex == 0)
    {
        p_foc->encoder.p_qepRegs = (struct EQEP_REGS *)MOTOR1_QEP_BASE;
        p_foc->encoder.mechScaler = 0.25 / QEP_MOTOR1_ENCODER_LINES_NB;
        p_foc->encoder.polePairs = M1_POLES_PAIRS;
    } else {
        p_foc->encoder.p_qepRegs = (struct EQEP_REGS *)MOTOR2_QEP_BASE;
        p_foc->encoder.mechScaler = 0.25 / QEP_MOTOR2_ENCODER_LINES_NB;
        p_foc->encoder.polePairs = M2_POLES_PAIRS;
    }

    // Speed estimation initialization
    p_foc->encoder.speedEst.highSpeedScaler = 2.0 * FM_PI * QEP_UNIT_TIMER_FREQ; // 2pi / dt
    p_foc->encoder.speedEst.lowSpeedScaler = 2.0 * FM_PI * p_foc->encoder.mechScaler * DEVICE_SYSCLK_FREQ * 4.0 / 128.0; // 4 is pos divider and 128 is clock divider
    LPF_setBandwidth(&p_foc->encoder.speedEst.lpf, PWM_FREQUENCY, 200); // 200Hz low-pass filter for velocity estimation

    // Initialize PI controllers
    if (motorIndex == 0)
    {
        p_foc->piId.kp = M1_LD * 2.0*FM_PI*M1_CUR_LOOP_BANDWIDTH;
        p_foc->piId.ki = M1_RS * 2.0*FM_PI*M1_CUR_LOOP_BANDWIDTH / PWM_FREQUENCY;
        p_foc->piIq.kp = M1_LQ * 2.0*FM_PI*M1_CUR_LOOP_BANDWIDTH;
        p_foc->piIq.ki = M1_RS * 2.0*FM_PI*M1_CUR_LOOP_BANDWIDTH / PWM_FREQUENCY;
    } else {
        p_foc->piId.kp = M2_LD * 2.0*FM_PI*M2_CUR_LOOP_BANDWIDTH;
        p_foc->piId.ki = M2_RS * 2.0*FM_PI*M2_CUR_LOOP_BANDWIDTH / PWM_FREQUENCY;
        p_foc->piIq.kp = M2_LQ * 2.0*FM_PI*M2_CUR_LOOP_BANDWIDTH;
        p_foc->piIq.ki = M2_RS * 2.0*FM_PI*M2_CUR_LOOP_BANDWIDTH / PWM_FREQUENCY;
    }

    // Controller saturations is 40% of bus voltage (10% margin). This is completely arbitrary.
    // DRVs must be initialized before CL_initFoc is called so vBus isn't zero
    if (motorIndex == 0)
    {
        p_foc->vBus = ADC_readResult(M12_VBUS_ADC_ADDR, M12_VBUS_SOC_NUM) * p_foc->voltageScalerReg;
    } else {
        p_foc->vBus = ADC_readResult(M12_VBUS_ADC_ADDR, M12_VBUS_SOC_NUM) * p_foc->voltageScalerReg;
    }

    p_foc->piId.sat = 0.4 * p_foc->vBus;
    p_foc->piIq.sat = 0.4 * p_foc->vBus;

    // Initialize position PD controller
    PD_reset(&p_foc->posPd);
    p_foc->posPd.kp = 1.0f;
    p_foc->posPd.kd = 0.02f;
    p_foc->posPd.sat = 4.0f; // 4 amps saturation

    // Initialize resistance estimation
    LPF_setBandwidth(&p_foc->resEstLpf, PWM_FREQUENCY, 0.2); // Very slow low-pass filter for resistance estimation

    // Initialize measures PWM and interrupt registers
    if (motorIndex == 0)
    {
        p_foc->pwmCompA = (volatile uint32_t *)(MOTOR1_PWM1_BASE + EPWM_O_CMPA);
        p_foc->pwmCompB = (volatile uint32_t *)(MOTOR1_PWM2_BASE + EPWM_O_CMPA);
        p_foc->pwmCompC = (volatile uint32_t *)(MOTOR1_PWM3_BASE + EPWM_O_CMPA);

        p_foc->isenAResult   = (volatile int32_t *)(M1_IA_ADC_PPB_RESULT_ADDR);
        p_foc->isenBResult   = (volatile int32_t *)(M1_IB_ADC_PPB_RESULT_ADDR);
        p_foc->isenCResult   = (volatile int32_t *)(M1_IC_ADC_PPB_RESULT_ADDR);
        p_foc->vBusSenResult = (volatile int32_t *)(M12_VBUS_ADC_PPB_RESULT_ADDR);

        p_foc->vsenaResult = (volatile int32_t *)(M1_VA_ADC_PPB_RESULT_ADDR);
        p_foc->vsenbResult = (volatile int32_t *)(M1_VB_ADC_PPB_RESULT_ADDR);
        p_foc->vsencResult = (volatile int32_t *)(M1_VC_ADC_PPB_RESULT_ADDR);

        p_foc->adcIntFlag = (union ADCINTFLG_REG *)(M1_IC_ADC_ADDR + ADC_O_INTFLG);
    }
    else
    {
        p_foc->pwmCompA = (volatile uint32_t *)(MOTOR2_PWM1_BASE + EPWM_O_CMPA);
        p_foc->pwmCompB = (volatile uint32_t *)(MOTOR2_PWM2_BASE + EPWM_O_CMPA);
        p_foc->pwmCompC = (volatile uint32_t *)(MOTOR2_PWM3_BASE + EPWM_O_CMPA);

        p_foc->isenAResult   = (volatile int32_t *)(M2_IA_ADC_PPB_RESULT_ADDR);
        p_foc->isenBResult   = (volatile int32_t *)(M2_IB_ADC_PPB_RESULT_ADDR);
        p_foc->isenCResult   = (volatile int32_t *)(M2_IC_ADC_PPB_RESULT_ADDR);

#if OMODRI_PAL_SETUP    // Second VSENVM measure is replaced by ISENVM, so use the same one than motor 1
        p_foc->vBusSenResult = (volatile int32_t *)(MOTOR1_VBUS_RESULT  + ADC_PPBxRESULT_OFFSET_BASE + 2 * MOTOR1_ADC_PPB_NUM1);
#else
        p_foc->vBusSenResult = (volatile int32_t *)(M12_VBUS_ADC_PPB_RESULT_ADDR);
#endif
        p_foc->vsenaResult = (volatile int32_t *)(M2_VA_ADC_PPB_RESULT_ADDR);
        p_foc->vsenbResult = (volatile int32_t *)(M2_VB_ADC_PPB_RESULT_ADDR);
        p_foc->vsencResult = (volatile int32_t *)(M2_VC_ADC_PPB_RESULT_ADDR);

        p_foc->adcIntFlag = (union ADCINTFLG_REG *)(M2_IC_ADC_ADDR + ADC_O_INTFLG);
    }
}

// Control loop reset
void CL_resetFoc(FOC* p_foc)
{
    // Reset controllers
    PI_reset(&p_foc->piId);
    PI_reset(&p_foc->piIq);

    // Reset alignment routine
    ALIGN_reset(&p_foc->alignRoutine);

    // Reset position PD controller
    PD_reset(&p_foc->posPd);

    // Reset encoder
    ENC_reset(&p_foc->encoder);

    // Reset currents and voltages
    p_foc->ia = 0.0f;
    p_foc->ib = 0.0f;
    p_foc->ic = 0.0f;

    p_foc->ialpha = 0.0f;
    p_foc->ibeta = 0.0f;

    p_foc->id = 0.0f;
    p_foc->iq = 0.0f;

    p_foc->vd = 0.0f;
    p_foc->vq = 0.0f;

    p_foc->valpha = 0.0f;
    p_foc->vbeta = 0.0f;

    p_foc->va = 0.0f;
    p_foc->vb = 0.0f;
    p_foc->vc = 0.0f;

    // Reset potentiometer voltage
    p_foc->auxAdc = 0.0f;

    // Reset resistance estimation
    p_foc->phaseAVolt = 0.0f;
    p_foc->phaseBVolt = 0.0f;
    p_foc->phaseCVolt = 0.0f;

    p_foc->resistanceEst = 0.0f;
}

