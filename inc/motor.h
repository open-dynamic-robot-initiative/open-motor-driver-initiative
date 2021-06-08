/*
 * motor.h
 *
 *  Created on: 20 mai 2021
 *      Author: jmanhes
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "driverlib.h"
#include "device.h"
#include "f2838x_device.h"

//#include "pid.h"
//#include <stdint.h>
//#include "lowpass_filter.h"
#include <math.h>
#include "hal.h"

#include "pd.h"
#include "pi.h"

#include "align_routine.h"
#include "foc.h"
#include "encoder.h"
#include "math_defines.h"
#include "lowpass_filter.h"
#include "anticoagging.h"
#include "uomodri_user_defines.h"

typedef struct __MOTOR_PARAMS__
{
    // Motor constants
    float32_t Rs;       // Stator resistance [Ohm]
    float32_t Ld;       // Direct axis inductance [H]
    float32_t Lq;       // Quadratic axis inductance [H]
    float32_t kv;       // Motor speed constant [RPM/V or rad/s/V]
    float32_t ke;       // Motor back EMF constant [V/rad/s] (electrical radians) = 1/kv
    float32_t ki;       // Motor torque constant [N.m/A]
    uint8_t polePairs;  // Number of pole pairs
} MOTOR_PARAMS;

typedef struct __FILTER_PARAMS__
{
    float32_t b;
    float32_t a;
} FILTER_PARAMS;

typedef struct __MOTOR_MEASURES__
{
    const MotorID motorID;
    float32_t va;       // [V]
    float32_t vb;       // [V]
    float32_t vc;       // [V]
    float32_t vbus;     // [V]

    float32_t ia;       // [A]
    float32_t ib;       // [A]
    float32_t ic;       // [A]

    float32_t* mechanicalAngle; // between 0 and 1

    // ADC result registers
    volatile int32_t* vaMeasReg;
    volatile int32_t* vbMeasReg;
    volatile int32_t* vcMeasReg;
    volatile int32_t* vBusMeasReg;

    volatile int32_t* iaMeasReg;
    volatile int32_t* ibMeasReg;
    volatile int32_t* icMeasReg;

    volatile int32_t* encoderPosReg;
} MOTOR_MEASURES;

typedef struct __MOTOR_STRUCT__
{
    const MotorID           motorID;
    const MOTOR_HAL_CFG*    motorHalCfg;
    const MOTOR_PARAMS*     motorParams;
    MOTOR_MEASURES*         motorMeas;
    ENCODER*                motorEncoder;
    DRV8353*                motorDRV;
    FOC*                    motorFOC;

    bool enabled;
    // Alignment routine structure
    AlignRoutine alignRoutine;

    // Utility variables
    float32_t currentScaler;    // [A/pu]
    float32_t currentScalerReg; // [A/register]
    float32_t voltageScaler;    // [V/pu]
    float32_t voltageScalerReg; // [V/register]
    float32_t adcScaler;        // [V/pu]
    float32_t adcScalerReg;     // [V/register]
    float32_t pwmCompScaler;    // [register/pu]

    // Duty-cycle saturation
    float32_t dutyCycleSat;

    // Motor constants
    float32_t Rs; // Stator resistance [Ohm]
    float32_t Ld; // Direct axis inductance [H]
    float32_t Lq; // Quadratic axis inductance [H]
    float32_t ke; // Motor back EMF constant [V/rad/s] (electrical radians)

     // Rotor electrical angle sine cosine
    float32_t sinTheta;
    float32_t cosTheta;

    // Low-pass control loop filter
    LowpassFilter controlLoopLpf;

    // Encoder
//    ENCODER encoder;

      // Duty cycles
    float32_t ducya;    // [pu] between -0.5 and 0.5
    float32_t ducyb;    // [pu] between -0.5 and 0.5
    float32_t ducyc;    // [pu] between -0.5 and 0.5


    // PI controllers
    float32_t idRef;    // [A]
    float32_t iqRef;    // [A]

    PI_controller piId;
    PI_controller piIq;

    // Position PD controller
    PD_controller posPd;

    // Phase measured voltages
    float32_t phaseAVolt;   // [V]
    float32_t phaseBVolt;   // [V]
    float32_t phaseCVolt;   // [V]

    // Q-axis resistance estimation
    float32_t resistanceEst;    // [Ohm]
    // Resistance estimation low-pass filter
    LowpassFilter resEstLpf;

    // Anti-coagging structure
    Anticoagging acog;

    // Auxiliary ADC voltage
    float32_t auxAdc;   // [V]

    // PWM result registers
    volatile uint32_t *pwmCompA;
    volatile uint32_t *pwmCompB;
    volatile uint32_t *pwmCompC;

    // ADC interrupt flag register
    volatile union ADCINTFLG_REG *adcIntFlag;
} MOTOR_STRUCT;

#endif /* INC_MOTOR_H_ */
