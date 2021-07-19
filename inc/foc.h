#ifndef __FOC_H__
#define __FOC_H__

#include <math.h>

#include "driverlib.h"
#include "device.h"
#include "f2838x_device.h"

//#include "hal.h"
//#include "motor.h"
//#include "align_routine.h"
//#include "communication_interface.h"
//#include "motor.h"
//#include "lowpass_filter.h"
#include "encoder.h"
#include "anticoagging.h"
#include "uomodri_user_defines.h"

//#ifndef FM_PI
//#define FM_PI 3.14159265358979f
//#endif
//
//#ifndef FM_SQRT3
//#define FM_SQRT3 1.7320508075689f
//#endif
//
//#ifndef FM_ISQRT3
//#define FM_ISQRT3 0.57735026919f
//#endif
//
//#ifndef FM_RPM2RADPS
//#define FM_RPM2RADPS (2.0 * FM_PI / 60.0)
//#endif
//
//#define FM_ABS(x)   ((x) >= 0.0f ? (x) : -(x))

/*********************************************************************************/
/**                                FOC STRUCTURE                                **/
/*********************************************************************************/
typedef struct __PID_CONTROLLER__
{
    float32_t err;          // Current error
    float32_t vErr;         // Previous error
    float32_t deltaT;       // Time step

    float32_t kp;           // Proportional gain
    float32_t ki;           // Integral gain
    float32_t kd;           // Derivative gain

    float32_t integral;     // Integral term partial computation
    float32_t derivative;   // Derivative term partial computation
    float32_t ff;           // Feed-forward term
    float32_t out;          // Controller output
    float32_t* sat;         // Controller saturation
} PID_CONTROLLER;

typedef struct __MOTOR_PARAMETERS__
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

typedef struct __MOTOR_ACQUISITIONS__
{
    // ADC result registers
    volatile uint16_t*  vaMeasReg;
    volatile uint16_t*  vbMeasReg;
    volatile uint16_t*  vcMeasReg;
    volatile uint16_t*  vBusMeasReg;
    volatile int16_t*   iaMeasReg;
    volatile int16_t*   ibMeasReg;
    volatile int16_t*   icMeasReg;

    // ADC values scaled
    float32_t va;   // [V]
    float32_t vb;   // [V]
    float32_t vc;   // [V]
    float32_t vbus; // [V]
    float32_t ia;   // [A]
    float32_t ib;   // [A]
    float32_t ic;   // [A]
    float32_t vmax; // [V]
    float32_t imax; // [A]
    LpfStruct vBusLpf;
    // ADC conversion done
    bool measNew;
} MOTOR_ACQ;

typedef struct __FOC__
{
    const MOTOR_ID      motorID;
    // Motor configuration parameters
    const MOTOR_PARAMS  motorParams;
    // Current & Voltage measures
    MOTOR_ACQ           motor_acq;
    // Encoder position & speed
    ENCODER             motor_enc;

    // Low-pass control loop filter
    LpfStruct           iParkLpf;

    // Rotor electrical angle sine/cosine
    float32_t           sinTheta;
    float32_t           cosTheta;
    // Direct Clarke transform result
    float32_t           ialpha; // [A]
    float32_t           ibeta;  // [A]
    // Direct Park transform result
    float32_t           id;     // [A]
    float32_t           iq;     // [A]
    // Inverse Park transform result
    float32_t           ud;     // [V]
    float32_t           uq;     // [V]
    // Inverse Clarke transform result
    float32_t           ualpha; // [V]
    float32_t           ubeta;  // [V]
    // Bus voltage
    //float32_t*          uBus;   // [V]
    // Phase commands
    float32_t           ua;     // [V]
    float32_t           ub;     // [V]
    float32_t           uc;     // [V]

    // PI set point values
    float32_t           idRef;  // [A]
    float32_t           iqRef;  // [A]
    // PI controllers for current control
    PID_CONTROLLER      piId;
    PID_CONTROLLER      piIq;

    //PD set point values
    float32_t           posRef; // [rpm/s]
    // PD controller for position control
    PID_CONTROLLER      pdPos;

    // Enable flag
    bool                enabled;
    // Alignment routine structure
//    AlignRoutine        alignRoutine;

    // Utility variables
//    float32_t currentScaler;    // [A/pu]
//    float32_t currentScalerReg; // [A/register]
//    float32_t voltageScaler;    // [V/pu]
//    float32_t voltageScalerReg; // [V/register]
//    float32_t adcScaler;        // [V/pu]
//    float32_t adcScalerReg;     // [V/register]
    float32_t pwmCompScaler;    // [register/pu]

    // Duty-cycle saturation
    float32_t dtcMax;
    float32_t dtcMin;

    // Motor constants
//    float32_t* Rs; // Stator resistance [Ohm]
//    float32_t* Ld; // Direct axis inductance [H]
//    float32_t* Lq; // Quadratic axis inductance [H]
//    float32_t* ke; // Motor back EMF constant [V/rad/s] (electrical radians)


    // Angles
//    float32_t* mechAngle; // [pu]
//    float32_t* elecAngle; // [pu]


    // Duty cycles
    float32_t dtcPhaseA;    // [pu] between -0.5 and 0.5
    float32_t dtcPhaseB;    // [pu] between -0.5 and 0.5
    float32_t dtcPhaseC;    // [pu] between -0.5 and 0.5


//    // PI controllers
//    float32_t idRef;    // [A]
//    float32_t iqRef;    // [A]
//
////    PI_controller piId;
////    PI_controller piIq;
//
//    PID_controller piId;
//    PID_controller piIq;
//
//    // Position PD controller
//    PD_controller posPd;

    // Phase measured voltages
    float32_t phaseAVolt;   // [V]
    float32_t phaseBVolt;   // [V]
    float32_t phaseCVolt;   // [V]

    // Q-axis resistance estimation
    float32_t resistanceEst;    // [Ohm]
    // Resistance estimation low-pass filter
    LpfStruct resEstLpf;

    // Anti-coagging structure
    Anticoagging acog;

    // Auxiliary ADC voltage
    float32_t auxAdc;   // [V]

    // PWM result registers
    volatile uint16_t *pwmCompA;
    volatile uint16_t *pwmCompB;
    volatile uint16_t *pwmCompC;

    // ADC interrupt flag register
    volatile union ADCINTFLG_REG *adcIntFlag;
} FOC;


//-------------------------------------------------------------------------------//
//                                    DEFINES                                    //
//-------------------------------------------------------------------------------//

// Resistance estimation uses Ohm's Law R = U/I so we have to ensure I != 0.
// This threshold corresponds to the minimum current for the measure to be considered as correct.
#define FOC_RE_CURRENT_THRESHOLD    0.1f


//-------------------------------------------------------------------------------//
//                             FUNCTIONS DECLARATION                             //
//-------------------------------------------------------------------------------//

//inline void FOC_getPosition(FOC*, MOTOR_HAL_CFG*);
inline void FOC_getMeasures(MOTOR_ACQ*);
//inline void FOC_getPhaseCurrent(MOTOR_ACQ*);
//inline void FOC_getPhaseVoltage(MOTOR_ACQ*);
//inline void FOC_getBusVoltage(MOTOR_ACQ*);
//inline void FOC_getMeasures(FOC*, MOTOR_HAL_CFG*);
//inline void FOC_sineCosine(FOC*);
//inline void FOC_parkClarke(FOC*);
//inline void FOC_runPi(FOC*);
//inline void FOC_invParkClarke(FOC*);
//inline void FOC_svm(FOC*);
inline void FOC_control(FOC*);
inline void FOC_resistanceEstimation(FOC*);
//static inline void FOC_PID(PID_CONTROLLER*);
inline float32_t FOC_pi(PID_CONTROLLER*);
inline float32_t FOC_pd(PID_CONTROLLER*);
void FOC_resetStruct(FOC*);
void limit_norm(float*, float*, float);

#endif
