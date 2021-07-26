#ifndef __FOC_H__
#define __FOC_H__

#include <math.h>
#include <stdbool.h>

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
//typedef struct __PID_CONTROLLER__
//{
//    float32_t* in;          // pointer on the loopback current
//    float32_t* set;         // pointer on the setpoint current
//    float32_t err;          // Current error
//    float32_t vErr;         // Previous error
//    float32_t deltaT;       // Time step
//
//    float32_t kp;           // Proportional gain
//    float32_t ki;           // Integral gain
//    float32_t kd;           // Derivative gain
//
//    float32_t integral;     // Integral term partial computation
//    float32_t derivative;   // Derivative term partial computation
//    float32_t ff;           // Feed-forward term
//    float32_t out;          // Controller output
//    float32_t* sat;         // Controller saturation
//} PID_CONTROLLER;

typedef struct __PI_CONTROLLER__
{
    float32_t* in;          // pointer on the loopback current
    float32_t* set;         // pointer on the setpoint current
    float32_t err;          // Current error

    float32_t kp;           // Proportional gain
    float32_t ki;           // Integral gain

    float32_t integral;     // Integral term partial computation
    float32_t ff;           // Feed-forward term
    float32_t out;          // Controller output
    float32_t* sat;         // Controller saturation
} PI_CONTROLLER;

typedef struct __PD_CONTROLLER__
{
    float32_t* inTheta;     // pointer on the loopback position
    float32_t* setTheta;    // pointer on the setpoint position
    float32_t errTheta;     // position error
    float32_t* inSpeed;     // pointer on the loopback speed
    float32_t* setSpeed;    // pointer on the setpoint speed
    float32_t errSpeed;     // speed error

    float32_t kp;           // Proportional gain
    float32_t kd;           // Derivative gain

    float32_t derivative;   // Derivative term partial computation
    float32_t ff;           // Feed-forward term
    float32_t out;          // Controller output
    float32_t* sat;         // Controller saturation
} PD_CONTROLLER;

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
    LpfStruct vBusLpf;
    // ADC conversion done
    bool    measNew;
} MOTOR_ACQ;

typedef struct __FOC__
{
    const MOTOR_ID      motorID;
    // Motor configuration parameters
    const MOTOR_PARAMS  motor_cfg;
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
    // Phase commands
    float32_t           ua;     // [V]
    float32_t           ub;     // [V]
    float32_t           uc;     // [V]


    float32_t           vmax; // [V]
    float32_t           imax; // [A]
    // Duty-cycle saturation
    float32_t           dtcMax;
    float32_t           dtcMin;
    // Duty cycles
    float32_t           dtc_u;  // [pu] between -0.5 and 0.5
    float32_t           dtc_v;  // [pu] between -0.5 and 0.5
    float32_t           dtc_w;  // [pu] between -0.5 and 0.5

    // PI set point values
    float32_t           idRef;  // [A]
    float32_t           iqRef;  // [A]
    // PI controllers for current control
    PI_CONTROLLER       piId;
    PI_CONTROLLER       piIq;
    // PD set point values
    float32_t           posRef; // [rpm/s]
    // PD controller for position control
    PD_CONTROLLER       pdPos;

    // Q-axis resistance estimation
    float32_t           resistanceEst;    // [Ohm]
    // Resistance estimation low-pass filter
    LpfStruct           resEstLpf;
    // Anti-coagging structure
    Anticoagging        acog;
    // Enable flag
    bool                enabled;

    // Phase measured voltages
//    float32_t phaseAVolt;   // [V]
//    float32_t phaseBVolt;   // [V]
//    float32_t phaseCVolt;   // [V]

    // Auxiliary ADC voltage
//    float32_t auxAdc;   // [V]

    // ADC interrupt flag register
//    volatile union ADCINTFLG_REG *adcIntFlag;
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
inline void FOC_getMeasures(MOTOR_ACQ*);
inline void FOC_resistanceEstimation(FOC*);
inline void FOC_runControl(FOC*);
inline void FOC_resetPI(PI_CONTROLLER*);
inline float32_t FOC_runPI(PI_CONTROLLER*);
inline void FOC_resetPD(PD_CONTROLLER*);
inline float32_t FOC_runPD(PD_CONTROLLER*);
void FOC_resetStruct(FOC*);
void limit_norm(float32_t*, float32_t*, float32_t);

#endif
