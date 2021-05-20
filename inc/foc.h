#ifndef __FOC_H__
#define __FOC_H__

#include "driverlib.h"
#include "device.h"

//#include "pid.h"
//#include <stdint.h>
#include "lowpass_filter.h"
#include "hal.h"

#include "pd.h"
#include "pi.h"

#include "align_routine.h"
#include "encoder.h"
#include "math_defines.h"
#include "lowpass_filter.h"
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

//-------------------------------------------------------------------------------//
//                                 FOC STRUCTURE                                 //
//-------------------------------------------------------------------------------//

typedef struct __FOC__
{
    // Enable flag
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

    // Bus voltage
    float32_t vBus; // [V]

    // Rotor electrical angle sine cosine
    float32_t sinTheta;
    float32_t cosTheta;

    // Low-pass control loop filter
    LowpassFilter controlLoopLpf;

    // Encoder
    ENCODER encoder;

    // Current transforms
    float32_t ia;       // [A]
    float32_t ib;       // [A]
    float32_t ic;       // [A]

    float32_t ialpha;   // [A]
    float32_t ibeta;    // [A]

    float32_t id;       // [A]
    float32_t iq;       // [A]

    // Voltage transforms
    float32_t vd;       // [V]
    float32_t vq;       // [V]

    float32_t valpha;   // [V]
    float32_t vbeta;    // [V]

    float32_t va;       // [V]
    float32_t vb;       // [V]
    float32_t vc;       // [V]
    float32_t vbus;     // [V]

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

    // ADC result registers
    volatile int32_t *isenAResult;
    volatile int32_t *isenBResult;
    volatile int32_t *isenCResult;
    volatile int32_t *vBusSenResult;

    volatile int32_t *vsenaResult;
    volatile int32_t *vsenbResult;
    volatile int32_t *vsencResult;
    volatile int32_t *potResult;

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
inline void FOC_getPosition(FOC*, MOTOR_HAL_CFG*);
inline void FOC_getPhaseCurrent(FOC*, MOTOR_HAL_CFG*);
inline void FOC_getPhaseVoltage(FOC*, MOTOR_HAL_CFG*);
inline void FOC_getBusVoltage(FOC*, MOTOR_HAL_CFG*);

inline void FOC_getMeasures(FOC*, MOTOR_HAL_CFG*);
inline void FOC_sineCosine(FOC *foc);
inline void FOC_parkClarke(FOC *foc);

inline void FOC_runPi(FOC *foc);

inline void FOC_invParkClarke(FOC *foc);
inline void FOC_svm(FOC *foc);

inline void FOC_resistanceEstimation(FOC *foc);

#endif
