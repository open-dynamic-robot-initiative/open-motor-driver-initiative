#ifndef __FOC_H__
#define __FOC_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#ifdef C28_CORE
#include "f2838x_device.h"
#include "f2838x_struct.h"
#include "f2838x_examples.h"
#include "driverlib.h"
#include "device.h"

#include "uomodri_user_defines.h"
#elif defined CM_CORE
#include "driverlib_cm.h"
#include "cm.h"

#include "uomodri_cm_user_defines.h"
#endif
#include "encoder.h"
#ifdef C28_CORE
#include "anticoagging.h"
#endif

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
typedef enum
{
    ALIGN_COUNT_UP,
    ALIGN_STAY,
    ALIGN_COUNT_DOWN,
    ALIGN_DONE
} AlignState;

typedef struct __PI_STRUCT__
{
    float32_t           in;         // Loopback current
    float32_t           set;        // Setpoint current
    float32_t           err;        // Current error

    float32_t           kp;         // Proportional gain
    float32_t           ki;         // Integral gain

    float32_t           integral;   // Integral term partial computation
    float32_t           ff;         // Feed-forward term
    float32_t           out;        // Controller output
    float32_t           sat;        // Controller saturation
} PI_STRUCT;

typedef struct __PD_STRUCT__
{
    float32_t           inTheta;    // Loopback position
    float32_t           setTheta;   // Setpoint position
    float32_t           errTheta;   // Error position
    float32_t           inSpeed;    // Loopback speed
    float32_t           setSpeed;   // Setpoint speed
    float32_t           errSpeed;   // Error speed

    float32_t           kp;         // Proportional gain
    float32_t           kd;         // Derivative gain

    float32_t           derivative; // Derivative term partial computation
    float32_t           ff;         // Feed-forward term
    float32_t           out;        // Controller output
    float32_t           sat;        // Controller saturation
} PD_STRUCT;

typedef struct __PARAMETERS_STRUCT__
{
    // Motor constants
    float32_t           Rs;         // Stator resistance [Ohm]
    float32_t           Ld;         // Direct axis inductance [H]
    float32_t           Lq;         // Quadratic axis inductance [H]
    float32_t           kv;         // Motor speed constant [RPM/V or rad/s/V]
    float32_t           ke;         // Motor back EMF constant [V/rad/s] (electrical radians) = 1/kv
    float32_t           ki;         // Motor torque constant [N.m/A]
    const uint8_t       polePairs;  // Number of pole pairs
} PARAMS_STRUCT;

typedef struct __ACQUISITIONS_STRUCT__
{
#ifdef C28_CORE
    volatile struct ADC_REGS*           const p_adcRegs[MAX_ADC];
    volatile struct ADC_RESULT_REGS*    const p_adcResultRegs[MAX_ADC];
#elif defined CM_CORE
    uint32_t                            rsvd1[5];
    uint32_t                            rsvd2[5];
#endif
    // ADC result registers
    volatile uint16_t*                  const p_vaMeasReg;
    volatile uint16_t*                  const p_vbMeasReg;
    volatile uint16_t*                  const p_vcMeasReg;
    volatile uint16_t*                  const p_vBusMeasReg;
    volatile int16_t*                   const p_iaMeasReg;
    volatile int16_t*                   const p_ibMeasReg;
    volatile int16_t*                   const p_icMeasReg;
    // ADC values scaled
    float32_t                           va;         // [V]
    float32_t                           vb;         // [V]
    float32_t                           vc;         // [V]
    float32_t                           vbus;       // [V]
    float32_t                           ia;         // [A]
    float32_t                           ib;         // [A]
    float32_t                           ic;         // [A]
    float32_t                           auxAcq;
    LPFSTRUCT                           vBusLpf;

//    bool                                measNew;    // ADC conversion done
} ACQ_STRUCT;

typedef struct __ALIGN_STRUCT__
{
    AlignState              state;

    uint32_t                cnt;
    uint32_t                cntMax;

    float32_t               idAlignRef;
    float32_t               idAlign;
} ALIGN_STRUCT;

typedef struct __FOC_STRUCT__
{
    const ID_ENUM       motor_id;
    // Motor configuration parameters
    PARAMS_STRUCT       motor_cfg;
//    const PARAMS_STRUCT motor_cfg;
    // Current & Voltage measures
    ACQ_STRUCT          motor_acq;
    // Encoder position & speed
    ENC_STRUCT          motor_enc;
    // Alignment calibration procedure
    ALIGN_STRUCT        motor_align;
    // Low-pass control loop filter
    LPFSTRUCT           iParkLpf;
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
    PI_STRUCT           piId;
    PI_STRUCT           piIq;
    // PD set point values
    //float32_t           posRef; // [rpm/s]
    // PD controller for position control
    PD_STRUCT           pdPos;

    // Q-axis resistance estimation
    float32_t           resistanceEst;    // [Ohm]
    // Resistance estimation low-pass filter
    LPFSTRUCT           resEstLpf;
    // Anti-coagging structure
#ifdef C28_CORE
    Anticoagging        acog;
#elif defined CM_CORE
    uint8_t             rsvd[11];
#endif
    // Error messages
    uint16_t            errorMsg;
    // Enable flag
//    bool                motor_enable;
//    bool                system_enable;

    // Phase measured voltages
//    float32_t phaseAVolt;   // [V]
//    float32_t phaseBVolt;   // [V]
//    float32_t phaseCVolt;   // [V]

    // Auxiliary ADC voltage
//    float32_t auxAdc;   // [V]

    // ADC interrupt flag register
//    volatile union ADCINTFLG_REG *adcIntFlag;
} FOC_STRUCT;


//-------------------------------------------------------------------------------//
//                                    DEFINES                                    //
//-------------------------------------------------------------------------------//

// Resistance estimation uses Ohm's Law R = U/I so we have to ensure I != 0.
// This threshold corresponds to the minimum current for the measure to be considered as correct.
#define FOC_RE_CURRENT_THRESHOLD    0.1f


//-------------------------------------------------------------------------------//
//                             FUNCTIONS DECLARATION                             //
//-------------------------------------------------------------------------------//
inline void FOC_getMeasures(ACQ_STRUCT*);
inline void FOC_resistanceEstimation(FOC_STRUCT*);
inline void FOC_runControl(FOC_STRUCT*);
inline void FOC_resetPI(PI_STRUCT*);
inline float32_t FOC_runPI(PI_STRUCT*);
inline void FOC_resetPD(PD_STRUCT*);
inline float32_t FOC_runPD(PD_STRUCT*);
void FOC_resetStruct(FOC_STRUCT*);
void limit_norm(float32_t*, float32_t*, float32_t);

#endif
