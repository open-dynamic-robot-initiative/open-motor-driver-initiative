#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include <math.h>
#include "driverlib.h"
#include "device.h"
#include "f2838x_device.h"

#include "hal.h"
#include "math_defines.h"
#include "uomodri_user_defines.h"

// This threshold corresponds to the middle speed for low/high speed estimation merge
// See the ENC_update function for more details
#define SPEED_ESTIMATION_THRESHOLD      80.4f // [rad/s]

typedef struct __SpeedEst__
{
    // High speed estimation
    float32_t highSpeedScaler;// [rad/s/pu], diff_angular * scaler = velocity
    float32_t highSpeed;    // [rad/s]
    // Low speed estimation
    float32_t lowSpeedScaler; // [rad.register], scaler / dtRegister = velocity
    float32_t lowSpeed;     // [rad/s]

    // Speed merge
    float32_t alpha;     // Speed merge coefficient
    float32_t refSpeed;  // [rad/s] Merged of low speed and high speed estimation

    // Low-pass filter
    LpfStruct lpf;

    // Output speeds
    float32_t mechSpeed; // [rad/s]
    float32_t elecSpeed; // [rad/s]

    // Utility variables
    float32_t theta[2]; // [pu]
    bool      thetaDir; // 0 = CCW/reverse, 1 = CW/forward
} SpeedEst;

typedef struct __Encoder__
{
    // pointers on HAL structure, QEP registers and direct access to QEP counter register
    const eQEP_TypeDef* p_qepHandle;
    struct EQEP_REGS*   p_qepRegs;
//    volatile int32_t*   p_qepCntPosReg;

    const uint8_t       polePairs;    // Number of pole pairs of the motor
    // Scalers
    const float32_t     mechScaler; // [pu/register]
    // Angles
    float32_t           thetaElec; // [pu]
    float32_t           thetaMech[2]; // [pu]
    bool                thetaDir; //0 = CCW/reverse, 1 = CW/forward
    bool                thetaNew; // New sample available?
    // Indexed angles
    bool                isIndexCompensated;   // Is index compensation activated
    float32_t           indexAngle;       // Index mech angle
    float32_t           indexedMechAngle; // Mech angle relative to the index [pu]
    float32_t           absoluteAngle; // [turns]

    // Speed estimation structure
    SpeedEst            speedEst;
    // Communication variables
    int16_t             turnNb;     // Number of full turns (signed)
    bool                indexDetected; // Has index been detected
    bool                indexToggle;   // Flag that toggles every time index is seen
    bool                rollOverFlag;  // Roll-over error flag
} ENCODER;

inline void encoder_resetStruct(ENCODER*);
inline void encoder_SetPosition(ENCODER*, uint32_t);
inline void encoder_GetPosition(ENCODER*);
inline void encoder_GetSpeed(ENCODER*);

#endif
