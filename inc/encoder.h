#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include "driverlib.h"
#include "device.h"
#include "f2838x_device.h"
//#include <f28x_project.h>
#include "lowpass_filter.h"
#include "math_defines.h"

#include "uomodri_user_defines.h"

// This threshold corresponds to the middle speed for low/high speed estimation merge
// See the ENC_update function for more details
#define SPEED_ESTIMATION_THRESHOLD      80.4f // [rad/s]

typedef struct __SpeedEst__
{
    // High speed estimation
    float32_t highSpeedScaler;  // [rad/s/pu], dAngle * scaler = velocity
    float32_t highMechSpeedEst; // [rad/s]
    // Low speed estimation
    float32_t lowSpeedScaler;  // [rad.register], scaler / dtRegister = velocity
    float32_t lowMechSpeedEst; // [rad/s]

    // Speed merge
    float32_t alpha;     // Speed merge coefficient
    float32_t speedRef;  // [rad/s] Merged of low speed and high speed estimation

    // Low-pass filter
    LowpassFilter lpf;

    // Output speeds
    float32_t mechSpeed; // [rad/s]
    float32_t elecSpeed; // [rad/s]

    // Utility variables
    float32_t angle[2];     // [pu]
} SpeedEst;

typedef struct __Encoder__
{
    const MotorID       motorID;
    // pointers on HAL structure, QEP registers and direct access to QEP counter register
    const eQEP_TypeDef* p_qepHandle;
    struct EQEP_REGS*   p_qepRegs;
    volatile int32_t*   p_encoderPosReg;
    // Scalers
    const float32_t     mechAngleNorm; // [pu/register]
    const uint8_t*      p_polePairs;  // Number of pole pairs of the motor

    // Angles
    float32_t elecAngle; // [pu]
    float32_t mechAngle; // [pu]

    // Indexed angles
    bool  isIndexCompensated;   // Is index compensation activated
    float32_t indexAngle;       // Index mech angle
    float32_t indexedMechAngle; // Mech angle relative to the index [pu]

    float32_t absoluteAngle; // [turns]
    float32_t lastMechAngle; // [turns] This is an utility variable for turnNb computation

    // Speed estimation structure
    SpeedEst speedEst;

    // Communication variables
    int16_t turnNb;     // Number of full turns (signed)
    bool indexDetected; // Has index been detected
    bool indexToggle;   // Flag that toggles every time index is seen
    bool rollOverFlag;  // Roll-over error flag
} ENCODER;

inline void ENC_reset(ENCODER *encoder);
inline void ENC_resetQepPos(ENCODER *encoder);
inline void encoder_GetPosition(ENCODER *encoder);
inline void ENC_update(ENCODER *encoder);



#endif
