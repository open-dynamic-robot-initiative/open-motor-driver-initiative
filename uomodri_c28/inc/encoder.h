#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef C28_CORE
#include "f2838x_device.h"
#include "driverlib.h"
#include "device.h"

#include "hal.h"
#include "uomodri_user_defines.h"
#elif defined CM_CORE
#include "driverlib_cm.h"
#include "cm.h"

#include "uomodri_cm_user_defines.h"
#endif

// This threshold corresponds to the middle speed for low/high speed estimation merge
// See the ENC_update function for more details
#define SPEED_ESTIMATION_THRESHOLD      80.4f // [rad/s]

typedef struct __SPEEDEST_STRUCT__
{
    // High speed estimation
    float32_t           speedHighScaler;// [rad/s/pu], diff_angular * scaler = velocity
    float32_t           speedHigh;    // [rad/s]
    // Low speed estimation
    float32_t           speedLowScaler; // [rad.register], scaler / dtRegister = velocity
    float32_t           speedLow;     // [rad/s]
    // Speed merge
    float32_t           alpha;     // Speed merge coefficient
    float32_t           speedRef;  // [rad/s] Merged of low speed and high speed estimation
    // Low-pass filter
    LPFSTRUCT           lpf;
    // Output speeds
    float32_t           speedMech; // [rad/s]
    float32_t           speedElec; // [rad/s]
    // Utility variables
    float32_t           theta[2]; // [pu]
    bool                thetaDir; // 0 = CCW/reverse, 1 = CW/forward
} SPEED_STRUCT;

typedef struct __ENCODER_STRUCT__
{
#ifdef C28_CORE
    // pointers on HAL structure, QEP registers and direct access to QEP counter register
    const eQEP_TypeDef*         p_qepHandle;
    volatile struct EQEP_REGS*  p_qepRegs;
#elif defined CM_CORE
    uint32_t                    rsvd1;
    uint32_t                    rsvd2;
#endif
    const uint8_t               polePairs;    // Number of pole pairs of the motor
    // Scalers
    float32_t                   thetaMechScaler; // [pu/register]
    // Angles
    float32_t                   thetaElec; // [pu]
    float32_t                   thetaMech[2]; // [pu]
    bool                        thetaDir; //0 = CCW/reverse, 1 = CW/forward
//    bool                thetaNew; // New sample available?
    // Indexed angles
    bool                        indexCompensated;   // Is index compensation activated
    float32_t                   indexTheta;       // Index mech angle
    float32_t                   indexedThetaMech; // Mech angle relative to the index [pu]
    float32_t                   thetaAbsolute; // [turns]
    // Speed estimation structure
    SPEED_STRUCT                speedEst;
    // Communication variables
    int16_t                     turnNb;     // Number of full turns (signed)
    bool                        indexDetected; // Has index been detected
    bool                        indexToggle;   // Flag that toggles every time index is seen
    bool                        rollOverErrorFlag;  // Roll-over error flag
    bool                        rollOverErrorFlagEnable; // Enable roll-over error flag
} ENC_STRUCT;

void ENC_resetStruct(ENC_STRUCT*);
inline void ENC_setPosition(ENC_STRUCT*, uint32_t);
inline void ENC_getPosition(ENC_STRUCT*);
inline void ENC_getSpeed(ENC_STRUCT*);

#endif
