#ifndef __ENCODERS_H__
#define __ENCODERS_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#ifdef USE_CM_CORE
#include "driverlib_cm.h"
#include "cm.h"

#include "uomodri_cm_user_defines.h"
#else
#include "f2838x_device.h"
#include "driverlib.h"
#include "device.h"

#include "hal.h"
#include "uomodri_user_defines.h"
#endif

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
typedef struct __SPEED_ESTIMATOR_STRUCT__
{
    float32_t   speedHighScaler;    /*!< High speed estimation - [rad/s/pu] - diff_angular * scaler = velocity */
    float32_t   speedHigh;          /*!< High speed estimation -  [rad/s] - diff_angular * scaler = velocity */
    float32_t   speedLowScaler;     /*!< Low speed estimation - [rad.register] - scaler / dtRegister = velocity */
    float32_t   speedLow;           /*!< Low speed estimation - [rad/s] - scaler / dtRegister = velocity */
    float32_t   alpha;              /*!< Speed merge coefficient */
//    float32_t   speedRef;           /*!< Speed merge - [rad/s] - Low speed and high speed merge estimation */
    lpf_t       speedFlt;           /*!< Filter structure for speed filtering */
    float32_t   speedMech;          /*!< Mechanical output speed - [rad/s] */
    float32_t   speedElec;          /*!< Electrical output speed (pole pairs integration) - [rad/s] */
    float32_t   theta[2];           /*!< Angular position of rotor - [pu] */
} speed_t;

typedef struct __ENCODER_STRUCT__
{
#ifdef USE_CM_CORE
    uint32_t                    rsvd1;
    uint32_t                    rsvd2;
#else
    // pointers on HAL structure, QEP registers and direct access to QEP counter register
    const eqep_cfg_t*           p_qepHandle;        /*!< Pointer on HAL structure */
    volatile struct EQEP_REGS*  p_qepRegs;          /*!< Pointer on the QEP registers */
#endif
    const uint8_t               polePairs;          /*!< Number of motor pole pairs */
    float32_t                   thetaElec;          /*!< Angular electrical position  - [pu] */
    float32_t                   thetaMech[2];       /*!< Angular mechanical position  - [pu] */
    float32_t                   thetaMechScaler;    /*!< Angular mechanical scaler - [pu/register] */
    float32_t                   thetaIndex;         /*!< Index mechanical angle */
    float32_t                   thetaMechIndexed;   /*!< Mechanical angle relative to the index - [pu] */
    float32_t                   thetaAbsolute;      /*!< Absolute angular position - [turns] */
    speed_t                     speed;              /*!< Speed estimation structure */
    int16_t                     turnNb;             /*!< Number of full turns (signed) */
    int16_t                     thetaDir;           /*!< Rotor direction - 0 (false) = CCW/reverse, 1(true) = CW/forward */
    bool_t                      indexDetected;      /*!< Has index been detected */
    bool_t                      indexCompensated;   /*!< Is index compensation activated */
    bool_t                      indexToggle;        /*!< Flag that toggles every time index is seen */
    bool_t                      rollOverFlag;       /*!< Roll-over error flag */
//    bool                        rollOverFlagEn;     /*!< Enable roll-over error flag */
} encoder_t;

/***********************************************************************
 * DEFINES
 ***********************************************************************/
// This threshold corresponds to the middle speed for low/high speed estimation merge
// See the ENC_update function for more details
#define SPEED_ESTIMATION_THRESHOLD      80.4f // [rad/s]
#define MAX_ROLLOVER                    (127)
#define MIN_ROLLOVER                    (-128)

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void ENC_resetStruct(encoder_t*);
inline void ENC_setPosition(encoder_t*, uint32_t);
#if (defined UOMODRI_USE_ENCODER_GET_REG_FUNCTION) && (UOMODRI_USE_ENCODER_GET_REG_FUNCTION)
inline void ENC_getRegisters(encoder_t*);
#endif
inline void ENC_getPosition(encoder_t*);
inline void ENC_getSpeed(encoder_t*);
inline void ENC_getTheta(encoder_t*);

#endif /* __ENCODERS_H__ */
