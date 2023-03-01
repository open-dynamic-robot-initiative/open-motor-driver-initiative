#ifndef __ENCODER_H__
#define __ENCODER_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#ifdef CPU1
#include "hal.h"
#include "uomodri_user_defines.h"
#else
#include "uomodri_cm_user_defines.h"
#endif

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
typedef struct __speed_t__
{
    float32_t       speedHighScaler;        /*!< High speed estimation - [rad/s/pu] - diff_angular * scaler = velocity */
    float32_t       speedHigh;              /*!< High speed estimation -  [rad/s] - diff_angular * scaler = velocity */
    float32_t       speedLowScaler;         /*!< Low speed estimation - [rad.register] - scaler / dtRegister = velocity */
    float32_t       speedLow;               /*!< Low speed estimation - [rad/s] - scaler / dtRegister = velocity */
    float32_t       alpha;                  /*!< Speed merge coefficient */
    lpf_t           speedFlt[2];            /*!< Filter structure for speed filtering */
    float32_t       speedMech[2];           /*!< Mechanical output speed - [rad/s] */
    float32_t       speedElec;              /*!< Electrical output speed (pole pairs integration) - [rad/s] */
    float32_t       theta[2];               /*!< Angular position of rotor - [pu] */
} speed_t;

typedef struct __encoder_t__
{
#ifdef CPU1
    // pointers on HAL structure, QEP registers and direct access to QEP counter register
    const eqep_cfg_t*   p_qepHandle;        /*!< Pointer on HAL structure */
#else
    uint32_t            rsvd1;
#endif
    const float32_t     polePairs;          /*!< Number of motor pole pairs */
    float32_t           thetaElec;          /*!< Angular electrical position  - [pu] */
    float32_t           thetaMech[2];       /*!< Angular mechanical position  - [pu] */
    float32_t           thetaMechScaler;    /*!< Angular mechanical scaler - [pu/register] */
    float32_t           thetaIndex;         /*!< Index mechanical angle */
    float32_t           thetaAbsolute;      /*!< Absolute angular position - [turns] */
    speed_t             speed;              /*!< Speed estimation structure */
    int16_t             turnNb;             /*!< Number of full turns (signed) */
    int16_t             thetaDir;           /*!< Rotor direction - -1 = CCW/reverse, 1 = CW/forward */
    bool_t              indexDetect;        /*!< Has index been detected */
    bool_t              indexOffset;        /*!< Is index compensation activated */
    bool_t              indexToggle;        /*!< Flag that toggles every time index is seen */
    bool_t              indexError;         /*!< Mismatch error between consecutive index mechanical angle capture */
    bool_t              rollOverError;      /*!< Roll-over error flag */
} encoder_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
inline void ENC_resetStruct(encoder_t*);
inline void ENC_resetPeriph(encoder_t*);
inline void ENC_getPosition(encoder_t*);
inline void ENC_getSpeed(encoder_t*);
inline void ENC_getTheta(encoder_t*);

#endif /* __ENCODER_H__ */
