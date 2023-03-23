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
#if (defined CPU1) && (!defined CLA1_CORE_ENABLE)
    // pointers on HAL structure, QEP registers and direct access to QEP counter register
    const uint32_t      eqepBaseAddr;       /*!< Address of EQEP peripheral used inHAL structure */
//    const eqep_cfg_t*   p_qepHandle;        /*!< Pointer on HAL structure */
#elif (defined CPU1) && (defined CLA1_CORE_ENABLE)
    // pointers on HAL structure, QEP registers and direct access to QEP counter register
    uint32_t            eqepBaseAddr;       /*!< Address of EQEP peripheral used inHAL structure */
//    const eqep_cfg_t*   p_qepHandle;        /*!< Pointer on HAL structure */
    #else
    uint32_t            rsvd1;
#endif
#if (CLA1_CORE_ENABLE)
    float32_t           polePairs;          /*!< Number of motor pole pairs */
#else
    const float32_t     polePairs;          /*!< Number of motor pole pairs */
#endif
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
 * DEFINES
 ***********************************************************************/
#define MAX_ROLLOVER                    (127)
#define MIN_ROLLOVER                    (-128)
//#define ENC_GET_POSITION(p_enc)                                                                             \
//        p_enc->thetaMech[OLD]   = p_enc->thetaMech[NEW];                                                    \
//        p_enc->thetaMech[NEW]   = (float32_t)HWREG(eqepBase + EQEP_O_QPOSSLAT) * p_enc->thetaMechScaler;    \
//        p_enc->thetaDir         = (HWREGH(eqepBase + EQEP_O_QEPSTS) & EQEP_QEPSTS_QDF)? (1) : (-1);         \
//        float32_t thetaElec     = (float32_t)p_enc->polePairs * p_enc->thetaMech[NEW];                      \
//        p_enc->thetaElec        = __mpy2pif32(__fracf32(thetaElec));                                        \
//        p_enc->thetaMech[NEW]   = __mpy2pif32(p_enc->thetaMech[NEW]);
//#define ENC_GET_SPEED(p_enc)                                                                                                        \
//        speed_t* p_speed                    = &p_enc->speed;                                                                        \
//        if(HWREGH(eqepBase + EQEP_O_QFLG) & EQEP_INT_UNIT_TIME_OUT)                                                                 \
//        {                                                                                                                           \
//            p_speed->theta[OLD]             = p_speed->theta[NEW];                                                                  \
//            p_speed->theta[NEW]             = __mpy2pif32((float32_t)HWREG(eqepBase + EQEP_O_QPOSLAT) * p_enc->thetaMechScaler);    \
//            HWREGH(eqepBase + EQEP_O_QCLR)  = EQEP_INT_UNIT_TIME_OUT;                                                               \
//            float32_t deltaTheta            = p_speed->theta[NEW] - p_speed->theta[OLD];                                            \
//            deltaTheta                      = (deltaTheta > M_PI)   ? (deltaTheta - (2.0f * M_PI)) : (deltaTheta);                  \
//            deltaTheta                      = (deltaTheta < -M_PI)  ? (deltaTheta + (2.0f * M_PI)) : (deltaTheta);                  \
//            p_speed->speedHigh              = deltaTheta * p_speed->speedHighScaler;                                                \
//        }                                                                                                                           \
//        float32_t speedFusion   = p_speed->speedHigh;                                                                               \
//        UOMODRI_SPEED_FLT(p_speed->speedFlt[0], p_speed->speedMech[0], speedFusion);                                                \
//        UOMODRI_SPEED_FLT(p_speed->speedFlt[1], p_speed->speedMech[1], speedFusion);                                                \
//        p_speed->speedElec      = (float32_t)p_enc->polePairs * p_speed->speedMech[0];
//#define ENC_GET_THETA(p_enc)                                                                                                                \
//        float32_t deltaThetaMech= p_enc->thetaMech[NEW] - p_enc->thetaMech[OLD];                                                            \
//        p_enc->turnNb           = (deltaThetaMech > M_PI)   ? (p_enc->turnNb - 1) : (p_enc->turnNb);                                        \
//        p_enc->turnNb           = (deltaThetaMech < -M_PI)  ? (p_enc->turnNb + 1) : (p_enc->turnNb);                                        \
//        bool_t maxRollOver      = (p_enc->turnNb > MAX_ROLLOVER)? (true) : (false);                                                         \
//        bool_t minRollOver      = (p_enc->turnNb < MIN_ROLLOVER)? (true) : (false);                                                         \
//        p_enc->rollOverError    = (maxRollOver || minRollOver)  ? (true) : (false);                                                         \
//        p_enc->turnNb           = (maxRollOver) ? (MIN_ROLLOVER) : (p_enc->turnNb);                                                         \
//        p_enc->turnNb           = (minRollOver) ? (MAX_ROLLOVER) : (p_enc->turnNb);                                                         \
//        if(EQEP_getInterruptStatus(eqepBase) & EQEP_INT_INDEX_EVNT_LATCH)                                                                   \
//        {                                                                                                                                   \
//            float32_t thetaIndex= __mpy2pif32((float32_t)EQEP_getIndexPositionLatch(eqepBase) * p_enc->thetaMechScaler);                    \
//            p_enc->thetaIndex   = (!p_enc->indexDetect) ? (thetaIndex) : (p_enc->thetaIndex);                                               \
//            p_enc->indexDetect  = true;                                                                                                     \
//            p_enc->indexError   = (fabsf(p_enc->thetaIndex - thetaIndex) < __mpy2pif32(4.0f / p_enc->thetaMechScaler)) ? (false) : (true);  \
//            p_enc->indexToggle  = !p_enc->indexToggle;                                                                                      \
//            EQEP_clearInterruptStatus(eqepBase, EQEP_INT_INDEX_EVNT_LATCH);                                                                 \
//        }                                                                                                                                   \
//        p_enc->thetaAbsolute    = p_enc->thetaMech[NEW] + __mpy2pif32((float32_t)p_enc->turnNb);

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
//inline void ENC_resetStruct(encoder_t*);
//inline void ENC_resetPeriph(encoder_t*);
//inline void ENC_getPosition(encoder_t*);
//inline void ENC_getSpeed(encoder_t*);
//inline void ENC_getTheta(encoder_t*);

#endif /* __ENCODER_H__ */
