#ifndef __ENCODER_H__
#define __ENCODER_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdint.h>
#include <hw_types.h>
#ifdef CPU1
#include <f2838x_device.h>
#include "uomodri_user_defines.h"
#else
#include "uomodri_cm_user_defines.h"
#endif

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
/**
 * @struct  speed_t
 * @brief   Speed estimation structure of encoder.
 */
typedef struct __speed_t__
{
    float32_t           speedHighScaler;    /*!< High speed estimation - [rad/s/pu] - diff_angular * scaler = velocity */
    float32_t           speedHigh;          /*!< High speed estimation -  [rad/s] - diff_angular * scaler = velocity */
    float32_t           speedLowScaler;     /*!< Low speed estimation - [rad.register] - scaler / dtRegister = velocity */
    float32_t           speedLow;           /*!< Low speed estimation - [rad/s] - scaler / dtRegister = velocity */
    float32_t           alpha;              /*!< Speed merge coefficient */
    lpf_t               speedFlt[2];        /*!< Filter structure for speed filtering */
    float32_t           speedMech[2];       /*!< Mechanical output speed - [rad/s] */
    float32_t           speedElec;          /*!< Electrical output speed (pole pairs integration) - [rad/s] */
    float32_t           theta[2];           /*!< Angular position of rotor - [pu] */
} speed_t;

/**
 * @struct  encoder_t
 * @brief   Position, speed, index detection \& error management of encoder.
 */
typedef struct __encoder_t__
{
#if (defined CPU1) && (!CLA1_CORE_ENABLE)
    const uint32_t      eqepBaseAddr;       /*!< Address of EQEP peripheral used in HAL structure */
    const float32_t     polePairs;          /*!< Number of motor pole pairs */
#else
    uint32_t            eqepBaseAddr;       /*!< Address of EQEP peripheral used in HAL structure */
    float32_t           polePairs;          /*!< Number of motor pole pairs */
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

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/

#endif /* __ENCODER_H__ */
