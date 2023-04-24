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
typedef union __enc_test_u__
{
    uint8_t             all;
    struct
    {
        uint8_t         indexDetect     : 1;    /*!< Bit 0      : R/W - Has index been detected? */
        uint8_t         indexOffset     : 1;    /*!< Bit 1      : R/W - Is index compensation activated? */
        uint8_t         indexToggle     : 1;    /*!< Bit 2      : R/W - Toggle flag every time index is seen. */
        uint8_t         indexError      : 1;    /*!< Bit 3      : R/W - Mismatch between index and mechanical angle capture. */
        uint8_t         rollOverError   : 1;    /*!< Bit 4      : R/W - Roll-over error flag. */
        uint8_t         phaseError      : 1;    /*!< Bit 5      : R/W - Phase A, phase B or both are missing. */
        uint8_t         ENC_TEST_RSV1   : 2;    /*!< Bit 6-7    : R/W - Reserved */
    };
} enc_test_u;

/**
 * @struct  speed_t
 * @brief   Speed estimation structure of encoder.
 */
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

/**
 * @struct  encoder_t
 * @brief   Position, speed, index detection \& error management of encoder.
 */
typedef struct __encoder_t__
{
#ifdef CPU1
    // pointers on HAL structure, QEP registers and direct access to QEP counter register
    const uint32_t      eqepBaseAddr;       /*!< Address of EQEP peripheral used inHAL structure */
#else
    uint32_t            rsvd1;
#endif
    const float32_t     polePairs;          /*!< Number of motor pole pairs */
    float32_t           thetaElec;          /*!< Angular electrical position  - [pu] */
    float32_t           thetaMech[2];       /*!< Angular mechanical position  - [pu] */
    float32_t           thetaMechScaler;    /*!< Angular mechanical scaler - [pu/register] */
    float32_t           thetaIndexRelative; /*!< Index mechanical angle */
    float32_t           thetaIndexAbsolute; /*!< First index mechanical angle */
    float32_t           thetaAbsolute;      /*!< Absolute angular position - [turns] */
    speed_t             speed;              /*!< Speed estimation structure */
    int16_t             turnNb;             /*!< Number of full turns (signed) */
    int16_t             thetaDir;           /*!< Rotor direction - -1 = CCW/reverse, 1 = CW/forward */
    enc_test_u          flags;
} encoder_t;

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define MAX_ROLLOVER                    (127)
#define MIN_ROLLOVER                    (-128)

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
inline void ENC_resetStruct(encoder_t*);
inline void ENC_resetPeriph(encoder_t*);
inline void ENC_getPosition(encoder_t*);
inline void ENC_getSpeed(encoder_t*);
inline void ENC_getTheta(encoder_t*);

#endif /* __ENCODER_H__ */
