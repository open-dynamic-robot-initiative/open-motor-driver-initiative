#ifndef __ENCODER_H__
#define __ENCODER_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#ifdef CPU1
//#include "f2838x_device.h"
//#include "driverlib.h"
#include "config/uomodri_user_defines.h"
#else
#include <hw_types.h>
#include <stdint.h>
#include "uomodri_cm_user_defines.h"
#endif

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
typedef union __enc_test_u__
{
    Uint16              all;
    struct
    {
        Uint16          indexDetect     : 1;    /*!< Bit 0      : R/W - Has index been detected? */
        Uint16          indexOffset     : 1;    /*!< Bit 1      : R/W - Is index compensation activated? */
        Uint16          indexToggle     : 1;    /*!< Bit 2      : R/W - Toggle flag each time index is seen. */
        Uint16          indexError      : 1;    /*!< Bit 3      : R/W - Mismatch between index and mechanical angle capture. */
        Uint16          rollOverError   : 1;    /*!< Bit 4      : R/W - Roll-over error flag. */
        Uint16          alignError      : 1;    /*!< Bit 5      : R/W - Detect an error on encoder (pole pairs mismatch, phases errors...) */
        Uint16          ENC_TEST_RSV1   : 10;   /*!< Bit 6-15   : R/W - Reserved */
    };
} enc_test_u;

/**
 * @struct  speed_t
 * @brief   Speed estimation structure of encoder.
 */
typedef struct __speed_t__
{
    float32             speedHighScaler;    /*!< High speed estimation - [rad/s/pu] - diff_angular * scaler = velocity */
    float32             speedHigh;          /*!< High speed estimation -  [rad/s] - diff_angular * scaler = velocity */
    float32             speedLowScaler;     /*!< Low speed estimation - [rad.register] - scaler / dtRegister = velocity */
    float32             speedLow;           /*!< Low speed estimation - [rad/s] - scaler / dtRegister = velocity */
    float32             alpha;              /*!< Speed merge coefficient */
    lpf_t               speedFlt[2];        /*!< Filter structure for speed filtering */
    float32             speedMech[2];       /*!< Mechanical output speed - [rad/s] */
    float32             speedElec;          /*!< Electrical output speed (pole pairs integration) - [rad/s] */
    float32             theta[2];           /*!< Angular position of rotor - [pu] */
} speed_t;

/**
 * @struct  encoder_t
 * @brief   Position, speed, index detection \& error management of encoder.
 */
typedef struct __encoder_t__
{
    Uint32              eqepBase;           /*!< Address of EQEP peripheral used in HAL structure */
    float32             polePairs;          /*!< Number of motor pole pairs */
    float32             thetaElec;          /*!< Angular electrical position  - [pu] */
    float32             thetaMech[2];       /*!< Angular mechanical position  - [pu] */
    float32             thetaMechScaler;    /*!< Angular mechanical scaler - [pu/register] */
    float32             thetaIndexRelative; /*!< Index mechanical angle */
    float32             thetaIndexAbsolute; /*!< First index mechanical angle */
    float32             thetaAbsolute;      /*!< Absolute angular position - [turns] */
    speed_t             speed;              /*!< Speed estimation structure */
    int16               turnNb;             /*!< Number of full turns (signed) */
    int16               thetaDir;           /*!< Rotor direction - -1 = CCW/reverse, 1 = CW/forward */
    enc_test_u          flags;
} encoder_t;

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define MAX_ROLLOVER    (127)
#define MIN_ROLLOVER    (-128)

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/

#endif /* __ENCODER_H__ */
