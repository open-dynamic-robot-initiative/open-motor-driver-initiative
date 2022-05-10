#ifndef __FOC_H__
#define __FOC_H__

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#ifdef USE_CM_CORE
#include "driverlib_cm.h"
#include "cm.h"

#include "uomodri_cm_user_defines.h"
#else
#include "uomodri_user_defines.h"
#endif
#include "encoder.h"

/***********************************************************************
 * TYPDEF ENUM
 ***********************************************************************/

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
/**
 * @struct  pi_t
 * @brief   Proportional-Integral controller structure for FOC algorithm.
 */
typedef struct __PROPORTIONAL_INTEGRAL_t__
{
    float32_t* const    p_fb;           /*!< Pointer on the feedback current */
    float32_t* const    p_set;          /*!< Pointer on the setpoint current */
    float32_t           err;            /*!< Current error */
    float32_t           kp;             /*!< Proportional gain */
    float32_t           ki;             /*!< Integral gain */
    float32_t           integral;       /*!< Integral term partial computation */
    float32_t           ff;             /*!< Feed-forward term */
    float32_t           out;            /*!< Controller output */
    float32_t* const    p_sat;          /*!< Pointer on the saturation value to use */
} pi_t;

/**
 * @struct  pd_t
 * @brief   Proportional-Derivative controller structure for FOC algorithm.
 */
typedef struct __PROPORTIONAL_DERIVATIVE_t__
{
    float32_t* const    p_fbTheta;      /*!< Pointer on the feedback position */
    float32_t* const    p_setTheta;     /*!< Pointer on the setpoint position */
    float32_t           errTheta;       /*!< Error position */
    float32_t* const    p_fbSpeed;      /*!< Pointer on the feedback speed */
    float32_t* const    p_setSpeed;     /*!< Pointer on the Set point speed */
    float32_t           errSpeed;       /*!< Error speed */
    float32_t* const    p_kp;           /*!< Pointer on the Proportional gain */
    float32_t* const    p_kd;           /*!< Pointer on the Derivative gain */
    float32_t           derivative;     /*!< Derivative term partial computation */
    float32_t* const    p_ff;           /*!< Pointer on the feed-forward term */
    float32_t           out;            /*!< Controller output */
    float32_t* const    p_sat;          /*!< Controller saturation */
} pd_t;

typedef struct __MOTOR_PARAMETERS_t__
{
    // Motor constants
    float32_t       Rs;                 /*!< Stator resistance [Ohm] */
    float32_t       Ls;                 /*!< Stator inductance [H] */
    float32_t       kv;                 /*!< Motor speed constant [RPM/V or rad/s/V] */
    float32_t       ke;                 /*!< Motor back EMF constant [V/rad/s] (electrical radians) = 1/kv */
    float32_t       ki;                 /*!< Motor torque constant [N.m/A] */
    const float32_t polePairs;          /*!< Number of pole pairs */
} params_t;

typedef struct __MOTOR_ACQUISITIONS_t__
{
    // ADC result registers
    volatile uint16_t*  const p_vaMeasReg;
    volatile uint16_t*  const p_vbMeasReg;
    volatile uint16_t*  const p_vcMeasReg;
    volatile uint16_t*  const p_vBusMeasReg;
    volatile uint16_t*  const p_vExtMeasReg;
    volatile int16_t*   const p_iaMeasReg;
    volatile int16_t*   const p_ibMeasReg;
    volatile int16_t*   const p_icMeasReg;
    // ADC values scaled
    float32_t           va;         // [V]
    float32_t           vb;         // [V]
    float32_t           vc;         // [V]
    float32_t           vbus;       // [V]
    float32_t           vExt;       // [V]
    float32_t           ia;         // [A]
    float32_t           ib;         // [A]
    float32_t           ic;         // [A]
    lpf_t               vBusFlt;
    lpf_t               vExtFlt;
} acq_t;

typedef struct __MOTOR_COMMAND_t__
{
    const motor_id_e    motor_id;
    float32_t           posRef;
    float32_t           velRef;
    float32_t           iqRef;
    float32_t           kpCoeff;
    float32_t           kdCoeff;
    float32_t           iSat;
    uint16_t            timeoutRef;
    uint16_t            cptTimeout;
    bool_t              encOffsetCompEnable;
    bool_t              rollOverEnable;
    bool_t              motorEnable;
    bool_t              systemEnable;
} cmd_t;

typedef struct __MOTOR_FOC_CONTROL_t__
{
    params_t            motor_cfg;      /*!< Motor configuration parameters */
    acq_t               motor_acq;      /*!< Current \& voltage acquisitions */
    encoder_t           motor_enc;      /*!< Encoder position & speed */
    cmd_t               motor_cmd;      /*!< Motor commands */
    lpf_t               iParkFlt;       /*!< Low-pass filter parameters affected to park transform */
    float32_t           sinTheta;       /*!< Sinus of the electrical rotor angle */
    float32_t           cosTheta;       /*!< Cosine of the electrical rotor angle  */
    float32_t           ialpha;         /*!< Direct Clarke transform result - [A] */
    float32_t           ibeta;          /*!< Direct Clarke transform result - [A] */
    float32_t           id;             /*!< Filtered Direct Park transform result - [A] */
    float32_t           iq;             /*!< Filtered Quadrature Park transform result - [A] */
    float32_t           ud;             /*!< Inverse Park transform result - [V] */
    float32_t           uq;             /*!< Inverse Park transform result - [V] */
    float32_t           ualpha;         /*!< Inverse Clarke transform result - [V] */
    float32_t           ubeta;          /*!< Inverse Clarke transform result - [V] */
    float32_t           ua;             /*!< Phase commands - Channel a - [V] */
    float32_t           ub;             /*!< Phase commands - Channel b - [V] */
    float32_t           uc;             /*!< Phase commands - Channel c - [V] */
    float32_t           vmax;           /*!< Bus voltage saturation - [V] */
    float32_t           imax;           /*!< Max current - [A] */
    float32_t           dtcMax;         /*!< Max saturation duty-cycle */
    float32_t           dtcMin;         /*!< Min saturation duty-cycle */
    float32_t           dtc_u;          /*!< Duty cycles on channel u - [pu] between -0.5 and 0.5 */
    float32_t           dtc_v;          /*!< Duty cycles on channel v - [pu] between -0.5 and 0.5 */
    float32_t           dtc_w;          /*!< Duty cycles on channel w - [pu] between -0.5 and 0.5 */
    float32_t           idRef;          /*!< PI controller set point values for D-axis current - [A] */
    float32_t           iqRef;          /*!< PI controller set point values for Q-axis current - [A] */
    float32_t           iAlignMax;      /*!< Maximum current required during alignment phase -[A] */
    pi_t                piId;           /*!< PI controller structure for current control */
    pi_t                piIq;           /*!< PI controller structure for current control */
    pd_t                pdPos;          /*!< PD controller structure for position/velocity control */
    float32_t           resEst;         /*!< Q-axis resistance estimation - [Ohm] */
    lpf_t               resEstFlt;      /*!< Q-axis resistance estimation filter structure */
//#ifdef USE_CM_CORE
//    uint8_t             rsvd[11];
//#else
//    Anticoagging        acog;           /*!< Anti-coagging structure */
//#endif
} foc_t;

/***********************************************************************
 * DEFINES
 ***********************************************************************/
// Resistance estimation uses Ohm's Law R = U/I so we have to ensure I != 0.
// This threshold corresponds to the minimum current for the measure to be considered as correct.
#define FOC_RE_CURRENT_THRESHOLD    0.1f

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
inline void FOC_getMeasures(acq_t*);
inline void FOC_runControl(foc_t*);
inline float32_t FOC_runPD(pd_t*);
void FOC_resetStruct(foc_t*);
void limit_norm(float32_t*, float32_t*, float32_t);

#endif
