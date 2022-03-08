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
#include "f2838x_device.h"
#include "f2838x_struct.h"
#include "f2838x_examples.h"
#include "driverlib.h"
#include "device.h"

#include "uomodri_user_defines.h"
#include "anticoagging.h"
#endif
#include "encoder.h"

/***********************************************************************
 * TYPDEF ENUM
 ***********************************************************************/
/**
 * @enum    align_state_e
 * @brief   Alignment procedure states.
 */
typedef enum
{
    ALIGN_COUNT_UP      = 0,    /*!< ALIGN_COUNT_UP */
    ALIGN_STAY          = 1,    /*!< ALIGN_STAY */
    ALIGN_COUNT_DOWN    = 2,    /*!< ALIGN_COUNT_DOWN */
    ALIGN_DONE          = 3,    /*!< ALIGN_DONE */
} align_state_e;

typedef enum
{
    MOTOR_STOP          = 0x00,
    MOTOR_ALIGN_UP      = 0x01,    /*!< ALIGN_COUNT_UP */
    MOTOR_ALIGN_STAY    = 0x02,    /*!< ALIGN_STAY */
    MOTOR_ALIGN_DOWN    = 0x04,    /*!< ALIGN_COUNT_DOWN */
//    MOTOR_ALIGN_DONE    = 0x08,    /*!< ALIGN_DONE */
    MOTOR_READY         = 0x10,
    MOTOR_ERROR         = 0x20,
} motor_state_e;

/**
 * @enum    status_error_e
 * @brief   Error flags list.
 */
typedef enum
{
    COM_ERROR_NO_ERROR          = 0,    /*!< No error */
    COM_ERROR_ENCODER1          = 1,    /*!< Encoder_1 error too high */
    COM_ERROR_SPI_RECV_TIMEOUT  = 2,    /*!< Timeout for receiving current references exceeded */
    COM_ERROR_CRIT_TEMP         = 3,    /*!< Motor temperature reached critical valued */
    COM_ERROR_POSCONV           = 4,    /*!< Unused error */
    COM_ERROR_POS_ROLLOVER      = 5,    /*!< Position roll-over occurred */
    COM_ERROR_ENCODER2          = 6,    /*!< Encoder_2 error too high */
    COM_ERROR_DRV_NFAULT        = 7,    /*!< Motor DRV nFault error */
} com_error_e;

//typedef enum
//{
//    FOC_ERROR_NO_ERROR          = 0x0000,    /*!< No error */
//    FOC_ERROR_ENCODER           = 0x0001,    /*!< Encoder_1 error too high */
//    FOC_ERROR_POS_ROLLOVER      = 0x0002,    /*!< Position roll-over occurred */
//    FOC_ERROR_DRV_NFAULT        = 0x0004,    /*!< Motor DRV nFault error */
////    FOC_ERROR_FAILURE_MOTOR1    = 0x0007,
////    FOC_ERROR_ENCODER2          = 0x0010,    /*!< Encoder_2 error too high */
////    FOC_ERROR_POS_ROLLOVER2     = 0x0020,    /*!< Position roll-over occurred */
////    FOC_ERROR_DRV_NFAULT2       = 0x0040,    /*!< Motor DRV nFault error */
////    FOC_ERROR_SPI_RECV_TIMEOUT  = 0x0100,    /*!< Timeout for receiving current references exceeded */
////    FOC_ERROR_CRIT_TEMP         = 0x0200,    /*!< Motor temperature reached critical valued */
//} foc_error_e;

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
//typedef struct __PID_CONTROLLER__
//{
//    float32_t* in;          // pointer on the loopback current
//    float32_t* set;         // pointer on the setpoint current
//    float32_t err;          // Current error
//    float32_t vErr;         // Previous error
//    float32_t deltaT;       // Time step
//
//    float32_t kp;           // Proportional gain
//    float32_t ki;           // Integral gain
//    float32_t kd;           // Derivative gain
//
//    float32_t integral;     // Integral term partial computation
//    float32_t derivative;   // Derivative term partial computation
//    float32_t ff;           // Feed-forward term
//    float32_t out;          // Controller output
//    float32_t* sat;         // Controller saturation
//} PID_CONTROLLER;
/**
 * @struct  pi_t
 * @brief   Proportional-Integral controller structure for FOC algorithm.
 */
typedef struct __PROPORTIONAL_INTEGRAL_t__
{
    float32_t*      p_fb;       /*!< Pointer on the feedback current */
    float32_t*      p_set;      /*!< Pointer on the setpoint current */
    float32_t       err;        /*!< Current error */
    float32_t       kp;         /*!< Proportional gain */
    float32_t       ki;         /*!< Integral gain */
    float32_t       integral;   /*!< Integral term partial computation */
    float32_t       ff;         /*!< Feed-forward term */
    float32_t       out;        /*!< Controller output */
    float32_t*      p_sat;      /*!< Pointer on the saturation value to use */
} pi_t;

/**
 * @struct  pd_t
 * @brief   Proportional-Derivative controller structure for FOC algorithm.
 */
typedef struct __PROPORTIONAL_DERIVATIVE_t__
{
    float32_t*      p_fbTheta;  /*!< Pointer on the feedback position */
    float32_t*      p_setTheta; /*!< Pointer on the setpoint position */
    float32_t       errTheta;   /*!< Error position */
    float32_t*      p_fbSpeed;  /*!< Pointer on the feedback speed */
    float32_t*      p_setSpeed; /*!< Pointer on the Set point speed */
    float32_t       errSpeed;   /*!< Error speed */
    float32_t*      p_kp;       /*!< Pointer on the Proportional gain */
    float32_t*      p_kd;       /*!< Pointer on the Derivative gain */
    float32_t       derivative; /*!< Derivative term partial computation */
    float32_t*      p_ff;       /*!< Pointer on the feed-forward term */
    float32_t       out;        /*!< Controller output */
    float32_t*      p_sat;      /*!< Controller saturation */
} pd_t;

typedef struct __MOTOR_PARAMETERS_t__
{
    // Motor constants
    float32_t       Rs;         /*!< Stator resistance [Ohm] */
    float32_t       Ld;         /*!< Direct axis inductance [H] */
    float32_t       Lq;         /*!< Quadratic axis inductance [H] */
    float32_t       kv;         /*!< Motor speed constant [RPM/V or rad/s/V] */
    float32_t       ke;         /*!< Motor back EMF constant [V/rad/s] (electrical radians) = 1/kv */
    float32_t       ki;         /*!< Motor torque constant [N.m/A] */
    const uint8_t   polePairs;  /*!< Number of pole pairs */
} params_t;

typedef struct __MOTOR_ACQUISITIONS_t__
{
#ifdef USE_CM_CORE
    uint32_t                            rsvd1[5];
    uint32_t                            rsvd2[5];
#else
    volatile struct ADC_REGS*           const p_adcRegs[MAX_ADC];
    volatile struct ADC_RESULT_REGS*    const p_adcResultRegs[MAX_ADC];
#endif
    // ADC result registers
    volatile uint16_t*                  const p_vaMeasReg;
    volatile uint16_t*                  const p_vbMeasReg;
    volatile uint16_t*                  const p_vcMeasReg;
    volatile uint16_t*                  const p_vBusMeasReg;
    volatile uint16_t*                  const p_vExtMeasReg;
    volatile int16_t*                   const p_iaMeasReg;
    volatile int16_t*                   const p_ibMeasReg;
    volatile int16_t*                   const p_icMeasReg;
    // ADC values scaled
    float32_t                           va;         // [V]
    float32_t                           vb;         // [V]
    float32_t                           vc;         // [V]
    float32_t                           vbus;       // [V]
    float32_t                           vExt;       // [V]
    float32_t                           ia;         // [A]
    float32_t                           ib;         // [A]
    float32_t                           ic;         // [A]
    lpf_t                               vBusFlt;
    lpf_t                               vExtFlt;
} acq_t;

typedef struct __MOTOR_ALIGNEMENT_t__
{
    align_state_e   state;

    uint32_t        cnt;
    uint32_t        cntMax;

    float32_t       idAlignRef;
    float32_t       idAlign;
} align_t;

typedef struct __MOTOR_FOC_CONTROL_t__
{
    const motor_id_e    motor_id;       /*!< Motor identification */
    params_t            motor_cfg;      /*!< Motor configuration parameters */
    acq_t               motor_acq;      /*!< Current \& voltage acquisitions */
    encoder_t           motor_enc;      /*!< Encoder position & speed */
//    align_t             motor_align;    /*!< Alignment calibration procedure */
    motor_state_e       motor_state[2];
    lpf_t               iParkFlt;       /*!< Low-pass filter parameters affected to park transform */
    float32_t           sinTheta;       /*!< Sinus of the electrical rotor angle */
    float32_t           cosTheta;       /*!< Cosine of the electrical rotor angle  */
    float32_t           ialpha;         /*!< Direct Clarke transform result - [A] */
    float32_t           ibeta;          /*!< Direct Clarke transform result - [A] */
    float32_t           id;             /*!< Direct Park transform result - [A] */
    float32_t           iq;             /*!< Direct Park transform result - [A] */
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
    pi_t                piId;           /*!< PI controller structure for current control */
    pi_t                piIq;           /*!< PI controller structure for current control */
    pd_t                pdPos;          /*!< PD controller structure for position/velocity control */
    float32_t           resEst;         /*!< Q-axis resistance estimation - [Ohm] */
    lpf_t               resEstFlt;      /*!< Q-axis resistance estimation filter structure */
#ifdef USE_CM_CORE
    uint8_t             rsvd[11];
#else
    Anticoagging        acog;           /*!< Anti-coagging structure */
#endif
    uint16_t            errorStatus;    /*!< Error messages */
//    uint16_t            errorStatus;
} foc_t;

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define FOC_ERROR_NO_ERROR          (0x0000)    /*!< No error */
#define FOC_ERROR_ENCODER           (0x0001)    /*!< Encoder_1 error too high */
#define FOC_ERROR_POS_ROLLOVER      (0x0002)    /*!< Position roll-over occurred */
#define FOC_ERROR_DRV_NFAULT        (0x0004)    /*!< Motor DRV nFault error */
// Resistance estimation uses Ohm's Law R = U/I so we have to ensure I != 0.
// This threshold corresponds to the minimum current for the measure to be considered as correct.
#define FOC_RE_CURRENT_THRESHOLD    0.1f

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
inline void FOC_getMeasures(acq_t*);
inline void FOC_resetMeasuresStruct(acq_t*);
inline void FOC_resistanceEstimation(foc_t*);
inline void FOC_runControl(foc_t*);
inline void FOC_resetPI(pi_t*);
inline float32_t FOC_runPI(pi_t*);
inline void FOC_resetPD(pd_t*);
inline float32_t FOC_runPD(pd_t*);
void FOC_resetStruct(foc_t*);
void limit_norm(float32_t*, float32_t*, float32_t);

#endif
