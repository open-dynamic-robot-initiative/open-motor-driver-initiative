#ifndef __FOC_H__
#define __FOC_H__

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <stdint.h>
#include <hw_types.h>
#ifdef CPU1
#include "uomodri_user_defines.h"
#else
#include "driverlib_cm.h"
#include "cm.h"

#include "uomodri_cm_user_defines.h"
#endif
#include "encoder.h"

/***********************************************************************
 * ENUM TYPES
 ***********************************************************************/
/**
 * @enum    svm_sector_e
 * @brief   Space Vector Modulation (SVM) - Sectors definitions
 */
typedef enum
{
    MOTOR_SVM_SECTOR_0  = 0x00, /*!< SECTOR_\#0 : Default sector state */
    MOTOR_SVM_SECTOR_1  = 0x06, /*!< SECTOR_\#1 : Ubeta > 0.0 && ((sqrt(3) x Ualpha) - Ubeta) > 0.0 && ((-sqrt(3) x Ualpha) - Ubeta) < 0.0 */
    MOTOR_SVM_SECTOR_2  = 0x04, /*!< SECTOR_\#2 : Ubeta > 0.0 && ((sqrt(3) x Ualpha) - Ubeta) < 0.0 && ((-sqrt(3) x Ualpha) - Ubeta) < 0.0 */
    MOTOR_SVM_SECTOR_3  = 0x05, /*!< SECTOR_\#3 : Ubeta > 0.0 && ((sqrt(3) x Ualpha) - Ubeta) < 0.0 && ((-sqrt(3) x Ualpha) - Ubeta) > 0.0 */
    MOTOR_SVM_SECTOR_4  = 0x01, /*!< SECTOR_\#4 : Ubeta < 0.0 && ((sqrt(3) x Ualpha) - Ubeta) < 0.0 && ((-sqrt(3) x Ualpha) - Ubeta) > 0.0 */
    MOTOR_SVM_SECTOR_5  = 0x03, /*!< SECTOR_\#5 : Ubeta < 0.0 && ((sqrt(3) x Ualpha) - Ubeta) > 0.0 && ((-sqrt(3) x Ualpha) - Ubeta) > 0.0 */
    MOTOR_SVM_SECTOR_6  = 0x02, /*!< SECTOR_\#6 : Ubeta < 0.0 && ((sqrt(3) x Ualpha) - Ubeta) > 0.0 && ((-sqrt(3) x Ualpha) - Ubeta) < 0.0 */
    MOTOR_SVM_SECTOR_7  = 0x07, /*!< SECTOR_\#7 : Default sector state */
} svm_sector_e;

/***********************************************************************
 * UNION TYPES
 ***********************************************************************/
/**
 * @union   ptr_32b_u
 * @brief   Memory padding (C28 to CLA pointer address translation).
 */
typedef union __ptr_32b_u__
{
    void*               ptr;
    uint32_t            addr32b;
} ptr_32b_u;

/**
 * @union   cmd_reg_u
 * @brief   Bit field associated to motor control (motor \& encoder).
 */
typedef union __cmd_reg_u__
{
    uint8_t             all;
    struct
    {
        uint8_t         systemEnable    : 1;    /*!< Bit 0      : R/W - Enable system to start. No power on motors.         */
        uint8_t         motorEnable     : 1;    /*!< Bit 1      : R/W - Enable power on motor.                              */
        uint8_t         rollOverEnable  : 1;    /*!< Bit 2      : R/W - Limit the rotations to +/- 128 rounds.              */
        uint8_t         encOffsetEnable : 1;    /*!< Bit 3      : R/W - Use encoder index in position computation.          */
        uint8_t         CMD_RSV1        : 4;    /*!< Bit 4-7    : R/W - Reserved                                            */
    };
} cmd_reg_u;

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
/**
 * @struct  pi_t
 * @brief   Proportional-Integral controller structure for FOC algorithm.
 */
typedef struct __pi_t__
{
#if (CLA1_CORE_ENABLE)
    ptr_32b_u           p_fb;           /*!< Pointer on the feedback current */
    ptr_32b_u           p_set;          /*!< Pointer on the setpoint current */
    ptr_32b_u           p_sat;          /*!< Pointer on the saturation value to use */
#else
    float32_t* const    p_fb;           /*!< Pointer on the feedback current */
    float32_t* const    p_set;          /*!< Pointer on the setpoint current */
    float32_t* const    p_sat;          /*!< Pointer on the saturation value to use */
#endif
    float32_t           err;            /*!< Current error */
    float32_t           kp;             /*!< Proportional gain */
    float32_t           ki;             /*!< Integral gain */
    float32_t           integral;       /*!< Integral term partial computation */
    float32_t           ff;             /*!< Feed-forward term */
    float32_t           out;            /*!< Controller output */
} pi_t;

/**
 * @struct  pd_t
 * @brief   Proportional-Derivative controller structure for FOC algorithm.
 */
typedef struct __pd_t__
{
#if (CLA1_CORE_ENABLE)
    ptr_32b_u      	    p_fbTheta;      /*!< Pointer on the feedback position */
    ptr_32b_u           p_setTheta;     /*!< Pointer on the setpoint position */
    ptr_32b_u           p_fbSpeed;      /*!< Pointer on the feedback speed */
    ptr_32b_u           p_setSpeed;     /*!< Pointer on the Set point speed */
    ptr_32b_u           p_kp;           /*!< Pointer on the Proportional gain */
    ptr_32b_u           p_kd;           /*!< Pointer on the Derivative gain */
    ptr_32b_u           p_ff;           /*!< Pointer on the feed-forward term */
    ptr_32b_u           p_sat;          /*!< Controller saturation */
#else
    float32_t* const    p_fbTheta;      /*!< Constant pointer on the feedback position */
    float32_t* const    p_setTheta;     /*!< Constant pointer on the setpoint position */
    float32_t* const    p_fbSpeed;      /*!< Constant pointer on the feedback speed */
    float32_t* const    p_setSpeed;     /*!< Constant pointer on the Set point speed */
    float32_t* const    p_kp;           /*!< Constant pointer on the Proportional gain */
    float32_t* const    p_kd;           /*!< Constant pointer on the Derivative gain */
    float32_t* const    p_ff;           /*!< Constant pointer on the feed-forward term */
    float32_t* const    p_sat;          /*!< Constant pointer on the controller saturation */
#endif
    float32_t           errTheta;       /*!< Error position */
    float32_t           errSpeed;       /*!< Error speed */
    float32_t           derivative;     /*!< Derivative term partial computation */
    float32_t           out;            /*!< Controller output */

} pd_t;

/**
 * @struct  params_t
 * @brief   Motor constants structure.
 */
typedef struct __params_t__
{
    float32_t           Rs;             /*!< Stator resistance [Ohm] */
    float32_t           Ls;             /*!< Stator inductance [H] */
    float32_t           kv;             /*!< Motor speed constant [RPM/V or rad/s/V] */
    float32_t           ke;             /*!< Motor back EMF constant [V/rad/s] (electrical radians) = 1/kv */
    float32_t           ki;             /*!< Motor torque constant [N.m/A] */
#if (CLA1_CORE_ENABLE)
    float32_t           polePairs;      /*!< Number of pole pairs */
#else
    const float32_t     polePairs;      /*!< Number of pole pairs */
#endif
} params_t;

/**
 * @struct  acq_t
 * @brief   Registers localization \& results
 */
typedef struct __acquisitions_t__
{
    // ADC result registers
#if (CLA1_CORE_ENABLE)
    volatile ptr_32b_u  p_vBusMeasReg;
    volatile ptr_32b_u  p_vExtMeasReg;
    volatile ptr_32b_u  p_iMeasReg[3];
#else
    volatile uint16_t*  const p_vBusMeasReg;
    volatile uint16_t*  const p_vExtMeasReg;
    volatile int16_t*   const p_iMeasReg[3];
#endif
    // ADC values scaled
    float32_t           vbus;       // [V]
    float32_t           vExt;       // [V]
    float32_t           ia;         // [A]
    float32_t           ib;         // [A]
    float32_t           ic;         // [A]
    lpf_t               vBusFlt;
    lpf_t               vExtFlt;
} acq_t;

/**
 * @struct  cmd_t
 * @brief   Commands for FOC process. Target the PD controller.
 */
typedef struct __cmd_t__
{
    float32_t           posRef;
    float32_t           velRef;
    float32_t           iqff;
    float32_t           kpCoeff;
    float32_t           kdCoeff;
    float32_t           iSat;
    uint16_t            timeoutRef;
    uint16_t            cptTimeout;
    uint16_t            index;
    cmd_reg_u           cmdArray;
} cmd_t;

/**
 * @struct  foc_t
 * @brief   General structure for FOC control.
 */
typedef struct __foc_control_t__
{
    params_t            motor_cfg;      /*!< Motor configuration parameters */
    acq_t               motor_acq;      /*!< Current \& voltage acquisitions */
    encoder_t           motor_enc;      /*!< Encoder position & speed */
    cmd_t               motor_cmd;      /*!< Motor commands */
    lpf_t               iParkFlt;       /*!< Low-pass filter parameters affected to park transform */
    float32_t           sinTheta;       /*!< Sinus of the electrical rotor angle */
    float32_t           cosTheta;       /*!< Cosine of the electrical rotor angle */
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
    float32_t           dtcMax;         /*!< Max saturation duty-cycle */
    float32_t           dtcMin;         /*!< Min saturation duty-cycle */
    float32_t           dtc_u;          /*!< Duty cycles on channel u - [pu] between -0.5 and 0.5 */
    float32_t           dtc_v;          /*!< Duty cycles on channel v - [pu] between -0.5 and 0.5 */
    float32_t           dtc_w;          /*!< Duty cycles on channel w - [pu] between -0.5 and 0.5 */
    float32_t           idRef;          /*!< PI controller set point values for D-axis current - [A] */
    float32_t           iqRef;          /*!< PI controller set point values for Q-axis current - [A] */
    float32_t           iAlignMax;      /*!< Maximum current required during the phase alignment - [A] */
    float32_t           thetaAlign;     /*!< Initial position to force a rotor movement during alignment - [RAD] */
    pi_t                piId;           /*!< PI controller structure for current control */
    pi_t                piIq;           /*!< PI controller structure for current control */
    pd_t                pdPosVel;       /*!< PD controller structure for position/velocity control */
    float32_t           resEst;         /*!< Q-axis resistance estimation - [Ohm] */
    lpf_t               resEstFlt;      /*!< Q-axis resistance estimation filter structure */
} foc_t;

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define FOC_RE_CURRENT_THRESHOLD    0.1f

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/

#endif /* __FOC_H__ */
