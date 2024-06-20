#ifndef __FOC_H__
#define __FOC_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#ifdef CPU1
#include "f2838x_device.h"
#include "uomodri_user_defines.h"
#else
#include <stdint.h>
#include <hw_types.h>
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
    Uint32              addr32b;
} ptr_32b_u;

typedef union __status_reg_u__
{
    Uint16              all;
    struct
    {
        Uint16          drv_initErr     : 1;    /*!< Bits 0   : R/W - DRV initialization went wrong */
        Uint16          status_RSV1     : 15;   /*!< Bits 1-15: R/W - Reserved */
    };
} status_reg_u;

/**
 * @union   cmd_reg_u
 * @brief   Bit field associated to motor control (motor \& encoder).
 */
typedef union __cmd_reg_u__
{
    Uint16              all;
    struct
    {
        Uint16          systemEnable    : 1;    /*!< Bit 0      : R/W - Enable system to start. No power on motors.         */
        Uint16          motorEnable     : 1;    /*!< Bit 1      : R/W - Enable power on motor.                              */
        Uint16          rollOverEnable  : 1;    /*!< Bit 2      : R/W - Limit the rotations to +/- 128 rounds.              */
        Uint16          encOffsetEnable : 1;    /*!< Bit 3      : R/W - Use encoder index in position computation.          */
//        Uint16        calibEnable     : 1;    /*!< Bit 4      : R/W - Ready for ADC calibration. (Offset compensation).   */
        Uint16          CMD_RSV1        : 12;   /*!< Bit 5-15   : R/W - Reserved                                            */
    };
} cmd_reg_u;

typedef union __ppb_num_u__
{
    Uint16              all;
    struct
    {
        Uint16          ia              : 2;    /*!< Bit 0-1    : R/W - Post Processing Block for ia current measure.   */
        Uint16          ib              : 2;    /*!< Bit 2-3    : R/W - Post Processing Block for ib current measure.   */
        Uint16          ic              : 2;    /*!< Bit 4-5    : R/W - Post Processing Block for ic current measure.   */
        Uint16          vBus            : 2;    /*!< Bit 6-7    : R/W - Post Processing Block for Vbus tension measure. */
        Uint16          vExt            : 2;    /*!< Bit 8-9    : R/W - Post Processing Block for Vext tension measure. */
        Uint16          PPB_RSV1        : 6;    /*!< Bit 10-15  : R/W - Reserved                                        */
    };
} ppb_num_u;

typedef union __soc_num_u__
{
    Uint32              all;
    struct
    {
        Uint32          ia              : 4;    /*!< Bit 0-3    : R/W - Start Of Conversion (SOC) Block for ia current measure.     */
        Uint32          ib              : 4;    /*!< Bit 4-7    : R/W - Start Of Conversion (SOC) Block for ib current measure.     */
        Uint32          ic              : 4;    /*!< Bit 8-11   : R/W - Start Of Conversion (SOC) Block for ic current measure.     */
        Uint32          vBus            : 4;    /*!< Bit 12-15  : R/W - Start Of Conversion (SOC) Block for Vbus tension measure.   */
        Uint32          vExt            : 4;    /*!< Bit 16-19  : R/W - Start Of Conversion (SOC) Block for Vext tension measure.   */
        Uint32          SOC_RSV1        : 12;   /*!< Bit 20-31  : R/W - Reserved                                                    */
    };
} soc_num_u;

typedef union __int_num_u__
{
    Uint32              all;
    ADC_IntNumber       adcInt;
} int_num_u;

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
/**
 * @struct  pi_t
 * @brief   Proportional-Integral controller structure for FOC algorithm.
 */
typedef struct __pi_t__
{
    ptr_32b_u           p_fb;           /*!< Pointer on the feedback current */
    ptr_32b_u           p_set;          /*!< Pointer on the setpoint current */
    ptr_32b_u           p_sat;          /*!< Pointer on the saturation value to use */
    float32             err;            /*!< Current error */
    float32             kp;             /*!< Proportional gain */
    float32             ki;             /*!< Integral gain */
    float32             integral;       /*!< Integral term partial computation */
    float32             ff;             /*!< Feed-forward term */
    float32             out;            /*!< Controller output */
} pi_t;

/**
 * @struct  pd_t
 * @brief   Proportional-Derivative controller structure for FOC algorithm.
 */
typedef struct __pd_t__
{
    ptr_32b_u           p_fbTheta;      /*!< Pointer on the feedback position */
    ptr_32b_u           p_setTheta;     /*!< Pointer on the setpoint position */
    ptr_32b_u           p_fbSpeed;      /*!< Pointer on the feedback speed */
    ptr_32b_u           p_setSpeed;     /*!< Pointer on the Set point speed */
    ptr_32b_u           p_kp;           /*!< Pointer on the Proportional gain */
    ptr_32b_u           p_kd;           /*!< Pointer on the Derivative gain */
    ptr_32b_u           p_ff;           /*!< Pointer on the feed-forward term */
    ptr_32b_u           p_sat;          /*!< Pointer on the controller saturation */
    float32             errTheta;       /*!< Error position */
    float32             errSpeed;       /*!< Error speed */
    float32             derivative;     /*!< Derivative term partial computation */
    float32             out;            /*!< Controller output */

} pd_t;

/**
 * @struct  params_t
 * @brief   Motor constants structure.
 */
typedef struct __params_t__
{
    float32             Rs;             /*!< Stator resistance [Ohm] */
    float32             Ls;             /*!< Stator inductance [H] */
    float32             kv;             /*!< Motor speed constant [RPM/V or rad/s/V] */
    float32             ke;             /*!< Motor back EMF constant [V/rad/s] (electrical radians) = 1/kv */
    float32             ki;             /*!< Motor torque constant [N.m/A] */
    float32             polePairs;      /*!< Number of pole pairs */
} params_t;

/**
 * @struct  acq_t
 * @brief   Registers localization \& results
 */
typedef struct __acquisitions_t__
{
    // ADC values scaled
    float32             ia;                 // [A]
    float32             ib;                 // [A]
    float32             ic;                 // [A]
    float32             vBus;               // [V]
    float32             vExt;               // [V]
    struct
    {
        lpf_t           vBusFlt;            // Low Pass Vbus filter parameters
        lpf_t           vExtFlt;            // Low Pass Vext filter parameters
    };
    struct
    {
        Uint32          iabcCfgBase[3];
        Uint32          iabcResultBase[3];  // ADC phase current result registers
        Uint32          vBusResultBase;     // ADC Vbus result register
        Uint32          vExtResultBase;     // ADC external voltage result register
        soc_num_u       socNum;
        int_num_u       intNum;
        ppb_num_u       ppbNum;
    };
} acq_t;

/**
 * @struct  cmd_t
 * @brief   Commands for FOC process. Target the PD controller.
 */
typedef struct __cmd_t__
{
    float32             posRef;
    float32             velRef;
    float32             iqff;
    float32             kpCoeff;
    float32             kdCoeff;
    float32             iSat;
    Uint16              timeoutRef;
    Uint16              cptTimeout;
    Uint16              index;
    cmd_reg_u           cmdField;
    status_reg_u        statusField;
} cmd_t;

/**
 * @struct  foc_t
 * @brief   General structure for FOC control.
 */
typedef struct __foc_t__
{
    params_t            cfg;            /*!< Motor configuration parameters */
    acq_t               acq;            /*!< Current \& voltage acquisitions */
    encoder_t           enc;            /*!< Encoder position & speed */
    cmd_t               cmd;            /*!< Motor commands */
    lpf_t               iParkFlt;       /*!< Low-pass filter parameters affected to park transform */
    struct
    {
        float32         sinTheta;       /*!< Sinus of the electrical rotor angle */
        float32         cosTheta;       /*!< Cosine of the electrical rotor angle */
        float32         ialpha;         /*!< Direct Clarke transform result - [A] */
        float32         ibeta;          /*!< Direct Clarke transform result - [A] */
        float32         id;             /*!< Filtered Direct Park transform result - [A] */
        float32         iq;             /*!< Filtered Quadrature Park transform result - [A] */
        float32         ud;             /*!< Inverse Park transform result - [V] */
        float32         uq;             /*!< Inverse Park transform result - [V] */
        float32         ualpha;         /*!< Inverse Clarke transform result - [V] */
        float32         ubeta;          /*!< Inverse Clarke transform result - [V] */
        float32         ua;             /*!< Phase commands - Channel a - [V] */
        float32         ub;             /*!< Phase commands - Channel b - [V] */
        float32         uc;             /*!< Phase commands - Channel c - [V] */
//        float32       vmax;           /*!< Bus voltage saturation - [V] */
        float32         dtcMax;         /*!< Max saturation duty-cycle */
        float32         dtcMin;         /*!< Min saturation duty-cycle */
        float32         dtc_u;          /*!< Duty cycles on channel u - [pu] between -0.5 and 0.5 */
        float32         dtc_v;          /*!< Duty cycles on channel v - [pu] between -0.5 and 0.5 */
        float32         dtc_w;          /*!< Duty cycles on channel w - [pu] between -0.5 and 0.5 */
        float32         idRef;          /*!< PI controller set point values for D-axis current - [A] */
        float32         iqRef;          /*!< PI controller set point values for Q-axis current - [A] */
    };
    struct
    {
        float32         initPosStep;    /*!< Initial position to force a rotor movement during alignment - [RAD] */
        Uint32          initPosCnt;     /*!< Initial position to force a rotor movement during alignment - [RAD] */
        Uint32          initFixCnt;     /*!< Initial position to force a rotor movement during alignment - [RAD] */
        float32         initPosEnc[2];  /*!< Initial positions of theta electric during alignment (forward \& reverse) - [RAD] */
        float32         initDiff;
        float32         initErr[2];
    };
    pi_t                piId;           /*!< PI controller structure for current control */
    pi_t                piIq;           /*!< PI controller structure for current control */
    pd_t                pdPosVel;       /*!< PD controller structure for position/velocity control */
    float32             resEst;         /*!< Q-axis resistance estimation - [Ohm] */
    lpf_t               resEstFlt;      /*!< Q-axis resistance estimation filter structure */
} foc_t;

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define FOC_RE_CURRENT_THRESHOLD    0.1f
#define CMD_SYSTEM_EN_POS           (0)
#define CMD_SYSTEM_EN_MASK          (1 << CMD_SYSTEM_EN_POS)
#define CMD_MOTOR_EN_POS            (1)
#define CMD_MOTOR_EN_MASK           (1 << CMD_MOTOR_EN_POS)
#define CMD_ROLLOVER_EN_POS         (2)
#define CMD_ROLLOVER_EN_MASK        (1 << CMD_ROLLOVER_EN_POS)
#define CMD_ENCODER_OFFSET_POS      (3)
#define CMD_ENCODER_OFFSET_MASK     (1 << CMD_ENCODER_OFFSET_POS)
//#define CMD_CALIBRATION_EN_POS      (4)
//#define CMD_CALIBRATION_EN_MASK     (1 << CMD_CALIBRATION_EN_POS)
//#define CMD_TIMEOUT_EN_POS          (4)
//#define CMD_TIMEOUT_EN_MASK         (1 << CMD_TIMEOUT_EN_POS)
#define CMD_POSITION_OFFSET_COMP    (CMD_SYSTEM_EN_MASK | CMD_MOTOR_EN_MASK | CMD_ENCODER_OFFSET_MASK)

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/

#endif /* __FOC_H__ */
