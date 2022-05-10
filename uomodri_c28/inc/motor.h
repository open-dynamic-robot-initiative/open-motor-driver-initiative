#ifndef __MOTOR_H__
#define __MOTOR_H__

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "f2838x_device.h"
#include "hal.h"
#include "foc.h"
#include "drv8353.h"
#include "uomodri_user_defines.h"

/***********************************************************************
 * TYPDEF ENUM
 ***********************************************************************/
/**
 * @enum    motor_svm_sector_e
 * @brief   Space Vector Modulation (SVM) - Sectors definitions
 */
typedef enum
{
    MOTOR_SVM_SECTOR_0          = 0x00,  /*!< SECTOR_\#0 : Default sector state */
    MOTOR_SVM_SECTOR_1          = 0x06,  /*!< SECTOR_\#1 : Ubeta >= 0.0 && ((sqrt(3) x Ualpha) - Ubeta) >= 0.0 */
    MOTOR_SVM_SECTOR_2          = 0x04,  /*!< SECTOR_\#2 : Ubeta >= 0.0 && (((sqrt(3) x Ualpha) - Ubeta) < 0.0 || (-(sqrt(3) x Ualpha) - Ubeta) < 0.0) */
    MOTOR_SVM_SECTOR_3          = 0x05,  /*!< SECTOR_\#3 : Ubeta >= 0.0 && (-(sqrt(3) x Ualpha) - Ubeta) >= 0.0 */
    MOTOR_SVM_SECTOR_4          = 0x01,  /*!< SECTOR_\#4 : Ubeta < 0.0 && ((sqrt(3) x Ualpha) - Ubeta) < 0.0 */
    MOTOR_SVM_SECTOR_5          = 0x03,  /*!< SECTOR_\#5 : Ubeta < 0.0 && (((sqrt(3) x Ualpha) - Ubeta) >= 0.0 || (-(sqrt(3) x Ualpha) - Ubeta) >= 0.0) */
    MOTOR_SVM_SECTOR_6          = 0x02,  /*!< SECTOR_\#6 : Ubeta < 0.0 && (-(sqrt(3) x Ualpha) - Ubeta) < 0.0 */
    MOTOR_SVM_SECTOR_7          = 0x07,   /*!< SECTOR_\#6 : ERROR */
} motor_svm_sector_e;

/**
 * @enum    motor_state_e
 * @brief   Motor states for the Finite State Machine (FSM).
 */
typedef enum
{
    MOTOR_STATE_INIT            = 1,
    MOTOR_STATE_ALIGN_UP        = 2,    /*!< ALIGN_COUNT_UP */
    MOTOR_STATE_ALIGN_FIX       = 3,    /*!< ALIGN_FIX */
    MOTOR_STATE_READY           = 4,
    MOTOR_STATE_STOP            = 5,
    MOTOR_STATE_ERROR           = 15,
} motor_state_e;

/**
 * @enum    motor_error_e
 * @brief   Motor error flags list.
 */
typedef enum
{
    MOTOR_ERROR_NO_ERROR        = 0,    /*!< No error */
    MOTOR_ERROR_ENCODER         = 1,    /*!< Encoder_1 error too high */
    MOTOR_ERROR_POS_ROLLOVER    = 2,    /*!< Position roll-over occurred */
    MOTOR_ERROR_DRV_NFAULT      = 4,    /*!< Motor DRV nFault error */
    MOTOR_ERROR_COM_TIMEOUT     = 8,    /*!< Communication timeout error */
} motor_error_e;

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
/**
 * @struct  mst2slv_reg_t
 * @brief   Master to Slave command structure (bits definition) for uOmodri
 */
typedef struct __svm_test_reg_t__
{
    uint16_t        NEG_UALPHA_UBETA_SIGN_TEST  : 1;    /*!< Bits 0     : R/W - */
    uint16_t        UALPHA_UBETA_SIGN_TEST      : 1;    /*!< Bits 1     : R/W - */
    uint16_t        UBETA_SIGN_TEST             : 1;    /*!< Bits 2     : R/W - */
    uint16_t        SVM_TEST_RSV1               : 13;   /*!< Bits 3-15  : R/W - Reserved    */
} svm_test_reg_t;

/**
 * @union   mst2slv_reg_u
 * @brief   Master to Slave command union (bits definition) for uOmodri. Access to bit field.
 */
typedef union __svm_test_reg_u__
{
  uint16_t          all;                /*!< Short (16bits) type access */
  svm_test_reg_t    bit;                /*!< @see command_mode_t        */
} svm_test_reg_u;

//typedef struct
//{
//    uint32_t b_TIM1PrdCnt;
//    // b_VrefA = b_Vm * cos(2*pi*b_freq*t)
//    // b_VrefB = b_Vm * cos(2*pi*b_freq*t - 2*pi/3)
//    // b_VrefC = b_Vm * cos(2*pi*b_freq*t + 2*pi/3)
//    float b_VrefA, b_VrefB, b_VrefC;
//    float b_Vm, b_freq;
//    float b_VrefAlpha, b_VrefBeta;
//
//    uint8_t b_sector;
//    float b_VrefAngle;
//
//    //space vector
//    //0 : 000       1: 001  2: 010  3: 011  4: 100  5: 101  6: 110  7: 111
//    uint8_t a_VectorOut[3];
//    float a_VectorOutDuty[3];
//    uint32_t a_VectorOutDutyCmpr[4];
//
//} BLDC_SVPWMTypeDef;

typedef struct __MOTOR_STRUCT_t__
{
    const motor_id_e                motor_id;       /*!< Motor identification */
    const hal_motor_cfg_t*  const   p_motorHalCfg;  /*!< Pointer on the HAL structure (PWM, ADC , IT) associated to the motor */
    drv8353_t* const                p_motorDRV;     /*!< Pointer on the DRV structure associated to the motor */
    foc_t* const                    p_motorFOC;     /*!< Pointer on the FOC structure associated to the motor */
    uint64_t                        itCnt;          /*!< Event counter incrementing on every IT call */
    uint32_t                        clCycleNb;
    // Q-axis resistance estimation
    float32_t                       statorResEst;    // [Ohm]
    // Resistance estimation low-pass filter
    lpf_t                           statorResEstFlt;
    // Current motor state for the FSM
    motor_state_e                   motor_state;    /*!< Current motor state for the FSM */
    // Error messages
    motor_error_e                   motor_error;    /*!< Error messages */
    // Anti-coagging structure
//    Anticoagging            acog;
    // PWM address register for FOC command
    volatile uint16_t* const        p_motorChAReg;
    volatile uint16_t* const        p_motorChBReg;
    volatile uint16_t* const        p_motorChCReg;
    bool_t                          itDone;
} motor_t;

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
inline void MOT_runCommand(motor_t*, float32_t, float32_t, float32_t);
inline void MOT_stopCommand(motor_t*);
inline bool_t MOT_runControl(motor_t*);
void MOT_resistorEstimation(motor_t*);

#endif /* __MOTOR_H__ */
