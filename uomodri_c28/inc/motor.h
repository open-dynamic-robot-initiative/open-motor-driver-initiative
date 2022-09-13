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
 * DEFINES
 ***********************************************************************/
#define MOTOR_ERROR_NO_ERROR        (0) /*!< No error */
#define MOTOR_ERROR_ENCODER         (1) /*!< Encoder error too high */
#define MOTOR_ERROR_POS_ROLLOVER    (2) /*!< Position roll-over occurred */
#define MOTOR_ERROR_DRV_NFAULT      (4) /*!< Motor DRV nFault error */
#define MOTOR_ERROR_COM_TIMEOUT     (8) /*!< Communication timeout error */

/***********************************************************************
 * TYPDEF ENUM
 ***********************************************************************/
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

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
typedef struct __error_reg_t__
{
    uint16_t    enc_mismatch    : 1;                /*!< Bits 0   : R/W - Encoder error too high */
    uint16_t    pos_rollover    : 1;                /*!< Bits 1   : R/W - Position roll-over occurred */
    uint16_t    drv_fault       : 1;                /*!< Bits 2   : R/W - Motor DRV nFault error */
    uint16_t    com_timeout     : 1;                /*!< Bits 3   : R/W - Communication timeout error */
} error_reg_t;

typedef union __error_reg_u__
{
    uint16_t    all;
    error_reg_t bit;
} error_reg_u;

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
    error_reg_u                     motor_error;    /*!< Error messages */
    // PWM address register for FOC command
    volatile uint16_t* const        p_motorChAReg;
    volatile uint16_t* const        p_motorChBReg;
    volatile uint16_t* const        p_motorChCReg;
    bool_t                          itDone;
} motor_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
inline void MOT_runCommand(motor_t*, float32_t, float32_t, float32_t);
inline void MOT_stopCommand(motor_t*);
inline bool_t MOT_runControl(motor_t*);

#endif /* __MOTOR_H__ */
