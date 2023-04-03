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
#define MOTOR_STATE_INIT        (0x00)
#define MOTOR_STATE_ALIGN_UP    (0x01)
#define MOTOR_STATE_ALIGN_FIX   (0x02)
#define MOTOR_STATE_READY       (0x04)
#define MOTOR_STATE_STOP        (0x08)
#define MOTOR_STATE_ERROR       (0xFF)

/***********************************************************************
 * TYPDEF ENUM
 ***********************************************************************/

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
typedef union __error_reg_u__
{
    uint8_t                     all;
    struct
    {
        uint8_t                 enc_mismatch    : 1;    /*!< Bits 0   : R/W - Encoder error too high */
        uint8_t                 pos_rollover    : 1;    /*!< Bits 1   : R/W - Position roll-over occurred */
        uint8_t                 drv_fault       : 1;    /*!< Bits 2   : R/W - Motor DRV nFault error */
        uint8_t                 com_timeout     : 1;    /*!< Bits 3   : R/W - Communication timeout error */
        uint8_t                 error_RSV1      : 4;    /*!< Bits 4-15: R/W - Reserved */
    };
} error_reg_u;

typedef struct __motor_struct_t__
{
#if (CLA1_CORE_ENABLE)
    foc_t*                      p_motorFOC;             /*!< Pointer on the FOC structure associated to the motor */
#else
    foc_t* const                p_motorFOC;             /*!< Pointer on the FOC structure associated to the motor */
#endif
    uint64_t                    itCnt;                  /*!< Event counter incrementing on every IT call */
    uint32_t                    epwmBaseAddr[3];
    uint32_t                    drvFaultNum;
    uint32_t                    drvEnNum;
    uint32_t                    clCycleNb;
    float32_t                   statorResEst;           /*!< Q-axis resistance estimation. [Ohm] */
    lpf_t                       statorResEstFlt;        /*!< Resistance estimation low-pass filter */
#if (CLA1_CORE_ENABLE)
    volatile cla_u16_ptr        p_motorCmdReg[3];       /*!< PWM address register for FOC command */
    uint8_t                     motor_id;               /*!< Motor identification */
#else
    volatile uint16_t* const    p_motorCmdReg[3];       /*!< PWM address register for FOC command */
    const uint8_t               motor_id;               /*!< Motor identification */
#endif
    uint8_t                     motor_state;            /*!< Current motor state for the FSM. */
    error_reg_u                 motor_error;            /*!< Error messages */
    bool_t                      itDone;
} motor_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
//inline void MOT_runCommand(motor_t*, float32_t, float32_t, float32_t);
//inline void MOT_stopCommand(motor_t*);
//inline bool_t MOT_runControl(motor_t*);

#endif /* __MOTOR_H__ */
