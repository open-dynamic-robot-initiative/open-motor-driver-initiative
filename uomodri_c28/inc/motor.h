#ifndef __MOTOR_H__
#define __MOTOR_H__

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <stdint.h>
#include <hw_types.h>
#include <f2838x_device.h>

#include "foc.h"
#include "uomodri_user_defines.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define MOTOR_STATE_START                           (0x00)
#define MOTOR_STATE_INIT_FW                         (0x01)
#define MOTOR_STATE_INIT_REV                        (0x02)
#define MOTOR_STATE_INIT_TEST                       (0x04)
#define MOTOR_STATE_INIT_FIX                        (0x08)
#define MOTOR_STATE_READY                           (0x10)
#define MOTOR_STATE_STOP                            (0x20)
#define MOTOR_STATE_ERROR                           (0xFF)

#define USE_STATE_MACHINE_4_ELECTRONCS_ENGINEERS    (1)

/***********************************************************************
 * ENUM TYPES
 ***********************************************************************/

/***********************************************************************
 * UNION TYPES
 ***********************************************************************/
/**
 * @union   error_reg_u
 * @brief   Bit field associated to all sources of motor errors.
 */
typedef union __error_reg_u__
{
    uint8_t                     all;
    struct
    {
        uint8_t                 enc_mismatch    : 1;    /*!< Bits 0   : R/W - Encoder error too high */
        uint8_t                 pos_rollover    : 1;    /*!< Bits 1   : R/W - Position roll-over occurred */
        uint8_t                 drv_fault       : 1;    /*!< Bits 2   : R/W - Motor DRV nFault error */
        uint8_t                 com_timeout     : 1;    /*!< Bits 3   : R/W - Communication timeout error */
        uint8_t                 enc_failure     : 1;    /*!< Bits 4   : R/W - encoder failure during alignment. */
        uint8_t                 error_RSV1      : 3;    /*!< Bits 5-7 : R/W - Reserved */
    };
} error_reg_u;

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
/**
 * @struct  motor_t
 * @brief   General structure for motor control.
 */
typedef struct __motor_t__
{
#if (CLA1_CORE_ENABLE)
    ptr_32b_u                   p_foc;                  /*!< Pointer on the FOC structure associated to the motor */
#else
    foc_t* const                p_foc;                  /*!< Pointer on the FOC structure associated to the motor */
#endif
    uint64_t                    itCnt;                  /*!< Event counter incrementing on every IT call */
    uint32_t                    epwmBaseAddr[3];        /*!< Address array for the PWM peripherals associated to the motor control */
    uint32_t                    drvFaultNum;            /*!< GPIO associated to the DRV fault line */
    uint32_t                    drvEnNum;               /*!< GPIO associated to the DRV enable line */
    uint32_t                    clCycleNb;
    float32_t                   statorResEst;           /*!< Q-axis resistance estimation. [Ohm] */
    lpf_t                       statorResEstFlt;        /*!< Resistance estimation low-pass filter */
#if (CLA1_CORE_ENABLE)
    volatile ptr_32b_u          p_cmdReg[3];            /*!< PWM address register for FOC command */
    uint8_t                     motor_id;               /*!< Motor identification */
#else
    volatile uint16_t* const    p_cmdReg[3];            /*!< PWM address register for FOC command */
    const uint8_t               motor_id;               /*!< Motor identification */
#endif
    uint8_t                     motor_state;            /*!< Current motor state for the FSM. */
    error_reg_u                 motor_error;            /*!< Bit field for the error sources. */
    bool_t                      itDone;
} motor_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/

#endif /* __MOTOR_H__ */
