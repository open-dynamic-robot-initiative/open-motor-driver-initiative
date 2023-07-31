#ifndef __MOTOR_H__
#define __MOTOR_H__

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <stdint.h>
#include <hw_types.h>
#include "f2838x_device.h"

#include "foc.h"
#include "uomodri_user_defines.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define MOTOR_STATE_IDLE                            (0x0000)
#define MOTOR_STATE_INIT_FW                         (0x0001)
#define MOTOR_STATE_INIT_REV                        (0x0002)
#define MOTOR_STATE_INIT_TEST                       (0x0004)
#define MOTOR_STATE_INIT_FIX                        (0x0008)
#define MOTOR_STATE_READY                           (0x0010)
#define MOTOR_STATE_STOP                            (0x0020)
#define MOTOR_STATE_ERROR                           (0xFFFF)

#define USE_STATE_MACHINE_4_ELECTRONCS_ENGINEERS    (0)

/***********************************************************************
 * ENUM TYPES
 ***********************************************************************/
//typedef enum __motor_states__
//{
//    MOTOR_STATE_START       = 0x0000,
//    MOTOR_STATE_INIT_FW     = 0x0001,
//    MOTOR_STATE_INIT_REV    = 0x0002,
//    MOTOR_STATE_INIT_TEST   = 0x0004,
//    MOTOR_STATE_INIT_FIX    = 0x0008,
//    MOTOR_STATE_READY       = 0x0010,
//    MOTOR_STATE_STOP        = 0x0020,
//    MOTOR_STATE_ERROR       = 0xFFFF,
//} motor_states;

/***********************************************************************
 * UNION TYPES
 ***********************************************************************/
/**
 * @union   error_reg_u
 * @brief   Bit field associated to all sources of motor errors.
 */
typedef union __error_reg_u__
{
    uint16_t                    all;
    struct
    {
        uint16_t                enc_indexErr    : 1;    /*!< Bits 0   : R/W - Encoder error too high */
        uint16_t                pos_rollover    : 1;    /*!< Bits 1   : R/W - Position roll-over occurred */
        uint16_t                drv_fault       : 1;    /*!< Bits 2   : R/W - Motor DRV nFault error */
        uint16_t                com_timeout     : 1;    /*!< Bits 3   : R/W - Communication timeout error */
        uint16_t                enc_alignErr    : 1;    /*!< Bits 4   : R/W - encoder failure during alignment. */
        uint16_t                error_RSV1      : 11;   /*!< Bits 5-15: R/W - Reserved */
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
    foc_t                       foc;                    /*!< FOC structure associated to the motor */
    uint32_t                    itCnt;                  /*!< Event counter incrementing on every IT call */
    uint32_t                    epwmBase[3];            /*!< Address array for the PWM peripherals associated to the motor control */
    uint32_t                    drvFaultNum;            /*!< GPIO associated to the DRV fault line */
    uint32_t                    drvEnNum;               /*!< GPIO associated to the DRV enable line */
    uint32_t                    clCycleNb;
    struct
    {
        float32_t               statorResEst;           /*!< Q-axis resistance estimation. [Ohm] */
        lpf_t                   statorResEstFlt;        /*!< Resistance estimation low-pass filter */
    };
    uint16_t                    id;                     /*!< Motor identification */
    uint16_t                    state;                  /*!< Current motor state for the FSM. */
    error_reg_u                 error;                  /*!< Bit field for the error sources. */
} motor_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void MOT_initStruct(motor_t*, uint16_t);

#endif /* __MOTOR_H__ */
