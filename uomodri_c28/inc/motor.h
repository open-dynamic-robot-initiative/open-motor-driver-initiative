#ifndef __MOTOR_H__
#define __MOTOR_H__

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "f2838x_device.h"
#include "foc.h"
#include "uomodri_user_defines.h"

/***********************************************************************
 * ENUM TYPES
 ***********************************************************************/
typedef enum __motor_state__
{
    MOTOR_STATE_IDLE        = 0x0000,
    MOTOR_STATE_INIT_FW     = 0x0001,
    MOTOR_STATE_INIT_REV    = 0x0002,
    MOTOR_STATE_INIT_TEST   = 0x0004,
    MOTOR_STATE_INIT_FIX    = 0x0008,
    MOTOR_STATE_READY       = 0x0010,
    MOTOR_STATE_STOP        = 0x0020,
    MOTOR_STATE_ERROR       = 0xFFFF,
} motor_state;

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define MOTOR_STATE_OK                              (MOTOR_STATE_READY | \
                                                     MOTOR_STATE_STOP)
#define MOTOR_STATE_INIT                            (MOTOR_STATE_INIT_FW | \
                                                     MOTOR_STATE_INIT_REV | \
                                                     MOTOR_STATE_INIT_TEST | \
                                                     MOTOR_STATE_INIT_FIX)
#define MOTOR_STATE_BOOT                            (MOTOR_STATE_IDLE)

#define USE_STATE_MACHINE_4_ELECTRONCS_ENGINEERS    (0)

/***********************************************************************
 * UNION TYPES
 ***********************************************************************/
/**
 * @union   error_reg_u
 * @brief   Bit field associated to all sources of motor errors.
 */
typedef union __error_reg_u__
{
    Uint16          all;
    struct
    {
        Uint16      enc_indexErr    : 1;    /*!< Bits 0   : R/W - Encoder error too high */
        Uint16      enc_alignErr    : 1;    /*!< Bits 1   : R/W - Encoder failure during alignment. */
        Uint16      pos_rollover    : 1;    /*!< Bits 2   : R/W - Position roll-over occurred */
        Uint16      drv_fault       : 1;    /*!< Bits 3   : R/W - Motor DRV nFault error */
        Uint16      com_timeout     : 1;    /*!< Bits 4   : R/W - Communication timeout error */
        Uint16      error_RSV1      : 11;   /*!< Bits 5-15: R/W - Reserved */
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
    foc_t           foc;                    /*!< FOC structure associated to the motor */
    Uint32          itCnt;                  /*!< Event counter incrementing on every IT call */
    Uint32          epwmBase[3];            /*!< Address array for the PWM peripherals associated to the motor control */
    Uint32          drvFaultNum;            /*!< GPIO associated to the DRV fault line */
    Uint32          drvEnNum;               /*!< GPIO associated to the DRV enable line */
    Uint32          clCycleNb;
    struct
    {
        float32     statorResEst;           /*!< Q-axis resistance estimation. [Ohm] */
        lpf_t       statorResEstFlt;        /*!< Resistance estimation low-pass filter */
    };
    Uint16          id;                     /*!< Motor identification */
    error_reg_u     error;                  /*!< Bit field for the error sources. */
    motor_state     state;                  /*!< Current motor state for the FSM. */
} motor_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void MOT_initStruct(motor_t*, Uint16);

#endif /* __MOTOR_H__ */
