#ifndef __MOTOR_H__
#define __MOTOR_H__

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "f2838x_device.h"
#include "driverlib.h"
#include "device.h"

#include "hal.h"
#include "foc.h"
#include "uomodri_user_defines.h"
#include "anticoagging.h"


typedef enum
{
    MOTOR_ALIGN_COUNT_UP    = 0,/*!< No error */
    MOTOR_ALIGN_STAY        = 1, /*!< Encoder_1 error too high */
    MOTOR_ALIGN_COUNT_DOWN  = 2,/*!< Timeout for receiving current references exceeded */
    MOTOR_ALIGN_DONE        = 3, /*!< Motor temperature reached critical valued */
    MOTOR_RUN               = 4,/*!< Unused error */
    MOTOR_TEMP              = 5,
    MOTOR_ANTICOGGING       = 6,
} motor_state;

typedef struct __MOTORS_STRUCT__
{
    const hal_motor_cfg_t*  p_motorHalCfg;
    drv8353_t*              p_motorDRV;
    foc_t*                  p_motorFOC;
    // Cycle counter events
    uint64_t                interruptCnt;
    uint32_t                clCycleNb;
    // Q-axis resistance estimation
    float32_t               statorResEst;    // [Ohm]
    // Resistance estimation low-pass filter
    lpf_t                   statorResEstFlt;
    // Anti-coagging structure
    Anticoagging            acog;
    bool_t                  interruptDone;
    // PWM address register for FOC command
    volatile uint16_t*      p_motorChAReg;
    volatile uint16_t*      p_motorChBReg;
    volatile uint16_t*      p_motorChCReg;
} motor_t;

//inline void MOTOR_getCurrent(MOTOR_STRUCT*);
//inline void MOTOR_getCurrent(MOTOR_MEASURES*);
//inline void MOTOR_getVoltage(MOTOR_MEASURES*);
void MOT_runCommand(motor_t*, float32_t, float32_t, float32_t);
void MOT_stopCommand(motor_t*);
void MOT_resetAlignStruct(align_t*);
void MOT_runAlign(align_t*);
//void MOTOR_interruptClearFlags(IT_TypeDef*);
//void ALIGN_run(AlignRoutine*);

//void MOT_emergencyStop(void);
//void MOT_emergencyStop(MOTOR_STRUCT*);
void MOT_resistorEstimation(motor_t*);

#endif /* __MOTOR_H__ */
