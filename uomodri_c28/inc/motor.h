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
 * CONTROL STRUCTURES
 ***********************************************************************/
typedef struct __MOTORS_STRUCT__
{
    const hal_motor_cfg_t*  const   p_motorHalCfg;
    drv8353_t* const                p_motorDRV;
    foc_t* const                    p_motorFOC;
    // Cycle counter events
    uint64_t                        itCnt;
    uint32_t                        clCycleNb;
    // Q-axis resistance estimation
    float32_t                       statorResEst;    // [Ohm]
    // Resistance estimation low-pass filter
    lpf_t                           statorResEstFlt;
    // Anti-coagging structure
//    Anticoagging            acog;
    bool_t                          itDone;
    // PWM address register for FOC command
    volatile uint16_t* const        p_motorChAReg;
    volatile uint16_t* const        p_motorChBReg;
    volatile uint16_t* const        p_motorChCReg;
} motor_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void MOT_runCommand(motor_t*, float32_t, float32_t, float32_t);
void MOT_stopCommand(motor_t*);
void MOT_resistorEstimation(motor_t*);

#endif /* __MOTOR_H__ */
