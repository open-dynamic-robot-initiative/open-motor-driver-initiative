/*
 * motor.h
 *
 *  Created on: 20 mai 2021
 *      Author: jmanhes
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "f2838x_device.h"
#include "driverlib.h"
#include "device.h"

#include <math.h>
#include "hal.h"
#include "foc.h"
#include "anticoagging.h"
#include "uomodri_user_defines.h"



typedef struct __MOTOR_STRUCT__
{
//    const ID_ENUM           motorID;
    const HAL_MOTOR_CFG*    p_motorHalCfg;
    DRV8353*                p_motorDRV;
    FOC_STRUCT*             p_motorFOC;
    //bool                    enabled;

    // Alignment routine structure
//    ALIGN_STRUCT            motorAlign;
    // Cycle counter events
    uint32_t                clCycleNb;
    uint64_t                interruptCnt;

    // Q-axis resistance estimation
    float32_t               statorResEst;    // [Ohm]
    // Resistance estimation low-pass filter
    LPFSTRUCT               statorResEstLpf;

    // Anti-coagging structure
    Anticoagging            acog;

    // PWM address register for FOC command
    volatile uint16_t*      p_motorChAReg;
    volatile uint16_t*      p_motorChBReg;
    volatile uint16_t*      p_motorChCReg;
} MOTOR_STRUCT;

//inline void MOTOR_getCurrent(MOTOR_STRUCT*);
//inline void MOTOR_getCurrent(MOTOR_MEASURES*);
//inline void MOTOR_getVoltage(MOTOR_MEASURES*);
void MOT_runCommand(MOTOR_STRUCT*, float32_t, float32_t, float32_t);
void MOT_stopCommand(MOTOR_STRUCT*);
void MOT_resetAlignStruct(ALIGN_STRUCT*);
void MOT_runAlign(ALIGN_STRUCT*);
//void MOTOR_interruptClearFlags(IT_TypeDef*);
//void ALIGN_run(AlignRoutine*);

//void MOT_emergencyStop(void);
//void MOT_emergencyStop(MOTOR_STRUCT*);
void MOT_resistorEstimation(MOTOR_STRUCT*);

#endif /* INC_MOTOR_H_ */
