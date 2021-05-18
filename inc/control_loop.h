#ifndef __CONTROL_LOOP_H__
#define __CONTROL_LOOP_H__

#include "hal.h"
#include "foc.h"
#include "pd.h"
#include "pi.h"

// Debug-only structure used to store motor data
typedef struct _MotorDebug_
{
    uint32_t clCycleNb;
} MotorDebug;


void MTR1_offsetComputation(MotorDebug* motorDebug);
void MTR2_offsetComputation(MotorDebug* motorDebug);

void CL_initFoc(FOC *foc, const uint16_t motorIndex);
void CL_resetFoc(FOC* foc);

#endif
