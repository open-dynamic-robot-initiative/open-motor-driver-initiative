#ifndef __UOMODRI_CLA_TASKS_SHARE_H__
#define __UOMODRI_CLA_TASKS_SHARE_H__

// drivers
#include "f2838x_device.h"
#include "f2838x_examples.h"
#include "driverlib.h"
#include "device.h"

#include "encoder.h"


//extern uint8_t  polePairs;    // Number of pole pairs of the motor
//extern float    thetaMechScaler; // [pu/register]
//extern float32_t    thetaMech[2]; // [pu]
extern uint8_t      thetaDir; //0 = CCW/reverse, 1 = CW/forward
extern float        thetaMech[2]; // [pu]
extern float        thetaElec; // [pu]
extern ENC_STRUCT   motor_enc;
//extern float32_t    thetaMechCal[2]; // [pu]
//extern float32_t    pdSat;

//__attribute__((interrupt)) void cla1_isr1(void);
//CLA C Tasks defined in Cla1Tasks_C.cla
__attribute__((interrupt)) void Cla1Task1(void);
__attribute__((interrupt)) void Cla1Task8(void);

#endif /* __UOMODRI_CLA_TASKS_SHARE_H__ */
