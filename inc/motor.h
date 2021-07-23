/*
 * motor.h
 *
 *  Created on: 20 mai 2021
 *      Author: jmanhes
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "driverlib.h"
#include "device.h"
#include "f2838x_device.h"

#include <math.h>
#include "hal.h"
#include "foc.h"
#include "anticoagging.h"
#include "uomodri_user_defines.h"

typedef enum
{
    ALIGN_COUNT_UP,
    ALIGN_STAY,
    ALIGN_COUNT_DOWN,
    ALIGN_DONE
} AlignState;

typedef struct __MOTOR_ALIGN__
{
    AlignState state;

    uint32_t counter;
    uint32_t max;

    float32_t idAlignRef;
    float32_t idAlign;
} MOTOR_ALIGN;

typedef struct __MOTOR_STRUCT__
{
    const MOTOR_ID          motorID;
    const HAL_MOTOR_CFG*    motorHalCfg;
    DRV8353*                motorDRV;
    FOC*                    motorFOC;
    bool                    enabled;
    // Alignment routine structure
    MOTOR_ALIGN             motorAlign;
    // Cycle counter events
    uint32_t clCycleNb;
    uint64_t interruptCounter;

    // Utility variables
//    float32_t currentScaler;    // [A/pu]
//    float32_t currentScalerReg; // [A/register]
//    float32_t voltageScaler;    // [V/pu]
//    float32_t voltageScalerReg; // [V/register]
//    float32_t adcScaler;        // [V/pu]
//    float32_t adcScalerReg;     // [V/register]
//    float32_t pwmCompScaler;    // [register/pu]

    // Duty-cycle saturation
    float32_t dutyCycleSat;

    // Motor constants
//    float32_t Rs; // Stator resistance [Ohm]
//    float32_t Ld; // Direct axis inductance [H]
//    float32_t Lq; // Quadratic axis inductance [H]
//    float32_t ke; // Motor back EMF constant [V/rad/s] (electrical radians)

     // Rotor electrical angle sine cosine
//    float32_t sinTheta;
//    float32_t cosTheta;

    // Low-pass control loop filter
    LpfStruct controlLoopLpf;

    // Encoder
//    ENCODER encoder;

      // Duty cycles
    float32_t ducya;    // [pu] between -0.5 and 0.5
    float32_t ducyb;    // [pu] between -0.5 and 0.5
    float32_t ducyc;    // [pu] between -0.5 and 0.5


    // PI controllers
//    float32_t idRef;    // [A]
//    float32_t iqRef;    // [A]

//    PI_controller piId;
//    PI_controller piIq;

    // Position PD controller
//    PD_controller posPd;

    // Phase measured voltages
//    float32_t phaseAVolt;   // [V]
//    float32_t phaseBVolt;   // [V]
//    float32_t phaseCVolt;   // [V]

    // Q-axis resistance estimation
    float32_t statorResEst;    // [Ohm]
    // Resistance estimation low-pass filter
    LpfStruct statorResEstLpf;

    // Anti-coagging structure
    Anticoagging acog;

    // Auxiliary ADC voltage
    float32_t auxAdc;   // [V]

    // PWM result registers
    volatile uint32_t *pwmCompA;
    volatile uint32_t *pwmCompB;
    volatile uint32_t *pwmCompC;

    volatile uint16_t* pwmChAReg;
    volatile uint16_t* pwmChBReg;
    volatile uint16_t* pwmChCReg;



    // ADC interrupt flag register
    volatile union ADCINTFLG_REG *adcIntFlag;
} MOTOR_STRUCT;

//inline void MOTOR_getCurrent(MOTOR_STRUCT*);
//inline void MOTOR_getCurrent(MOTOR_MEASURES*);
//inline void MOTOR_getVoltage(MOTOR_MEASURES*);
void MOT_runCommand(MOTOR_STRUCT*, float32_t, float32_t, float32_t);
void MOT_stopCommand(MOTOR_STRUCT*);
void MOT_resetAlignStruct(MOTOR_ALIGN*);
void MOT_runAlign(MOTOR_ALIGN*);
void MOT_interruptClearFlags(void);
//void ALIGN_run(AlignRoutine*);

void MOT_emergencyStop(void);
void MOT_StatorResEst(MOTOR_STRUCT*);

#endif /* INC_MOTOR_H_ */
