/*
 * File name: myactuator_rmdx8h.h
 * Description: Header file containing definitions of RMDx8H motor from MyACTUATOR.
 */

#ifndef __MYACTUATOR_RMDX8H_DEFINES_H__
#define __MYACTUATOR_RMDX8H_DEFINES_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "../../uomodri_user_defines.h"

/***********************************************************************
 * MOTORS DEFINES
 ***********************************************************************/
#define MYACTUATOR_RMDX8H_KV                                (33.0f)     // Motor constant (rpm/V)
#define MYACTUATOR_RMDX8H_KE                                (FM_SQRT3 / (MYACTUATOR_RMDX8H_KV * FM_RPM2RADPS)) // Motor back EMF constant (V/rad/s)
#define MYACTUATOR_RMDX8H_KI                                (0.3f)      // Motor torque constant (N.m/A)
#define MYACTUATOR_RMDX8H_RS                                (0.52f)     // Stator resistance (ohm)
#define MYACTUATOR_RMDX8H_LS                                (260e-6)    // Stator d-axis inductance (H)
#define MYACTUATOR_RMDX8H_POLES_PAIRS                       (20.0f)     // Number of poles
#define MYACTUATOR_RMDX8H_CURRENT_INIT_MAX                  (2.0f)      // Current on d-axis for motor alignment (A)
#define MYACTUATOR_RMDX8H_CURRENT_CMD_SAT_MAX               (10.0f)

#define MYACTUATOR_RMDX8H_CURRENT_CUTOFF_FREQ               (1000.0f)   // Current loop bandwidth (Hz)
#define MYACTUATOR_RMDX8H_CURRENT_TIME_CONST                (FM_2MULTPI * MYACTUATOR_RMDX8H_CURRENT_CUTOFF_FREQ * PWM_PERIOD)
#define MYACTUATOR_RMDX8H_CURRENT_LPF_ALPHA                 (MYACTUATOR_RMDX8H_CURRENT_TIME_CONST / (1.0f + MYACTUATOR_RMDX8H_CURRENT_TIME_CONST))
#define MYACTUATOR_RMDX8H_CURRENT_LPF_ONE_M_ALPHA           (1.0f - MYACTUATOR_RMDX8H_CURRENT_LPF_ALPHA)
#define MYACTUATOR_RMDX8H_FIX_INIT_DELAY                    (2.0f)      // in seconds
#define MYACTUATOR_RMDX8H_FIX_INIT_EVT                      (MYACTUATOR_RMDX8H_FIX_INIT_DELAY * PWM_FREQ)
#define MYACTUATOR_RMDX8H_CURRENT_INIT_DELAY                (0.5f)      // in seconds
#define MYACTUATOR_RMDX8H_CURRENT_INIT_EVT                  (MYACTUATOR_RMDX8H_CURRENT_INIT_DELAY * PWM_FREQ)
#define MYACTUATOR_RMDX8H_CURRENT_INIT_STEP                 (MYACTUATOR_RMDX8H_CURRENT_INIT_MAX / MYACTUATOR_RMDX8H_CURRENT_INIT_EVT)
#define MYACTUATOR_RMDX8H_POSITION_INIT_MAX                 FM_2MULTPI
#define MYACTUATOR_RMDX8H_POSITION_INIT_DELAY               MYACTUATOR_RMDX8H_CURRENT_INIT_DELAY
#define MYACTUATOR_RMDX8H_POSITION_INIT_EVT                 (MYACTUATOR_RMDX8H_POSITION_INIT_DELAY * PWM_FREQ)
#define MYACTUATOR_RMDX8H_POSITION_INIT_STEP                (MYACTUATOR_RMDX8H_POSITION_INIT_MAX / MYACTUATOR_RMDX8H_POSITION_INIT_EVT)
#define MYACTUATOR_RMDX8H_POSITION_INIT_ERR_MAX             ((MYACTUATOR_RMDX8H_POSITION_INIT_MAX + FM_2MULTPIDIV10) / MYACTUATOR_RMDX8H_POLES_PAIRS)
#define MYACTUATOR_RMDX8H_POSITION_INIT_ERR_MIN             ((MYACTUATOR_RMDX8H_POSITION_INIT_MAX - FM_2MULTPIDIV10) / MYACTUATOR_RMDX8H_POLES_PAIRS)
#define MYACTUATOR_RMDX8H_POSITION_INIT_ERR                 (FM_2MULTPI / 10.0f / MYACTUATOR_RMDX8H_POLES_PAIRS)
#define MYACTUATOR_RMDX8H_PI_ID_KP_COEF                     (MYACTUATOR_RMDX8H_LS * FM_2MULTPI * MYACTUATOR_RMDX8H_CURRENT_CUTOFF_FREQ)
#define MYACTUATOR_RMDX8H_PI_ID_KI_COEF                     (MYACTUATOR_RMDX8H_RS * FM_2MULTPI * MYACTUATOR_RMDX8H_CURRENT_CUTOFF_FREQ)
#define MYACTUATOR_RMDX8H_PI_IQ_KP_COEF                     (MYACTUATOR_RMDX8H_LS * FM_2MULTPI * MYACTUATOR_RMDX8H_CURRENT_CUTOFF_FREQ)
#define MYACTUATOR_RMDX8H_PI_IQ_KI_COEF                     (MYACTUATOR_RMDX8H_RS * FM_2MULTPI * MYACTUATOR_RMDX8H_CURRENT_CUTOFF_FREQ)
#define MYACTUATOR_RMDX8H_DTC_MAX                           (0.9f)      // Max duty cycle
#define MYACTUATOR_RMDX8H_DTC_MIN                           (0.1f)      // Min duty cycle
#define MYACTUATOR_RMDX8H_STATOR_RESISTOR_CUTOFF_FREQ       (0.2f)
#define MYACTUATOR_RMDX8H_STATOR_RESISTOR_TIME_CONST        (FM_2MULTPI * MYACTUATOR_RMDX8H_STATOR_RESISTOR_CUTOFF_FREQ * PWM_PERIOD)
#define MYACTUATOR_RMDX8H_STATOR_RESISTOR_LPF_ALPHA         (MYACTUATOR_RMDX8H_STATOR_RESISTOR_TIME_CONST / (1.0f + MYACTUATOR_RMDX8H_STATOR_RESISTOR_TIME_CONST))
#define MYACTUATOR_RMDX8H_STATOR_RESISTOR_LPF_ONE_M_ALPHA   (1.0f - MYACTUATOR_RMDX8H_STATOR_RESISTOR_LPF_ALPHA)

/*** Motor 1 constants ***/
#if (MOTOR1_PARAMS == USE_MYACTUATOR_RMDX8H)
#define MOTOR1_KV                                           MYACTUATOR_RMDX8H_KV
#define MOTOR1_KE                                           MYACTUATOR_RMDX8H_KE
#define MOTOR1_KI                                           MYACTUATOR_RMDX8H_KI
#define MOTOR1_RS                                           MYACTUATOR_RMDX8H_RS
#define MOTOR1_LS                                           MYACTUATOR_RMDX8H_LS
#define MOTOR1_POLES_PAIRS                                  MYACTUATOR_RMDX8H_POLES_PAIRS
#define MOTOR1_CURRENT_INIT_MAX                             MYACTUATOR_RMDX8H_CURRENT_INIT_MAX
#define MOTOR1_CURRENT_CMD_SAT_MAX                          MYACTUATOR_RMDX8H_CURRENT_CMD_SAT_MAX

#define MOTOR1_CURRENT_CUTOFF_FREQ                          MYACTUATOR_RMDX8H_CURRENT_CUTOFF_FREQ
#define MOTOR1_CURRENT_TIME_CONST                           MYACTUATOR_RMDX8H_CURRENT_TIME_CONST
#define MOTOR1_CURRENT_LPF_ALPHA                            MYACTUATOR_RMDX8H_CURRENT_LPF_ALPHA
#define MOTOR1_CURRENT_LPF_ONE_M_ALPHA                      MYACTUATOR_RMDX8H_CURRENT_LPF_ONE_M_ALPHA
#define MOTOR1_FIX_INIT_DELAY                               MYACTUATOR_RMDX8H_FIX_INIT_DELAY
#define MOTOR1_FIX_INIT_EVT                                 MYACTUATOR_RMDX8H_FIX_INIT_EVT
#define MOTOR1_CURRENT_INIT_DELAY                           MYACTUATOR_RMDX8H_CURRENT_INIT_DELAY
#define MOTOR1_CURRENT_INIT_EVT                             MYACTUATOR_RMDX8H_CURRENT_INIT_EVT
#define MOTOR1_CURRENT_INIT_STEP                            MYACTUATOR_RMDX8H_CURRENT_INIT_STEP
#define MOTOR1_POSITION_INIT_MAX                            MYACTUATOR_RMDX8H_POSITION_INIT_MAX
#define MOTOR1_POSITION_INIT_DELAY                          MYACTUATOR_RMDX8H_CURRENT_INIT_DELAY
#define MOTOR1_POSITION_INIT_EVT                            MYACTUATOR_RMDX8H_POSITION_INIT_EVT
#define MOTOR1_POSITION_INIT_STEP                           MYACTUATOR_RMDX8H_POSITION_INIT_STEP
#define MOTOR1_POSITION_INIT_ERR_MAX                        MYACTUATOR_RMDX8H_POSITION_INIT_ERR_MAX
#define MOTOR1_POSITION_INIT_ERR_MIN                        MYACTUATOR_RMDX8H_POSITION_INIT_ERR_MIN
#define MOTOR1_POSITION_INIT_ERR                            MYACTUATOR_RMDX8H_POSITION_INIT_ERR
#define MOTOR1_PI_ID_KP_COEF                                MYACTUATOR_RMDX8H_PI_ID_KP_COEF
#define MOTOR1_PI_ID_KI_COEF                                MYACTUATOR_RMDX8H_PI_ID_KI_COEF
#define MOTOR1_PI_IQ_KP_COEF                                MYACTUATOR_RMDX8H_PI_IQ_KP_COEF
#define MOTOR1_PI_IQ_KI_COEF                                MYACTUATOR_RMDX8H_PI_IQ_KI_COEF
#define MOTOR1_DTC_MAX                                      MYACTUATOR_RMDX8H_DTC_MAX
#define MOTOR1_DTC_MIN                                      MYACTUATOR_RMDX8H_DTC_MIN
#define MOTOR1_STATOR_RESISTOR_CUTOFF_FREQ                  MYACTUATOR_RMDX8H_STATOR_RESISTOR_CUTOFF_FREQ
#define MOTOR1_STATOR_RESISTOR_TIME_CONST                   MYACTUATOR_RMDX8H_STATOR_RESISTOR_TIME_CONST
#define MOTOR1_STATOR_RESISTOR_LPF_ALPHA                    MYACTUATOR_RMDX8H_STATOR_RESISTOR_LPF_ALPHA
#define MOTOR1_STATOR_RESISTOR_LPF_ONE_M_ALPHA              MYACTUATOR_RMDX8H_STATOR_RESISTOR_LPF_ONE_M_ALPHA
#endif /* USE_MYACTUATOR_RMDX8H_ON_MOTOR1 */

/*** Motor 2 constants ***/
#if (MOTOR2_PARAMS == USE_MYACTUATOR_RMDX8H)
#define MOTOR2_KV                                           MYACTUATOR_RMDX8H_KV
#define MOTOR2_KE                                           MYACTUATOR_RMDX8H_KE
#define MOTOR2_KI                                           MYACTUATOR_RMDX8H_KI
#define MOTOR2_RS                                           MYACTUATOR_RMDX8H_RS
#define MOTOR2_LS                                           MYACTUATOR_RMDX8H_LS
#define MOTOR2_POLES_PAIRS                                  MYACTUATOR_RMDX8H_POLES_PAIRS
#define MOTOR2_CURRENT_INIT_MAX                             MYACTUATOR_RMDX8H_CURRENT_INIT_MAX
#define MOTOR2_CURRENT_CMD_SAT_MAX                          MYACTUATOR_RMDX8H_CURRENT_CMD_SAT_MAX

#define MOTOR2_CURRENT_CUTOFF_FREQ                          MYACTUATOR_RMDX8H_CURRENT_CUTOFF_FREQ
#define MOTOR2_CURRENT_TIME_CONST                           MYACTUATOR_RMDX8H_CURRENT_TIME_CONST
#define MOTOR2_CURRENT_LPF_ALPHA                            MYACTUATOR_RMDX8H_CURRENT_LPF_ALPHA
#define MOTOR2_CURRENT_LPF_ONE_M_ALPHA                      MYACTUATOR_RMDX8H_CURRENT_LPF_ONE_M_ALPHA
#define MOTOR2_FIX_INIT_DELAY                               MYACTUATOR_RMDX8H_FIX_INIT_DELAY
#define MOTOR2_FIX_INIT_EVT                                 MYACTUATOR_RMDX8H_FIX_INIT_EVT
#define MOTOR2_CURRENT_INIT_DELAY                           MYACTUATOR_RMDX8H_CURRENT_INIT_DELAY
#define MOTOR2_CURRENT_INIT_EVT                             MYACTUATOR_RMDX8H_CURRENT_INIT_EVT
#define MOTOR2_CURRENT_INIT_STEP                            MYACTUATOR_RMDX8H_CURRENT_INIT_STEP
#define MOTOR2_POSITION_INIT_MAX                            MYACTUATOR_RMDX8H_POSITION_INIT_MAX
#define MOTOR2_POSITION_INIT_DELAY                          MYACTUATOR_RMDX8H_CURRENT_INIT_DELAY
#define MOTOR2_POSITION_INIT_EVT                            MYACTUATOR_RMDX8H_POSITION_INIT_EVT
#define MOTOR2_POSITION_INIT_STEP                           MYACTUATOR_RMDX8H_POSITION_INIT_STEP
#define MOTOR2_POSITION_INIT_ERR_MAX                        MYACTUATOR_RMDX8H_POSITION_INIT_ERR_MAX
#define MOTOR2_POSITION_INIT_ERR_MIN                        MYACTUATOR_RMDX8H_POSITION_INIT_ERR_MIN
#define MOTOR2_POSITION_INIT_ERR                            MYACTUATOR_RMDX8H_POSITION_INIT_ERR
#define MOTOR2_PI_ID_KP_COEF                                MYACTUATOR_RMDX8H_PI_ID_KP_COEF
#define MOTOR2_PI_ID_KI_COEF                                MYACTUATOR_RMDX8H_PI_ID_KI_COEF
#define MOTOR2_PI_IQ_KP_COEF                                MYACTUATOR_RMDX8H_PI_IQ_KP_COEF
#define MOTOR2_PI_IQ_KI_COEF                                MYACTUATOR_RMDX8H_PI_IQ_KI_COEF
#define MOTOR2_DTC_MAX                                      MYACTUATOR_RMDX8H_DTC_MAX
#define MOTOR2_DTC_MIN                                      MYACTUATOR_RMDX8H_DTC_MIN
#define MOTOR2_STATOR_RESISTOR_CUTOFF_FREQ                  MYACTUATOR_RMDX8H_STATOR_RESISTOR_CUTOFF_FREQ
#define MOTOR2_STATOR_RESISTOR_TIME_CONST                   MYACTUATOR_RMDX8H_STATOR_RESISTOR_TIME_CONST
#define MOTOR2_STATOR_RESISTOR_LPF_ALPHA                    MYACTUATOR_RMDX8H_STATOR_RESISTOR_LPF_ALPHA
#define MOTOR2_STATOR_RESISTOR_LPF_ONE_M_ALPHA              MYACTUATOR_RMDX8H_STATOR_RESISTOR_LPF_ONE_M_ALPHA
#endif /* USE_MYACTUATOR_RMDX8H_ON_MOTOR2 */

#endif /* __MYACTUATOR_RMDX8H_DEFINES_H__ */