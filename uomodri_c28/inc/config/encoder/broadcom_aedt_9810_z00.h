/*
 * File name: broadcom_aedt_9810_z00.h
 * Description: Header file containing definitions of AEDT-9810-Z00 encoder from BROADCOM.
 */

#ifndef __BROADCOM_AEDT_9810_Z00_DEFINES_H__
#define __BROADCOM_AEDT_9810_Z00_DEFINES_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
//#include "driverlib.h"
//#include "device.h"
//#include "../uomodri_math.h"
#include "../../uomodri_user_defines.h"
//#if (USE_UOMODRI_REV == USE_UOMODRI_V1)
////
//#elif (USE_UOMODRI_REV == USE_UOMODRI_V2)
////
//#elif (USE_UOMODRI_REV == USE_UOMODRI_V3)
//#include "config/uomodri_3_1/uomodri_3_1_hal_defines.h" // uOmodri v3.1 definitions
//#include "config/uomodri_3_1/uomodri_3_1_handlers.h"    // uOmodri v3.1 configuration include file
//#elif (USE_UOMODRI_REV == USE_UOMODRI_SA_V1)
////
//#else
//#error "@ least one config must be set @ a time."
//#endif

/***********************************************************************
 * ENCODERS DEFINES
 ***********************************************************************/
#define BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION                   (5000.0f)
#define BROADCOM_AEDT_9810_Z00_ENC_QUADRATURE_SCALE             (4.0f * BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION)
#define BROADCOM_AEDT_9810_Z00_ENC_QUADRATURE_SCALE_MAX         (BROADCOM_AEDT_9810_Z00_ENC_QUADRATURE_SCALE - 1)
#define BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION_SCALE             (1.0f / BROADCOM_AEDT_9810_Z00_ENC_QUADRATURE_SCALE)
#define BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION_RAD               (FM_ROUND2RAD * BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION_SCALE)
#define BROADCOM_AEDT_9810_Z00_ENC_CONFIG                       (EQEP_CONFIG_2X_RESOLUTION | \
                                                                 EQEP_CONFIG_QUADRATURE | \
                                                                 EQEP_CONFIG_IGATE_DISABLE)
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_FREQ       (1000U) // [Hz] Velocity computation frequency
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_TICKS      (DEVICE_SYSCLK_FREQ / \
                                                                 BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_FREQ)
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_TICKS_MAX  (BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_TICKS - 1)
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SCALE             (BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_FREQ) // 2pi / dt
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_LOW_SCALE              (2.0f * UOMODRI_PI * DEVICE_SYSCLK_FREQ * \
                                                                 (1 << QEP_UNIT_POS_EVENT_DIV) / \
                                                                 (1 << (QEP_CAPTURE_CLOCK_DIV >> 4)) / \
                                                                 BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION_SCALE)
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_CUTOFF_FREQ            (100.0f) // cutoff frequency for velocity estimation
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_TIME_CONST             (2.0f * UOMODRI_PI * \
                                                                 BROADCOM_AEDT_9810_Z00_ENC_SPEED_CUTOFF_FREQ * \
                                                                 PWM_PERIOD)
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_LPF_ALPHA              (BROADCOM_AEDT_9810_Z00_ENC_SPEED_TIME_CONST / \
                                                                 (1.0f + BROADCOM_AEDT_9810_Z00_ENC_SPEED_TIME_CONST))
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_LPF_ONE_M_ALPHA        (1.0f - BROADCOM_AEDT_9810_Z00_ENC_SPEED_LPF_ALPHA)
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_LOCAL_LPF_ALPHA        (BROADCOM_AEDT_9810_Z00_ENC_SPEED_TIME_CONST / \
                                                                 (1.0f + BROADCOM_AEDT_9810_Z00_ENC_SPEED_TIME_CONST))
#define BROADCOM_AEDT_9810_Z00_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA  (1.0f - BROADCOM_AEDT_9810_Z00_ENC_SPEED_LPF_ALPHA)

/*** Encoder 1 constants ***/
#if (ENCODER1_PARAMS == USE_BROADCOM_AEDT_9810_Z00)
#define MOTOR1_ENC_RESOLUTION                                   BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION
#define MOTOR1_ENC_QUADRATURE_SCALE                             BROADCOM_AEDT_9810_Z00_ENC_QUADRATURE_SCALE
#define MOTOR1_ENC_QUADRATURE_SCALE_MAX                         BROADCOM_AEDT_9810_Z00_ENC_QUADRATURE_SCALE_MAX
#define MOTOR1_ENC_RESOLUTION_SCALE                             BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION_SCALE
#define MOTOR1_ENC_RESOLUTION_RAD                               BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION_RAD
#define MOTOR1_ENC_CONFIG                                       BROADCOM_AEDT_9810_Z00_ENC_CONFIG
#define MOTOR1_ENC_SPEED_HIGH_SAMPLE_FREQ                       BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_FREQ
#define MOTOR1_ENC_SPEED_HIGH_SAMPLE_TICKS                      BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_TICKS
#define MOTOR1_ENC_SPEED_HIGH_SAMPLE_TICKS_MAX                  BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_TICKS_MAX
#define MOTOR1_ENC_SPEED_HIGH_SCALE                             BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SCALE
#define MOTOR1_ENC_SPEED_LOW_SCALE                              BROADCOM_AEDT_9810_Z00_ENC_SPEED_LOW_SCALE
#define MOTOR1_ENC_SPEED_CUTOFF_FREQ                            BROADCOM_AEDT_9810_Z00_ENC_SPEED_CUTOFF_FREQ
#define MOTOR1_ENC_SPEED_TIME_CONST                             BROADCOM_AEDT_9810_Z00_ENC_SPEED_TIME_CONST
#define MOTOR1_ENC_SPEED_LPF_ALPHA                              BROADCOM_AEDT_9810_Z00_ENC_SPEED_LPF_ALPHA
#define MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA                        BROADCOM_AEDT_9810_Z00_ENC_SPEED_LPF_ONE_M_ALPHA
#define MOTOR1_ENC_SPEED_LOCAL_LPF_ALPHA                        BROADCOM_AEDT_9810_Z00_ENC_SPEED_LOCAL_LPF_ALPHA
#define MOTOR1_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA                  BROADCOM_AEDT_9810_Z00_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA
#endif /* USE_BROADCOM_AEDT_9810_Z00_ON_MOTOR1 */
/*** Encoder 2 constants ***/
#if (ENCODER2_PARAMS == USE_BROADCOM_AEDT_9810_Z00)
#define MOTOR2_ENC_RESOLUTION                                   BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION
#define MOTOR2_ENC_QUADRATURE_SCALE                             BROADCOM_AEDT_9810_Z00_ENC_QUADRATURE_SCALE
#define MOTOR2_ENC_QUADRATURE_SCALE_MAX                         BROADCOM_AEDT_9810_Z00_ENC_QUADRATURE_SCALE_MAX
#define MOTOR2_ENC_RESOLUTION_SCALE                             BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION_SCALE
#define MOTOR2_ENC_RESOLUTION_RAD                               BROADCOM_AEDT_9810_Z00_ENC_RESOLUTION_RAD
#define MOTOR2_ENC_CONFIG                                       BROADCOM_AEDT_9810_Z00_ENC_CONFIG
#define MOTOR2_ENC_SPEED_HIGH_SAMPLE_FREQ                       BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_FREQ
#define MOTOR2_ENC_SPEED_HIGH_SAMPLE_TICKS                      BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_TICKS
#define MOTOR2_ENC_SPEED_HIGH_SAMPLE_TICKS_MAX                  BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SAMPLE_TICKS_MAX
#define MOTOR2_ENC_SPEED_HIGH_SCALE                             BROADCOM_AEDT_9810_Z00_ENC_SPEED_HIGH_SCALE
#define MOTOR2_ENC_SPEED_LOW_SCALE                              BROADCOM_AEDT_9810_Z00_ENC_SPEED_LOW_SCALE
#define MOTOR2_ENC_SPEED_CUTOFF_FREQ                            BROADCOM_AEDT_9810_Z00_ENC_SPEED_CUTOFF_FREQ
#define MOTOR2_ENC_SPEED_TIME_CONST                             BROADCOM_AEDT_9810_Z00_ENC_SPEED_TIME_CONST
#define MOTOR2_ENC_SPEED_LPF_ALPHA                              BROADCOM_AEDT_9810_Z00_ENC_SPEED_LPF_ALPHA
#define MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA                        BROADCOM_AEDT_9810_Z00_ENC_SPEED_LPF_ONE_M_ALPHA
#define MOTOR2_ENC_SPEED_LOCAL_LPF_ALPHA                        BROADCOM_AEDT_9810_Z00_ENC_SPEED_LOCAL_LPF_ALPHA
#define MOTOR2_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA                  BROADCOM_AEDT_9810_Z00_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA
#endif /* USE_BROADCOM_AEDT_9810_Z00_ON_MOTOR2 */

#endif /* __BROADCOM_AEDT_9810_Z00_DEFINES_H__ */
