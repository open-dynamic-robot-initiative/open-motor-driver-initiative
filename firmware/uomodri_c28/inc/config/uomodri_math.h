/*
 * File name: omodri_math.h
 * Description: Header file containing specific constants for uOmodri project
 */

#ifndef __UOMODRI_MATH_H__
#define __UOMODRI_MATH_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "../uomodri_user_defines.h"
#ifdef __TMS320C28XX_CLA__
#include "CLAmath.h"
#else
#include "math.h"
#endif

/***********************************************************************
 * GLOBAL MACROS
 ***********************************************************************/
/** @defgroup CONSTANT_defines
 *  @brief    General constants definitions.
 * @{
 */
#ifndef FM_SQRT3
#define FM_SQRT3                        (1.7320508075688772935274463415059f)  /* sqrt(3) */
#endif

#ifndef FM_1DIVSQRT3
#define FM_1DIVSQRT3                    (0.57735026918962576450914878050196f) /* 1/sqrt(3) */
#endif

#ifndef FM_SQRT3DIV2
#define FM_SQRT3DIV2                    (0.86602540378443864676372317075294f) /* sqrt(3)/2 */
#endif

#ifndef FM_1DIV3
#define FM_1DIV3                        (0.33333333333333333333333333333333f) /* 1/3 */
#endif

#ifndef FM_PI
#define FM_PI                           (3.1415926535897932384626433832795f) /* PI */
#endif

#ifndef FM_2MULTPI
#define FM_2MULTPI                      (6.2831853071795864769252867665590f) /* 2xPI */
#endif

#ifndef FM_2MULTPIDIV10
#define FM_2MULTPIDIV10                 (0.6283185307179586476925286766559f) /* 2xPI/10 */
#endif

#ifndef FM_ROUND2RAD
#define FM_ROUND2RAD                    FM_2MULTPI
#endif

#ifndef FM_RPM2RADPS
#define FM_RPM2RADPS                    ((float32_t)(FM_ROUND2RAD / 60.0f))
#endif

#ifndef FM_RADPS2RPM
#define FM_RADPS2RPM                    ((float32_t)(1.0f / FM_RPM2RADPS))
#endif

#ifndef FM_RAD2ROUND
#define FM_RAD2ROUND                    ((float32_t)(1.0f / FM_ROUND2RAD))
#endif

#ifndef FM_KRPM2RADPS
#define FM_KRPM2RADPS                   ((float32_t)(1000.0f * FM_RPM2RADPS))
#endif

#ifndef FM_RADPS2KRPM
#define FM_RADPS2KRPM                   ((float32_t)(1.0f / FM_KRPM2RADPS))
#endif

#ifndef FM_ABS
#define FM_ABS(x)                       ((x) >= 0.0f ? (x) : -(x))
#endif

//#define HWSREGH(x)                              (*((volatile int16_t *)((uintptr_t)(x))))
/* Initialize control loop low-pass filter
   alpha = 1 / (1 + (1 / (2 * PI * CUTTOFF_FREQ / PWM_FREQ)))
   Simplifying :
   alpha = 1 - (1 / (1 + (2 * PI * CUTTOFF_FREQ / PWM_FREQ)))
   1 - alpha = 1 / (1 + (2 * PI * CUTTOFF_FREQ / PWM_FREQ)) */
#ifndef LPF_FILTER_1
#define LPF_FILTER_1(flt_coef, y, x)    (y) = ((y) + (flt_coef.a * ((x) - (y))))
#endif

#ifndef LPF_FILTER_2
#define LPF_FILTER_2(flt_coef, y, x)    (y) = ((flt_coef.a * (x)) + (flt_coef.one_minus_a * (y)))
//#define LPF_FILTER_2(flt_coef, y, x)            (y) = ((flt_coef.a * (x)) + ((1.0f - flt_coef.a) * (y)))
#endif

#ifndef LPF_FILTER_3
#define LPF_FILTER_3(flt_coef, y, x)    (y) = (x)
#endif

/* (((x) > (y)) ? (((x) > (z)) ? (x) : (z)) : (((y) > (z)) ? (y) : (z))) */
#ifndef FMAX3
#define FMAX3(x, y, z)                  (__fmax(__fmax((x), (y)), (z)))
#endif

/* (((x) < (y)) ? (((x) < (z)) ? (x) : (z)) : (((y) < (z)) ? (y) : (z))) */
#ifndef FMIN3
#define FMIN3(x, y, z)                  (__fmin(__fmin((x), (y)), (z)))
#endif

#ifndef FMINMAX
#define FMINMAX(x, y, z)                (__fmax(__fmin((x), (y)), (z)))
#endif

// 16 bits macros
#define LSB_16(x)                       ((x) & 0xFF)
#define MSB_16(x)                       (((x) >> 8) & 0xFF)
#define FUS_16_TO_32(x, y)              (((uint32_t)(x) << 16) | (uint32_t)(y))
//#define SEP_32_TO_16(x, y)
// 32 bits macros
#define LSB_32(x)                       ((x) & 0xFFFF)
#define MSB_32(x)                       (((x) >> 16) & 0xFFFF)
#define LSB_64(x)                       ((x) & 0xFFFFFFFF)
#define MSB_64(x)                       (((x) >> 32) & 0xFFFFFFFF)


#define SWAP(x,y)                       do { \
                                            typeof(x) _x = x; \
                                            typeof(y) _y = y; \
                                            x = _y; \
                                            y = _x; \
                                           } while(0)

#endif /* __UOMODRI_MATH_H__ */
