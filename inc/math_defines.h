// This file contains float math defines

#ifndef __MATH_DEFINES_H__
#define __MATH_DEFINES_H__

#ifndef FM_PI
#define FM_PI 3.14159265358979f
#endif

#ifndef FM_SQRT3
#define FM_SQRT3 1.7320508075689f
#endif

#ifndef FM_ISQRT3
#define FM_ISQRT3 0.57735026919f
#endif

#ifndef FM_RPM2RADPS
#define FM_RPM2RADPS (2.0 * FM_PI / 60.0)
#endif

#define FM_ABS(x)   ((x) >= 0.0f ? (x) : -(x))

#endif
