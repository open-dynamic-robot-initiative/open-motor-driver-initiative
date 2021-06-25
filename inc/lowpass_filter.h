#ifndef __LOWPASS_FILTER_H__
#define __LOWPASS_FILTER_H__

#include <math.h>
//#include "math_defines.h"

typedef struct __LowpassFilter__
{
    float b;
    float a;
} LowpassFilter;

// freq: Frequency in Hertz at which the filter will be executed
// bw:   Filter bandwidth in Hertz
static void LPF_setBandwidth(LowpassFilter *lpf, float freq, float bw)
{
    lpf->a = 1.0 / (1.0 + 2.0 * M_PI * bw / freq);
    lpf->b = 1.0 - lpf->a;
}

#define LPF_FILTER(filt, y, x)     (y) = filt.b * (x) + filt.a * (y);

#endif
