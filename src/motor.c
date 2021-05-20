/*
 * motor.c
 *
 *  Created on: 20 mai 2021
 *      Author: jmanhes
 */

#include "motor.h"

static void LPF_setBandwidth_simple(FILTER_PARAMS* p_flt, float32_t freq, float32_t bw)
{
    p_flt->a = 1.0 / (1.0 + 2.0 * M_PI * bw / freq);
    p_flt->b = 1.0 - p_flt->a;
}

#define LPF_FILTER(filt, x, y)     (x) = filt.b * (y) + filt.a * (x);



