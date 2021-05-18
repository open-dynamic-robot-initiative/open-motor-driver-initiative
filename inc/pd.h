#ifndef __PD_H__
#define __PD_H__

typedef struct __PD_controller__
{
    float kp;   // Proportional gain
    float kd;   // Derivative gain

    float err;  // Position error
    float vErr; // Velocity error

    float ff;   // Feed-forward
    float out;  // Controller output
    float sat;  // Controller saturation
} PD_controller;

// PD controller initialization
static inline void PD_reset(PD_controller *pd)
{
    pd->err = 0.0f;
    pd->vErr = 0.0f;
    pd->out = 0.0f;
}

// PD controller execution
static inline void PD_run(PD_controller *pd)
{
    pd->out = pd->kp * pd->err + pd->kd * pd->vErr + pd->ff;
    pd->out = __fmax(__fmin(pd->out, pd->sat), -pd->sat);
}

#endif
