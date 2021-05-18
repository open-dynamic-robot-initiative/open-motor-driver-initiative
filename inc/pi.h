#ifndef __PI_CONTROLLER_H__
#define __PI_CONTROLLER_H__

typedef struct __PI_controller__
{
    float kp;
    float ki;
    float sat;

    float integral;

    float err;
    float ff;
    float out;
} PI_controller;

static inline void PI_reset(PI_controller *pi)
{
    pi->integral = 0.0f;
    pi->err = 0.0f;
    pi->ff = 0.0f;
    pi->out = 0.0f;
}

static inline void PI_run(PI_controller *pi)
{
    pi->integral += pi->ki * pi->err;

    pi->out = pi->kp * pi->err + pi->integral + pi->ff;
    pi->out = __fmin(__fmax(pi->out, -pi->sat), pi->sat);
}

#endif
