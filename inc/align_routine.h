#ifndef __ALIGN_ROUTINE_H__
#define __ALIGN_ROUTINE_H__

typedef enum
{
    ALIGN_COUNT_UP,
    ALIGN_STAY,
    ALIGN_COUNT_DOWN,
    ALIGN_DONE
} AlignState;

typedef struct __AlignRoutine__
{
    AlignState state;

    uint32_t counter;
    uint32_t max;

    float32_t idAlignRef;
    float32_t idAlign;
} AlignRoutine;

static void ALIGN_reset(AlignRoutine *align)
{
    align->state = ALIGN_COUNT_UP;
    align->counter = 0;
    align->idAlign = 0.0f;
}

// Return true if calibration is finished
static void ALIGN_run(AlignRoutine *align)
{
    switch (align->state)
    {
    case ALIGN_COUNT_UP:
        // Increment alignment current
        align->idAlign += 1e-5;

        if(align->idAlign >= align->idAlignRef)
        {
            align->state = ALIGN_STAY;
        }
        break;

    case ALIGN_STAY:
        // Maintain alignment current
        if(++align->counter >= align->max)
        {
            align->state = ALIGN_COUNT_DOWN;
        }
        break;

    case ALIGN_COUNT_DOWN:
        // Decrement alignment current
        align->idAlign -= 1e-5;

        if(align->idAlign <= 0.0f)
        {
            align->counter = 0;
            align->idAlign = 0.0f;
            align->state = ALIGN_DONE;
        }
        break;
    }
}

#endif
