#ifndef __ANTICOAGGING_H__
#define __ANTICOAGGING_H__

//**********************************************************************************************//
//                                    SCRIPT GENERATED DATA                                     //
//**********************************************************************************************//

// MOTOR 1
#define M1_ACOG_ENABLE        0 // Set this to 1 to use anticoagging for motor 1
#define M1_ACOG_ARRAY_LEN     2

static const float m1_acogFrequencies[M1_ACOG_ARRAY_LEN] = {
 72,
 18,
};


static const float m1_acogCoeffs[2 * M1_ACOG_ARRAY_LEN] = {
 0.0, 0.0,
 0.0, 0.0,
};

// MOTOR 2
#define M2_ACOG_ENABLE        0 // Set this to 1 to use anticoagging for motor 2
#define M2_ACOG_ARRAY_LEN     2

static const float m2_acogFrequencies[M2_ACOG_ARRAY_LEN] = {
 72,
 18,
};


static const float m2_acogCoeffs[2 * M2_ACOG_ARRAY_LEN] = {
 0.15314338134673652, -0.052750250737860924,
 0.0378368985046421, -0.07095689846939295,
};

//**********************************************************************************************//
//                                   ANTICOAGGING STRUCTURE                                     //
//**********************************************************************************************//

typedef struct __Anticoagging__
{
    // Enabled flag
    bool_t enabled;

    // Number of coefficients
    uint16_t coeffNb;

    // Frequency array
    const float *frequencies;
    // Fourier coefficients array
    const float *coeffs;
} Anticoagging;


//**********************************************************************************************//
//                                  ANTICOAGGING FUNCTIONS                                      //
//**********************************************************************************************//

// Anti-coagging structure initialization
// Set the corresponding arrays for each motor
static void ACOG_init(Anticoagging* acog, const uint16_t motorIndex)
{
    // Enable anticoagging
    acog->enabled = true;

    // Copy frequencies and coefficients
    if (motorIndex == 0)
    {
        acog->coeffNb = M1_ACOG_ARRAY_LEN;
        acog->frequencies = m1_acogFrequencies;
        acog->coeffs = m1_acogCoeffs;
    }
    else // motorIndex == 1
    {
        acog->coeffNb = M2_ACOG_ARRAY_LEN;
        acog->frequencies = m2_acogFrequencies;
        acog->coeffs = m2_acogCoeffs;
    }
}

// Inverse Fourier transform to compute the corresponding coagging current
// The angle used must be indexed so the configuration is the same than
// during the measurements
static inline float ACOG_getAcogCurrent(Anticoagging* acog, float position)
{
    uint16_t freqIndex;
    float acogCurrent = 0.0f;

    for(freqIndex = 0; freqIndex < acog->coeffNb; freqIndex++)
    {
        acogCurrent += acog->coeffs[2*freqIndex  ] * __cospuf32(acog->frequencies[freqIndex] * position)
                      -acog->coeffs[2*freqIndex+1] * __sinpuf32(acog->frequencies[freqIndex] * position);
    }

    return acogCurrent;
}

#endif
