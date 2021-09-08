#ifndef __CONUNICATION_INTERFACE_H__
#define __CONUNICATION_INTERFACE_H__

#include <stdint.h>
#include <math.h>
//#include "math_defines.h"
#include "legacy_spi.h"

//-------------------------------------------------------------------------------//
//                               LSB DEFINITIONS                                 //
//-------------------------------------------------------------------------------//
/*
 * These values are extracted from the Open-Dynamic-Robot-Initiative Github project
 * Documentation:
 * https://github.com/open-dynamic-robot-initiative/master-board/blob/master/documentation/BLMC_%C2%B5Driver_SPI_interface.md
 */

#define CI_POSITION_LSB         5.960464477539063e-08                   // 2**(-24)
#define CI_VELOCITY_LSB         0.00048828125 * 1000.0 * FM_RPM2RADPS   // 2**(-11) // Added rad/s to krpm conversion
#define CI_IQ_LSB               0.0009765625                            // 2**(-10)
#define CI_CURRENT_SAT_LSB      0.125                                   // 2**(-3)
#define CI_LSBISTANCE_LSB       3.0517578125e-05                        // 2**(-15)
#define CI_VOLTAGE_LSB          6.103515625e-05                         // 2**(-14)
#define CI_KP_LSB               0.00048828125                           // 2**(-11)
#define CI_KD_LSB               0.0009765625                            // 2**(-10)

//-------------------------------------------------------------------------------//
//                             STRUCTURE DEFINITION                              //
//-------------------------------------------------------------------------------//

// Structure that contains useful variables for master communication
typedef struct __ComInterface__
{
    // Enabled system flag
    bool systemEnabled;

    // Last received command index
    uint16_t lastCommandIndex;

    // Time variables
    float32_t time;             // Time in seconds since startup
    uint64_t lastMessageMs;     // Last time stamp (milliseconds) at which a message has been received
    uint16_t timeoutThreshold;  // Duration in milliseconds before triggering timeout

    // Position roll-over error flags
    bool rolloverErrorFlag;

    // CAN error
    uint16_t canError;

    // Controller references
    // Position references
    float32_t positionRef[2];

    // Velocity references
    float32_t velocityRef[2];

    // Quadrature current references
    float32_t iqRef[2];

    // Data resolutions
    const float32_t positionLsb;
    const float32_t velocityLsb;
    const float32_t iqLsb;
    const float32_t currentSatLsb;
    const float32_t resistanceLsb;
    const float32_t voltageLsb;
    const float32_t kpLsb;
    const float32_t kdLsb;

} ComInterface;

// Initialization structure
#define DEFAULT_COM_INTERFACE       { 0, 0, 0, 0, 0xFFFF, 0, SPI_SENSOR_STATUS_ERROR_NO_ERROR, 0, 0, 0, 0, 0, 0, \
                                    CI_POSITION_LSB, CI_VELOCITY_LSB, CI_IQ_LSB, CI_CURRENT_SAT_LSB, CI_LSBISTANCE_LSB, CI_VOLTAGE_LSB, CI_KP_LSB, CI_KD_LSB }

#endif
