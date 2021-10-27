#ifndef __LEGACY_SPI_H__
#define __LEGACY_SPI_H__

#include "uomodri_cm_user_defines.h"

//**********************************************************************************************//
//                                        INCLUDES                                              //
//**********************************************************************************************//

//#include "lsb_msb_macros.h"
//#include "foc.h"
//#include "motor.h"
//#include "uomodri_user_defines.h"
//#include "uomodri_prj_config_handlers.h"

//**********************************************************************************************//
//                                       ERROR FLAGS                                            //
//**********************************************************************************************//
// No error
#define SPI_SENSOR_STATUS_ERROR_NO_ERROR            0
// Encoder1 error too high
#define SPI_SENSOR_STATUS_ERROR_ENCODER1            1
// Timeout for receiving current references exceeded
#define SPI_SENSOR_STATUS_ERROR_SPI_RECV_TIMEOUT    2
// Motor temperature reached critical value // TODO implement
#define SPI_SENSOR_STATUS_ERROR_CRIT_TEMP           3
// Unused error
#define SPI_SENSOR_STATUS_ERROR_POSCONV             4
// Position roll-over occurred
#define SPI_SENSOR_STATUS_ERROR_POS_ROLLOVER        5
// Encoder2 error too high
#define SPI_SENSOR_STATUS_ERROR_ENCODER2            6
// DRV nFault error
#define SPI_SENSOR_STATUS_ERROR_DRV_NFAULT          7

//**********************************************************************************************//
//                                   FUNCTION SIGNATURES                                        //
//**********************************************************************************************//

#if UOMODRI_SPI_DEBUG_CONFIG
// Updates the sensor packet
void UpdateLogPacket();

#else

// Updated sensor packet for SPI communication
void UpdateSensorPacket();
// Read command packet for SPI communication
void ReadCommandPacket();

#endif

#endif
