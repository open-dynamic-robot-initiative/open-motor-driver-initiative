#include "legacy_spi.h"
#include "motor.h"
#include "foc.h"
#include "spi_communication.h"
//#include "communication_interface.h"
#include "uomodri_user_defines.h"
#include "uomodri_prj_config_handlers.h"

//**********************************************************************************************//
//                                     EXTERN DECLARATIONS                                      //
//**********************************************************************************************//

//**********************************************************************************************//
//                                    FUNCTION DEFINITIONS                                      //
//**********************************************************************************************//

#if UOMODRI_SPI_DEBUG_CONFIG

// Choose which motor is logged
#define DEBUG_LOGGED_MOTOR      0

// Used for real-time logging when the master board isn't connected
void UpdateLogPacket()
{
    // Get available packet
    ComPacketW* comPacket = SPI_getAvailableWPacket();

    // Update fields
    uint32_t timestamp = motor[DEBUG_LOGGED_MOTOR].alignRoutine.state == ALIGN_DONE ? (uint32_t)(comInterface.time * 10000.0f) : 0;
    //uint32_t timestamp = TIME_S * 10000.0f;
    comPacket->timestamp_msb = MSB_32(timestamp);
    comPacket->timestamp_lsb = LSB_32(timestamp);

    comPacket->idRef = (int16_t)(motor[DEBUG_LOGGED_MOTOR].motorFOC->idRef);// / comInterface.iqResolution);
    comPacket->idFbk = (int16_t)(motor[DEBUG_LOGGED_MOTOR].motorFOC->id);// / comInterface.iqResolution);
    comPacket->iqRef = (int16_t)(motor[DEBUG_LOGGED_MOTOR].motorFOC->iqRef);// / comInterface.iqResolution);
    comPacket->iqFbk = (int16_t)(motor[DEBUG_LOGGED_MOTOR].motorFOC->iq);// / comInterface.iqResolution);

    int32_t position = (int32_t)(motor[DEBUG_LOGGED_MOTOR].motorFOC->motor_enc.thetaAbsolute);// / comInterface.positionResolution);
    comPacket->position_msb = MSB_32(position);
    comPacket->position_lsb = LSB_32(position);

    int32_t refPosition = (int32_t)(comInterface.positionRef[DEBUG_LOGGED_MOTOR]);// / comInterface.positionResolution);
    comPacket->refPos_msb = MSB_32(refPosition);
    comPacket->refPos_lsb = LSB_32(refPosition);

    comPacket->velocity = (int16_t)(motor[DEBUG_LOGGED_MOTOR].motorFOC->motor_enc.speedEst.speedMech);// / comInterface.velocityResolution);

    comPacket->refVel = (int16_t)(comInterface.velocityRef[DEBUG_LOGGED_MOTOR]);// / comInterface.velocityResolution);

    comPacket->ud = (int16_t)(motor[DEBUG_LOGGED_MOTOR].motorFOC->ud);// / comInterface.iqResolution);
    comPacket->uq = (int16_t)(motor[DEBUG_LOGGED_MOTOR].motorFOC->uq);// / comInterface.iqResolution);

    comPacket->resistance = (uint16_t)(motor[DEBUG_LOGGED_MOTOR].motorFOC->resistanceEst); // / comInterface.resistanceResolution);

    // Push packet
    SPI_pushWPacket();
}

#else
// Function signatures
static void timeoutRoutine();

// Update communication packet
void UpdateSensorPacket()
{
    // Get available packet
    ComPacketW* p_sensorPacket  = SPI_getAvailableWPacket();
    FOC*        p_foc_m1        = motor[MOTOR_1].motorFOC;
    FOC*        p_foc_m2        = motor[MOTOR_2].motorFOC;

    // Update status field
    // System enabled
    p_sensorPacket->status.bit.SE       = comInterface.systemEnabled;
    // Motor 1 enabled
    p_sensorPacket->status.bit.M1E      = motor[MOTOR_1].enabled;
    // Motor 1 ready
    p_sensorPacket->status.bit.M1R      = motor[MOTOR_1].alignRoutine.state == ALIGN_DONE;
    // Motor 1 index toggle
    p_sensorPacket->status.bit.IDX1T    = p_foc_m1->motor_enc.indexToggle;
    // Motor 1 index detected
    p_sensorPacket->status.bit.IDX1D    = p_foc_m1->motor_enc.indexDetected;
    // Motor 2 enabled
    p_sensorPacket->status.bit.M2E      = motor[MOTOR_2].enabled;
    // Motor 2 ready
    p_sensorPacket->status.bit.M2R      = motor[MOTOR_2].alignRoutine.state == ALIGN_DONE;
    // Motor 2 index toggle
    p_sensorPacket->status.bit.IDX2T    = p_foc_m2->motor_enc.indexToggle;
    // Motor 2 index detected
    p_sensorPacket->status.bit.IDX2D    = p_foc_m2->motor_enc.indexDetected;
    // TODO Implement CAN errors
    p_sensorPacket->status.bit.EC       = comInterface.canError;
    // Update time stamp in milliseconds
    p_sensorPacket->timeStamp           = (uint16_t)(comInterface.time * 1000.0f);

    // Update motor 1 position
    // convert position to signed value
    int32_t position                    = (int32_t)(p_foc_m1->motor_enc.thetaAbsolute / comInterface.positionLsb);
    p_sensorPacket->position1_msb       = MSB_32(position);
    p_sensorPacket->position1_lsb       = LSB_32(position);
    // Update motor 2 position
    position                            = (int32_t)(p_foc_m2->motor_enc.thetaAbsolute / comInterface.positionLsb);
    p_sensorPacket->position2_msb       = MSB_32(position);
    p_sensorPacket->position2_lsb       = LSB_32(position);
    // Update motor velocities
    p_sensorPacket->velocity1           = (int16_t)(p_foc_m1->motor_enc.speedEst.speedMech / comInterface.velocityLsb);
    p_sensorPacket->velocity2           = (int16_t)(p_foc_m2->motor_enc.speedEst.speedMech / comInterface.velocityLsb);
    // Update quadrature currents
    p_sensorPacket->iq1                 = (int16_t)(p_foc_m1->iq / comInterface.iqLsb);
    p_sensorPacket->iq2                 = (int16_t)(p_foc_m2->iq / comInterface.iqLsb);
    // Update coil resistances
    p_sensorPacket->coilResistance1     = (uint16_t)(p_foc_m1->resistanceEst / comInterface.resistanceLsb);
    p_sensorPacket->coilResistance2     = (uint16_t)(p_foc_m2->resistanceEst / comInterface.resistanceLsb);
    // Update potentiometer voltages
    p_sensorPacket->adc1                = (uint16_t)(p_foc_m1->auxAdc / comInterface.voltageLsb);
    p_sensorPacket->adc2                = (uint16_t)(p_foc_m2->auxAdc / comInterface.voltageLsb);
    // Update last received command index
    p_sensorPacket->lastCmdIndex        = comInterface.lastCommandIndex;
    // Push packet
    SPI_pushWPacket();
}


void ReadCommandPacket()
{
    // Get read packet
    ComPacketRPtr   commandPacket   = SPI_getAvailableRPacket();
    FOC*            p_foc_m1        = motor[MOTOR_1].motorFOC;
    FOC*            p_foc_m2        = motor[MOTOR_2].motorFOC;

//-------------------- TIMEOUT CHECK -------------------//
    // If error flag is set, check for timeout
    if (commandPacket == RPACKET_ERROR)
    {
        // Timeout check
        if (comInterface.systemEnabled && ((uint64_t)(comInterface.time * 1000.0f) - comInterface.lastMessageMs >= comInterface.timeoutThreshold))
        {
            // Trigger timeout infinite loop
            timeoutRoutine();
        }

        // Don't read packet
        return;
    }
    else
    {
        // Update last read packet time stamp
        comInterface.lastMessageMs = (uint64_t)(comInterface.time * 1000.0f);
    }

//-------------------- PACKET READING -------------------//

    // Enable system
    if (commandPacket->mode.bit.ES != comInterface.systemEnabled)
    {
        if (commandPacket->mode.bit.ES)
        {
            // System enable
            FOC_resetStruct(p_foc_m1);
            FOC_resetStruct(p_foc_m2);
            comInterface.systemEnabled = 1;
        }
        else
        {
            // System disable
            comInterface.systemEnabled = 0;
        }
    }

    // Enable motor 1
    p_foc_m1->enabled = commandPacket->mode.bit.EM1;

    // Enable motor 2
    p_foc_m2->enabled = commandPacket->mode.bit.EM2;

    // Position roll-over errors
    comInterface.rolloverErrorFlag = commandPacket->mode.bit.EPRE;

    // Index offset compensation
    p_foc_m1->motor_enc.indexCompensated = commandPacket->mode.bit.EI1OC;
    p_foc_m2->motor_enc.indexCompensated = commandPacket->mode.bit.EI2OC;

    // Update timeout
    comInterface.timeoutThreshold = commandPacket->mode.bit.TIMEOUT;

    // Update position references
    comInterface.positionRef[MOTOR_1] = comInterface.positionLsb * (int32_t)(FUS_16_TO_32(commandPacket->refPos1_msb, commandPacket->refPos1_lsb));
    comInterface.positionRef[MOTOR_2] = comInterface.positionLsb * (int32_t)(FUS_16_TO_32(commandPacket->refPos2_msb, commandPacket->refPos2_lsb));

    // Update velocity references
    comInterface.velocityRef[MOTOR_1] = comInterface.velocityLsb * (int16_t)commandPacket->refVel1;
    comInterface.velocityRef[MOTOR_2] = comInterface.velocityLsb * (int16_t)commandPacket->refVel2;

    // Update current references
    comInterface.iqRef[MOTOR_1] = comInterface.iqLsb * (int16_t)commandPacket->refIq1;
    comInterface.iqRef[MOTOR_2] = comInterface.iqLsb * (int16_t)commandPacket->refIq2;

    // Update Kp and Kd
    p_foc_m1->pdPos.kp = comInterface.kpLsb * commandPacket->Kp1;
    p_foc_m1->pdPos.kd = comInterface.kdLsb * commandPacket->Kd1;

    p_foc_m2->pdPos.kp = comInterface.kpLsb * commandPacket->Kp2;
    p_foc_m2->pdPos.kd = comInterface.kdLsb * commandPacket->Kd2;

    // Update current saturations // TODO Unused for the moment
    //p_foc_m1.posPd.sat = comInterface.currentSatLsb * MSB_16(commandPacket->iSat12);
    //motor_foc[MOTOR_2].posPd.sat = comInterface.currentSatLsb * LSB_16(commandPacket->iSat12);

    // Update last received index
    comInterface.lastCommandIndex = commandPacket->index;
}

// Function called if a timeout happened
static void timeoutRoutine()
{
    // Trigger emergency stop
    MOTOR_emergencyStop();//FOC_emergencyStop();

    // Set timeout CAN error
    comInterface.canError = SPI_SENSOR_STATUS_ERROR_SPI_RECV_TIMEOUT;

    // Update sensor packet
    UpdateSensorPacket();

    // Infinite loop
    while(1);
}

#endif
