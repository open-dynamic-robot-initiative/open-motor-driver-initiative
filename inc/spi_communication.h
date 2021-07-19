#ifndef __SPI_COMMUNICATION_H__
#define __SPI_COMMUNICATION_H__

#include "f2838x_device.h"
#include "hal.h"
#include "uomodri_user_defines.h"

// Word number in a full communication


//-----------------------------------------------------------------------
//                      COMMUNICATION FIELDS
//-----------------------------------------------------------------------

// MasterBoard MOSI communication mode bits definition
struct MB_MODE_BITS
{
    uint16_t TIMEOUT:8; // Timeout in milliseconds
    uint16_t rsvd   :2; // reserved
    uint16_t EI2OC  :1; // Enable index 2 offset compensation
    uint16_t EI1OC  :1; // Enable index 1 offset compensation
    uint16_t EPRE   :1; // Enable position roll-over error
    uint16_t EM2    :1; // Enable motor 2
    uint16_t EM1    :1; // Enable motor 1
    uint16_t ES     :1; // Enable system
};

union MB_MODE
{
    uint16_t all;
    struct MB_MODE_BITS bit;
};

// MasterBoard MISO communication status bits definition
struct MB_STATUS_BITS
{
    uint16_t EC   :4; // Error code
    uint16_t rsvd :3; // reserved
    uint16_t IDX2T:1; // Index 2 toggle
    uint16_t IDX1T:1; // Index 1 toggle
    uint16_t IDX2D:1; // Index 2 detected
    uint16_t IDX1D:1; // Index 1 detected
    uint16_t M2R  :1; // Motor 2 ready
    uint16_t M2E  :1; // Motor 2 enabled
    uint16_t M1R  :1; // Motor 1 ready
    uint16_t M1E  :1; // Motor 1 enabled
    uint16_t SE   :1; // System enabled
};

union MB_STATUS
{
    uint16_t all;
    struct MB_STATUS_BITS bit;
};

//-----------------------------------------------------------------------
//                      COMMUNICATION STRUCTURES
//-----------------------------------------------------------------------

#if UOMODRI_SPI_DEBUG_CONFIG

// Debug packet for real-time logging
typedef struct __ComPacketW__
{
    uint16_t timestamp_msb;
    uint16_t timestamp_lsb;
    uint16_t idRef;
    uint16_t idFbk;
    uint16_t iqRef;
    uint16_t iqFbk;
    uint16_t position_msb;
    uint16_t position_lsb;
    uint16_t refPos_msb;
    uint16_t refPos_lsb;
    uint16_t velocity;
    uint16_t refVel;
    uint16_t ud;
    uint16_t uq;
    uint16_t resistance;
    uint16_t crc_msb;
    uint16_t crc_lsb;
} ComPacketW;

#else

// Sensor packet sent to master board
typedef struct __ComPacketW__
{
    union MB_STATUS status;
    uint16_t timeStamp;
    uint16_t position1_msb;
    uint16_t position1_lsb;
    uint16_t position2_msb;
    uint16_t position2_lsb;
    uint16_t velocity1;
    uint16_t velocity2;
    uint16_t iq1;
    uint16_t iq2;
    uint16_t coilResistance1;
    uint16_t coilResistance2;
    uint16_t adc1;
    uint16_t adc2;
    uint16_t lastCmdIndex;
    uint16_t crc_lsb; // MSB and LSB are swapped for SOLO master board
    uint16_t crc_msb;
} ComPacketW;

#endif


// Packet received from master board
typedef struct __ComPacketR__
{
    union MB_MODE mode;
    uint16_t refPos1_msb;
    uint16_t refPos1_lsb;
    uint16_t refPos2_msb;
    uint16_t refPos2_lsb;
    uint16_t refVel1;
    uint16_t refVel2;
    uint16_t refIq1;
    uint16_t refIq2;
    uint16_t Kp1;
    uint16_t Kp2;
    uint16_t Kd1;
    uint16_t Kd2;
    uint16_t iSat12; // Concatenated saturations for both controllers
    uint16_t index;
    uint16_t crc_msb;
    uint16_t crc_lsb;
} ComPacketR;


//-----------------------------------------------------------------------
//                      STRUCTURES HANDLERS
//-----------------------------------------------------------------------

// Pointer to received packet typedef
typedef ComPacketR* ComPacketRPtr;
#define RPACKET_ERROR       (ComPacketRPtr)0xFFFFFFFF

// Pointer to sent packet typedef
typedef ComPacketW* ComPacketWPtr;

//-----------------------------------------------------------------------
//                          DUAL BUFFERS
//-----------------------------------------------------------------------
/*
 * Dual buffers are used to avoid writing on data while it'is being transmitted
 * or received through SPI.
 *
 * They contain three attributes:
 *  - ptrDmaCh: Pointer to the associated DMA channel register structure
 *  - index: Index of the currently used packet
 *  - comPackets: Two communication packets, one of which is constantly
 *                being used by the master board communication.
 *                When a user packet update occurs, the second packet is the
 *                one updated by the user and becomes the packet used for the SPI communication.
 */


// Dual source buffer for received data
typedef struct __DualBufferR__
{
    volatile struct CH_REGS* ptrDmaCh;
    uint16_t index;
    ComPacketR comPackets[2];
} DualBufferR;

// Dual source buffer for sent data
typedef struct __DualBufferW__
{
    volatile struct CH_REGS* ptrDmaCh;
    uint16_t index;
    ComPacketW comPackets[2];
} DualBufferW;


//#pragma DATA_SECTION(dualBufferW, "ramgs0");
//extern DualBufferW dualBufferW;
//#pragma DATA_SECTION(dualBufferR, "ramgs0");
//extern DualBufferR dualBufferR;

//-----------------------------------------------------------------------
//                      FUNCTION SIGNATURES
//-----------------------------------------------------------------------
// Communication initialization routine
void SPI_initCommunication(const DMA_TypeDef*);

// Returns a pointer to the packet that isn't sent to the master board
inline ComPacketWPtr SPI_getAvailableWPacket();
// Compute CRC and swap Tx packets after the unused one has been updated
inline void SPI_pushWPacket();

// Returns a pointer to the packet that doesn't receive master board data
// If the CRC of the returned packet isn't correct, return (ComPacketRPtr)0xFFFFFFFF pointer instead.
// The user MUST check if the returned pointer is (ComPacketRPtr)0xFFFF before using it.
inline ComPacketRPtr SPI_getAvailableRPacket();

#endif
