#ifndef __COMMUNICATION_CM_H__
#define __COMMUNICATION_CM_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
//#include "math_defines.h"
#include "foc.h"
//#include "uomodri_user_defines.h"
//#include "legacy_spi.h"

typedef struct _mode_reg_t_
{
    uint16_t        TIMEOUT     : 8;    /*!< Bits 7-0   : R/W - Timeout disable system after no valid command in milliseconds           */
    uint16_t        MODE_RSV1   : 2;    /*!< Bits 9-8   : R/W - Reserved                                                                */
    uint16_t        EI2OC       : 1;    /*!< Bits 10    : R/W - Enable/Disable incremental encoder index offset compensation - MOTOR_2  */
    uint16_t        EI1OC       : 1;    /*!< Bits 11    : R/W - Enable/Disable incremental encoder index offset compensation - MOTOR_1  */
    uint16_t        EPRE        : 1;    /*!< Bits 12    : R/W - Enable/Disable position rollover error - position range : -128 to +128  */
    uint16_t        EM2         : 1;    /*!< Bits 13    : R/W - Enable(calibration procedure, alignment \& run)/Disable - MOTOR_2       */
    uint16_t        EM1         : 1;    /*!< Bits 14    : R/W - Enable(calibration procedure, alignment \& run)/Disable - MOTOR_1       */
    uint16_t        ES          : 1;    /*!< Bits 15    : R/W - Enable/Disable System                                                   */
} mode_reg_t;

typedef union _mode_reg_u_
{
  uint16_t          all;                /*!< Short (16bits) type access */
  mode_reg_t        bit;                /*!< @see command_mode_t        */
} mode_reg_u;

typedef struct _rx_payload_t_
{
    mode_reg_u      modeReg;
    uint32_t        positionRef[2];
    uint16_t        velocityRef[2];
    uint16_t        iqRef[2];
    uint16_t        kpCoeff[2];
    uint16_t        kdCoeff[2];
    uint8_t         iSat;
    uint16_t        index;
} rx_payload_t;

typedef struct _rx_uomodri_t_
{
    mode_reg_u      modeReg;
    float32_t       positionRef[2];
    float32_t       velocityRef[2];
    float32_t       iqRef[2];
    float32_t       kpCoeff[2];
    float32_t       kdCoeff[2];
    float32_t       iSat[2];
} rx_uomodri_t;

typedef struct _rx_msg_t_
{
    rx_payload_t    payload;
    uint32_t        crc;
} rx_msg_t;

typedef struct _status_reg_t_
{
    uint16_t        ERROR_CODE  : 4;    /*!< Bits 3-0   : R/W - Timeout disable system after no valid command               */
    uint16_t        STATUS_RSV1 : 3;    /*!< Bits 6-4   : R/W - Reserved                                                    */
    uint16_t        IDX2T       : 1;    /*!< Bits 7     : R/W - incremental encoder status toggle on index detect - MOTOR_2 */
    uint16_t        IDX1T       : 1;    /*!< Bits 8     : R/W - incremental encoder status toggle on index detect - MOTOR_1 */
    uint16_t        IDX2D       : 1;    /*!< Bits 9     : R/W - incremental encoder status index detected - MOTOR_2         */
    uint16_t        IDX1D       : 1;    /*!< Bits 10    : R/W - incremental encoder status index detected - MOTOR_1         */
    uint16_t        M2R         : 1;    /*!< Bits 11    : R/W - MOTOR_2 ready/not ready                                     */
    uint16_t        M2E         : 1;    /*!< Bits 12    : R/W - MOTOR_2 enabled/not enabled                                 */
    uint16_t        M1R         : 1;    /*!< Bits 13    : R/W - MOTOR_1 ready/not ready                                     */
    uint16_t        M1E         : 1;    /*!< Bits 14    : R/W - MOTOR_1 enabled/not enabled                                 */
    uint16_t        SE          : 1;    /*!< Bits 15    : R/W - System enabled/disabled                                     */
} status_reg_t;

typedef union _status_reg_u_
{
  uint16_t          all;                /*!< Short (16bits) type access */
  status_reg_t      bit;                /*!< @see DRV_Stat00_t          */
} status_reg_u;

typedef struct _tx_payload_t_
{
    status_reg_u    statusReg;
    uint16_t        timeStamp;
    uint32_t        position[2];
    uint16_t        velocity[2];
    uint16_t        iq[2];
    uint16_t        coilRes[2];
    uint16_t        adcSamples[2];
    uint16_t        lastCmdIndex;
} tx_payload_t;

typedef struct _tx_msg_t_
{
    tx_payload_t    payload;
    uint32_t        crc;
//    float32_t       time[2];             // Time in seconds since startup
} tx_msg_t;
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
//typedef struct __DualBufferR__
//{
//    volatile struct CH_REGS* ptrDmaCh;
//    uint16_t index;
//    rx_msg_t com_rx_msg[2];
//} DualBufferR;

// Dual source buffer for sent data
//typedef struct __DualBufferW__
//{
//    volatile struct CH_REGS* ptrDmaCh;
//    uint16_t index;
//    tx_msg_t com_tx_msg[2];
//} DualBufferW;


//// Packet received from master board
//typedef struct __ComPacketR__
//{
//    mode_reg_u mode;
//    uint16_t refPos1_msb;
//    uint16_t refPos1_lsb;
//    uint16_t refPos2_msb;
//    uint16_t refPos2_lsb;
//    uint16_t refVel1;
//    uint16_t refVel2;
//    uint16_t refIq1;
//    uint16_t refIq2;
//    uint16_t Kp1;
//    uint16_t Kp2;
//    uint16_t Kd1;
//    uint16_t Kd2;
//    uint16_t iSat12; // Concatenated saturations for both controllers
//    uint16_t index;
//    uint16_t crc_msb;
//    uint16_t crc_lsb;
//} ComPacketR;
//
//// Debug packet for real-time logging
//typedef struct __ComPacketW__
//{
//    //sensor_status_u status;
//    uint16_t timeStamp;
//     uint16_t position1_msb;
//     uint16_t position1_lsb;
//     uint16_t position2_msb;
//     uint16_t position2_lsb;
//     uint16_t velocity1;
//     uint16_t velocity2;
//     uint16_t iq1;
//     uint16_t iq2;
//     uint16_t coilResistance1;
//     uint16_t coilResistance2;
//     uint16_t adc1;
//     uint16_t adc2;
//     uint16_t lastCmdIndex;
//     uint16_t crc_lsb; // MSB and LSB are swapped for SOLO master board
//     uint16_t crc_msb;
// } ComPacketW;
//
//// Dual source buffer for received data
//typedef struct __DualBufferR__
//{
//    //volatile struct CH_REGS* ptrDmaCh;
//    uint16_t index;
//    ComPacketR comPackets[2];
//} DualBufferR;
//
//// Dual source buffer for sent data
//typedef struct __DualBufferW__
//{
//    //volatile struct CH_REGS* ptrDmaCh;
//    uint16_t index;
//    ComPacketW comPackets[2];
//} DualBufferW;
//
//// Pointer to received packet typedef
//typedef ComPacketR* ComPacketRPtr;
//#define RPACKET_ERROR       (ComPacketRPtr)0xFFFFFFFF
//// Pointer to sent packet typedef
//typedef ComPacketW* ComPacketWPtr;

// Initialization structure
#define DEFAULT_COM_INTERFACE       { 0, 0, 0, 0, 0xFFFF, 0, SPI_SENSOR_STATUS_ERROR_NO_ERROR, 0, 0, 0, 0, 0, 0, \
                                    CI_POSITION_LSB, CI_VELOCITY_LSB, CI_IQ_LSB, CI_CURRENT_SAT_LSB, CI_LSBISTANCE_LSB, CI_VOLTAGE_LSB, CI_KP_LSB, CI_KD_LSB }

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



//// Debug
//uint32_t comError = 0;
//uint32_t comSuccess = 0;

bool com_cmdRx(rx_msg_t*, rx_uomodri_t*);
bool com_msgTx(FOC_STRUCT*, tx_msg_t*);
//uint32_t com_crc32(uint16_t*, uint16_t);
//void com_msgDbgTx(FOC_STRUCT*, uint8_t*);

#endif /*__COMMUNICATION_CM_H__*/
