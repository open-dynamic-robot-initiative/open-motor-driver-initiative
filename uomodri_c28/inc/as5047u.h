#ifndef AS5047U_h
#define AS5047U_h

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#ifdef USE_CM_CORE
#include "uomodri_cm_user_defines.h"
#else
#include "hal.h"
#include "uomodri_user_defines.h"
#endif

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
typedef struct __asenc_cfg_t__
{
    uint32_t  spiHandle;            /*!< handle for the serial peripheral interface */
    uint32_t  gpioNumber_CS;        /*!< GPIO connected to the AS5047U CS pin       */
} asenc_cfg_t;

// ERRFL Register Definition________________________________________________________________
typedef struct _Errfl_t
{
    uint16_t    agcw    : 1;    /* Agc-warning */
    uint16_t    maghalf : 1;
    uint16_t    p2ramw  : 1;    /* P2ram_warning */
    uint16_t    p2ramer : 1;    /* P2ram_error */
    uint16_t    framer  : 1;    /* Framing_error */
    uint16_t    comerr  : 1;    /* Command_error */
    uint16_t    crcerr  : 1;    /* CRC error */
    uint16_t    wdtst   : 1;    /* Watchdog Information */
    uint16_t    unused1 : 1;
    uint16_t    ofnf    : 1;    /* OffCompNotFinished */
    uint16_t    cordov  : 1;    /* Cordic overflow */
    uint16_t    unused2 : 5;
} Errfl_t;

typedef union _Errfl_u
{
    uint16_t    all;
    Errfl_t     bit;
} Errfl_u;

// PROG Register Definition________________________________________________________________
typedef struct _Prog_t
{
    uint16_t    progen  : 1;
    uint16_t    unused1 : 1;
    uint16_t    otpref  : 1;
    uint16_t    progotp : 1;
    uint16_t    unused2 : 2;
    uint16_t    progver : 1;
    uint16_t    unused3 : 9;
} Prog_t;

typedef union _Prog_u
{
    uint16_t    all;
    Prog_t      bit;
} Prog_u;

// DIA Register Definition________________________________________________________________
typedef struct _Dia_t
{
    uint16_t    vddm    : 1;
    uint16_t    loopend : 1;
    uint16_t    cordov  : 1;
    uint16_t    compl   : 1;
    uint16_t    comph   : 1;
    uint16_t    maghalf : 1;
    uint16_t    cosoffs : 1;
    uint16_t    sinoffs : 1;
    uint16_t    offcomp : 1;
    uint16_t    agcf    : 1;
    uint16_t    unused1 : 1;
    uint16_t    spicnt  : 2;
    uint16_t    unused2 : 3;
} Dia_t;

typedef union _Dia_u
{
    uint16_t    all;
    Dia_t       bit;
} Dia_u;

// AGC Register Definition________________________________________________________________
typedef struct _Agc_t
{
    uint8_t     agc;
} Agc_t;

typedef union _Agc_u
{
    uint8_t     all;
    Agc_t       bit;
} Agc_u;

// VEL Register Definition____________________________________________________________
typedef struct _Vel_t
{
    uint16_t    vel     : 14;
    uint16_t    unused1 : 2;
} Vel_t;

typedef union _Vel_u
{
    uint16_t    all;
    Vel_t       bit;
} Vel_u;

// Mag Register Definition____________________________________________________________
typedef struct _Mag_t
{
    uint16_t    mag     : 14;
    uint16_t    unused1 : 2;
} Mag_t;

typedef union _Mag_u
{
    uint16_t    all;
    Mag_t       bit;
} Mag_u;

// ANGLEUNC Register Definition____________________________________________________________
typedef struct _Angle_t
{
    uint16_t    cordicang   : 14;
    uint16_t    unused1     : 2;
} Angle_t;

typedef union _Angle_u
{
    uint16_t    all;
    Angle_t     bit;
} Angle_u;

// ECC_S Register Definition____________________________________________________________
typedef struct _Eccs_t
{
    uint16_t    cordicang   : 14;
    uint16_t    unused1     : 2;
} Eccs_t;

typedef union _Eccs_u
{
    uint16_t    all;
    Eccs_t      bit;
} Eccs_u;

// ANGLECOM Register Definition________________________________________________________________
typedef struct _Anglecom_t
{
    uint16_t    daecang : 14;
    uint16_t    unused1 : 2;
} Anglecom_t;

typedef union _Anglecom_u
{
    uint16_t    all;
    Anglecom_t  bit;
} Anglecom_u;

// DISABLE Register Definition________________________________________________________________
typedef struct _Disable_t
{
    uint8_t     uvwoff  : 1;
    uint8_t     abioff  : 1;
    uint8_t     na      : 4;
    uint8_t     filtdis : 1;
    uint8_t     unused1 : 1;
} Disable_t;

typedef union _Disable_u
{
    uint8_t     all;
    Disable_t   bit;
} Disable_u;

// ZPOSM Register Definition________________________________________________________________
typedef struct _Zposm_t
{
    uint8_t     zposm;
} Zposm_t;

typedef union _Zposm_u
{
    uint8_t     all;
    Zposm_t     bit;
} Zposm_u;

// ZPOSL Register Definition________________________________________________________________
typedef struct _Zposl_t
{
    uint8_t     zposl   :6;
    uint8_t     dia1en  :1;
    uint8_t     dia2en  :1;
} Zposl_t;

typedef union _Zposl_u
{
    uint8_t     all;
    Zposl_t     bit;
} Zposl_u;

// SETTINGS1 Register Definition________________________________________________________________
typedef struct _Settings1_t
{
    uint8_t     kmax    : 3;
    uint8_t     kmin    : 3;
    uint8_t     dia3en  : 1;
    uint8_t     dia4en  : 1;
} Settings1_t;

typedef union _Settings1_u
{
    uint8_t     all;
    Settings1_t bit;
} Settings1_u;

// SETTINGS2 Register Definition________________________________________________________________
typedef struct _Settings2_t
{
    uint8_t     iwidth      : 1;
    uint8_t     noiseset    : 1;
    uint8_t     dir         : 1;
    uint8_t     uvwabi      : 1;
    uint8_t     daecdis     : 1;
    uint8_t     abidec      : 1;
    uint8_t     data_slct   : 1;
    uint8_t     pwmon       : 1;
} Settings2_t;

typedef union _Settings2_u
{
    uint8_t     all;
    Settings2_t bit;
} Settings2_u;

// SETTINGS3 Register Definition________________________________________________________________
typedef struct _Settings3_t
{
    uint8_t     uvwpp   : 3;
    uint8_t     hys     : 2;
    uint8_t     abires  : 3;
} Settings3_t;

typedef union _Settings3_u
{
    uint8_t     all;
    Settings3_t bit;
} Settings3_u;

// 16 bit Command Frame  Definition________________________________________________________________
typedef struct _CmdFrame16_t
{
    uint16_t    cmdFrame    : 14;
    uint16_t    rw          : 1;
    uint16_t    dnc         : 1; /*do not care*/
} CmdFrame16_t;

typedef union _CmdFrame16_u
{
    uint16_t        all;
    CmdFrame16_t    bit;
} CmdFrame16_u;

// 16 bit BitData Frame  Definition________________________________________________________________
typedef struct _BitDataFrame16_t
{
    uint16_t    data    : 14;
    uint16_t    er      : 2;
} BitDataFrame16_t;

typedef union _BitDataFrame16_u
{
    uint16_t            all;
    BitDataFrame16_t    bit;
} BitDataFrame16_u;

// 24 bit Command Frame  Definition________________________________________________________________
typedef struct _CmdFrame24_t
{
    uint32_t    crc         : 8;
    uint32_t    cmdFrame    : 14;
    uint32_t    rw          : 1;
    uint32_t    dnc         : 1; /*do not care*/
} CmdFrame24_t;

typedef union _CmdFrame24_u
{
    uint32_t        all;
    CmdFrame24_t    bit;
} CmdFrame24_u;

// 24 bit BitData Frame  Definition________________________________________________________________
typedef struct _BitDataFrame24_t
{
    uint32_t    crc     : 8;
    uint32_t    data    : 14;
    uint32_t    er      : 2;
} BitDataFrame24_t;

typedef union _BitDataFrame24_u
{
    uint32_t            all;
    BitDataFrame24_t    bit;
} BitDataFrame24_u;

// 32 bit Command Frame  Definition________________________________________________________________
typedef struct _CommandFrame32_t
{
    uint32_t    crc         : 8;
    uint32_t    cmdFrame    : 14;
    uint32_t    rw          : 1;
    uint32_t    dnc         : 1; /*do not care*/
    uint32_t    pad         : 8;
} CmdFrame32_t;

typedef union _CmdFrame32_u
{
    uint32_t        all;
    CmdFrame32_t    bit;
} CmdFrame32_u;

// 32 bit BitData Frame  Definition________________________________________________________________
typedef struct _BitDataFrame32_t
{
    uint32_t    pad     : 8;
    uint32_t    crc     : 8;
    uint32_t    data    : 14;
    uint32_t    er      : 2;
} BitDataFrame32_t;

typedef struct _ByteDataFrame32_t
{
    uint32_t    byte_0  : 8;
    uint32_t    byte_1  : 8;
    uint32_t    byte_2  : 8;
    uint32_t    byte_3  : 8;
} ByteDataFrame32_t;

typedef struct _ShortDataFrame32_t
{
    uint32_t    short_0 : 16;
    uint32_t    short_1 : 16;
} ShortDataFrame32_t;

typedef union _BitDataFrame32_u
{
    uint32_t            all;
    ShortDataFrame32_t  byte_16;
    ByteDataFrame32_t   byte_8;
    BitDataFrame32_t    bit;
} BitDataFrame32_u;

typedef struct __AS_ENC_STRUCT__
{
    // pointers on HAL structure, SPI registers and direct access to SPI counter register
    const asenc_cfg_t*  asencHandle; //= MST_SPI_BASE;

    BitDataFrame32_u    angleData;
    BitDataFrame32_u    velData;
    float32_t           degAngle;
    float32_t           prevAngle;
    float32_t           startAngle;
    float32_t           totalAngle;
    float32_t           turnNum;
    float32_t           rndCount;
    float32_t           vel;
    uint16_t            quadNum;
    uint16_t            prevQuadNum;

} as_enc_t;

/***********************************************************************
 * DEFINES
 ***********************************************************************/
// Volatile Registers Addresses
#define NOP_REG             0x0000
#define ERRFL_REG           0x0001
#define PROG_REG            0x0003
#define DIA_REG             0x3FF5
#define AGC_REG             0x3FF9
#define SINDATA_REG         0x3FFA
#define COSDATA_REG         0x3FFB
#define VEL_REG             0x3FFC
#define MAG_REG             0x3FFD
#define ANGLE_REG           0x3FFE
#define ANGLECOM_REG        0x3FFF
#define ECC_CHECKSUM_REG    0x00D1


// Non-Volatile Registers Addresses
#define DISABLE_REG         0x0015
#define ZPOSM_REG           0x0016
#define ZPOSL_REG           0x0017
#define SETTINGS1_REG       0x0018
#define SETTINGS2_REG       0x0019
#define SETTINGS3_REG       0x001A
#define ECC_REG             0x001B

#define WRITE               0
#define READ                1

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void AS5047UReset(as_enc_t*);

uint16_t readData(as_enc_t* , uint16_t command);

void writeData32(as_enc_t* , uint32_t command, uint32_t data);
uint32_t readData32(as_enc_t* , uint32_t command);

BitDataFrame16_u readRegister(as_enc_t*, uint16_t registerAddress);
BitDataFrame32_u readRegister32(as_enc_t* p_enc, uint16_t registerAddress);

void readAngle(as_enc_t*);
void readAngle32(as_enc_t*);

uint8_t calcCRC(uint8_t* command);
bool checkCRC(uint8_t* frame, uint8_t crc);

uint16_t send16(as_enc_t* , uint16_t command);
uint32_t send32(as_enc_t* , uint32_t command);
void getAngleandVelocity(as_enc_t*);
void writeRegister(as_enc_t*,uint16_t registerAddress, uint16_t registerValue);
void writeRegister32(as_enc_t*,uint16_t registerAddress, uint16_t registerValue);
void writeSettings1(as_enc_t*, Settings1_u values);
void writeSettings2(as_enc_t*, Settings2_u values);
void writeZeroPosition(as_enc_t*, Zposm_u zposm, Zposl_u zposl);



#endif // #AS5047U_h
