#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#ifdef CPU1
#include "f2838x_device.h"
#include "motor.h"
#else
#include <stddef.h>
#endif

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define POLYNOMIAL_32_P1                            0x04C11DB7
#define CRC_INIT_CRC32                              0xFFFFFFFF

#define MSG_RX_ES_POS                               (15)
#define MSG_RX_ES_MASK                              (1 << MSG_RX_ES_POS)
#define MSG_RX_EM1_POS                              (14)
#define MSG_RX_EM1_MASK                             (1 << MSG_RX_EM1_POS)
#define MSG_RX_EM2_POS                              (13)
#define MSG_RX_EM2_MASK                             (1 << MSG_RX_EM2_POS)
#define MSG_RX_EPRE_POS                             (12)
#define MSG_RX_EPRE_MASK                            (1 << MSG_RX_EPRE_POS)
#define MSG_RX_EI1OC_POS                            (11)
#define MSG_RX_EI1OC_MASK                           (1 << MSG_RX_EI1OC_POS)
#define MSG_RX_EI2OC_POS                            (10)
#define MSG_RX_EI2OC_MASK                           (1 << MSG_RX_EI2OC_POS)
#define MSG_RX_TIMEOUT_POS                          (0)
#define MSG_RX_TIMEOUT_MASK                         (0xFF << MSG_RX_TIMEOUT_POS)

#define MSG_RX_ES                                   (0x8000)
#define MSG_RX_EM1                                  (0x4000)
#define MSG_RX_EM2                                  (0x2000)
#define MSG_RX_EPRE                                 (0x1000)
#define MSG_RX_EI1OC                                (0x0800)
#define MSG_RX_EI2OC                                (0x0400)
#define MSG_RX_TIMEOUT                              (0x00FF)

#define STATUS_ERROR_NO_ERROR                       (0) /*!< No error */
#define STATUS_ERROR_ENCODER1                       (1) /*!< Encoder_1 error too high */
#define STATUS_ERROR_SPI_RECV_TIMEOUT               (2) /*!< Timeout for receiving current references exceeded */
#define STATUS_ERROR_CRIT_TEMP                      (3) /*!< Motor temperature reached critical valued */
#define STATUS_ERROR_POSCONV                        (4) /*!< Unused error */
#define STATUS_ERROR_POS_ROLLOVER                   (5) /*!< Position roll-over occurred */
#define STATUS_ERROR_ENCODER2                       (6) /*!< Encoder_2 error too high */
#define STATUS_ERROR_DRV_NFAULT                     (7) /*!< Motor DRV nFault error */

/***********************************************************************
 * COMMUNICATION CRC TABLE
 ***********************************************************************/
static const uint32_t crc32_table[] =
{
 0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61, 0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
 0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
 0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039, 0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
 0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
 0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1, 0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
 0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
 0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
 0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde, 0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
 0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
 0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6, 0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
 0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
 0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637, 0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
 0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
 0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff, 0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
 0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
 0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7, 0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
 0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
 0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8, 0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
 0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
 0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
 0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0, 0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
 0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
 0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668, 0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};

static unsigned char const crc8_table[] =
{
 0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
 0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
 0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
 0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
 0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
 0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
 0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
 0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
 0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
 0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
 0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
 0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
 0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
 0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
 0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35
};

/***********************************************************************
 * COMMUNICATION ERROR FLAGS
 ***********************************************************************/
/**
 * @enum    com_statusError
 * @brief   Error flags list.
 */
//typedef enum
//{
//    STATUS_ERROR_NO_ERROR           = 0, /*!< No error */
//    STATUS_ERROR_ENCODER1           = 1, /*!< Encoder_1 error too high */
//    STATUS_ERROR_SPI_RECV_TIMEOUT   = 2, /*!< Timeout for receiving current references exceeded */
//    STATUS_ERROR_CRIT_TEMP          = 3, /*!< Motor temperature reached critical valued */
//    STATUS_ERROR_POSCONV            = 4, /*!< Unused error */
//    STATUS_ERROR_POS_ROLLOVER       = 5, /*!< Position roll-over occurred */
//    STATUS_ERROR_ENCODER2           = 6, /*!< Encoder_2 error too high */
//    STATUS_ERROR_DRV_NFAULT         = 7, /*!< Motor DRV nFault error */
//} com_statusError;

/**
 * @enum    com_rx_state_e
 * @brief   States of the state machine in COM_msgExtract_cla for message extraction.
 */
typedef enum
{
    COM_RX_STATE_MODE               = 0, //!< COM_RX_STATE_MODE
    COM_RX_STATE_POS_MSB_MOT1       = 1, //!< COM_RX_STATE_POS_MSB_MOT1
    COM_RX_STATE_POS_LSB_MOT1       = 2, //!< COM_RX_STATE_POS_LSB_MOT1
    COM_RX_STATE_POS_MSB_MOT2       = 3, //!< COM_RX_STATE_POS_MSB_MOT2
    COM_RX_STATE_POS_LSB_MOT2       = 4, //!< COM_RX_STATE_POS_LSB_MOT2
    COM_RX_STATE_VEL_MOT1           = 5, //!< COM_RX_STATE_VEL_MOT1
    COM_RX_STATE_VEL_MOT2           = 6, //!< COM_RX_STATE_VEL_MOT2
    COM_RX_STATE_IQ_MOT1            = 7, //!< COM_RX_STATE_IQ_MOT1
    COM_RX_STATE_IQ_MOT2            = 8, //!< COM_RX_STATE_IQ_MOT2
    COM_RX_STATE_KP_MOT1            = 9, //!< COM_RX_STATE_KP_MOT1
    COM_RX_STATE_KP_MOT2            = 10,//!< COM_RX_STATE_KP_MOT2
    COM_RX_STATE_KD_MOT1            = 11,//!< COM_RX_STATE_KD_MOT1
    COM_RX_STATE_KD_MOT2            = 12,//!< COM_RX_STATE_KD_MOT2
    COM_RX_STATE_ISAT_MOT12         = 13,//!< COM_RX_STATE_ISAT_MOT12
    COM_RX_STATE_INDEX              = 14,//!< COM_RX_STATE_INDEX
    COM_RX_STATE_CRC_MSB            = 15,//!< COM_RX_STATE_CRC_MSB
    COM_RX_STATE_CRC_LSB            = 16,//!< COM_RX_STATE_CRC_LSB
} com_rx_state_e;

/***********************************************************************
 * COMMUNICATION STATE MACHINES FOR CM CORE
 ***********************************************************************/
#ifndef CPU1
typedef enum
{
    COM_DBG_RX_WORD_A               = 0,    //!< COM_RX_STATE_MODE
    COM_DBG_RX_WORD_B               = 1,
    COM_DBG_RX_CNT                  = 2,
    COM_DBG_RX_MSG_TYPE             = 3,
    COM_DBG_RX_CFG_LIST_4_TX        = 4,
    COM_DBG_RX_CMD_LIST_4_RX        = 5,
    COM_DBG_RX_CMD_MSG              = 6,
    COM_DBG_RX_CRC                  = 7,
} com_dbg_rx_state_e;

typedef enum
{
    COM_DBG_TX_HEADER               = 0,    //!< COM_RX_STATE_MODE
//    COM_DBG_TX_WORD_B               = 1,
    COM_DBG_TX_CNT                  = 2,
    COM_DBG_TX_MSG_TYPE             = 3,
    COM_DBG_TX_CFG_ACK              = 4,
    COM_DBG_TX_CMD_LIST_4_RX        = 5,
    COM_DBG_TX_CMD_MSG              = 7,
    COM_DBG_TX_CRC                  = 8,
} com_dbg_tx_state_e;
#endif

/***********************************************************************
 * COMMUNICATION STRUCTURES \& UNIONS BIT FILEDS
 ***********************************************************************/
/**
 * @struct  mst2slv_reg_t
 * @brief   Master to Slave command structure (bits definition) for uOmodri
 */
typedef struct __mst2slv_reg_t__
{
    uint16_t        TIMEOUT     : 8;    /*!< Bits 7-0   : R/W - Timeout disable system after no valid command in milliseconds           */
    uint16_t        CMD_RSV1    : 2;    /*!< Bits 9-8   : R/W - Reserved                                                                */
//    uint16_t        RST         : 1;    /*!< Bits 9     : R/W - Force a reset of CPU1, CPU1 periph, CPU2, CM, CM periph                 */
    uint16_t        EI2OC       : 1;    /*!< Bits 10    : R/W - Enable/Disable incremental encoder index offset compensation - MOTOR_2  */
    uint16_t        EI1OC       : 1;    /*!< Bits 11    : R/W - Enable/Disable incremental encoder index offset compensation - MOTOR_1  */
    uint16_t        EPRE        : 1;    /*!< Bits 12    : R/W - Enable/Disable position rollover error - position range : -128 to +128  */
    uint16_t        EM2         : 1;    /*!< Bits 13    : R/W - Enable(calibration procedure, alignment \& run)/Disable - MOTOR_2       */
    uint16_t        EM1         : 1;    /*!< Bits 14    : R/W - Enable(calibration procedure, alignment \& run)/Disable - MOTOR_1       */
    uint16_t        ES          : 1;    /*!< Bits 15    : R/W - Enable/Disable System                                                   */
} mst2slv_reg_t;

/**
 * @union   mst2slv_reg_u
 * @brief   Master to Slave command union (bits definition) for uOmodri. Access to bit field.
 */
typedef union __mst2slv_reg_u__
{
  uint16_t          all;                /*!< Short (16bits) type access */
  mst2slv_reg_t     bit;                /*!< @see command_mode_t        */
} mst2slv_reg_u;

/**
 * @struct  slv2mst_reg_t
 * @brief   Slave to Master status structure (bits definition) from uOmodri.
 */
typedef struct __slv2mst_reg_t__
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
} slv2mst_reg_t;

/**
 * @union   slv2mst_reg_u
 * @brief   Slave to Master status structure (bits definition) from uOmodri. Access to bit field.
 */
typedef union __slv2mst_reg_u__
{
  uint16_t          all;                /*!< Short (16bits) type access */
  slv2mst_reg_t     bit;                /*!< @see DRV_Stat00_t          */
} slv2mst_reg_u;

/**
 * @struct  U32toU16_reg_t
 * @brief   32bits to 16bits down conversion.
 */
typedef struct __U32toU16_reg_t__
{
    uint16_t        u16_msb;
    uint16_t        u16_lsb;
} U32toU16_reg_t;

/***********************************************************************
 * MAIN COMMUNICATION STRUCTURES
 ***********************************************************************/
/**
 * @struct  mst2slv_msg_t
 * @brief   Master 2 Slave full command message including CRC for uOmodri
 */
typedef struct __mst2slv_msg_t__
{
    mst2slv_reg_u   mode;
    U32toU16_reg_t  position[2];
    int16_t         velocity[2];
    int16_t         current[2];
    uint16_t        kpCoeff[2];
    uint16_t        kdCoeff[2];
    uint16_t        iSat;
    uint16_t        index;
    U32toU16_reg_t  crc;
} mst2slv_msg_t;

/**
 * @struct  mst2slv_msg_cla_t
 * @brief   Master 2 Slave structure associated with COM_msgExtract_cla (state machine)
 */
typedef struct __mst2slv_msg_cla_t__
{
    com_rx_state_e  msg_rx_state;
    uint32_t        msg_crc32;
    uint16_t        msg_rx;
    uint16_t        msg_tmp;
} mst2slv_msg_cla_t;

/**
 * @struct  slv2mst_msg_t
 * @brief   Slave to master full status message including CRC from uOmodri
 */
typedef struct __slv2mst_msg_t__
{
    slv2mst_reg_u   status;
    uint16_t        timeStamp;
    U32toU16_reg_t  position[2];
    int16_t         velocity[2];
    int16_t         current[2];
    uint16_t        coilRes[2];
    uint16_t        adcSamples[2];
    uint16_t        lastCmdIndex;
    U32toU16_reg_t  crc;
} slv2mst_msg_t;

/**
 * @struct  cmd_uomodri_t
 * @brief   Structure of all commands necessaries for uOmodri
 */
typedef struct __cmd_uomodri_t__
{
    mst2slv_reg_u   mode;
    float           posRef[2];
    float           velRef[2];
    float           iqRef[2];
    float           kpCoeff[2];
    float           kdCoeff[2];
    float           iSat[2];
    uint32_t        cptTimeout;
    uint16_t        index;
    bool            cmdAccesValid;
} cmd_uomodri_t;

/***********************************************************************
 * COMMUNICATION STRUCTURES FOR SHARED DATA EXCHANGE (CPU1<->CM)
 ***********************************************************************/
typedef union __uint64_u__
{
    uint64_t        uint64b;
    uint8_t         byte[sizeof(uint64_t)];
} uint64_u;

typedef union __uint32_u__
{
    uint32_t        uint32b;
    uint8_t         byte[sizeof(uint32_t)];
} uint32_u;
#ifdef CPU1
typedef union __float32_u__
{
    float32_t       float32b;
    uint8_t         byte[sizeof(float32_t)];
} float32_u;
#else
typedef union __float32_u__
{
    float_t         float32b;
    uint8_t         byte[sizeof(float_t)];
} float32_u;
#endif

/**
 * @struct  com_cpu1_to_cm_t
 * @brief   Structure for address transfer between CPU (16bits) and CM(8bits)
 */
typedef struct __com_cpu_2_cm_t__
{
    uint32_u    itcnt;
    float32_u   ia;
    float32_u   ib;
    float32_u   ic;
    float32_u   vbus;
    float32_u   ialpha;
    float32_u   ibeta;
    float32_u   id;
    float32_u   iq;
    float32_u   iqRef;
    float32_u   ud;
    float32_u   uq;
    float32_u   ualpha;
    float32_u   ubeta;
    float32_u   ua;
    float32_u   ub;
    float32_u   uc;
    float32_u   theta;
    float32_u   posRef;
    float32_u   speed;
    float32_u   velRef;
    float32_u   pd_kp;
    float32_u   pd_kd;
    float32_u   iqff;
    float32_u   isat;
    uint32_u    status;
} com_cpu_2_cm_t;

typedef union __word_32b_u__
{
#ifdef CPU1
    float32_t       float_32b;
#else
    float_t         float_32b;
#endif
    uint32_t        uint_32b;
    uint16_t        uint_16b[2];
    uint8_t         uint_8b[4];    /*!< */
} word_32b_u;

#ifndef CPU1
typedef struct __bit_config_t__
{
    uint64_t        m1_ia       : 1;    /*!< Bit 0      : R/W - Enable system to start. No power on motors.         */
    uint64_t        m1_ib       : 1;    /*!< Bit 1      : R/W - Enable power on motor.                              */
    uint64_t        m1_ic       : 1;    /*!< Bit 2      : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m1_vbus     : 1;    /*!< Bit 3      : R/W - Use encoder index in position computation.          */
    uint64_t        m1_ialpha   : 1;    /*!< Bit 4      : R/W - Enable system to start. No power on motors.         */
    uint64_t        m1_ibeta    : 1;    /*!< Bit 5      : R/W - Enable power on motor.                              */
    uint64_t        m1_id       : 1;    /*!< Bit 6      : R/W - Enable power on motor.                              */
    uint64_t        m1_iq       : 1;    /*!< Bit 7      : R/W - Enable power on motor.                              */
    uint64_t        m1_iqref    : 1;    /*!< Bit 8      : R/W - Enable power on motor.                              */
    uint64_t        m1_ud       : 1;    /*!< Bit 9      : R/W - Enable power on motor.                              */
    uint64_t        m1_uq       : 1;    /*!< Bit 10     : R/W - Enable power on motor.                              */
    uint64_t        m1_ualpha   : 1;    /*!< Bit 11     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m1_ubeta    : 1;    /*!< Bit 12     : R/W - Enable power on motor.                              */
    uint64_t        m1_ua       : 1;    /*!< Bit 13     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m1_ub       : 1;    /*!< Bit 14     : R/W - Enable power on motor.                              */
    uint64_t        m1_uc       : 1;    /*!< Bit 15     : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m1_pos      : 1;    /*!< Bit 16     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m1_posref   : 1;    /*!< Bit 17     : R/W - Enable power on motor.                              */
    uint64_t        m1_vel      : 1;    /*!< Bit 18     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m1_velref   : 1;    /*!< Bit 19     : R/W - Enable power on motor.                              */
    uint64_t        m1_itcnt    : 1;    /*!< Bit 20     : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m1_pd_kp    : 1;    /*!< Bit 21     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m1_pd_kd    : 1;    /*!< Bit 22     : R/W - Enable power on motor.                              */
    uint64_t        m1_iqff     : 1;    /*!< Bit 23     : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m1_isat     : 1;    /*!< Bit 24     : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m1_status   : 1;    /*!< Bit 25     : R -                                                       */
    uint64_t        DISP_RSV1   : 6;    /*!< Bit 26-31  : R/W - Reserved                                            */
    uint64_t        m2_ia       : 1;    /*!< Bit 32     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m2_ib       : 1;    /*!< Bit 33     : R/W - Enable power on motor.                              */
    uint64_t        m2_ic       : 1;    /*!< Bit 34     : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m2_vbus     : 1;    /*!< Bit 35     : R/W - Use encoder index in position computation.          */
    uint64_t        m2_ialpha   : 1;    /*!< Bit 36     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m2_ibeta    : 1;    /*!< Bit 37     : R/W - Enable power on motor.                              */
    uint64_t        m2_id       : 1;    /*!< Bit 38     : R/W - Enable power on motor.                              */
    uint64_t        m2_iq       : 1;    /*!< Bit 39     : R/W - Enable power on motor.                              */
    uint64_t        m2_iqref    : 1;    /*!< Bit 40     : R/W - Enable power on motor.                              */
    uint64_t        m2_ud       : 1;    /*!< Bit 41     : R/W - Enable power on motor.                              */
    uint64_t        m2_uq       : 1;    /*!< Bit 42     : R/W - Enable power on motor.                              */
    uint64_t        m2_ualpha   : 1;    /*!< Bit 43     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m2_ubeta    : 1;    /*!< Bit 44     : R/W - Enable power on motor.                              */
    uint64_t        m2_ua       : 1;    /*!< Bit 45     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m2_ub       : 1;    /*!< Bit 46     : R/W - Enable power on motor.                              */
    uint64_t        m2_uc       : 1;    /*!< Bit 47     : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m2_pos      : 1;    /*!< Bit 48     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m2_posref   : 1;    /*!< Bit 49     : R/W - Enable power on motor.                              */
    uint64_t        m2_vel      : 1;    /*!< Bit 50     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m2_velref   : 1;    /*!< Bit 51     : R/W - Enable power on motor.                              */
    uint64_t        m2_itcnt    : 1;    /*!< Bit 52     : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m2_pd_kp    : 1;    /*!< Bit 53     : R/W - Enable system to start. No power on motors.         */
    uint64_t        m2_pd_kd    : 1;    /*!< Bit 54     : R/W - Enable power on motor.                              */
    uint64_t        m2_iqff     : 1;    /*!< Bit 55     : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m2_isat     : 1;    /*!< Bit 56     : R/W - Limit the rotations to +/- 128 rounds.              */
    uint64_t        m2_status   : 1;    /*!< Bit 57     : R -                                                       */
    uint64_t        DISP_RSV2   : 6;    /*!< Bit 58-63  : R/W - Reserved                                            */
} bit_config_t;
//typedef struct __bit_config_t__
//{
//    uint32_t        ia          : 1;    /*!< Bit 0      : R/W - Enable system to start. No power on motors.         */
//    uint32_t        ib          : 1;    /*!< Bit 1      : R/W - Enable power on motor.                              */
//    uint32_t        ic          : 1;    /*!< Bit 2      : R/W - Limit the rotations to +/- 128 rounds.              */
//    uint32_t        vbus        : 1;    /*!< Bit 3      : R/W - Use encoder index in position computation.          */
//    uint32_t        ialpha      : 1;    /*!< Bit 4      : R/W - Enable system to start. No power on motors.         */
//    uint32_t        ibeta       : 1;    /*!< Bit 5      : R/W - Enable power on motor.                              */
//    uint32_t        id          : 1;    /*!< Bit 6      : R/W - Enable power on motor.                              */
//    uint32_t        iq          : 1;    /*!< Bit 7      : R/W - Enable power on motor.                              */
//    uint32_t        iqref       : 1;    /*!< Bit 8      : R/W - Enable power on motor.                              */
//    uint32_t        ud          : 1;    /*!< Bit 9      : R/W - Enable power on motor.                              */
//    uint32_t        uq          : 1;    /*!< Bit 10     : R/W - Enable power on motor.                              */
//    uint32_t        ualpha      : 1;    /*!< Bit 11     : R/W - Enable system to start. No power on motors.         */
//    uint32_t        ubeta       : 1;    /*!< Bit 12     : R/W - Enable power on motor.                              */
//    uint32_t        ua          : 1;    /*!< Bit 13     : R/W - Enable system to start. No power on motors.         */
//    uint32_t        ub          : 1;    /*!< Bit 14     : R/W - Enable power on motor.                              */
//    uint32_t        uc          : 1;    /*!< Bit 15     : R/W - Limit the rotations to +/- 128 rounds.              */
//    uint32_t        pos         : 1;    /*!< Bit 16     : R/W - Enable system to start. No power on motors.         */
//    uint32_t        posref      : 1;    /*!< Bit 17     : R/W - Enable power on motor.                              */
//    uint32_t        vel         : 1;    /*!< Bit 18     : R/W - Enable system to start. No power on motors.         */
//    uint32_t        velref      : 1;    /*!< Bit 19     : R/W - Enable power on motor.                              */
//    uint32_t        itcnt       : 1;    /*!< Bit 20     : R/W - Limit the rotations to +/- 128 rounds.              */
//    uint32_t        pd_kp       : 1;    /*!< Bit 21     : R/W - Enable system to start. No power on motors.         */
//    uint32_t        pd_kd       : 1;    /*!< Bit 22     : R/W - Enable power on motor.                              */
//    uint32_t        iqff        : 1;    /*!< Bit 23     : R/W - Limit the rotations to +/- 128 rounds.              */
//    uint32_t        isat        : 1;    /*!< Bit 24     : R/W - Limit the rotations to +/- 128 rounds.              */
//    uint32_t        DISP_RSV1   : 7;    /*!< Bit 25-31  : R/W - Reserved                                            */
//} bit_config_t;

typedef struct __bit_command_t__
{
    uint32_t        m1_pd_kp    : 1;    /*!< Bit 0      : R/W - */
    uint32_t        m1_pd_ki    : 1;    /*!< Bit 1      : R/W - */
    uint32_t        m1_posref   : 1;    /*!< Bit 2      : R/W - */
    uint32_t        m1_velref   : 1;    /*!< Bit 3      : R/W - */
    uint32_t        m1_iqff     : 1;    /*!< Bit 4      : R/W - */
    uint32_t        m1_isat     : 1;    /*!< Bit 5      : R/W - */
    uint32_t        m1_cmd      : 1;    /*!< Bit 6      : R/W - */
    uint32_t        m1_pi_kp_id : 1;    /*!< Bit 7      : R/W - */
    uint32_t        m1_pi_ki_id : 1;    /*!< Bit 8      : R/W - */
    uint32_t        m1_pi_kp_iq : 1;    /*!< Bit 9      : R/W - */
    uint32_t        m1_pi_ki_iq : 1;    /*!< Bit 10     : R/W - */
    uint32_t        DISP_RSV1   : 5;    /*!< Bit 11-15  : R/W - */
    uint32_t        m2_pd_kp    : 1;    /*!< Bit 16     : R/W - */
    uint32_t        m2_pd_ki    : 1;    /*!< Bit 17     : R/W - */
    uint32_t        m2_posref   : 1;    /*!< Bit 18     : R/W - */
    uint32_t        m2_velref   : 1;    /*!< Bit 19     : R/W - */
    uint32_t        m2_iqff     : 1;    /*!< Bit 20     : R/W - */
    uint32_t        m2_isat     : 1;    /*!< Bit 21     : R/W - */
    uint32_t        m2_cmd      : 1;    /*!< Bit 22     : R/W - */
    uint32_t        m2_pi_kp_id : 1;    /*!< Bit 23     : R/W - */
    uint32_t        m2_pi_ki_id : 1;    /*!< Bit 24     : R/W - */
    uint32_t        m2_pi_kp_iq : 1;    /*!< Bit 25     : R/W - */
    uint32_t        m2_pi_ki_iq : 1;    /*!< Bit 26     : R/W - */
    uint32_t        DISP_RSV2   : 5;    /*!< Bit 27-31  : R/W - Reserved*/
} bit_command_t;
//typedef struct __bit_command_t__
//{
//    uint16_t        pd_kp       : 1;    /*!< Bit 0      : R/W - */
//    uint16_t        pd_ki       : 1;    /*!< Bit 1      : R/W - */
//    uint16_t        posref      : 1;    /*!< Bit 2      : R/W - */
//    uint16_t        velref      : 1;    /*!< Bit 3      : R/W - */
//    uint16_t        iqff        : 1;    /*!< Bit 4      : R/W - */
//    uint16_t        isat        : 1;    /*!< Bit 5      : R/W - */
//    uint16_t        cmd         : 1;    /*!< Bit 6      : R/W - */
//    uint16_t        pi_kp_id    : 1;    /*!< Bit 7      : R/W - */
//    uint16_t        pi_ki_id    : 1;    /*!< Bit 8      : R/W - */
//    uint16_t        pi_kp_iq    : 1;    /*!< Bit 9      : R/W - */
//    uint16_t        pi_ki_iq    : 1;    /*!< Bit 10     : R/W - */
//    uint16_t        DISP_RSV1   : 5;    /*!< Bit 11-15  : R/W - */
//} bit_command_t;

typedef union __counter_u__
{
    uint32_t        all;        /*!< Short (16bits) type access */
    uint8_t         byte[4];    /*!< @see command_mode_t        */
} counter_u;

typedef union __config_u__
{
    bit_config_t    bit;
    uint64_t        all;        /*!< Short (16bits) type access */
    uint8_t         byte[8];    /*!< @see command_mode_t        */
} config_u;
//typedef union __config_u__
//{
//    bit_display_t   bit;
//    uint32_t        display;    /*!< Short (16bits) type access */
//    uint8_t         byte[4];    /*!< @see command_mode_t        */
//} config_u;

typedef union __command_u__
{
    bit_command_t   bit;
    uint32_t        all;        /*!< Short (16bits) type access */
    uint8_t         byte[4];    /*!< @see command_mode_t        */
} command_u;
//typedef union __command_u__
//{
//    bit_command_t   bit;
//    uint16_t        command;    /*!< Short (16bits) type access */
//    uint8_t         byte[2];    /*!< @see command_mode_t        */
//} command_u;

typedef struct __bit_command_mem_t__
{
    word_32b_u m1_pd_kp;
    word_32b_u m1_pd_ki;
    word_32b_u m1_posref;
    word_32b_u m1_velref;
    word_32b_u m1_iqff;
    word_32b_u m1_isat;
    word_32b_u m1_cmd;
    word_32b_u m1_pi_kp_id;
    word_32b_u m1_pi_ki_id;
    word_32b_u m1_pi_kp_iq;
    word_32b_u m1_pi_ki_iq;
    word_32b_u m2_pd_kp;
    word_32b_u m2_pd_ki;
    word_32b_u m2_posref;
    word_32b_u m2_velref;
    word_32b_u m2_iqff;
    word_32b_u m2_isat;
    word_32b_u m2_cmd;
    word_32b_u m2_pi_kp_id;
    word_32b_u m2_pi_ki_id;
    word_32b_u m2_pi_kp_iq;
    word_32b_u m2_pi_ki_iq;
} bit_command_mem_t;
//typedef struct __bit_command_mem_t__
//{
//    word_32b_u pd_kp;
//    word_32b_u pd_ki;
//    word_32b_u posref;
//    word_32b_u velref;
//    word_32b_u iqff;
//    word_32b_u isat;
//    word_32b_u cmd;
//    word_32b_u pi_kp_id;
//    word_32b_u pi_ki_id;
//    word_32b_u pi_kp_iq;
//    word_32b_u pi_ki_iq;
//} bit_command_mem_t;

typedef struct __attribute__((__packed__)) __dbg_uart_msg_t__
{
    uint8_t         header[2];
//    uint32_t        counter;
//    uint8_t         msgtype;

    float_t         ia;
    float_t         ib;
    float_t         ic;
//    float_t         vbus;
//    float_t         ialpha;
//    float_t         ibeta;
    float_t         id;
    float_t         iq;
    float_t         iqref;
    float_t         ud;
    float_t         uq;
//    float_t         ualpha;
//    float_t         ubeta;
//    float_t         ua;
//    float_t         ub;
//    float_t         uc;
    float_t         pos;
//    float_t         posref;
    float_t         vel;
//    float_t         velref;
//    uint32_t        itcnt;
    uint8_t         crc;
//    uint8_t         padding;
} dbg_uart_msg_t;

typedef struct __attribute__((__packed__)) __dbg_uart_addr_t__
{
    float_t*        p_ia;
    float_t*        p_ib;
    float_t*        p_ic;
    float_t*        p_vbus;
    float_t*        p_ialpha;
    float_t*        p_ibeta;
    float_t*        p_id;
    float_t*        p_iq;
    float_t*        p_iqref;
    float_t*        p_ud;
    float_t*        p_uq;
    float_t*        p_ualpha;
    float_t*        p_ubeta;
    float_t*        p_ua;
    float_t*        p_ub;
    float_t*        p_uc;
    float_t*        p_pos;
    float_t*        p_posref;
    float_t*        p_vel;
    float_t*        p_velref;
//    float_t*        p_pd_kp;
//    float_t*        p_pd_kd;
//    float_t*        p_iqff;
//    float_t*        p_isat;
//    float_t*        p_status;
//    uint64_t*       p_itcnt;
} dbg_uart_addr_t;

typedef struct __com_dbg_tx_msg_t__
{
//    dbg_uart_msg_t      msg;
    com_dbg_tx_state_e  state;
    word_32b_u          val[26];
//    com_cpu_2_cm_t

    dbg_uart_addr_t     addr;
    counter_u           counter;
    config_u            cfg_lst;
    command_u           cmd_lst;
    uint8_t             msg[200];
    uint8_t             msg_size;
    bool                valid;
} com_dbg_tx_msg_t;

typedef struct __com_dbg_rx_msg_t__
{
    word_32b_u          cmd_msg[32];
    com_dbg_rx_state_e  state;
    counter_u           counter;
    config_u            cfg_lst;
    command_u           cmd_lst;
    uint8_t             msgType;
    uint8_t             crc;
    bool                valid;
} com_dbg_rx_msg_t;

#endif

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
#ifdef CPU1
bool_t COM_msgExtract(mst2slv_msg_t*,  cmd_t*, cmd_t*);
bool_t COM_msgExtract_cla(mst2slv_msg_cla_t*, cmd_t*, cmd_t*);
void COM_msgCreate(motor_t*, motor_t*, slv2mst_msg_t*);
void COM_resetCmdStruct(cmd_t*);
void COM_resetMsgExtractStruct(mst2slv_msg_cla_t*);
uint32_t COM_crc32(uint16_t*, uint16_t);
uint32_t COM_crc32_cla(mst2slv_msg_cla_t*);
#else
void COM_msgDbgTx(dbg_uart_addr_t*, dbg_uart_msg_t*);
//void COM_msgDbgTx(com_dbg_tx_msg_t*, uint8_t);
void COM_msgDbgRx(com_dbg_rx_msg_t*, uint8_t);
uint32_t COM_crc32(uint16_t*, size_t);
uint8_t COM_crc8(uint8_t*, size_t);
//uint8_t com_crc8_fast(uint8_t*, size_t);
uint8_t COM_crc8Fast(uint8_t, uint8_t*, size_t);
#endif

#endif
