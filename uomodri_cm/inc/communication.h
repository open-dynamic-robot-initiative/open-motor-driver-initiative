#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#ifdef CPU1
//#include <stdbool.h>
//#include <stddef.h>
//#include <stdint.h>
#include "f2838x_device.h"
//#include "motor.h"
#include "uomodri_shared.h"
//#include "uomodri_user_defines.h"
#else
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <stddef.h>
#include "uomodri_cm_user_defines.h"
#endif

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define POLYNOMIAL_32_P1                        (0x04C11DB7)
#define CRC_INIT_CRC32                          (0xFFFFFFFF)

#define MSG_RX_ES_POS                           (15)
#define MSG_RX_ES_MASK                          (1U << MSG_RX_ES_POS)
#define MSG_RX_EM1_POS                          (14)
#define MSG_RX_EM1_MASK                         (1U << MSG_RX_EM1_POS)
#define MSG_RX_EM2_POS                          (13)
#define MSG_RX_EM2_MASK                         (1U << MSG_RX_EM2_POS)
#define MSG_RX_EPRE_POS                         (12)
#define MSG_RX_EPRE_MASK                        (1U << MSG_RX_EPRE_POS)
#define MSG_RX_EI1OC_POS                        (11)
#define MSG_RX_EI1OC_MASK                       (1U << MSG_RX_EI1OC_POS)
#define MSG_RX_EI2OC_POS                        (10)
#define MSG_RX_EI2OC_MASK                       (1U << MSG_RX_EI2OC_POS)
#define MSG_RX_TIMEOUT_POS                      (0)
#define MSG_RX_TIMEOUT_MASK                     (0xFFU << MSG_RX_TIMEOUT_POS)

#define MSG_RX_ES                               (0x8000)
#define MSG_RX_EM1                              (0x4000)
#define MSG_RX_EM2                              (0x2000)
#define MSG_RX_EPRE                             (0x1000)
#define MSG_RX_EI1OC                            (0x0800)
#define MSG_RX_EI2OC                            (0x0400)
#define MSG_RX_TIMEOUT                          (0x00FF)

#define STATUS_ERROR_NO_ERROR                   (0) /*!< No error */
#define STATUS_ERROR_ENCODER1                   (1) /*!< Encoder_1 error too high */
#define STATUS_ERROR_SPI_RECV_TIMEOUT           (2) /*!< Timeout for receiving current references exceeded */
#define STATUS_ERROR_CRIT_TEMP                  (3) /*!< Motor temperature reached critical valued */
#define STATUS_ERROR_POSCONV                    (4) /*!< Unused error */
#define STATUS_ERROR_POS_ROLLOVER               (5) /*!< Position roll-over occurred */
#define STATUS_ERROR_ENCODER2                   (6) /*!< Encoder_2 error too high */
#define STATUS_ERROR_DRV_FAULT                  (7) /*!< Motor DRV nFault error */
//#define STATUS_ERROR_DRV1_FAULT                     (7) /*!< Motor1 DRV Fault error */
//#define STATUS_ERROR_DRV2_FAULT                     (8) /*!< Motor2 DRV Fault error */

#define COM_MSG_EXTRACT_TX_RX_MAX_SIZE          (64)
#define COM_MSG_RS485_TX_RX_MAX_SIZE            (20)
#define COM_MSG_RS485_RX_ADDRESS_1              (0x42)
#define COM_MSG_RS485_RX_ADDRESS_2              (0x00)
#define COM_MSG_RS485_TX_ADDRESS                (0xFF)
#define COM_MSG_RS485_TX_TYPE_STANDARD          (0x00)

#define COM_MSG_SPI_TX_16BIT_FULL_LENGTH        (sizeof(slv2mst_msg_t) + \
                                                 sizeof(crc_reg_t))
#define COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH     (sizeof(slv2mst_msg_t))
#define COM_MSG_SPI_TX_8BIT_FULL_LENGTH         (COM_MSG_SPI_TX_16BIT_FULL_LENGTH * 2)
#define COM_MSG_SPI_TX_8BIT_PAYLOAD_LENGTH      (COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH * 2)

#define COM_MSG_SPI_RX_16BIT_FULL_LENGTH        (sizeof(mst2slv_msg_t) + \
                                                 sizeof(crc_reg_t))
#define COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH     (sizeof(mst2slv_msg_t))
#define COM_MSG_SPI_RX_8BIT_FULL_LENGTH         (COM_MSG_SPI_RX_16BIT_FULL_LENGTH * 2)
#define COM_MSG_SPI_RX_8BIT_PAYLOAD_LENGTH      (COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH * 2)

#define COM_MSG_RS485_TX_16BIT_FULL_LENGTH      (sizeof(header_reg_u) + \
                                                 sizeof(slv2mst_msg_t) + \
                                                 sizeof(crc_reg_t))
#define COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH   (sizeof(header_reg_u) + \
                                                 sizeof(slv2mst_msg_t))
#define COM_MSG_RS485_TX_8BIT_FULL_LENGTH       (COM_MSG_RS485_TX_16BIT_FULL_LENGTH * 2)
#define COM_MSG_RS485_TX_8BIT_PAYLOAD_LENGTH    (COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH * 2)

#define COM_MSG_RS485_RX_16BIT_FULL_LENGTH      (sizeof(header_reg_u) + \
                                                 sizeof(mst2slv_msg_t) + \
                                                 sizeof(crc_reg_t))
#define COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH   (sizeof(header_reg_u) + \
                                                 sizeof(mst2slv_msg_t))
#define COM_MSG_RS485_RX_8BIT_FULL_LENGTH       (COM_MSG_RS485_RX_16BIT_FULL_LENGTH * 2)
#define COM_MSG_RS485_RX_8BIT_PAYLOAD_LENGTH    (COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH * 2)

#define LED_MSG_TX_16BIT_LENGTH                 (6)

/***********************************************************************
 * COMMUNICATION CONSTANTS
 ***********************************************************************/
#define POSITION_LSB                            (5.960464477539063e-08f)    // 2**(-24)
#define VELOCITY_LSB                            (4.8828125e-04f)            // 2**(-11)
#define IQ_LSB                                  (9.765625e-04f)             // 2**(-10)
#define CURRENT_SAT_LSB                         (1.25e-01f)                 // 2**(-3)
#define RESISTANCE_LSB                          (3.0517578125e-05f)         // 2**(-15)
#define VOLTAGE_LSB                             (6.103515625e-05f)          // 2**(-14)
#define KP_LSB                                  (4.8828125e-04f)            // 2**(-11)
#define KD_LSB                                  (9.765625e-04f)             // 2**(-10)

/***********************************************************************
 * COMMUNICATION CRC TABLE
 ***********************************************************************/
static const uint32_t crc32_table[] =
{
 (uint32_t)0x00000000, (uint32_t)0x04c11db7, (uint32_t)0x09823b6e, (uint32_t)0x0d4326d9, (uint32_t)0x130476dc, (uint32_t)0x17c56b6b, (uint32_t)0x1a864db2, (uint32_t)0x1e475005,
 (uint32_t)0x2608edb8, (uint32_t)0x22c9f00f, (uint32_t)0x2f8ad6d6, (uint32_t)0x2b4bcb61, (uint32_t)0x350c9b64, (uint32_t)0x31cd86d3, (uint32_t)0x3c8ea00a, (uint32_t)0x384fbdbd,
 (uint32_t)0x4c11db70, (uint32_t)0x48d0c6c7, (uint32_t)0x4593e01e, (uint32_t)0x4152fda9, (uint32_t)0x5f15adac, (uint32_t)0x5bd4b01b, (uint32_t)0x569796c2, (uint32_t)0x52568b75,
 (uint32_t)0x6a1936c8, (uint32_t)0x6ed82b7f, (uint32_t)0x639b0da6, (uint32_t)0x675a1011, (uint32_t)0x791d4014, (uint32_t)0x7ddc5da3, (uint32_t)0x709f7b7a, (uint32_t)0x745e66cd,
 (uint32_t)0x9823b6e0, (uint32_t)0x9ce2ab57, (uint32_t)0x91a18d8e, (uint32_t)0x95609039, (uint32_t)0x8b27c03c, (uint32_t)0x8fe6dd8b, (uint32_t)0x82a5fb52, (uint32_t)0x8664e6e5,
 (uint32_t)0xbe2b5b58, (uint32_t)0xbaea46ef, (uint32_t)0xb7a96036, (uint32_t)0xb3687d81, (uint32_t)0xad2f2d84, (uint32_t)0xa9ee3033, (uint32_t)0xa4ad16ea, (uint32_t)0xa06c0b5d,
 (uint32_t)0xd4326d90, (uint32_t)0xd0f37027, (uint32_t)0xddb056fe, (uint32_t)0xd9714b49, (uint32_t)0xc7361b4c, (uint32_t)0xc3f706fb, (uint32_t)0xceb42022, (uint32_t)0xca753d95,
 (uint32_t)0xf23a8028, (uint32_t)0xf6fb9d9f, (uint32_t)0xfbb8bb46, (uint32_t)0xff79a6f1, (uint32_t)0xe13ef6f4, (uint32_t)0xe5ffeb43, (uint32_t)0xe8bccd9a, (uint32_t)0xec7dd02d,
 (uint32_t)0x34867077, (uint32_t)0x30476dc0, (uint32_t)0x3d044b19, (uint32_t)0x39c556ae, (uint32_t)0x278206ab, (uint32_t)0x23431b1c, (uint32_t)0x2e003dc5, (uint32_t)0x2ac12072,
 (uint32_t)0x128e9dcf, (uint32_t)0x164f8078, (uint32_t)0x1b0ca6a1, (uint32_t)0x1fcdbb16, (uint32_t)0x018aeb13, (uint32_t)0x054bf6a4, (uint32_t)0x0808d07d, (uint32_t)0x0cc9cdca,
 (uint32_t)0x7897ab07, (uint32_t)0x7c56b6b0, (uint32_t)0x71159069, (uint32_t)0x75d48dde, (uint32_t)0x6b93dddb, (uint32_t)0x6f52c06c, (uint32_t)0x6211e6b5, (uint32_t)0x66d0fb02,
 (uint32_t)0x5e9f46bf, (uint32_t)0x5a5e5b08, (uint32_t)0x571d7dd1, (uint32_t)0x53dc6066, (uint32_t)0x4d9b3063, (uint32_t)0x495a2dd4, (uint32_t)0x44190b0d, (uint32_t)0x40d816ba,
 (uint32_t)0xaca5c697, (uint32_t)0xa864db20, (uint32_t)0xa527fdf9, (uint32_t)0xa1e6e04e, (uint32_t)0xbfa1b04b, (uint32_t)0xbb60adfc, (uint32_t)0xb6238b25, (uint32_t)0xb2e29692,
 (uint32_t)0x8aad2b2f, (uint32_t)0x8e6c3698, (uint32_t)0x832f1041, (uint32_t)0x87ee0df6, (uint32_t)0x99a95df3, (uint32_t)0x9d684044, (uint32_t)0x902b669d, (uint32_t)0x94ea7b2a,
 (uint32_t)0xe0b41de7, (uint32_t)0xe4750050, (uint32_t)0xe9362689, (uint32_t)0xedf73b3e, (uint32_t)0xf3b06b3b, (uint32_t)0xf771768c, (uint32_t)0xfa325055, (uint32_t)0xfef34de2,
 (uint32_t)0xc6bcf05f, (uint32_t)0xc27dede8, (uint32_t)0xcf3ecb31, (uint32_t)0xcbffd686, (uint32_t)0xd5b88683, (uint32_t)0xd1799b34, (uint32_t)0xdc3abded, (uint32_t)0xd8fba05a,
 (uint32_t)0x690ce0ee, (uint32_t)0x6dcdfd59, (uint32_t)0x608edb80, (uint32_t)0x644fc637, (uint32_t)0x7a089632, (uint32_t)0x7ec98b85, (uint32_t)0x738aad5c, (uint32_t)0x774bb0eb,
 (uint32_t)0x4f040d56, (uint32_t)0x4bc510e1, (uint32_t)0x46863638, (uint32_t)0x42472b8f, (uint32_t)0x5c007b8a, (uint32_t)0x58c1663d, (uint32_t)0x558240e4, (uint32_t)0x51435d53,
 (uint32_t)0x251d3b9e, (uint32_t)0x21dc2629, (uint32_t)0x2c9f00f0, (uint32_t)0x285e1d47, (uint32_t)0x36194d42, (uint32_t)0x32d850f5, (uint32_t)0x3f9b762c, (uint32_t)0x3b5a6b9b,
 (uint32_t)0x0315d626, (uint32_t)0x07d4cb91, (uint32_t)0x0a97ed48, (uint32_t)0x0e56f0ff, (uint32_t)0x1011a0fa, (uint32_t)0x14d0bd4d, (uint32_t)0x19939b94, (uint32_t)0x1d528623,
 (uint32_t)0xf12f560e, (uint32_t)0xf5ee4bb9, (uint32_t)0xf8ad6d60, (uint32_t)0xfc6c70d7, (uint32_t)0xe22b20d2, (uint32_t)0xe6ea3d65, (uint32_t)0xeba91bbc, (uint32_t)0xef68060b,
 (uint32_t)0xd727bbb6, (uint32_t)0xd3e6a601, (uint32_t)0xdea580d8, (uint32_t)0xda649d6f, (uint32_t)0xc423cd6a, (uint32_t)0xc0e2d0dd, (uint32_t)0xcda1f604, (uint32_t)0xc960ebb3,
 (uint32_t)0xbd3e8d7e, (uint32_t)0xb9ff90c9, (uint32_t)0xb4bcb610, (uint32_t)0xb07daba7, (uint32_t)0xae3afba2, (uint32_t)0xaafbe615, (uint32_t)0xa7b8c0cc, (uint32_t)0xa379dd7b,
 (uint32_t)0x9b3660c6, (uint32_t)0x9ff77d71, (uint32_t)0x92b45ba8, (uint32_t)0x9675461f, (uint32_t)0x8832161a, (uint32_t)0x8cf30bad, (uint32_t)0x81b02d74, (uint32_t)0x857130c3,
 (uint32_t)0x5d8a9099, (uint32_t)0x594b8d2e, (uint32_t)0x5408abf7, (uint32_t)0x50c9b640, (uint32_t)0x4e8ee645, (uint32_t)0x4a4ffbf2, (uint32_t)0x470cdd2b, (uint32_t)0x43cdc09c,
 (uint32_t)0x7b827d21, (uint32_t)0x7f436096, (uint32_t)0x7200464f, (uint32_t)0x76c15bf8, (uint32_t)0x68860bfd, (uint32_t)0x6c47164a, (uint32_t)0x61043093, (uint32_t)0x65c52d24,
 (uint32_t)0x119b4be9, (uint32_t)0x155a565e, (uint32_t)0x18197087, (uint32_t)0x1cd86d30, (uint32_t)0x029f3d35, (uint32_t)0x065e2082, (uint32_t)0x0b1d065b, (uint32_t)0x0fdc1bec,
 (uint32_t)0x3793a651, (uint32_t)0x3352bbe6, (uint32_t)0x3e119d3f, (uint32_t)0x3ad08088, (uint32_t)0x2497d08d, (uint32_t)0x2056cd3a, (uint32_t)0x2d15ebe3, (uint32_t)0x29d4f654,
 (uint32_t)0xc5a92679, (uint32_t)0xc1683bce, (uint32_t)0xcc2b1d17, (uint32_t)0xc8ea00a0, (uint32_t)0xd6ad50a5, (uint32_t)0xd26c4d12, (uint32_t)0xdf2f6bcb, (uint32_t)0xdbee767c,
 (uint32_t)0xe3a1cbc1, (uint32_t)0xe760d676, (uint32_t)0xea23f0af, (uint32_t)0xeee2ed18, (uint32_t)0xf0a5bd1d, (uint32_t)0xf464a0aa, (uint32_t)0xf9278673, (uint32_t)0xfde69bc4,
 (uint32_t)0x89b8fd09, (uint32_t)0x8d79e0be, (uint32_t)0x803ac667, (uint32_t)0x84fbdbd0, (uint32_t)0x9abc8bd5, (uint32_t)0x9e7d9662, (uint32_t)0x933eb0bb, (uint32_t)0x97ffad0c,
 (uint32_t)0xafb010b1, (uint32_t)0xab710d06, (uint32_t)0xa6322bdf, (uint32_t)0xa2f33668, (uint32_t)0xbcb4666d, (uint32_t)0xb8757bda, (uint32_t)0xb5365d03, (uint32_t)0xb1f740b4
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
 * COMMUNICATION BUS IDENTIFIER
 ***********************************************************************/
/**
 * @enum    com_bus_e
 * @brief   Define which peripheral bus has been used for communication.
 */
typedef enum
{
    COM_NO_BUS_DEFINED  = 0,
    COM_USE_SPI_BUS     = 1,
    COM_USE_RS485_BUS   = 2,
} com_bus_e;

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
 * COMMUNICATION UNIONS BIT FILEDS
 ***********************************************************************/
/**
 * @union  U32toU16_reg_u
 * @brief   32bits to 16bits down conversion.
 */
typedef struct __U32toU16_reg_t__
{
    uint16_t        u16_msb;
    uint16_t        u16_lsb;
} U32toU16_reg_t;

/**
 * @union   mst2slv_reg_u
 * @brief   Master to Slave command union (bits definition) for uOmodri. Access to bit field.
 */
typedef union __mst2slv_reg_u__
{
    uint16_t        all;                /*!< Short (16bits) type access */
    struct
    {
        uint16_t    TIMEOUT     : 8;    /*!< Bits 7-0   : Timeout disable system after no valid command in milliseconds           */
        uint16_t    CMD_RSV1    : 2;    /*!< Bits 9-8   : Reserved                                                                */
        uint16_t    EI2OC       : 1;    /*!< Bits 10    : Enable/Disable incremental encoder index offset compensation - MOTOR_2  */
        uint16_t    EI1OC       : 1;    /*!< Bits 11    : Enable/Disable incremental encoder index offset compensation - MOTOR_1  */
        uint16_t    EPRE        : 1;    /*!< Bits 12    : Enable/Disable position rollover error - position range : -128 to +128  */
        uint16_t    EM2         : 1;    /*!< Bits 13    : Enable(calibration procedure, alignment \& run)/Disable - MOTOR_2       */
        uint16_t    EM1         : 1;    /*!< Bits 14    : Enable(calibration procedure, alignment \& run)/Disable - MOTOR_1       */
        uint16_t    ES          : 1;    /*!< Bits 15    : Enable/Disable System                                                   */
    };
} mst2slv_reg_u;

/**
 * @union   slv2mst_reg_u
 * @brief   Slave to Master status structure (bits definition) from uOmodri. Access to bit field.
 */
typedef union __slv2mst_reg_u__
{
    uint16_t        all;                /*!< Short (16bits) type access */
    struct
    {
        uint16_t    ERROR_CODE  : 4;    /*!< Bits 3-0   : Timeout disable system after no valid command               */
        uint16_t    STATUS_RSV1 : 3;    /*!< Bits 6-4   : Reserved                                                    */
        uint16_t    IDX2T       : 1;    /*!< Bits 7     : incremental encoder status toggle on index detect - MOTOR_2 */
        uint16_t    IDX1T       : 1;    /*!< Bits 8     : incremental encoder status toggle on index detect - MOTOR_1 */
        uint16_t    IDX2D       : 1;    /*!< Bits 9     : incremental encoder status index detected - MOTOR_2         */
        uint16_t    IDX1D       : 1;    /*!< Bits 10    : incremental encoder status index detected - MOTOR_1         */
        uint16_t    M2R         : 1;    /*!< Bits 11    : MOTOR_2 ready/not ready                                     */
        uint16_t    M2E         : 1;    /*!< Bits 12    : MOTOR_2 enabled/not enabled                                 */
        uint16_t    M1R         : 1;    /*!< Bits 13    : MOTOR_1 ready/not ready                                     */
        uint16_t    M1E         : 1;    /*!< Bits 14    : MOTOR_1 enabled/not enabled                                 */
        uint16_t    SE          : 1;    /*!< Bits 15    : System enabled/disabled                                     */
    };
} slv2mst_reg_u;

/**
 * @union   header_reg_u
 * @brief   Slave to Master header structure for RS485 communication.
 */
typedef union __header_reg_u__
{
    uint16_t        all;
    struct
    {
        uint16_t    size : 6;   /*!< Bits 5-0   : R/W - Number message elements expressed in bytes.  */
        uint16_t    type : 2;   /*!< Bits 7-6   : R/W - Type of message (TBD).                       */
        uint16_t    addr : 8;   /*!< Bits 15-8  : R/W - Target address                               */
    };
} header_reg_u;

/***********************************************************************
 * MAIN COMMUNICATION STRUCTURES
 ***********************************************************************/
/**
 * @struct  pos_reg_t
 *  @brief  32bits position value managed as 2x 16bits registers.
 */
typedef U32toU16_reg_t pos_reg_t;

/**
 * @struct  crc_reg_t
 *  @brief  32bits CRC value managed as 2x 16bits registers.
 */
typedef U32toU16_reg_t crc_reg_t;

/**
 * @struct  mst2slv_msg_t
 * @brief   Master 2 Slave full command message including CRC for uOmodri
 */
typedef struct __mst2slv_msg_t__
{
    mst2slv_reg_u   mode;
    pos_reg_t       position[2];
    int16_t         velocity[2];
    int16_t         current[2];
    uint16_t        kpCoeff[2];
    uint16_t        kdCoeff[2];
    uint16_t        iSat;
    uint16_t        index;
} mst2slv_msg_t;

/**
 * @struct  slv2mst_msg_t
 * @brief   Slave to master full status message including CRC from uOmodri
 */
typedef struct __slv2mst_msg_t__
{
    slv2mst_reg_u   status;
    uint16_t        timeStamp;
    pos_reg_t       position[2];
    int16_t         velocity[2];
    int16_t         current[2];
    uint16_t        coilRes[2];
    uint16_t        adcSamples[2];
    uint16_t        lastCmdIndex;
} slv2mst_msg_t;

/**
 * @struct  cmd_uomodri_t
 * @brief   Structure of all commands necessaries for uOmodri
 */
#ifdef CPU1
typedef struct __cmd_uomodri_t__
{
    mst2slv_reg_u   mode;
    float32_t       posRef[2];
    float32_t       velRef[2];
    float32_t       iqRef[2];
    float32_t       kpCoeff[2];
    float32_t       kdCoeff[2];
    float32_t       iSat[2];
    uint32_t        cptTimeout;
    uint16_t        index;
} cmd_uomodri_t;
#else
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
} cmd_uomodri_t;
#endif

/***********************************************************************
 * COMMUNICATION FOR SPI
 ***********************************************************************/
/**
 * @struct  spi_rx_msg_t
 * @brief   SPI reception message structure.
 */
typedef struct __spi_rx_msg_t__
{
    mst2slv_msg_t   data;
    crc_reg_t       crc;
} spi_rx_msg_t;

/**
 * @struct  spi_tx_msg_t
 * @brief   SPI transmission message structure.
 */
typedef struct __spi_tx_msg_t__
{
    slv2mst_msg_t   data;
    crc_reg_t       crc;
} spi_tx_msg_t;

/**
 * @struct  spi_rx_t
 * @brief   SPI global reception structure
 */
typedef struct __spi_rx_t__
{
    spi_rx_msg_t    msg;
} spi_rx_t;

/**
 * @struct  spi_tx_t
 * @brief   SPI global transmission structure
 */
typedef struct __spi_tx_t__
{
    spi_tx_msg_t    msg[2];
    uint16_t        msg_cr;
    uint16_t        msg_nx;
//    struct
//    {
//        uint16_t    msg_cr  : 1;
//        uint16_t    msg_nx  : 1;
//        uint16_t    msg_RSV :14;
//    };
} spi_tx_t;

/***********************************************************************
 * COMMUNICATION FOR RS485
 ***********************************************************************/
/**
 * @union   rs485_tx_msg_u
 * @brief   RS485 reception message structure
 */
typedef union __rs485_rx_msg_u__
{
    uint16_t            raw[COM_MSG_RS485_TX_RX_MAX_SIZE];
    struct
    {
        header_reg_u    header;
        mst2slv_msg_t   data;
        crc_reg_t       crc;
    };
} rs485_rx_msg_u;

/**
 * @union   rs485_tx_msg_u
 * @brief   RS485 transmission message structure
 */
typedef union __rs485_tx_msg_u__
{
    uint16_t            raw[COM_MSG_RS485_TX_RX_MAX_SIZE];
    struct
    {
        header_reg_u    header;
        slv2mst_msg_t   data;
        crc_reg_t       crc;
    };
} rs485_tx_msg_u;

/**
 * @struct  rs485_rx_t
 * @brief   RS485 global reception structure
 */
typedef struct __rs485_rx_t__
{
    rs485_rx_msg_u      msg;
    uint16_t            index;
    struct
    {
        uint16_t        addr1   : 1;
        uint16_t        addr2   : 1;
        uint16_t        rsv     : 14;
    };
} rs485_rx_t;

/**
 * @struct  rs485_tx_t
 * @brief   RS485 global transmission structure
 */
typedef struct __rs485_tx_t__
{
    rs485_tx_msg_u      msg[2];
    uint16_t            index;
    uint16_t            msg_cr;
    uint16_t            msg_nx;
//    struct
//    {
//        uint16_t    msg_cr  : 1;
//        uint16_t    msg_nx  : 1;
//        uint16_t    msg_RSV :14;
//    };
} rs485_tx_t;

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
//bool_t COM_msgExtract(uint32_t, mst2slv_msg_t*, cmd_t*, cmd_t*);
void COM_msgExtract(mst2slv_msg_t*, cmd_t*, cmd_t*);
bool_t COM_msgCreate(motor_t*, slv2mst_msg_t*, uint32_t);
void COM_msgSwapBytes(uint16_t*, uint16_t*, uint16_t);
void COM_initCmdStruct(cmd_t*);
uint32_t COM_vCRC32(uint16_t*, uint16_t);
uint32_t COM_msgCRC(uint16_t*, uint16_t);
inline void RS485_msgReceiveInit(rs485_rx_t*);
inline void RS485_msgReceive(uint32_t, rs485_rx_t*);
void RS485_msgTransmitInit(rs485_tx_t*);
void RS485_msgTransmit(uint32_t, rs485_tx_t*);
void SCI_rxDisable(uint32_t);
void SCI_rxEnable(uint32_t);
void SCI_txDisable(uint32_t);
void SCI_txEnable(uint32_t);
#else
void COM_msgDbgTx(dbg_uart_addr_t*, dbg_uart_msg_t*);
void COM_msgDbgRx(com_dbg_rx_msg_t*, uint8_t);
uint32_t COM_crc32(uint16_t*, size_t);
uint8_t COM_crc8(uint8_t*, size_t);
uint8_t COM_crc8Fast(uint8_t, uint8_t*, size_t);
#endif

#endif /*__COMMUNICATION_H__*/
