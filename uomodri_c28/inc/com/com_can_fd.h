#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#ifdef CPU1
#include "f2838x_device.h"
#include "motor_ctrl.h"
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

#define COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH     (sizeof(slv2mst_msg_t))
#define COM_MSG_SPI_TX_16BIT_FULL_LENGTH        (COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH + sizeof(crc_reg_t))

#define COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH     (sizeof(mst2slv_msg_t))
#define COM_MSG_SPI_RX_16BIT_FULL_LENGTH        (COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH + sizeof(crc_reg_t))

#define COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH   (sizeof(header_reg_u) + sizeof(slv2mst_msg_t))
#define COM_MSG_RS485_TX_8BIT_PAYLOAD_LENGTH    (COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH * 2)
#define COM_MSG_RS485_TX_16BIT_FULL_LENGTH      (COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH + sizeof(crc_reg_t))
#define COM_MSG_RS485_TX_8BIT_FULL_LENGTH       (COM_MSG_RS485_TX_16BIT_FULL_LENGTH * 2)

#define COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH   (sizeof(header_reg_u) + sizeof(mst2slv_msg_t))
#define COM_MSG_RS485_RX_8BIT_PAYLOAD_LENGTH    (COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH * 2)
#define COM_MSG_RS485_RX_16BIT_FULL_LENGTH      (COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH + sizeof(crc_reg_t))
#define COM_MSG_RS485_RX_8BIT_FULL_LENGTH       (COM_MSG_RS485_RX_16BIT_FULL_LENGTH * 2)

#define LED_MSG_TX_16BIT_LENGTH                 (16)

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

/***********************************************************************
 * COMMUNICATION BUS IDENTIFIER
 ***********************************************************************/
/**
 * @enum    com_bus_e
 * @brief   Define which peripheral bus has been used for communication.
 */
typedef enum __com_bus_enum__
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
 * @union   U32toU16_reg_u
 * @brief   32bits to 16bits type down conversion.
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
        uint16_t    ERROR_CODE  : 4;    /*!< Bits 3-0   : Timeout disable system after no valid command         */
        uint16_t    STATUS_RSV1 : 3;    /*!< Bits 6-4   : Reserved                                              */
        uint16_t    IDX2T       : 1;    /*!< Bits 7     : incremental encoder toggle on index detect - MOTOR_2  */
        uint16_t    IDX1T       : 1;    /*!< Bits 8     : incremental encoder toggle on index detect - MOTOR_1  */
        uint16_t    IDX2D       : 1;    /*!< Bits 9     : incremental encoder index detected - MOTOR_2          */
        uint16_t    IDX1D       : 1;    /*!< Bits 10    : incremental encoder index detected - MOTOR_1          */
        uint16_t    M2R         : 1;    /*!< Bits 11    : MOTOR_2 ready/not ready                               */
        uint16_t    M2E         : 1;    /*!< Bits 12    : MOTOR_2 enabled/not enabled                           */
        uint16_t    M1R         : 1;    /*!< Bits 13    : MOTOR_1 ready/not ready                               */
        uint16_t    M1E         : 1;    /*!< Bits 14    : MOTOR_1 enabled/not enabled                           */
        uint16_t    SE          : 1;    /*!< Bits 15    : System enabled/disabled                               */
    };
} slv2mst_reg_u;

/**
 * @union   header_reg_u
 * @brief   Slave to Master header structure for RS485 communication.
 */
typedef union __header_reg_u__
{
    uint16_t        all;        /*!< Short (16bits) type access */
    struct
    {
        uint16_t    size : 6;   /*!< Bits 5-0   : R/W - Number of elements in the message expressed in bytes.   */
        uint16_t    type : 2;   /*!< Bits 7-6   : R/W - Type of message (TBD).                                  */
        uint16_t    addr : 8;   /*!< Bits 15-8  : R/W - Target address                                          */
    };
} header_reg_u;

/***********************************************************************
 * MAIN COMMUNICATION STRUCTURES
 ***********************************************************************/
/**
 * @struct  pos_reg_t
 *  @brief  32bits position value managed as 2x16bits registers.
 */
typedef U32toU16_reg_t pos_reg_t;

/**
 * @struct  crc_reg_t
 *  @brief  32bits CRC value managed as 2x16bits registers.
 */
typedef U32toU16_reg_t crc_reg_t;

/**
 * @struct  mst2slv_msg_t
 * @brief   Master to Slave full command message including CRC for uOmodri.
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
 * @brief   Slave to master full status message including CRC from uOmodri.
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
 * @brief   uOmodri internal structure command including mst2slv_msg + info status.
 */
#ifndef CPU1
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
#else
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
#endif

/***********************************************************************
 * COMMUNICATION FOR SPI
 ***********************************************************************/
/**
 * @struct  spi_rx_msg_t
 * @brief   SPI reception message structure. Separating received values and CRC to be computed.
 */
typedef struct __spi_rx_msg_t__
{
    mst2slv_msg_t   data;
    crc_reg_t       crc;
} spi_rx_msg_t;

/**
 * @struct  spi_tx_msg_t
 * @brief   SPI transmission message structure. Separating values to transmit and computed CRC.
 */
typedef struct __spi_tx_msg_t__
{
    slv2mst_msg_t   data;
    crc_reg_t       crc;
} spi_tx_msg_t;

/**
 * @struct  spi_rx_t
 * @brief   SPI global reception structure.
 */
typedef struct __spi_rx_t__
{
    spi_rx_msg_t    msg;
} spi_rx_t;

/**
 * @struct  spi_tx_t
 * @brief   SPI global transmission structure.
 */
typedef struct __spi_tx_t__
{
    spi_tx_msg_t    msg[2];
    uint16_t        msg_cr;
    uint16_t        msg_nx;
} spi_tx_t;

/***********************************************************************
 * COMMUNICATION FOR RS485
 ***********************************************************************/
/**
 * @union   rs485_tx_msg_u
 * @brief   RS485 reception message structure. Separating header, received values and CRC to be computed.
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
 * @brief   RS485 transmission message structure. Separating header and values to transmit plus computed CRC.
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
 * @brief   RS485 global reception structure.
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
 * @brief   RS485 global transmission structure.
 */
typedef struct __rs485_tx_t__
{
    rs485_tx_msg_u      msg[2];
    uint16_t            index;
    uint16_t            msg_cr;
    uint16_t            msg_nx;
} rs485_tx_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
#ifndef CPU1
void COM_msgDbgTx(dbg_uart_addr_t*, dbg_uart_msg_t*);
void COM_msgDbgRx(com_dbg_rx_msg_t*, uint8_t);
uint32_t COM_crc32(uint16_t*, size_t);
uint8_t COM_crc8(uint8_t*, size_t);
uint8_t COM_crc8Fast(uint8_t, uint8_t*, size_t);
#else
void COM_msgExtract(mst2slv_msg_t*, cmd_t*, cmd_t*);
bool_t COM_msgCreate(motor_t*, slv2mst_msg_t*, uint32_t);
void COM_initCmdStruct(cmd_t*);
uint32_t COM_msgCRC(uint16_t*, uint16_t);
inline void RS485_msgReceive(uint32_t, rs485_rx_t*);
inline void RS485_msgTransmit(uint32_t, rs485_tx_t*);
//void SCI_rxDisable(uint32_t);
//void SCI_rxEnable(uint32_t);
//void SCI_txDisable(uint32_t);
//void SCI_txEnable(uint32_t);
#endif

#endif /*__COMMUNICATION_H__*/
