/*
 * File name: communication_cm.c
 * Description: Source file containing extraction & build/update message functions
 */

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <strings.h>
#include "communication.h"
#include "uomodri_cm_user_defines.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define LSB_32(x)           ((x) & 0x0000FFFF)
#define MSB_32(x)           (((x) >> 16) & 0x0000FFFF)
#define LSB_16(x)           ((x) & 0x00FF)
#define MSB_16(x)           (((x) >> 8) & 0x00FF)
#define MSG_CFG             (0x01)
#define MSG_CMD             (0x02)
#define MSG_CMD_LIST_MASK   (0x07FF07FF)
#define PD_KP_MASK          (0x0001)
#define PD_KI_MASK          (0x0002)
#define POS_REF_MASK        (0x0004)
#define VEL_REF_MASK        (0x0008)
#define IQ_FF_MASK          (0x0010)
#define ISAT_MASK           (0x0020)
#define CMD_MASK            (0x0040)
#define PI_KP_ID_MASK       (0x0080)
#define PI_KI_ID_MASK       (0x0100)
#define PI_KP_IQ_MASK       (0x0200)
#define PI_KI_IQ_MASK       (0x0400)
#define PD_KP_POS           (0)
#define PD_KI_POS           (1)
#define POS_REF_POS         (2)
#define VEL_REF_POS         (3)
#define IQ_FF_POS           (4)
#define ISAT_POS            (5)
#define CMD_POS             (6)
#define PI_KP_ID_POS        (7)
#define PI_KI_ID_POS        (8)
#define PI_KP_IQ_POS        (9)
#define PI_KI_IQ_POS        (10)

typedef enum
{
    COM_DBG_RX_WORD_A   = 0,    //!< COM_RX_STATE_MODE
    COM_DBG_RX_WORD_B   = 1,
    COM_DBG_RX_CNT      = 2,
    COM_DBG_RX_MSG_TYPE = 3,
    COM_DBG_RX_CFG_LIST_4_TX = 4,
    COM_DBG_RX_CMD_LIST_4_RX = 5,
    COM_DBG_RX_CMD_MSG  = 7,
    COM_DBG_RX_CRC      = 8,
} com_dbg_rx_state_e;

typedef enum
{
    m1_pd_kp            = 0,
    m1_pd_ki            = 1,
    m1_posref           = 2,
    m1_velref           = 3,
    m1_iqff             = 4,
    m1_isat             = 5,
    m1_cmd              = 6,
    m1_pi_kp_id         = 7,
    m1_pi_ki_id         = 8,
    m1_pi_kp_iq         = 9,
    m1_pi_ki_iq         = 10,
    DISP_RSV1_0         = 11,
    DISP_RSV1_1         = 12,
    DISP_RSV1_2         = 13,
    DISP_RSV1_3         = 14,
    DISP_RSV1_4         = 15,
    m2_pd_kp            = 16,
    m2_pd_ki            = 17,
    m2_posref           = 18,
    m2_velref           = 19,
    m2_iqff             = 20,
    m2_isat             = 21,
    m2_cmd              = 22,
    m2_pi_kp_id         = 23,
    m2_pi_ki_id         = 24,
    m2_pi_kp_iq         = 25,
    m2_pi_ki_iq         = 26,
    DISP_RSV2_0         = 27,
    DISP_RSV2_1         = 28,
    DISP_RSV2_2         = 29,
    DISP_RSV2_3         = 30,
    DISP_RSV2_4         = 31,
} com_dbg_rx_command_bit_mask_e;

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
    uint64_t        DISP_RSV1   : 7;    /*!< Bit 25-31  : R/W - Reserved                                            */
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
    uint64_t        DISP_RSV2   : 7;    /*!< Bit 57-31  : R/W - Reserved                                            */
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

typedef union __aggregate_32b_u__
{
    float_t         float_32b;
    uint32_t        uint_32b;
    uint8_t         byte[4];    /*!< */
} aggregate_32b_u;

typedef struct __bit_command_mem_t__
{
    aggregate_32b_u m1_pd_kp;
    aggregate_32b_u m1_pd_ki;
    aggregate_32b_u m1_posref;
    aggregate_32b_u m1_velref;
    aggregate_32b_u m1_iqff;
    aggregate_32b_u m1_isat;
    aggregate_32b_u m1_cmd;
    aggregate_32b_u m1_pi_kp_id;
    aggregate_32b_u m1_pi_ki_id;
    aggregate_32b_u m1_pi_kp_iq;
    aggregate_32b_u m1_pi_ki_iq;
    aggregate_32b_u m2_pd_kp;
    aggregate_32b_u m2_pd_ki;
    aggregate_32b_u m2_posref;
    aggregate_32b_u m2_velref;
    aggregate_32b_u m2_iqff;
    aggregate_32b_u m2_isat;
    aggregate_32b_u m2_cmd;
    aggregate_32b_u m2_pi_kp_id;
    aggregate_32b_u m2_pi_ki_id;
    aggregate_32b_u m2_pi_kp_iq;
    aggregate_32b_u m2_pi_ki_iq;
} bit_command_mem_t;
//typedef struct __bit_command_mem_t__
//{
//    aggregate_32b_u pd_kp;
//    aggregate_32b_u pd_ki;
//    aggregate_32b_u posref;
//    aggregate_32b_u velref;
//    aggregate_32b_u iqff;
//    aggregate_32b_u isat;
//    aggregate_32b_u cmd;
//    aggregate_32b_u pi_kp_id;
//    aggregate_32b_u pi_ki_id;
//    aggregate_32b_u pi_kp_iq;
//    aggregate_32b_u pi_ki_iq;
//} bit_command_mem_t;

typedef struct __com_dbg_rx_msg_t__
{
    aggregate_32b_u     cmd_msg[22];
    com_dbg_rx_state_e  state;
    counter_u           counter;
    config_u            cfg_lst;
    command_u           cmd_lst;
    uint8_t             crc;
//    uint8_t             msg[sizeof(bit_command_mem_t)];
    bool                valid;
} com_dbg_rx_msg_t;

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Message created/updates for debug
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
void com_msgDbgTx(dbg_uart_addr_t* p_addr, dbg_uart_msg_t* p_msg)
{
//    static uint32_t cntMsg  = 0;
//    p_msg->count            = cntMsg++;
//    p_msg->ia               = (float_t)(*p_addr->p_ia);
//    p_msg->ib               = (float_t)(*p_addr->p_ib);
//    p_msg->ic               = (float_t)(*p_addr->p_ic);
//    p_msg->vbus             = (float_t)(*p_addr->p_vbus);
//    p_msg->ialpha           = (float_t)(*p_addr->p_ialpha);
//    p_msg->ibeta            = (float_t)(*p_addr->p_ibeta);
//    p_msg->id               = (float_t)(*p_addr->p_id);
    p_msg->iq               = (float_t)(*p_addr->p_iq);
    p_msg->iqref            = (float_t)(*p_addr->p_iqref);
//    p_msg->ud               = (float_t)(*p_addr->p_ud);
//    p_msg->uq               = (float_t)(*p_addr->p_uq);
//    p_msg->ualpha           = (float_t)(*p_addr->p_ualpha);
//    p_msg->ubeta            = (float_t)(*p_addr->p_ubeta);
//    p_msg->ua               = (float_t)(*p_addr->p_ua);
//    p_msg->ub               = (float_t)(*p_addr->p_ub);
//    p_msg->uc               = (float_t)(*p_addr->p_uc);
    p_msg->pos              = (float_t)(*p_addr->p_pos);
    p_msg->posref           = (float_t)(*p_addr->p_posref);
    p_msg->vel              = (float_t)(*p_addr->p_vel);
    p_msg->velref           = (float_t)(*p_addr->p_velref);
//    p_msg->itcnt            = (uint32_t)(*p_addr->p_itcnt);
    p_msg->crc              = com_crc8_fast(0, (uint8_t *)p_msg, (sizeof(dbg_uart_msg_t) - 1));

    return;
}

/**
 * @brief       Message extract for debug (display \& command)
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
void com_msg_DbgRx(com_dbg_rx_msg_t* p_msg, uint8_t rx_byte)
{
//    static command_u cmd_lst = {.all = 0U};
    static uint32_t cmd_lst = 0;
    static uint8_t  i = 0, j = 0;

    switch(p_msg->state)
    {
    case COM_DBG_RX_WORD_A:
    default:
        i                           = 0;
        j                           = 0;
        cmd_lst                     = 0;
        p_msg->state                = (rx_byte == 'a')  ? (COM_DBG_RX_WORD_B)               : (COM_DBG_RX_WORD_A);
        p_msg->crc                  = (rx_byte == 'a')  ? (com_crc8_fast(0, &rx_byte, 1))   : (0);
        break;

    case COM_DBG_RX_WORD_B:
        p_msg->state                = (rx_byte == 'b')  ? (COM_DBG_RX_CNT)                          : (COM_DBG_RX_WORD_A);
        p_msg->crc                  = (rx_byte == 'b')  ? (com_crc8_fast(p_msg->crc, &rx_byte, 1))  : (0);
        break;

    case COM_DBG_RX_CNT:
        p_msg->counter.byte[i++] = rx_byte;
        p_msg->state                = (i < sizeof(counter_u))   ? (COM_DBG_RX_CNT)  : (COM_DBG_RX_MSG_TYPE);
        i                           = (i < sizeof(counter_u))   ? (i)               : (0);
        p_msg->crc                  = com_crc8_fast(p_msg->crc, &rx_byte, 1);
        break;

     case COM_DBG_RX_MSG_TYPE:
         i                          = 0;
         j                          = 0;
         cmd_lst                    = (rx_byte == MSG_CMD)  ? (p_msg->cmd_lst.all & MSG_CMD_LIST_MASK)  : (0);
         p_msg->state               = (rx_byte == MSG_CMD)  ? (COM_DBG_RX_CMD_MSG)                      : (COM_DBG_RX_WORD_A);
         p_msg->state               = (rx_byte == MSG_CMD) && (!cmd_lst)    ? (COM_DBG_RX_CRC)          : (p_msg->state);
         p_msg->state               = (rx_byte == MSG_CFG)  ? (COM_DBG_RX_CFG_LIST_4_TX)                : (p_msg->state);
         p_msg->crc                 = (rx_byte == MSG_CMD)  ? (com_crc8_fast(p_msg->crc, &rx_byte, 1))  : (p_msg->crc);
         p_msg->crc                 = (rx_byte == MSG_CFG)  ? (com_crc8_fast(p_msg->crc, &rx_byte, 1))  : (0);
         break;

     case COM_DBG_RX_CFG_LIST_4_TX:
         p_msg->cfg_lst.byte[i++]   = rx_byte;
         p_msg->state               = (i < sizeof(config_u))    ? (COM_DBG_RX_CFG_LIST_4_TX)    : (COM_DBG_RX_CMD_LIST_4_RX);
         i                          = (i < sizeof(config_u))    ? (i)                           : (0);
         p_msg->crc                 = com_crc8_fast(p_msg->crc, &rx_byte, 1);
         break;

     case COM_DBG_RX_CMD_LIST_4_RX:
         p_msg->cmd_lst.byte[i++]   = rx_byte;
         p_msg->state               = (i < sizeof(command_u))   ? (COM_DBG_RX_CMD_LIST_4_RX)    : (COM_DBG_RX_CRC);
         i                          = (i < sizeof(command_u))   ? (i)                           : (0);
         p_msg->crc                 = com_crc8_fast(p_msg->crc, &rx_byte, 1);
         break;

     case COM_DBG_RX_CMD_MSG:
         i                          = (i % 4)   ? (i)       : (0);
         j                          = (i % 4)   ? (j)       : (ffs(cmd_lst) - 1);
         cmd_lst                   &= (i % 4)   ? (cmd_lst) : (~(1U << j));
         p_msg->cmd_msg[j].byte[i++]= rx_byte;
         p_msg->state               = (cmd_lst || (i % 4))  ? (COM_DBG_RX_CMD_MSG)  : (COM_DBG_RX_CRC);
         p_msg->crc                 = com_crc8_fast(p_msg->crc, &rx_byte, 1);
         break;

     case COM_DBG_RX_CRC:
         p_msg->state               = COM_DBG_RX_WORD_A;
         p_msg->valid               = (p_msg->crc == rx_byte)   ? (true)    : (false);
         break;
     }

}

uint32_t com_crc32(uint16_t* p_buf, size_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  while (len--)
    {
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ MSB_16(*p_buf)) & 0xFF];
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ LSB_16(*p_buf)) & 0xFF];
      p_buf++;
    }

  return(crc);
}


uint8_t com_crc8(uint8_t *p_buf, size_t len)
{
    uint8_t crc = 0, i = 0, j = 0;
    for(i = 0; i < len; i++)
    {
        uint8_t inbyte = p_buf[i];
        for(j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if(mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return(crc);
}

//uint8_t com_crc8_fast(uint8_t crc, void const *p_buf, size_t len)
//{
//    uint8_t const *data = p_buf;
//    if (data == NULL)
//        return 0xff;
//    crc &= 0xff;
//    while (len--)
//        crc = crc8_table[crc ^ *data++];
//    return crc;
//}

uint8_t com_crc8_fast(uint8_t crc, uint8_t* p_buf, size_t len)
{
//    uint8_t crc = 0;
    size_t i = 0;
    for(i = 0; i < len; i++)
        crc = crc8_table[p_buf[i] ^ crc];
    return crc;
}
