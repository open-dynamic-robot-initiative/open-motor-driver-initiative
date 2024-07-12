#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdint.h>
#include <string.h>
#include <hw_types.h>
#include "communication.h"

/***********************************************************************
 * ENUM TYPES
 ***********************************************************************/

/***********************************************************************
 * UNION TYPES
 ***********************************************************************/
typedef union __uart_addr_u__
{
    uint8_t             addr;
    struct
    {
        uint8_t         addr_0          : 1;    /*!< Bit 0      : R/W - Address bit 0. */
        uint8_t         addr_1          : 1;    /*!< Bit 1      : R/W - Address bit 1. */
        uint8_t         addr_2          : 1;    /*!< Bit 2      : R/W - Address bit 2. */
    };
} uart_addr_u;

/***********************************************************************
 * DEFINES
 ***********************************************************************/

/***********************************************************************
 * INTERRUPTS DEFINES
 ***********************************************************************/
#define INT_TIMER_LED                   INT_TIMER0
#define INT_TIMER_LED_CMD_GROUP         INTERRUPT_ACK_GROUP1
#define INT_TIMER_LED_CMD_HANDLER       &tmr_isrLedCmd

#define INT_RS485_Rx_Tx_CLB             INT_CLB1
#define INT_RS485_RX_Tx_CLB_GROUP       INTERRUPT_ACK_GROUP5
#define INT_RS485_RX_Tx_CLB_HANDLER     &clb_isrTxRx_RS485

#define INT_DMA_RX_MSG                  INT_DMA_CH6
#define INT_DMA_RX_MSG_GROUP            INTERRUPT_ACK_GROUP7
#define INT_DMA_RX_MSG_HANDLER          &dma_isrSpiComRx

#define INT_RS485_RX_PIE_FLAG           PIE_IFR8_INTX5
#define INT_RS485_RX_MSG                INT_SCIC_RX
#define INT_RS485_RX_MSG_GROUP          INTERRUPT_ACK_GROUP8
#define INT_RS485_RX_MSG_HANDLER        &sci_isrRx_RS485

#define INT_RS485_TX_PIE_FLAG           PIE_IFR8_INTX6
#define INT_RS485_TX_MSG                INT_SCIC_TX
#define INT_RS485_TX_MSG_GROUP          INTERRUPT_ACK_GROUP8
#define INT_RS485_TX_MSG_HANDLER        &sci_isrTx_RS485

#define INT_CLA_ADC_CALIB               INT_CLA1_1
#define INT_CLA_ADC_CALIB_GROUP         INTERRUPT_ACK_GROUP11
#define INT_CLA_ADC_CALIB_HANDLER       &cla_ClearTask

#define INT_CLA_MOTOR1                  INT_CLA1_3
#define INT_CLA_MOTOR1_GROUP            INTERRUPT_ACK_GROUP11
#define INT_CLA_MOTOR1_HANDLER          &cla_isrMotor1

#define INT_CLA_MOTOR2                  INT_CLA1_4
#define INT_CLA_MOTOR2_GROUP            INTERRUPT_ACK_GROUP11
#define INT_CLA_MOTOR2_HANDLER          &cla_isrMotor2

#define INT_TIMER_LED_DEF               { \
                                            .intNum             = INT_TIMER_LED, \
                                            .intAckGroup        = INT_TIMER_LED_CMD_GROUP, \
                                            .p_intHandler       = INT_TIMER_LED_CMD_HANDLER \
                                        }
#define INT_RS485_Rx_Tx_CLB_DEF         { \
                                            .intNum             = INT_RS485_Rx_Tx_CLB, \
                                            .intAckGroup        = INT_RS485_RX_Tx_CLB_GROUP, \
                                            .p_intHandler       = INT_RS485_RX_Tx_CLB_HANDLER \
                                        }
#define INT_DMA_RX_MSG_DEF              { \
                                            .intNum             = INT_DMA_RX_MSG, \
                                            .intAckGroup        = INT_DMA_RX_MSG_GROUP, \
                                            .p_intHandler       = INT_DMA_RX_MSG_HANDLER \
                                        }
#define INT_RS485_RX_MSG_DEF            { \
                                            .intNum             = INT_RS485_RX_MSG, \
                                            .intAckGroup        = INT_RS485_RX_MSG_GROUP, \
                                            .p_intHandler       = INT_RS485_RX_MSG_HANDLER \
                                        }
#define INT_RS485_TX_MSG_DEF            { \
                                            .intNum             = INT_RS485_TX_MSG, \
                                            .intAckGroup        = INT_RS485_TX_MSG_GROUP, \
                                            .p_intHandler       = INT_RS485_TX_MSG_HANDLER \
                                        }
#define INT_CLA_ADC_CALIB_DEF           { \
                                            .intNum             = INT_CLA_ADC_CALIB, \
                                            .intAckGroup        = INT_CLA_ADC_CALIB_GROUP, \
                                            .p_intHandler       = INT_CLA_ADC_CALIB_HANDLER \
                                        }
#define INT_CLA_MOTOR1_DEF              { \
                                            .intNum             = INT_CLA_MOTOR1, \
                                            .intAckGroup        = INT_CLA_MOTOR1_GROUP, \
                                            .p_intHandler       = INT_CLA_MOTOR1_HANDLER \
                                         }
#define INT_CLA_MOTOR2_DEF              { \
                                            .intNum             = INT_CLA_MOTOR2, \
                                            .intAckGroup        = INT_CLA_MOTOR2_GROUP, \
                                            .p_intHandler       = INT_CLA_MOTOR2_HANDLER \
                                         }


/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
/** @var    MOTOR_STRUCT    motor[]
 *  @brief  General structure array configuration structure for the Hardware Abstraction Layer.
 */
#pragma DATA_ALIGN(motor_m1, 4)
#pragma DATA_SECTION(motor_m1, "ClaDataMotor1");
motor_t     motor_m1;
#pragma DATA_ALIGN(motor_m2, 4)
#pragma DATA_SECTION(motor_m2, "ClaDataMotor2");
motor_t     motor_m2;

#pragma DATA_ALIGN(cmd_uomodri, 4)
#pragma DATA_SECTION(cmd_uomodri, "MSGRAM_CPU_TO_CLA")
cmd_t       cmd_uomodri[2];

#pragma DATA_ALIGN(com_spi_rx, 4)
#pragma DATA_SECTION(com_spi_rx, "ramgs0")
spi_rx_t    com_spi_rx;
#pragma DATA_ALIGN(com_spi_tx, 4)
#pragma DATA_SECTION(com_spi_tx, "ramgs0")
spi_tx_t    com_spi_tx          = {.msg_cr = 0U, .msg_nx = 1U,
                                   .msg[0].crc.u16_msb = 0xDFFF, .msg[0].crc.u16_lsb = 0x5283,
                                   .msg[1].crc.u16_msb = 0xDFFF, .msg[1].crc.u16_lsb = 0x5283};

#pragma DATA_ALIGN(com_rs485_rx, 4)
#pragma DATA_SECTION(com_rs485_rx, "ramgs0")
rs485_rx_t  com_rs485_rx        = {.index = 0U,
                                   .addr1 = false, .addr2 = false};
#pragma DATA_ALIGN(com_rs485_tx, 4)
#pragma DATA_SECTION(com_rs485_tx, "ramgs0")
rs485_tx_t  com_rs485_tx        = {.index = 0U,
                                   .msg_cr = 0U, .msg_nx = 1U,
                                   .msg[0].crc.u16_msb = 0x8CA0, .msg[0].crc.u16_lsb = 0x22CF,
                                   .msg[1].crc.u16_msb = 0x8CA0, .msg[1].crc.u16_lsb = 0x22CF};

#pragma DATA_ALIGN(led_spi_tx, 4)
#pragma DATA_SECTION(led_spi_tx, "ramgs0")
uint16_t    led_spi_tx[LED_MSG_TX_16BIT_LENGTH];
#pragma DATA_ALIGN(com_bus_select, 2)
#pragma DATA_SECTION(com_bus_select, "ramgs0")
com_bus_e   com_bus_select      = COM_NO_BUS_DEFINED;
#pragma DATA_ALIGN(global_1ms_tmr_cnt, 2)
#pragma DATA_SECTION(global_1ms_tmr_cnt, "ramgs0")
uint32_t    global_1ms_tmr_cnt  = 0U;

#if (defined DEBUG) && (CM_CORE_ENABLE) && (0)
#pragma DATA_SECTION(cpu2cm_dbg_msg, "MSGRAM_CPU_TO_CM");
com_cpu_2_cm_t  cpu2cm_dbg_msg;
#endif

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
//__attribute__((interrupt)) void adc_isrMotor1(void);
//__attribute__((interrupt)) void adc_isrMotor2(void);

#endif /* __MAIN_H__ */
