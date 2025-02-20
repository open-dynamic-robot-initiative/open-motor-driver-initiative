#ifndef __MAIN_H__
#define __MAIN_H__

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
 * SCI DEFINES
 ***********************************************************************/
#define COM_SCI_FIFO_TX_REG_ADDR        (COM_SCI_BASE + SCI_O_FFTX)
#define COM_SCI_FIFO_RX_REG_ADDR        (COM_SCI_BASE + SCI_O_FFRX)
#define COM_SCI_CTRL1_REG_ADDR          (COM_SCI_BASE + SCI_O_CTL1)

/***********************************************************************
 * INTERRUPTS DEFINES
 ***********************************************************************/
#define INT_CLA_MOTOR1                  INT_CLA1_3
#define INT_CLA_MOTOR1_GROUP            INTERRUPT_ACK_GROUP11
#define INT_CLA_MOTOR1_HANDLER          &cla_isrMotor1

#define INT_CLA_MOTOR2                  INT_CLA1_4
#define INT_CLA_MOTOR2_GROUP            INTERRUPT_ACK_GROUP11
#define INT_CLA_MOTOR2_HANDLER          &cla_isrMotor2

#define INT_TIMER_LED                   INT_TIMER0
#define INT_TIMER_LED_CMD_GROUP         INTERRUPT_ACK_GROUP1
#define INT_TIMER_LED_CMD_HANDLER       &tmr_isrLedCmd

#define INT_GPIO_SPI_CS                 INT_XINT1
#define INT_GPIO_SPI_CS_GROUP           INTERRUPT_ACK_GROUP1
#define INT_GPIO_SPI_CS_HANDLER         &gpio_isrSpiComRx

#define INT_RS485_RX_TX_CLB             INT_CLB1
#define INT_RS485_RX_TX_CLB_GROUP       INTERRUPT_ACK_GROUP5
#define INT_RS485_RX_TX_CLB_HANDLER     &clb_isrTxRx_RS485

#define INT_RS485_RX_PIE_FLAG           PIE_IFR8_INTX5
#define INT_RS485_RX_MSG                INT_SCIC_RX
#define INT_RS485_RX_MSG_GROUP          INTERRUPT_ACK_GROUP8
#define INT_RS485_RX_MSG_HANDLER        &sci_isrRx_RS485

#define INT_RS485_TX_PIE_FLAG           PIE_IFR8_INTX6
#define INT_RS485_TX_MSG                INT_SCIC_TX
#define INT_RS485_TX_MSG_GROUP          INTERRUPT_ACK_GROUP8
#define INT_RS485_TX_MSG_HANDLER        &sci_isrTx_RS485

#define INT_MCAN_RX_TX_MSG              INT_MCANA_1
#define INT_MCAN_RX_TX_MSG_GROUP        INTERRUPT_ACK_GROUP9
#define INT_MCAN_RX_TX_MSG_HANDLER      &mcan_isr_canfd

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
#define INT_TIMER_LED_DEF               { \
                                            .intNum             = INT_TIMER_LED, \
                                            .intAckGroup        = INT_TIMER_LED_CMD_GROUP, \
                                            .p_intHandler       = INT_TIMER_LED_CMD_HANDLER \
                                        }
#define INT_GPIO_SPI_CS_DEF             { \
                                            .intNum             = INT_GPIO_SPI_CS, \
                                            .intAckGroup        = INT_GPIO_SPI_CS_GROUP, \
                                            .p_intHandler       = INT_GPIO_SPI_CS_HANDLER \
                                        }
#define INT_RS485_RX_TX_CLB_DEF         { \
                                            .intNum             = INT_RS485_RX_TX_CLB, \
                                            .intAckGroup        = INT_RS485_RX_TX_CLB_GROUP, \
                                            .p_intHandler       = INT_RS485_RX_TX_CLB_HANDLER \
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
#define INT_MCAN_RX_TX_MSG_DEF          { \
                                            .intNum             = INT_MCAN_RX_TX_MSG, \
                                            .intAckGroup        = INT_MCAN_RX_TX_MSG_GROUP, \
                                            .p_intHandler       = INT_MCAN_RX_TX_MSG_HANDLER \
                                        }

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
#if (defined DEBUG) && (!defined CPU1) && (0)
#pragma DATA_SECTION(cpu2cm_dbg_msg, "MSGRAM_CPUx_TO_CM");
com_cpu_2_cm_t  cpu2cm_dbg_msg;
#endif





#endif /* __MAIN_H__ */
