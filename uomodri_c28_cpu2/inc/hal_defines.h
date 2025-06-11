/*
 * File name: hal_defines.h
 * Description: Header file containing CPU2 HAL configuration defines.
 */
#ifndef INC_HAL_DEFINES_CPU2_H_
#define INC_HAL_DEFINES_CPU2_H_

/***********************************************************************
 * IPC DEFINES
 ***********************************************************************/
#define IPC_SYNC_FLAG                           IPC_FLAG31

/***********************************************************************
 * GPIO DEFINES
 ***********************************************************************/

// Control of these by CPU2 should be enabled during CPU1 initialization:
#define DBG_PIN0_SET        (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO82)
#define DBG_PIN0_CLEAR      (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO82)
#define DBG_PIN3_SET        (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO77)
#define DBG_PIN3_CLEAR      (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO77)


/***********************************************************************
 * INTERRUPT DEFINES
 ***********************************************************************/
#define INT_FSI_RX_GROUP    INTERRUPT_ACK_GROUP7
#define INT_FSI_TX_GROUP    INTERRUPT_ACK_GROUP4
#define INT_DMA_GROUP       INTERRUPT_ACK_GROUP7

//#define INT_FSI_TX1         (INT_FSITXA1)
//#define INT_FSI_TX2         (INT_FSITXA2)
#define INT_FSI_RX1         (INT_FSIRXC1)
#define INT_FSI_RX2         (INT_FSIRXC2)

// NOTE: Timer 1 does not use PIE grouping.
#define INT_TIMER           (INT_TIMER1)

#define INT_FSI_RX_DATA_FRAME_DEF   {                                                 \
                                      .interruptNumber    = INT_FSI_RX1,              \
                                      .handler            = &fsi_rx_data_frame_isr,   \
}

#define INT_FSI_RX_ERROR_DEF        {                                                 \
                                      .interruptNumber    = INT_FSI_RX2,              \
                                      .handler            = &fsi_rx_error_isr,        \
}

#define INT_TIMER_DEF               {                                                 \
                                      .interruptNumber    = INT_TIMER,                \
                                      .handler            = &timer_timeout_isr,       \
}

/***********************************************************************
 * DMA DEFINES
 ***********************************************************************/
// FSI TX buffer DMA channel: one packet per burst / transfer.
// Source is either FSIRX (to forward a packet) or memory (to send our state).
// This will be set up during runtime. Channel is manually triggered after tag test.
#define DMA_FSI_TO_TX_CH     (DMA_CH5_BASE)
#define DMA_FSI_TO_TX_DEF    {                                                                                                  \
                                .dmaBase           = DMA_FSI_TO_TX_CH,                                                          \
                                .destAddr          = (const void*)(FSI_TX_CPU2_BASE + FSI_O_TX_BUF_BASE(0U)),                   \
                                .srcAddr           = NULL,                                                                      \
                                .burstSize         = FSI_PACKET_SZ_WORDS,                                                       \
                                .burstSrcStep      = 1,                                                                         \
                                .burstDestStep     = 1,                                                                         \
                                .transferSize      = 1,                                                                         \
                                .transferSrcStep   = 0,                                                                         \
                                .transferDestStep  = 0,                                                                         \
                                .trigger           = DMA_TRIGGER_SOFTWARE,                                                      \
                                .config            = DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT,  \
                                .interruptEnable   = false,                                                                     \
                                .channelStart      = false,                                                                     \
                            }

// FSI RX to memory channel: one packet per burst, #(boards in daisy chain) packets per transfer.
// Triggered by FSIRX to save data to GS3 cyclically, starting with the packet for board ID = 0.
// Source points back to the beginning of the FSIRX buffer after each burst.
// TODO(rafael): Do 32-bit unit size to avoid float corruption when CPU1 DMA copies.
#define DMA_FSI_TO_MEM_CH    (DMA_CH6_BASE)
#define DMA_FSI_TO_MEM_DEF    {                                                                                                 \
                                .dmaBase           = DMA_FSI_TO_MEM_CH,                                                         \
                                .destAddr          = (const void*)(&gs_rw_fsi_recv_pack_buf.packets[0]),                        \
                                .srcAddr           = (const void*)(FSI_RX_CPU2_BASE + FSI_O_RX_BUF_BASE(0U)),                   \
                                .burstSize         = FSI_PACKET_SZ_WORDS,                                                       \
                                .burstSrcStep      = 1,                                                                         \
                                .burstDestStep     = 1,                                                                         \
                                .transferSize      = DAISY_CHAIN_DRIVER_COUNT,                                                  \
                                .transferSrcStep   = -(int16_t)FSI_PACKET_SZ_WORDS + 1,                                         \
                                .transferDestStep  = +1,                                                                        \
                                .trigger           = DMA_TRIGGER_FSIRXC,                                                        \
                                .config            = DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT,  \
                                .interruptEnable   = false,                                                                     \
                                .channelStart      = false,                                                                     \
                            }

/***********************************************************************
 * FSI DEFINES
 ***********************************************************************/
/** @defgroup Daisy-chain FSI configuration.
 *  @brief    Configure FSI modules associated with motor driver daisy-chain.
 *
 * @{
 */
/*
 * We use FSI_TXA and FSI_RXC for board-to-board communication.
 * Both modules are controlled by CPU2 to enable low latency message
 * reception and forwarding.
 *
 * This is necessary because we cannot check and forward an incoming
 * message using CPU1 with low latency since the core is often running ISR code.
 * Alternatively, the CLB could check packet tags and decide whether to
 * forward as is or trigger a DMA transfer from memory to the TX buffer.
 * However, CLB is not connected to FSI_RXC tag bits.
 *
 * Another solution to avoid the need for packet forwarding would be
 * to enable RX->TX passthrough, and set up a time-sharing scheme
 * for boards to share their information one after another
 * (without propagation delay). However, RXC->TXA passthrough is not
 * possible and we don't expose RXB on our PCB.
 *
 * Single-lane FSI is used with 50MHz clock (200Mbps). In single-lane
 * mode, the same data is sent through both TXD0 and TXD1. Due to a
 * hardware bug, TXD0 is connected to RXD1 instead of RXD0. For this
 * reason, we use single-lane mode so that data from TXD1 reaches RXD0.
 *
 * Alternatively, we could flip bits when writing to the TX buffer
 * and calculate message CRC in software. This would work because
 * frame pre/postamble, tag and udata are always repeated across both
 * TX channels.
 */

#define FSI_BUF_SZ_WORDS                        (16U)
#define FSI_DATA_WIDTH                          FSI_DATA_WIDTH_1_LANE

// Board-to-board TX (controlled by CPU2):
#define FSI_TX_CPU2_BASE                        FSITXA_BASE
#define FSI_TX_CPU2_PRESCALE                    FSI_PRESCALE_25MHZ

#define FSI_TX_HANDSHAKE_PACKET_DELAY_US        25.0
#define FSI_TX_ACTIVE_PACKET_DELAY_US           2.0
#define FSI_TX_FLUSH_DELAY_US                   0.5

#define FSI_TX_CPU2_DEF(EVT_LIST)               {                                                                       \
                                                    .fsiBase           = FSI_TX_CPU2_BASE,                              \
                                                    .direction         = FSI_TX_DIR,                                    \
                                                    .dmaTrigger        = false,                                         \
                                                    .clkPrescaler      = FSI_TX_CPU2_PRESCALE,                          \
                                                    .dataWidth         = FSI_DATA_WIDTH,                                \
                                                    .eventInterrupts   = (EVT_LIST)                                     \
                                                }

// Board-to-board RX (controlled by CPU2):
#define FSI_RX_CPU2_BASE                        FSIRXC_BASE
#define FSI_RX_DATA_PACKET_TIMEOUT_US           100.

#define FSI_RX_CPU2_DEF(EVT_LIST)               {                                                                       \
                                                    .fsiBase                   = FSI_RX_CPU2_BASE,                      \
                                                    .direction                 = FSI_RX_DIR,                            \
                                                    .rxLoopback                = false,                                 \
                                                    .rxEOFWatchdogTimeoutUs    = 10.0,                                  \
                                                    .dmaTrigger                = false,                                 \
                                                    .dataWidth                 = FSI_DATA_WIDTH,                        \
                                                    .eventInterrupts           = (EVT_LIST),                            \
                                                }

/***********************************************************************
 * TIMERS DEFINES
 ***********************************************************************/
// FSI TX timer. Initially, this is used to time ping packets during handshaking. Afterwards,
// we reconfigure the period to ensure there's enough time between data packets.
#define TIMER_FSI_TX_BASE   (CPUTIMER1_BASE)
#define TIMER_FSI_TX_DEF    {                                                             \
                                .timerBase          = TIMER_FSI_TX_BASE,                  \
                                .periodUs           = FSI_TX_HANDSHAKE_PACKET_DELAY_US,   \
                                .enableInterrupt    = true,                               \
}

#endif /* INC_HAL_DEFINES_CPU2_H_ */
