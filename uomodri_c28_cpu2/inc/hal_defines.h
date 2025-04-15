/*
 * File name: hal_defines.h
 * Description: Header file containing CPU2 HAL configuration defines.
 */
#ifndef INC_HAL_DEFINES_H_
#define INC_HAL_DEFINES_H_

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
 * FSI_RXA is also enabled and is controlled by CPU1. RXA is connected
 * via internal loopback to FSI_TXA. This is set up so that all forwarded
 * messages (via TXA), will be received on RXA as well, and will be saved
 * by DMA to CPU1.CLA memory to be used during FOC.
 *
 * This is necessary because we cannot filter and forward an incoming
 * message using CPU1 with low latency, since CPU1 is often running ISR code.
 * An alternative solution would be to use CLB to check whether a
 * received frame needs to be forwarded and if so, also trigger a DMA
 * transfer to CPU1 memory. However, CLB is not connected to FSI_RXC tag
 * bits.
 *
 * Dual-lane FSI is used with 50MHz clock (200Mbps).
 */

#define FSI_DATA_WIDTH                          FSI_DATA_WIDTH_1_LANE

// Board-to-board TX (controlled by CPU2):
#define FSI_TX_CPU2_BASE                        FSITXA_BASE
#define FSI_TX_CPU2_PRESCALE                    FSI_PRESCALE_25MHZ
#define FSI_TX_CPU2_DEF                         { \
                                                    .fsiBase           = FSI_TX_CPU2_BASE, \
                                                    .direction         = FSI_TX_DIR, \
                                                    .loopback          = false, \
                                                    .clkPrescaler      = FSI_TX_CPU2_PRESCALE, \
                                                    .dataWidth         = FSI_DATA_WIDTH, \
                                                }

// Board-to-board RX (controlled by CPU2):
#define FSI_RX_CPU2_BASE                        FSIRXC_BASE
#define FSI_RX_CPU2_DEF                         { \
                                                    .fsiBase           = FSI_RX_CPU2_BASE, \
                                                    .direction         = FSI_RX_DIR, \
                                                    .loopback          = false, \
                                                    .dataWidth         = FSI_DATA_WIDTH, \
                                                }


#endif /* INC_HAL_DEFINES_H_ */
