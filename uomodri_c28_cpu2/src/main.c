/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <device.h>
#include <string.h>

#include "globals.h"
#include "hal_handlers.h"

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
#pragma DATA_ALIGN(gs_rw_fsi_recv_pack_buf, 4)
#pragma DATA_SECTION(gs_rw_fsi_recv_pack_buf, "ramgs3")
volatile fsi_recv_data_t gs_rw_fsi_recv_pack_buf = {.comm_state = COMM_INACTIVE, .packets = {0}};

#pragma DATA_ALIGN(gs_r_fsi_cla_state_ring_buf, 4)
#pragma DATA_SECTION(gs_r_fsi_cla_state_ring_buf, "ramgs4")
volatile fsi_data_ring_buf_t gs_r_fsi_cla_state_ring_buf;

// Packets should be received cyclically. If this doesn't happen we must
// have lost something and we trigger an error. The FSI would have
// gone wrong anyway for a packet to be lost and it would probably need
// a reset:
volatile uint16_t next_packet_id = 0U;

// Mark whether this is the first packet after handshaking. This will
// be used to determine whether we need to wait for a complete frame
// transmission before queuing another one:
volatile uint16_t is_first_packet_tx = 1U;

/***********************************************************************
 * UTILITIES
 ***********************************************************************/
inline void _set_timer_period_us(float32_t us)
{
    CPUTimer_stopTimer(TIMER_FSI_TX_BASE);
    CPUTimer_setPeriod(
            TIMER_FSI_TX_BASE,
            (uint32_t)((float32_t)DEVICE_SYSCLK_FREQ / 1000000.0 * us) - 1
    );
}

/***********************************************************************
 * FUNCTIONS
 ***********************************************************************/
#if DAISY_CHAIN_DRIVER_ID == 0    // Primary board handshake routine

void fsi_perform_handshake()
{
    // 25 us between ping packets:
    _set_timer_period_us(FSI_TX_HANDSHAKE_PACKET_DELAY_US);

    // Prepare transmitter:
    FSI_setTxFrameTag(FSI_TX_CPU2_BASE, FSI_FRAME_TAG0);
    FSI_setTxFrameType(FSI_TX_CPU2_BASE, FSI_FRAME_TYPE_PING);
//    FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVTMASK);
//    FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVTMASK);

    // Send P0 periodically to ensure chain is fully online:
    CPUTimer_startTimer(CPUTIMER1_BASE);

    while (true)
    {
        if (FSI_COMM_STATE_VAR == COMM_ERROR)
        {
            CPUTimer_stopTimer(CPUTIMER1_BASE);
            return;
        }

        // If we've received a ping packet, the communication link is up:
        if (FSI_getRxEventStatus(FSI_RX_CPU2_BASE) & FSI_RX_EVT_PING_FRAME)
            break;

        // If the packet we've previously queued to send is done, send another one:
        if (FSI_getTxEventStatus(FSI_TX_CPU2_BASE) & FSI_TX_EVT_FRAME_DONE)
        {
            FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVT_FRAME_DONE);
            CPUTimer_startTimer(CPUTIMER1_BASE);
        }
    }

    // Daisy chain is up. Stop any queued Ping 0 transmission and inform all nodes
    // to end the handshake using a Ping 1 packet:
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    DEVICE_DELAY_US(1);

    FSI_setTxFrameTag(FSI_TX_CPU2_BASE, FSI_FRAME_TAG1);
    FSI_startTxTransmit(FSI_TX_CPU2_BASE);
}

#else   // DAISY_CHAIN_DRIVER_ID != 0, secondary board handshake routine

void fsi_perform_handshake()
{
    FSI_setTxFrameType(FSI_TX_CPU2_BASE, FSI_FRAME_TYPE_PING);
//    FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVTMASK);

    uint16_t recv_tag = 0U;

    while (true)
    {
        if (FSI_COMM_STATE_VAR == COMM_ERROR)
            return;

        if (FSI_getRxEventStatus(FSI_RX_CPU2_BASE) & FSI_RX_EVT_PING_FRAME)
        {
            FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVT_PING_FRAME | FSI_RX_EVT_FRAME_DONE);

            // Forward any ping packets received:
            recv_tag = FSI_getRxPingTag(FSI_RX_CPU2_BASE);
            FSI_setTxFrameTag(FSI_TX_CPU2_BASE, (FSI_FrameTag)recv_tag);

            // If we received a Ping 1 packet, the link is up and we must
            // end the handshake. Forward packet without flush and exit:
            if (recv_tag == FSI_FRAME_TAG1)
            {
                FSI_startTxTransmit(FSI_TX_CPU2_BASE);
                return;
            }

            FSI_executeTxFlushSequence(FSI_TX_CPU2_BASE, FSI_TX_CPU2_PRESCALE);
            DEVICE_DELAY_US(FSI_TX_FLUSH_DELAY_US);
            FSI_startTxTransmit(FSI_TX_CPU2_BASE);
        }
    }
}

#endif

int main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    FSI_COMM_STATE_VAR = COMM_INACTIVE;

    // Synchronize with CPU1:
	IPC_sync(IPC_CPU2_L_CPU1_R, IPC_SYNC_FLAG);

#if DAISY_CHAIN_ENABLE
    // FSI packet must fit in RX / TX buffer:
    ASSERT(FSI_PACKET_SZ_WORDS <= FSI_BUF_SZ_WORDS);

    // 16 boards max in daisy chain due to tag size limitation:
    ASSERT(DAISY_CHAIN_DRIVER_COUNT <= 0xFU);

	// Now peripherals are available for CPU2 to configure:
	HAL_ini(&hal);

    /**********************************************************************
    *                          HANDSHAKE                                  *
    **********************************************************************/
    // Start handshake procedure and repeat until there's an error:
COMM_HANDSHAKE_start:
    FSI_COMM_STATE_VAR = COMM_HANDSHAKE;

    // If this is not the first time we're executing a handshake
    // reinitialize the FSI modules (reset registers, events, etc).
    // Also, wait for the next (and all after) boards to timeout
    // otherwise our ping packets will keep them in active mode:
    if (FSI_COMM_STATE_VAR != COMM_INACTIVE)
    {
        DEVICE_DELAY_US(2.0 * FSI_RX_DATA_PACKET_TIMEOUT_US);
        HAL_FSI_ini(hal.p_fsiHandle);
    }

    fsi_perform_handshake();

    if (FSI_COMM_STATE_VAR == COMM_ERROR)
        goto COMM_HANDSHAKE_start;

    // Wait for the last ping packet to be sent before modifying FSI settings.
    // The ping packet can be malformed otherwise:
    DEVICE_DELAY_US(1);

    /**********************************************************************
    *                            SETUP                                    *
    **********************************************************************/
    // Handshake complete, setup data transmission and reception:
    is_first_packet_tx = 1U;
    next_packet_id = 0U;

    // Clear all transmitter events:
    FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVTMASK);
    FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVTMASK);

    // Reset buffer pointers:
    FSI_setTxBufferPtr(FSI_TX_CPU2_BASE, 0U);
    FSI_setRxBufferPtr(FSI_RX_CPU2_BASE, 0U);

    // Setup frame parameters:
    FSI_setTxUserDefinedData(FSI_TX_CPU2_BASE, 0x0);
    FSI_setTxFrameType(FSI_TX_CPU2_BASE, FSI_FRAME_TYPE_NWORD_DATA);
    FSI_setTxSoftwareFrameSize(FSI_TX_CPU2_BASE, FSI_PACKET_SZ_WORDS);
    FSI_setRxSoftwareFrameSize(FSI_RX_CPU2_BASE, FSI_PACKET_SZ_WORDS);

    // Clear packet buffer (shared with CPU1):
    memset(
            (void*)gs_rw_fsi_recv_pack_buf.packets,
            0x0U,
            sizeof(uint16_t) * DAISY_CHAIN_DRIVER_COUNT * FSI_PACKET_SZ_WORDS
    );

    // Reset channel state for both DMAs - this ensures that all pointers
    // are reset to the beginning of each buffer:
    DMA_triggerSoftReset(DMA_FSI_TO_TX_CH);
    DMA_triggerSoftReset(DMA_FSI_TO_MEM_CH);

    // Start all DMA channels:
    DMA_startChannel(DMA_FSI_TO_TX_CH);
    DMA_startChannel(DMA_FSI_TO_MEM_CH);

    // Set up packet transmission timer delay:
    _set_timer_period_us(FSI_TX_ACTIVE_PACKET_DELAY_US);

    // Enable RX events:
    FSI_enableRxDMAEvent(FSI_RX_CPU2_BASE);
    FSI_enableRxInterrupt(FSI_RX_CPU2_BASE, FSI_INT1, FSI_RX_EVT_DATA_FRAME);

    // Set up data packet timeout (we've configured this watchdog to be reset
    // by all frames, not just ping):
    FSI_enableRxPingWatchdog(
            FSI_RX_CPU2_BASE,
            (uint32_t)((float32_t)DEVICE_SYSCLK_FREQ / 1000000.0 * FSI_RX_DATA_PACKET_TIMEOUT_US
    ));

    // If there was any error in the meantime, fall back to handshake:
    if (FSI_COMM_STATE_VAR == COMM_ERROR)
        goto COMM_HANDSHAKE_start;

    FSI_COMM_STATE_VAR = COMM_ACTIVE;

    /**********************************************************************
    *                       PACKET INJECTION                              *
    **********************************************************************/
    // At this point, a packet train can be initialized. The primary board
    // sends empty packets that will be forwarded by all boards from then on:

#if DAISY_CHAIN_DRIVER_ID == 0
    // Wait a bit to ensure boards have seen the last handshake packet:
    DEVICE_DELAY_US(5.);

    // Set up an empty packet to be injected:
    memset((void*)FSI_getTxBufferAddress(FSI_TX_CPU2_BASE), 0x0U, sizeof(uint16_t) * 16);

    uint16_t cur_idx = 0U;
    for (cur_idx = 0U; cur_idx < DAISY_CHAIN_DRIVER_COUNT; cur_idx++)
    {
        FSI_setTxBufferPtr(FSI_TX_CPU2_BASE, 0U);
        FSI_setTxFrameTag(FSI_TX_CPU2_BASE, (FSI_FrameTag)cur_idx);
        CPUTimer_startTimer(CPUTIMER1_BASE);

        while (true)
        {
            if (FSI_COMM_STATE_VAR == COMM_ERROR)
                goto COMM_HANDSHAKE_start;

            // Wait until packet is sent to continue with the next one:
            if (FSI_getTxEventStatus(FSI_TX_CPU2_BASE) & FSI_TX_EVT_FRAME_DONE)
                break;
        }

        FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVT_FRAME_DONE);
    }

#endif // DAISY_CHAIN_DRIVER_ID == 0

    /**********************************************************************
    *                       ACTIVE OPERATION                              *
    **********************************************************************/
    while(true)
    {
       if (FSI_COMM_STATE_VAR == COMM_ERROR)
       {
           goto COMM_HANDSHAKE_start;
       }
    }

#else // DAISY_CHAIN_ENABLE
    while(true);
#endif
}
