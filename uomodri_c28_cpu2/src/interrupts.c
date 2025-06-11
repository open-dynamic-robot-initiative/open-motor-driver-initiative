/*
 * File name: interrupts.c
 * Description: Source file containing IT ISRs running on CPU2 (C28).
 */

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <device.h>

#include "interrupts.h"

#include "globals.h"
#include "hal_defines.h"
#include "daisy_chain.h"

/***********************************************************************
 * INTERRUPT SERVICE ROUTINES
 ***********************************************************************/

// Triggered when CPU Timer 1 times out to start an FSI TX trasmission.
// Depending on whether FSI is executing a handshake, it will also
// send the flush sequence before the packet:
__attribute__((interrupt)) void timer_timeout_isr(void)
{
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    if (FSI_COMM_STATE_VAR == COMM_HANDSHAKE)
    {
        FSI_executeTxFlushSequence(FSI_TX_CPU2_BASE, FSI_TX_CPU2_PRESCALE);
        DEVICE_DELAY_US(FSI_TX_FLUSH_DELAY_US);
    }

//    // Check whether a DMA transfer is pending for safety:
//    while(DMA_getBurstStatusFlag(DMA_FSI_TO_TX_CH));

    FSI_startTxTransmit(FSI_TX_CPU2_BASE);
    // NOTE: No group acknowledgement is required for CPU Timer 1
}

// Called whenever there's an error during communication.
// Will mark the error in addition to disabling DMA transfers,
// timers, and FSI events:
inline void _disable_system_on_error()
{
    // Stop TX timer and DMA channels:
    CPUTimer_stopTimer(TIMER_FSI_TX_BASE);
    DMA_stopChannel(DMA_FSI_TO_MEM_CH);
    DMA_stopChannel(DMA_FSI_TO_TX_CH);

    // Disable all FSI interrupts:
    FSI_disableRxInterrupt(FSI_RX_CPU2_BASE, FSI_INT1, FSI_RX_EVTMASK);
    FSI_disableRxInterrupt(FSI_RX_CPU2_BASE, FSI_INT2, FSI_RX_EVTMASK);
    FSI_disableTxInterrupt(FSI_TX_CPU2_BASE, FSI_INT1, FSI_TX_EVTMASK);
    FSI_disableTxInterrupt(FSI_TX_CPU2_BASE, FSI_INT2, FSI_TX_EVTMASK);

    // Disable FSI DMA events:
    FSI_disableRxDMAEvent(FSI_RX_CPU2_BASE);
    FSI_disableTxDMAEvent(FSI_TX_CPU2_BASE);

    // Mark the error so that we fall back to handshaking:
    FSI_COMM_STATE_VAR = COMM_ERROR;
}

// Triggered when an FSI error occurs. A TX/RX reset is required
// along with the handshake procedure to establish the daisy chain:
__attribute__((interrupt)) void fsi_rx_error_isr(void)
{
    _disable_system_on_error();

    Interrupt_clearACKGroup(INT_FSI_RX_GROUP);

    // NOTE: We add this in case we handle TX errors as well:
    Interrupt_clearACKGroup(INT_FSI_TX_GROUP);
}

// Triggered as soon as a data packet is received at FSI RX.
// A DMA channel will at the same time copy data to GS memory.
__attribute__((interrupt)) void fsi_rx_data_frame_isr(void)
{
    uint16_t rx_tag = 0U;

    // For safety, skip processing the packet and fail if we
    // receive it when the FSI is not active. This shouldn't
    // happen under normal conditions:
    if (FSI_COMM_STATE_VAR != COMM_ACTIVE)
    {
        _disable_system_on_error();
        goto RETURN_ISR;
    }

    // If the timer is already running, that means a packet is already pending
    // for transmission. Receiving a fresh one means the transmitter
    // we're listening to is sending packets too fast. They should only complete
    // a fresh packet transmission after our packet transmission has started:
    if(!(HWREGH(CPUTIMER1_BASE + CPUTIMER_O_TCR) & CPUTIMER_TCR_TSS))
    {
        _disable_system_on_error();

        // We don't need to clear the FSI event as the module will be reset:
        goto RETURN_ISR;
    }

    // Wait until a pending TX is complete to avoid corruption.
    // This applies only to transmissions after the first:
    if (!is_first_packet_tx)
    {
        while(true)
        {
            // To avoid getting stuck, fail if there's an RX timeout:
            if (FSI_getRxEventStatus(FSI_RX_CPU2_BASE) & FSI_RX_EVT_PING_WD_TIMEOUT)
                goto RETURN_ISR;

            if (FSI_getTxEventStatus(FSI_TX_CPU2_BASE) & FSI_TX_EVT_FRAME_DONE)
                break;
        }
    }

    // Clear the TX done event. It will be set when the next packet is complete:
    FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVT_FRAME_DONE);

    // Start timer to count delay before sending the packet.
    // NOTE: This assumes that the delay is enough for the packet
    //       checking and DMA transfers to complete before
    //       triggering TX!
    CPUTimer_startTimer(CPUTIMER1_BASE);

    // In the meantime, check the packet and send it to the TX buffer:
    rx_tag = FSI_getRxFrameTag(FSI_RX_CPU2_BASE);

    // Check if there was an error in packet ordering:
    if (rx_tag != next_packet_id)
    {
        _disable_system_on_error();
        goto RETURN_ISR;
    }

    next_packet_id = (next_packet_id + 1) % DAISY_CHAIN_DRIVER_COUNT;

    // Copy data to be transmitted to TX buffer. Send data from the CLA/CPU1
    // ring buffer if the packet has our ID, otherwise forward the packet as is:
    DMA_configSourceAddress(
            DMA_FSI_TO_TX_CH,
            rx_tag == DAISY_CHAIN_DRIVER_ID ?
                    (const void*)(&(gs_r_fsi_cla_state_ring_buf.buf[gs_r_fsi_cla_state_ring_buf.cur_buf][0])) :
                    (const void*)FSI_getRxBufferAddress(FSI_RX_CPU2_BASE)
    );

    DMA_forceTrigger(DMA_FSI_TO_TX_CH);

    // Prepare TX module for transmission:
    FSI_setTxBufferPtr(FSI_TX_CPU2_BASE, 0U);
    FSI_setTxFrameTag(FSI_TX_CPU2_BASE, (FSI_FrameTag)rx_tag);

//    while (DMA_getBurstStatusFlag(DMA_FSI_TO_MEM_CH) || DMA_getBurstStatusFlag(DMA_FSI_TO_TX_CH));

    // Prepare the RX module for the next packet reception:
    FSI_setRxBufferPtr(FSI_RX_CPU2_BASE, 0U);
    FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVT_DATA_FRAME | FSI_RX_EVT_FRAME_DONE);

    is_first_packet_tx = 0U;

RETURN_ISR:
    Interrupt_clearACKGroup(INT_FSI_RX_GROUP);
}
