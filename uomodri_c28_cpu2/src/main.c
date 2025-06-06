// TODO(rafael): Clean all of this - create HAL functions, etc
// TODO(rafael): Allocation to global shared memory doesn't work, unless we allow from CPU1

#include <device.h>
#include <string.h>

#include "hal.h"
#include "hal_handlers.h"

#include "daisy_chain.h"

//#define INT_FSI_TX_GROUP                       INTERRUPT_ACK_GROUP4
//#define INT_FSI_RX1_GROUP                      INTERRUPT_ACK_GROUP7

#define DBG_PIN0_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO82)
#define DBG_PIN0_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO82)

#define DBG_PIN3_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO77)
#define DBG_PIN3_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO77)

//
//#define FSI_TX_FRAME_TAG_UDATA_REGADDR (FSI_TX_CPU2_BASE + FSI_O_TX_FRAME_TAG_UDATA)
//

#define INT_FSI_RX_GROUP                        INTERRUPT_ACK_GROUP7
#define INT_DMA_GROUP                           INTERRUPT_ACK_GROUP7

#define DMA_FSI_TO_TX_CH       (DMA_CH5_BASE)
#define DMA_FSI_TO_MEM_CH      (DMA_CH6_BASE)

#define INT_FRAME_RX           (INT_FSIRXC1)

void *tx_buf_addr, *rx_buf_addr;

typedef enum __fsi_comm_state_enum__
{
    COMM_INACTIVE   = 0,
    COMM_HANDSHAKE,
    COMM_ACTIVE,
} fsi_comm_state_e;

fsi_comm_state_e fsi_comm_state = COMM_INACTIVE;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define FSI_BUF_SZ_WORDS      (16U)

#pragma DATA_ALIGN(gs_rw_fsi_pack_buf, 4)
#pragma DATA_SECTION(gs_rw_fsi_pack_buf, "ramgs3")
volatile fsi_recv_data_t gs_rw_fsi_pack_buf = {.error = false, .packets = {0}};

#pragma DATA_ALIGN(gs_r_fsi_state_ring_buf, 4)
#pragma DATA_SECTION(gs_r_fsi_state_ring_buf, "ramgs4")
volatile fsi_data_ring_buf_t gs_r_fsi_state_ring_buf;

// TODO: Write the CLA data to GS and set the DMA to copy from there:

#pragma DATA_ALIGN(my_pack, 4)
#pragma DATA_SECTION(my_pack, "ramgs3")
uint16_t my_pack[FSI_PACKET_SZ_WORDS] = {
      DAISY_CHAIN_DRIVER_ID,
      DAISY_CHAIN_DRIVER_ID,
      DAISY_CHAIN_DRIVER_ID,
      DAISY_CHAIN_DRIVER_ID,
      DAISY_CHAIN_DRIVER_ID,
      DAISY_CHAIN_DRIVER_ID,
      DAISY_CHAIN_DRIVER_ID,
      DAISY_CHAIN_DRIVER_ID
};

// Packets should be received cyclically. If this doesn't happen we must
// have lost something and we trigger an error. The FSI would have
// gone wrong anyway for a packet to be lost and it would probably need
// a reset:
uint16_t next_pack_id = 0U;

// Whether we have completed one packet transmission:
uint16_t first_tx = 1U;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Triggered as soon as a data packet is received at FSI RX.
// A DMA channel will at the same time copy data to GS memory.
//
// TODO: Verify that the DMA is not triggered when this happens.
//
__attribute__((interrupt)) void fsi_rx_frame_isr(void)
{
    DBG_PIN3_SET;

    // If the timer is already running, that means that a packet is already
    // pending for transmission. Receiving a fresh one means that the transmitter
    // we're listening to is sending packets too fast. They should only complete
    // sending a fresh packet after our packet transmission is started:
    if(!(HWREGH(CPUTIMER1_BASE + CPUTIMER_O_TCR) & CPUTIMER_TCR_TSS))
    {
        gs_rw_fsi_pack_buf.error = true;
        return;
    }

    // Wait until a pending TX is complete to avoid corruption.
    // This applies only to transmissions after the first:
    while(!first_tx && !(FSI_getTxEventStatus(FSI_TX_CPU2_BASE) & FSI_TX_EVT_FRAME_DONE));

    // Clear the TX done event that will be set when the next packet is complete:
    FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVT_FRAME_DONE);

    // Start timer to count the delay before sending the packet:
    CPUTimer_startTimer(CPUTIMER1_BASE);

    DBG_PIN0_SET;

    // In the meantime, check the packet and send it to the TX buffer:
    uint16_t rx_tag = FSI_getRxFrameTag(FSI_RX_CPU2_BASE);

    // Check if there was an error in packet ordering:
    if (gs_rw_fsi_pack_buf.error || rx_tag != next_pack_id)
    {
        gs_rw_fsi_pack_buf.error = true;
        return;
    }

    next_pack_id = (next_pack_id + 1) % DAISY_CHAIN_DRIVER_COUNT;

    // Copy data to be transmitted to TX buffer:
    uint16_t cur_ring_buf = gs_r_fsi_state_ring_buf.cur_buf;
    DMA_configSourceAddress(
            DMA_FSI_TO_TX_CH,
            rx_tag == DAISY_CHAIN_DRIVER_ID ? (const void*)(&(gs_r_fsi_state_ring_buf.buf[cur_ring_buf][0])) : rx_buf_addr
    );

    DMA_forceTrigger(DMA_FSI_TO_TX_CH);

    // Prepare TX module for transmission:
    FSI_setTxBufferPtr(FSI_TX_CPU2_BASE, 0U);
    FSI_setTxFrameTag(FSI_TX_CPU2_BASE, (FSI_FrameTag)rx_tag);

    // Reset the RX pointer to prepare for next transmission:
//    while (DMA_getBurstStatusFlag(DMA_FSI_TO_MEM_CH) || DMA_getBurstStatusFlag(DMA_FSI_TO_TX_CH));
    FSI_setRxBufferPtr(FSI_RX_CPU2_BASE, 0U);
    FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVT_FRAME_DONE);

    first_tx = 0U;

    Interrupt_clearACKGroup(INT_FSI_RX_GROUP);
    DBG_PIN3_CLEAR;
}

__attribute__((interrupt)) void timer_end_tx_flush_isr(void)
{
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    FSI_executeTxFlushSequence(FSI_TX_CPU2_BASE, FSI_TX_CPU2_PRESCALE);
    DEVICE_DELAY_US(1);

////     Check whether a DMA transfer is pending for safety:
//    while(DMA_getBurstStatusFlag(DMA_FSI_TO_TX_CH));

    FSI_startTxTransmit(FSI_TX_CPU2_BASE);
}

__attribute__((interrupt)) void timer_end_tx_noflush_isr(void)
{
    DBG_PIN0_CLEAR;

    FSI_startTxTransmit(FSI_TX_CPU2_BASE);
    CPUTimer_stopTimer(CPUTIMER1_BASE);

//    // Check whether a DMA transfer is pending for safety:
//    while(DMA_getBurstStatusFlag(DMA_FSI_TO_TX_CH));
}

//__attribute__((interrupt)) void clear_dbg_pin(void)
//{
//    DBG_PIN0_CLEAR;
//    CPUTimer_stopTimer(CPUTIMER1_BASE);
//}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if DAISY_CHAIN_DRIVER_ID==0    // Master handshake routine

void chain_handshake()
{
    // 25 us between ping packets:
    CPUTimer_setPeriod(CPUTIMER1_BASE, (DEVICE_SYSCLK_FREQ / 1000000 * 25) - 1);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    Interrupt_register(INT_TIMER1, &timer_end_tx_flush_isr);

    FSI_setTxFrameTag(FSI_TX_CPU2_BASE, FSI_FRAME_TAG0);
    FSI_setTxFrameType(FSI_TX_CPU2_BASE, FSI_FRAME_TYPE_PING);

    FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVTMASK);
    FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVTMASK);

    // Send P0 periodically to ensure chain is fully online:
    CPUTimer_startTimer(CPUTIMER1_BASE);

    while (!(FSI_getRxEventStatus(FSI_RX_CPU2_BASE) & FSI_RX_EVT_PING_FRAME))
    {
        // If ping packet TX finished, send another one:
        if (FSI_getTxEventStatus(FSI_TX_CPU2_BASE) & FSI_TX_EVT_FRAME_DONE)
        {
            FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVT_FRAME_DONE);
            CPUTimer_startTimer(CPUTIMER1_BASE);
        }
    }

    // If we've received a forwarded ping packet, that means the chain is online.
    // Stop Ping 0 transmission and inform all nodes to end handshake with a Ping 1 packet:
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    DEVICE_DELAY_US(1);

    // FSI_setTxPingTag() is not for manually generated ping frames!
    FSI_setTxFrameTag(FSI_TX_CPU2_BASE, FSI_FRAME_TAG1);
    FSI_startTxTransmit(FSI_TX_CPU2_BASE);
}

#else                           // Slave handshake routine

void chain_handshake()
{
    FSI_setTxFrameType(FSI_TX_CPU2_BASE, FSI_FRAME_TYPE_PING);
    FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVTMASK);

    uint16_t recv_tag = 0U;

    while (true)
    {
        if (FSI_getRxEventStatus(FSI_RX_CPU2_BASE) & FSI_RX_EVT_PING_FRAME)
        {
            FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVT_PING_FRAME);

            // Forward any ping packets received. Send flush
            // sequence if the daisy chain is not yet fully online:
            recv_tag = FSI_getRxPingTag(FSI_RX_CPU2_BASE);
            FSI_setTxFrameTag(FSI_TX_CPU2_BASE, (FSI_FrameTag)recv_tag);

            if (recv_tag == FSI_FRAME_TAG1)
            {
                FSI_startTxTransmit(FSI_TX_CPU2_BASE);
                return;
            }

            FSI_executeTxFlushSequence(FSI_TX_CPU2_BASE, FSI_TX_CPU2_PRESCALE);
            DEVICE_DELAY_US(1);
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

    // TODO(rafael): While error, reset. until recv something successful (flush / ping)
    //            FSI_setRxDataWidth(FSI_RX_BASE, FSI_DATA_WIDTH_2_LANE);
    //            FSI_performRxInitialization(FSI_RX_BASE);

    ASSERT(FSI_PACKET_SZ_WORDS <= FSI_BUF_SZ_WORDS); // Packet must fit in RX / TX buffer
    ASSERT(DAISY_CHAIN_DRIVER_COUNT <= 0xFU);    // Due to tag size limitation

	// Synchronize with CPU1:
	IPC_sync(IPC_CPU2_L_CPU1_R, IPC_SYNC_FLAG);

	// Now FSI is available for us to configure:
	HAL_ini(&hal);

	// TODO(rafael): do this in the hal, the dma before the FSI? or maybe after
    DMA_initController();
//    DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);

    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    tx_buf_addr = (void*)FSI_getTxBufferAddress(FSI_TX_CPU2_BASE);
    rx_buf_addr = (void*)FSI_getRxBufferAddress(FSI_RX_CPU2_BASE);

    //////////////////////////////////// DMA CHANNELS ///////////////////////////////////////////////////////

    // FSI TX channel, source either FSIRX to forward or memory to send our info.
    // Manually triggered after source tag test:
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    DMA_configAddresses(DMA_FSI_TO_TX_CH, tx_buf_addr, rx_buf_addr);
    DMA_configBurst(DMA_FSI_TO_TX_CH, FSI_PACKET_SZ_WORDS, 1, 1);                // One packet per burst

    // Each transfer is a single burst of data (packet):
    DMA_configTransfer(DMA_FSI_TO_TX_CH, 1, 0, 0);

    DMA_configMode(
            DMA_FSI_TO_TX_CH,
            DMA_TRIGGER_SOFTWARE,
            DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT
    );

    DMA_disableInterrupt(DMA_FSI_TO_TX_CH);
    DMA_enableTrigger(DMA_FSI_TO_TX_CH);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    // FSI data to memory channel, triggered when a packet is received over FSI.
    // This should save the received data to GS3, cyclically. The previous board
    // should be sending a packet train starting from ID = 0 continuously.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    DMA_configAddresses(DMA_FSI_TO_MEM_CH, (uint16_t*)(&gs_rw_fsi_pack_buf.packets[0]), rx_buf_addr);
    DMA_configBurst(DMA_FSI_TO_MEM_CH, FSI_PACKET_SZ_WORDS, 1, 1);
    DMA_configTransfer(DMA_FSI_TO_MEM_CH, DAISY_CHAIN_DRIVER_COUNT, -(int16_t)FSI_PACKET_SZ_WORDS + 1, +1);

    // 32-bit unit size to avoid float corruption when CPU1 DMA copies.
    // TODO: This seems to only copy one word!
    DMA_configMode(
            DMA_FSI_TO_MEM_CH,
//            DMA_TRIGGER_SOFTWARE,
            DMA_TRIGGER_FSIRXC,
            DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT
    );

    DMA_disableInterrupt(DMA_FSI_TO_MEM_CH);
    DMA_enableTrigger(DMA_FSI_TO_MEM_CH);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////// FSI INITIALIZATION ///////////////////////////////////////////////////

    // Reset modules before starting handshake:
    FSI_resetTxModule(FSI_TX_CPU2_BASE, FSI_TX_MAIN_CORE_RESET);
    FSI_resetRxModule(FSI_RX_CPU2_BASE, FSI_RX_MAIN_CORE_RESET);

    DEVICE_DELAY_US(1);

    FSI_clearTxModuleReset(FSI_TX_CPU2_BASE, FSI_TX_MAIN_CORE_RESET);
    FSI_clearRxModuleReset(FSI_RX_CPU2_BASE, FSI_RX_MAIN_CORE_RESET);

    // TX settings:
    FSI_setTxDataWidth(FSI_TX_CPU2_BASE, FSI_DATA_WIDTH);
    FSI_setTxUserDefinedData(FSI_TX_CPU2_BASE, 0x0);
    FSI_configPrescalar(FSI_TX_CPU2_BASE, FSI_TX_CPU2_PRESCALE);      // Note: This disables the TX clock!
    FSI_enableTxClock(FSI_TX_CPU2_BASE);

    // Interrupts:
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    Interrupt_enable(INT_TIMER1);

    Interrupt_register(INT_FRAME_RX, &fsi_rx_frame_isr);
    Interrupt_enable(INT_FRAME_RX);

    asm(" NOP");
    EINT;
    ERTM;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Start handshaking procedure:
    // TODO: Add an LED or something to show what's happening.
    fsi_comm_state = COMM_HANDSHAKE;
    chain_handshake();

    // Handshake complete, setup data transmission and reception:
    // ----------------------------------------------------------
#if DAISY_CHAIN_DRIVER_ID==0

    // Delay a bit so that everyone gets ready:
    DEVICE_DELAY_US(2);
#endif

    // Set up TX delay timer (2us):
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_setPeriod(CPUTIMER1_BASE, (DEVICE_SYSCLK_FREQ / 1000000 * 2) - 1);
    Interrupt_register(INT_TIMER1, &timer_end_tx_noflush_isr);

    // Set up RX:
    FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVTMASK);
    FSI_enableRxFrameWatchdog(FSI_RX_CPU2_BASE, 4000U);               // RX every at least 20us
    FSI_setRxBufferPtr(FSI_RX_CPU2_BASE, 0U);
    FSI_setRxSoftwareFrameSize(FSI_RX_CPU2_BASE, FSI_PACKET_SZ_WORDS);

    // Set up TX:
    FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVTMASK);
    FSI_setTxFrameType(FSI_TX_CPU2_BASE, FSI_FRAME_TYPE_NWORD_DATA);
    FSI_setTxSoftwareFrameSize(FSI_TX_CPU2_BASE, FSI_PACKET_SZ_WORDS);

    // Start all DMA channels:
    DMA_startChannel(DMA_FSI_TO_TX_CH);
    DMA_startChannel(DMA_FSI_TO_MEM_CH);

    ////////////////////////////////// PACKET INJECTION //////////////////////////////////////////////////////
    // Inject packet train if you're board ID == 0:
    // --------------------------------------------

#if DAISY_CHAIN_DRIVER_ID==0
    // Set up an empty packet to be injected:
    memset((void*)FSI_getTxBufferAddress(FSI_TX_CPU2_BASE), 0x0U, sizeof(uint16_t) * 16);

    uint16_t cur_idx = 0U;
    for (cur_idx = 0U; cur_idx < DAISY_CHAIN_DRIVER_COUNT; cur_idx++)
    {
        FSI_setTxBufferPtr(FSI_TX_CPU2_BASE, 0U);
        FSI_setTxFrameTag(FSI_TX_CPU2_BASE, (FSI_FrameTag)cur_idx);
        CPUTimer_startTimer(CPUTIMER1_BASE);

        while (!(FSI_getTxEventStatus(FSI_TX_CPU2_BASE) & FSI_TX_EVT_FRAME_DONE));
        FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVT_FRAME_DONE);
    }

#endif
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

   while(true)
   {
       if (gs_rw_fsi_pack_buf.error)
       {
           CPUTimer_stopTimer(CPUTIMER1_BASE);

           DMA_stopChannel(DMA_FSI_TO_MEM_CH);
           DMA_stopChannel(DMA_FSI_TO_TX_CH);

           // TODO: Recover from a failure.

           __error__(__FILE__, __LINE__);
       }
   }
}
