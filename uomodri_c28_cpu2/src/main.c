// TODO(rafael): Clean all of this - create HAL functions, etc
// TODO(rafael): Allocation to global shared memory doesn't work, unless we allow from CPU1

#include <device.h>

#include "hal.h"
#include "hal_handlers.h"

//#define INT_FSI_TX_GROUP                       INTERRUPT_ACK_GROUP4
//
//#define INT_FSI_RX1_GROUP                       INTERRUPT_ACK_GROUP7
//
#define DBG_PIN0_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO82)
#define DBG_PIN0_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO82)

#define DBG_PIN3_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO77)
#define DBG_PIN3_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO77)
//
//#define FSI_TX_FRAME_TAG_UDATA_REGADDR (FSI_TX_CPU2_BASE + FSI_O_TX_FRAME_TAG_UDATA)
//
//#define DEVICE_ID (0x9U)
//
//
//

#define INT_FSI_RX_GROUP                        INTERRUPT_ACK_GROUP7
#define INT_DMA_GROUP                           INTERRUPT_ACK_GROUP7


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PACKET_SZ_WORDS       (4U)
#define PACK_BUF_CAPACITY     (16U)  // Packets
#define FSI_BUF_SZ_WORDS      (16U)

#define FSI_RX_FRAME_TAG_UDATA_REGADDR (FSI_RX_CPU2_BASE + FSI_O_RX_FRAME_TAG_UDATA)

// TODO: make sure this was linked correctly to gs3
#pragma DATA_ALIGN(pack_buf, 4)
#pragma DATA_SECTION(pack_buf, "ramgs3")
uint16_t pack_buf[PACK_BUF_CAPACITY + 1][PACKET_SZ_WORDS] = {0};

#pragma DATA_ALIGN(tag_buf, 4)
#pragma DATA_SECTION(tag_buf, "ramgs3")
uint16_t tag_buf[PACK_BUF_CAPACITY + 1] = {0};

volatile uint16_t pending_packs = 0;
volatile bool_t   error = false;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Triggered as soon as a packet is received at FSI RX. DMA should
// pick up the tag and data at the same time.
__attribute__((interrupt)) void fsi_rx_frame_isr(void)
{
    DBG_PIN3_SET;

    // Mark there will be more data available:
    if (++pending_packs > PACK_BUF_CAPACITY)
    {
        error = true;
    }

    // Reset the FSI RX buffer pointer to the beginning for next packet:
    FSI_setRxBufferPtr(FSI_RX_CPU2_BASE, 0U);

    Interrupt_clearACKGroup(INT_FSI_RX_GROUP);
    DBG_PIN3_CLEAR;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    // TODO(rafael): While error, reset. until recv something successful (flush / ping)
    //            FSI_setRxDataWidth(FSI_RX_BASE, FSI_DATA_WIDTH_2_LANE);
    //            FSI_performRxInitialization(FSI_RX_BASE);

    ASSERT(PACKET_SZ_WORDS <= FSI_BUF_SZ_WORDS); // Packet must fit in RX / TX buffer

	// Synchronize with CPU1:
	IPC_sync(IPC_CPU2_L_CPU1_R, IPC_SYNC_FLAG);

	// Now FSI is available for us to configure:
	HAL_ini(&hal);

	// TODO(rafael): do this in the hal, the dma before the FSI? or maybe after
    DMA_initController();

    // TODO(rafael): Do we need to set the data channel to be high prio?
#define DMA_DATA_RX_CH  (DMA_CH1_BASE)
#define DMA_TAG_RX_CH   (DMA_CH2_BASE)
#define DMA_DATA_TX_CH  (DMA_CH3_BASE)

//#define INT_DATA_RX_DMA (INT_DMA_CH1)
//#define INT_TAG_RX_DMA  (INT_DMA_CH2)
#define INT_FRAME_RX      (INT_FSIRXC1)

    // Data RX channel:
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    void* rx_buf_addr = (void*)FSI_getRxBufferAddress(FSI_RX_CPU2_BASE);

    DMA_configAddresses(DMA_DATA_RX_CH, pack_buf, rx_buf_addr);
    DMA_configBurst(DMA_DATA_RX_CH, PACKET_SZ_WORDS, 1, 1);                // One packet per burst

    // Full buffer per transfer so it will wrap to the beginning when it's
    // transferred BUF_CAPACITY packets:
    DMA_configTransfer(
            DMA_DATA_RX_CH,
            PACK_BUF_CAPACITY,
            -3,   // Src  must wrap to beginning
            1     // Dest can continue forward
    );

    DMA_configMode(
            DMA_DATA_RX_CH,
            DMA_TRIGGER_FSIRXC,
            DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT
    );

    /*
    // Trigger interrupt at the beginning to mark the fact that a new packet is
    // being stored in the buffer.
    DMA_setInterruptMode(DMA_DATA_RX_CH, DMA_INT_AT_BEGINNING);
    Interrupt_register(INT_DATA_RX_DMA, &fsi_rx_data_dma_start_isr);

    DMA_enableInterrupt(DMA_DATA_RX_CH);
    */

    DMA_disableInterrupt(DMA_DATA_RX_CH);
    DMA_enableTrigger(DMA_DATA_RX_CH);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Tag RX channel:
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    void* rx_tag_addr = (void*)FSI_RX_FRAME_TAG_UDATA_REGADDR;

    DMA_configAddresses(DMA_TAG_RX_CH, tag_buf, rx_tag_addr);
    DMA_configBurst(DMA_TAG_RX_CH, 1, 0, 1);

    // TODO(rafael): how can this 4 offset fill up the buffer correctly?!?!?!
    DMA_configTransfer(DMA_TAG_RX_CH, PACK_BUF_CAPACITY, 0, 1);

    DMA_configMode(
            DMA_TAG_RX_CH,
            DMA_TRIGGER_FSIRXC,
            DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT
    );

    DMA_disableInterrupt(DMA_TAG_RX_CH);
    DMA_enableTrigger(DMA_TAG_RX_CH);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    // Data TX channel (manual buffer addressing):
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    void* tx_buf_addr = (void*)FSI_getTxBufferAddress(FSI_TX_CPU2_BASE);

    DMA_configAddresses(DMA_DATA_TX_CH, tx_buf_addr, pack_buf); // Always copy to the start of FSITX buffer
    DMA_configBurst(DMA_DATA_TX_CH, PACKET_SZ_WORDS, 1, 1);
    DMA_configTransfer(DMA_DATA_TX_CH, 1, 0, 0);

    // Disable channel after sending a packet. CPU main loop controls transfers.
    DMA_configMode(
            DMA_DATA_TX_CH,
            DMA_TRIGGER_SOFTWARE,
            DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT
    );

    // TODO(rafael): Enable this channel to trigger FSI transfer!

    DMA_disableInterrupt(DMA_DATA_TX_CH);
    DMA_enableTrigger(DMA_DATA_TX_CH);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Initialize FSI forwarding settings. These settings don't need
    // to be adjusted again:

    // RX:
    FSI_enableRxFrameWatchdog(FSI_RX_CPU2_BASE, 4000U); // 20us

    // TX:
    FSI_setTxDataWidth(FSI_TX_CPU2_BASE, FSI_DATA_WIDTH);
    FSI_configPrescalar(FSI_TX_CPU2_BASE, FSI_TX_CPU2_PRESCALE);      // Note: This disables the TX clock!
    FSI_setTxFrameType(FSI_TX_CPU2_BASE, FSI_FRAME_TYPE_4WORD_DATA);  // TODO(rafael): Synchronize this to CPU1
    // FSI_setTxUserDefinedData(FSI_TX_CPU2_BASE, 0x0);
    // FSI_enableTxInterrupt(FSI_TX_CPU2_BASE, FSI_INT1, FSI_TX_EVT_FRAME_DONE);
    FSI_enableTxClock(FSI_TX_CPU2_BASE);


    // Interrupts:
    Interrupt_register(INT_FRAME_RX, &fsi_rx_frame_isr);
    Interrupt_enable(INT_FRAME_RX);

    asm(" NOP");
    EINT;
    ERTM;

    // Start all DMA channels:
    DMA_startChannel(DMA_DATA_RX_CH);
    DMA_startChannel(DMA_TAG_RX_CH);
    DMA_startChannel(DMA_DATA_TX_CH);

    // TODO(rafael): Do these in the proper order in the HAL:
    while(FSI_getRxFrameWatchdogCounter(FSI_RX_CPU2_BASE));     // This is not necessary, if we do HAL_ini()
    FSI_setRxBufferPtr(FSI_RX_CPU2_BASE, 0U);
    FSI_enableRxDMAEvent(FSI_RX_CPU2_BASE);

    uint16_t next_pack_idx = 0U;

    bool_t set = false;
	while (1)
	{
	    if (set)
	        DBG_PIN0_CLEAR;
	    else
	        DBG_PIN0_SET;

	    set = !set;

	    if (error)
	    {
	        DMA_stopChannel(DMA_DATA_RX_CH);
	        DMA_stopChannel(DMA_TAG_RX_CH);
	        DMA_stopChannel(DMA_DATA_TX_CH);

	        __error__(__FILE__, __LINE__);
	    }

	    if (pending_packs)
	    {
	        // Wait for RX DMA transfers to finish so that we're sure we have data available.
	        // DMA can trigger interrupt only at the end of a transfer, not a burst.
	        // We technically only have to wait for the current transfer to finish only if
	        // it's transferring the first packet. However, since there's no way to mark
	        // that the DMA has finished a packet burst, we do it every time.
	        //
	        // TODO(rafael): Make sure it's not a bottleneck if we unnecessarily wait
	        //               for an RX when we have data available already.
	        //               DMA takes 4 cycles / word + 1 cycle / burst ->
	        //                  10 words = 41 cycles = 205ns, meh :)
	        while (DMA_getBurstStatusFlag(DMA_DATA_RX_CH) || DMA_getBurstStatusFlag(DMA_TAG_RX_CH));

            uint16_t next_tag = (tag_buf[next_pack_idx] & 0x1e) >> 1;

	        // TODO(rafael): device id
	        if (next_tag != 0x42)
	        {
	            // Set DMA source address:
	            DMA_configSourceAddress(DMA_DATA_TX_CH, &pack_buf[next_tag]);

	            // SW trigger TX DMA and wait until done to trigger FSITX:
	            // FSITX can be triggered by DMA if DMA writes UDATA field,
	            // but in this case we write in SW due to RX <-> TX tag reg offset.
	            DMA_forceTrigger(DMA_DATA_TX_CH);

	            // While DMA is transferring, configure FSI TX:
	            // --------------------------------------------
	            // Set TX module to start reading data from its beginning,
	            // where DMA is placing the packet.
	            FSI_setTxBufferPtr(FSI_TX_CPU2_BASE, 0U);

	            // Set tag and clear TX done flag:
	            FSI_setTxFrameTag(FSI_TX_CPU2_BASE, (FSI_FrameTag)next_tag);
	            FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVT_FRAME_DONE);

	            // Wait for DMA to finish and start packet transmission:
	            while (DMA_getBurstStatusFlag(DMA_DATA_TX_CH));
                FSI_startTxTransmit(FSI_TX_CPU2_BASE);

                // Wait for packet to finish before sending another one:
                while (! (FSI_getTxEventStatus(FSI_TX_CPU2_BASE) & FSI_TX_EVT_FRAME_DONE));

                // TODO(rafael): Add some delay for next board to transfer the data.
	        }

	        next_pack_idx++;
	        next_pack_idx %= PACK_BUF_CAPACITY;

	        pending_packs--;
	    }
	}

	// TODO(rafael): the last packet needs to check sts for tag & data dma before tx!!!
	// TODO(rafael): need to set tx ptr address to beginning as well!
}
