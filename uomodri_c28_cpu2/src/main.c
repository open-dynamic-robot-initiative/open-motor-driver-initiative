// TODO(rafael): Clean all of this - create HAL functions, etc
// TODO(rafael): Allocation to global shared memory doesn't work, unless we allow from CPU1

#include <device.h>

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PACKET_SZ_WORDS       (4U)
#define FSI_BUF_SZ_WORDS      (16U)

#pragma DATA_ALIGN(gs_fsi_pack_buf, 4)
#pragma DATA_SECTION(gs_fsi_pack_buf, "ramgs3")
fsi_recv_data_t gs_fsi_pack_buf = {.error = false, .packets = {0}};

//#pragma DATA_ALIGN(def_data, 4)
//#pragma DATA_SECTION(def_data, "ramgs3")
//uint16_t def_data[] = {0x0061, 0x0062, 0x0063, 0x0064, 0x0065, 0x0066, 0x0067, 0x0068, 0x0069, 0x0070, 0x0071, 0x0072, 0x0073, 0x0074, 0x0075, 0x0076};

// TODO: is this actually copied by the dma?!
uint16_t my_pack[4] = {0x42, 0x43, 0x44, 0x45};

// Packets should be received cyclically. If this doesn't happen we must
// have lost something and we trigger an error. The FSI would have
// gone wrong anyway for a packet to be lost and it would probably need
// a reset:
uint16_t next_pack_id = 0U;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Triggered as soon as a packet is received at FSI RX.
// A DMA channel will at the same time copy data to GS memory.
__attribute__((interrupt)) void fsi_rx_frame_isr(void)
{
    DBG_PIN3_SET;

    uint16_t rx_tag = FSI_getRxFrameTag(FSI_RX_CPU2_BASE);

    // Copy data to be transmitted to TX buffer:
    DMA_configSourceAddress(DMA_FSI_TO_TX_CH, rx_tag == 0x42 ? &my_pack[0] : rx_buf_addr);
    DMA_forceTrigger(DMA_FSI_TO_TX_CH);

//    // Check if there was an error in packet ordering:
//    if (pack_buf.error || rx_tag != next_pack_id)
//    {
//        pack_buf.error = true;
//        Interrupt_clearACKGroup(INT_FSI_RX_GROUP);
//        return;
//    }

    next_pack_id = (next_pack_id + 1) % DAISY_CHAIN_DRIVER_COUNT;

    // Prepare TX module for transmission:
    FSI_setTxBufferPtr(FSI_TX_CPU2_BASE, 0U);
    FSI_setTxFrameTag(FSI_TX_CPU2_BASE, (FSI_FrameTag)rx_tag);
    FSI_clearTxEvents(FSI_TX_CPU2_BASE, FSI_TX_EVT_FRAME_DONE);

    // When TX DMA finishes transmit packet:
    while (DMA_getBurstStatusFlag(DMA_FSI_TO_TX_CH));
    FSI_startTxTransmit(FSI_TX_CPU2_BASE);

    // TODO: Record TX timestamp here.

    // Check whether DMA saving the data finished too, and reset
    // RX pointer if so:
    while (DMA_getBurstStatusFlag(DMA_FSI_TO_MEM_CH));
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
    DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);

    tx_buf_addr = (void*)FSI_getTxBufferAddress(FSI_TX_CPU2_BASE);
    rx_buf_addr = (void*)FSI_getRxBufferAddress(FSI_RX_CPU2_BASE);

    // FSI TX channel, source either FSIRX to forward or memory to send our info.
    // Manually triggered after source tag test:
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    DMA_configAddresses(DMA_FSI_TO_TX_CH, tx_buf_addr, rx_buf_addr);
    DMA_configBurst(DMA_FSI_TO_TX_CH, PACKET_SZ_WORDS, 1, 1);                // One packet per burst

    // Each transfer is a single burst of data (packet):
    DMA_configTransfer(DMA_FSI_TO_TX_CH, 1, 0, 0);

    DMA_configMode(
            DMA_FSI_TO_TX_CH,
            DMA_TRIGGER_SOFTWARE,
            DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT
    );

    DMA_disableInterrupt(DMA_FSI_TO_TX_CH);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    // FSI data to memory channel, triggered when a packet is received over FSI.
    // This should save the received data to GS3, cyclically. The previous board
    // should be sending a packet train starting from ID = 0 continuously.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    DMA_configAddresses(DMA_FSI_TO_MEM_CH, (uint16_t*)(&gs_fsi_pack_buf.packets[0]), rx_buf_addr);
    DMA_configBurst(DMA_FSI_TO_MEM_CH, PACKET_SZ_WORDS, 1, 1);
    DMA_configTransfer(DMA_FSI_TO_MEM_CH, DAISY_CHAIN_DRIVER_COUNT, -(int16_t)PACKET_SZ_WORDS + 1, +1);

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

    // Initialize FSI forwarding settings. These settings don't need
    // to be adjusted again:

    // RX:
    FSI_enableRxFrameWatchdog(FSI_RX_CPU2_BASE, 4000U); // 20us

    // TX:
    FSI_setTxDataWidth(FSI_TX_CPU2_BASE, FSI_DATA_WIDTH);
    FSI_configPrescalar(FSI_TX_CPU2_BASE, FSI_TX_CPU2_PRESCALE);      // Note: This disables the TX clock!
    FSI_setTxFrameType(FSI_TX_CPU2_BASE, FSI_FRAME_TYPE_4WORD_DATA);  // TODO(rafael): Synchronize this to CPU1
    FSI_setTxUserDefinedData(FSI_TX_CPU2_BASE, 0x0);
    FSI_enableTxClock(FSI_TX_CPU2_BASE);

    // Interrupts:
    Interrupt_register(INT_FRAME_RX, &fsi_rx_frame_isr);
    Interrupt_enable(INT_FRAME_RX);

    asm(" NOP");
    EINT;
    ERTM;


    // TODO(rafael): Do these in the proper order in the HAL:
    while(FSI_getRxFrameWatchdogCounter(FSI_RX_CPU2_BASE));     // This is not necessary, if we do HAL_ini()
    FSI_setRxBufferPtr(FSI_RX_CPU2_BASE, 0U);

    // Start all DMA channels:
    DMA_startChannel(DMA_FSI_TO_TX_CH);
    DMA_startChannel(DMA_FSI_TO_MEM_CH);

    while(true)
    {
        if (gs_fsi_pack_buf.error)
        {
            DMA_stopChannel(DMA_FSI_TO_MEM_CH);
            DMA_stopChannel(DMA_FSI_TO_TX_CH);

            // TODO: Recover from a failure.

            __error__(__FILE__, __LINE__);
        }
    }
}
