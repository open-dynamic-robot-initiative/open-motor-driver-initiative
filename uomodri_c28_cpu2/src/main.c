// TODO(rafael): Clean all of this - create HAL functions, etc
// TODO(rafael): Allocation to global shared memory doesn't work, unless we allow from CPU1

#include <device.h>

#include "hal.h"
#include "hal_handlers.h"

#define INT_FSI_RX1_GROUP                       INTERRUPT_ACK_GROUP7

#define DBG_PIN0_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO82)
#define DBG_PIN0_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO82)

#define DBG_PIN3_SET                            (HWREG(GPIODATA_BASE + GPIO_O_GPCSET)   = GPIO_GPCSET_GPIO77)
#define DBG_PIN3_CLEAR                          (HWREG(GPIODATA_BASE + GPIO_O_GPCCLEAR) = GPIO_GPCCLEAR_GPIO77)

//#pragma DATA_ALIGN(fsi_test_rxint1_recv, 4)
//#pragma DATA_SECTION(fsi_test_rxint1_recv, "ramgs1")
uint16_t   fsi_test_rxint1_recv      = 0U;

// Example ISR:
// ==========================================================================
//extern bool_t fsi_test_txint1_recv;

//__attribute__((interrupt)) void fsi_tx1_isr(void)
//{
//    fsi_test_txint1_recv = true;
//
//    // TODO(rafael): Call and save FSI_getTxEventStatus()
//    // and clear only the handled events (see 32.2.3.4)
//    FSI_clearTxEvents(FSI_TX_BASE, FSI_TX_EVTMASK);
//
//    // TODO(rafael): Can handle errors by disabling
//    // transfers and interrupts (eg. disableAllFSIInterrupts()).

//    Interrupt_clearACKGroup(INT_FSI_TX1_GROUP);
//}
// ==========================================================================

__attribute__((interrupt)) void test_isr(void)
{
    fsi_test_rxint1_recv = 1U;

    FSI_clearRxEvents(FSI_RX_CPU2_BASE, FSI_RX_EVTMASK);
    Interrupt_clearACKGroup(INT_FSI_RX1_GROUP);
}

int main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    // TODO(rafael): While error, reset. until recv something successful (flush / ping)
    //            FSI_setRxDataWidth(FSI_RX_BASE, FSI_DATA_WIDTH_2_LANE);
    //            FSI_performRxInitialization(FSI_RX_BASE);

	// Synchronize with CPU1:
	IPC_sync(IPC_CPU2_L_CPU1_R, IPC_SYNC_FLAG);

	HAL_ini(&hal);

	FSI_enableRxInterrupt(FSI_RX_CPU2_BASE, FSI_INT1, FSI_RX_EVT_FRAME_DONE);

    Interrupt_register(INT_FSIRXC1, &test_isr);
    Interrupt_enable(INT_FSIRXC1);
    asm(" NOP");
    EINT;
    ERTM;

    uint16_t i = 0;

	while (1)
	{
	    if (fsi_test_rxint1_recv)
	    {
            DBG_PIN0_SET;
	        for (i = 0; i < 1000; i++);
            DBG_PIN0_CLEAR;

            fsi_test_rxint1_recv = 0;
	    }
	}

}
