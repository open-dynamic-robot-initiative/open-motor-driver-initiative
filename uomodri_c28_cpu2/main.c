// TODO(rafael): Clean all of this - create HAL functions, etc
// TODO(rafael): Allocation to global shared memory doesn't work, unless we allow from CPU1

#include <device.h>


#define USE_UOMODRI_V1                          (1)
#define USE_UOMODRI_V2                          (2)
#define USE_UOMODRI_V3                          (3)
#define USE_UOMODRI_SA_V1                       (4)
#define USE_UOMODRI_REV                         USE_UOMODRI_V3

#if (USE_UOMODRI_REV == USE_UOMODRI_V1)
#include "config/uomodri_1/uomodri_1_handlers.h"    // uOmodri v1.x configuration include file
#elif (USE_UOMODRI_REV == USE_UOMODRI_V2)
#include "config/uomodri_2/uomodri_2_handlers.h"    // uOmodri v2.x configuration include file
#elif (USE_UOMODRI_REV == USE_UOMODRI_V3)
#include "config/uomodri_3/uomodri_3_handlers.h"    // uOmodri v3.x configuration include file
#endif

//#define IPC_SYNC_FLAG                           IPC_FLAG31
#define FSI_RX_CPU2_BASE                        FSIRXC_BASE
#define FSI_RX_CPU2_CPUSEL_INSTANCE             SYSCTL_CPUSEL_FSIRXC

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
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();


    // TODO(rafael): While error, reset. until recv something successful (flush / ping)
    //            FSI_setRxDataWidth(FSI_RX_BASE, FSI_DATA_WIDTH_2_LANE);
    //            FSI_performRxInitialization(FSI_RX_BASE);

	// Synchronize with CPU1:
	IPC_sync(IPC_CPU2_L_CPU1_R, IPC_SYNC_FLAG);

	// We now know the FSI has been setup:
    FSI_disableRxInternalLoopback(FSI_RX_CPU2_BASE);

    FSI_resetRxModule(FSI_RX_CPU2_BASE, FSI_RX_MAIN_CORE_RESET);
    DEVICE_DELAY_US(1);
    FSI_clearRxModuleReset(FSI_RX_CPU2_BASE, FSI_RX_MAIN_CORE_RESET);

    FSI_performRxInitialization(FSI_RX_CPU2_BASE);

    FSI_setRxDataWidth(FSI_RX_CPU2_BASE, FSI_DATA_WIDTH_1_LANE);
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
