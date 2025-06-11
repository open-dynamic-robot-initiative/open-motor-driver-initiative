/*
 * File name: hal.c
 * Description: Source file containing CPU2 Hardware Abstraction Layer source code
 */

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <string.h>
#include <device.h>

#include "hal.h"

/***********************************************************************
 * FUNCTIONS
 ***********************************************************************/

/**
 * @brief       Initialize all HAL elements
 * @param[in]   *p_hal  Pointer to the general HAL structure definition
 */
void HAL_ini(const hal_cfg_t* p_hal)
{
    if(p_hal != NULL)
    {
        HAL_TMR_ini(p_hal->p_tmrHandle);
        HAL_DMA_ini(p_hal->p_dmaHandle);
        HAL_FSI_ini(p_hal->p_fsiHandle);
        HAL_INT_ini(p_hal->p_intHandle);
    }

    return;
}

/**
 * @brief       Interrupt management workaround according to errata sprz458.pdf:
 */
static volatile uint32_t errata_cnt = 0U;
__attribute__((interrupt)) void errata_isr(void)
{
    errata_cnt = (errata_cnt < UINT32_MAX) ? (errata_cnt + 1) : (0U);
}

/**
 * @brief       Initialize interrupts and handlers
 * @param[in]   *p_intCfg  Pointer to the interrupt config structure
 */
void HAL_INT_ini(const int_cfg_t* p_intCfg)
{
    ASSERT(p_intCfg != NULL);
    if(!p_intCfg) return;
    EALLOW;

    // Workaround for IT management according to silicon errata (see sprz458.pdf):
    Interrupt_register(INT_ADCA1,   errata_isr);
    Interrupt_register(INT_EPWM1_TZ,errata_isr);
    Interrupt_register(INT_EPWM1,   errata_isr);
    Interrupt_register(INT_ECAP1,   errata_isr);
    Interrupt_register(INT_EQEP1,   errata_isr);
    Interrupt_register(INT_SPIA_RX, errata_isr);
    Interrupt_register(INT_DMA_CH1, errata_isr);
    Interrupt_register(INT_I2CA,    errata_isr);
    Interrupt_register(INT_SCIA_RX, errata_isr);
    Interrupt_register(INT_ADCA_EVT,errata_isr);
    Interrupt_register(INT_CLA1_1,  errata_isr);
    Interrupt_register(INT_XINT3,   errata_isr);

    // Register interrupts:
    while (p_intCfg->interruptNumber != UINT32_MAX)
    {
        Interrupt_enable(p_intCfg->interruptNumber);
        Interrupt_register(p_intCfg->interruptNumber, p_intCfg->handler);

        p_intCfg++;
    }

    asm(" NOP");
    EINT;   // Globally enable interrupts (INTM)
    ERTM;   // Enable real time debug mode (DGBM)

    EDIS;
    return;
}

/**
 * @brief       DMA initialization function
 * @param[in]   *p_dmaCfg Pointer to the DMA configuration structure
 */
void HAL_DMA_ini(const dma_cfg_t* p_dmaCfg)
{
    ASSERT(p_dmaCfg != NULL);
    if(!p_dmaCfg) return;
    EALLOW;

    DMA_initController();

    while (p_dmaCfg->dmaBase != UINT32_MAX)
    {
        DMA_configAddresses(p_dmaCfg->dmaBase, p_dmaCfg->destAddr, p_dmaCfg->srcAddr);
        DMA_configBurst(p_dmaCfg->dmaBase, p_dmaCfg->burstSize, p_dmaCfg->burstSrcStep, p_dmaCfg->burstDestStep);
        DMA_configTransfer(p_dmaCfg->dmaBase, p_dmaCfg->transferSize, p_dmaCfg->transferSrcStep, p_dmaCfg->transferDestStep);

        DMA_configMode(p_dmaCfg->dmaBase, p_dmaCfg->trigger, p_dmaCfg->config);
        DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);

        p_dmaCfg->interruptEnable ?
                DMA_enableInterrupt(p_dmaCfg->dmaBase) :
                DMA_disableInterrupt(p_dmaCfg->dmaBase);

        DMA_enableTrigger(p_dmaCfg->dmaBase);
        p_dmaCfg->channelStart ? DMA_startChannel(p_dmaCfg->dmaBase) : NULL;

        p_dmaCfg++;
    }

    EDIS;
    return;
}

/**
 * @brief       CPU timers initialization function
 * @param[in]   *p_tmrCfg Pointer to the CPU timer configuration structure
 */
void HAL_TMR_ini(const tmr_cfg_t* p_tmrCfg)
{
    ASSERT(p_tmrCfg != NULL);
    if(!p_tmrCfg) return;
    EALLOW;

    while (p_tmrCfg->timerBase != UINT32_MAX)
    {
        CPUTimer_stopTimer(p_tmrCfg->timerBase);
        CPUTimer_setEmulationMode(p_tmrCfg->timerBase, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

        CPUTimer_setPeriod(
                p_tmrCfg->timerBase,
                (uint32_t)((float32_t)DEVICE_SYSCLK_FREQ / 1000000.0 * p_tmrCfg->periodUs) - 1
        );

        if (p_tmrCfg->enableInterrupt)
        {
            CPUTimer_enableInterrupt(p_tmrCfg->timerBase);
        }

        p_tmrCfg++;
    }

    EDIS;
    return;
}

/**
 * @brief       FSI module initialization function
 * @param[in]   *p_fsiCfg Pointer to the FSI configuration structure
 */
void HAL_FSI_ini(const fsi_cfg_t* p_fsiCfg)
{
    ASSERT(p_fsiCfg != NULL);
    if(!p_fsiCfg) return;
    EALLOW;

    while (p_fsiCfg->fsiBase != UINT32_MAX)
    {
        // Receiver module:
        if (p_fsiCfg->direction == FSI_RX_DIR)
        {

            // Force a full receiver reset at the beginning.
            FSI_resetRxModule(p_fsiCfg->fsiBase, FSI_RX_MAIN_CORE_RESET);
            FSI_resetRxModule(p_fsiCfg->fsiBase, FSI_RX_FRAME_WD_CNT_RESET);
            FSI_resetRxModule(p_fsiCfg->fsiBase, FSI_RX_PING_WD_CNT_RESET);

            DEVICE_DELAY_US(1);

            FSI_clearRxModuleReset(p_fsiCfg->fsiBase, FSI_RX_MAIN_CORE_RESET);
            FSI_clearRxModuleReset(p_fsiCfg->fsiBase, FSI_RX_FRAME_WD_CNT_RESET);
            FSI_clearRxModuleReset(p_fsiCfg->fsiBase, FSI_RX_PING_WD_CNT_RESET);

            // Disable FSI internal loopback if unused (internal TX->RX):
            if (!p_fsiCfg->rxLoopback)
            {
                FSI_disableRxInternalLoopback(p_fsiCfg->fsiBase);
            }

            // For some reason, the ping watchdog enable and event status
            // are not being cleared by the reset! Clear manually:
            FSI_disableRxPingWatchdog(p_fsiCfg->fsiBase);
            FSI_clearRxEvents(p_fsiCfg->fsiBase, FSI_RX_EVTMASK);

            // Initialize receiver and set data width for transmissions:
            FSI_performRxInitialization(p_fsiCfg->fsiBase);
            FSI_setRxDataWidth(p_fsiCfg->fsiBase, p_fsiCfg->dataWidth);

            if (p_fsiCfg->rxEOFWatchdogTimeoutUs > 0)
            {
                FSI_enableRxFrameWatchdog(
                        p_fsiCfg->fsiBase,
                        (uint32_t)((float32_t)DEVICE_SYSCLK_FREQ / 1000000.0 * p_fsiCfg->rxEOFWatchdogTimeoutUs)
                );
            }

            // Set ping watchdog to reset on any type of frame (the naming is unfortunate).
            // We will use this to timeout if there's a problem with a board in the chain.
            FSI_setRxPingTimeoutMode(p_fsiCfg->fsiBase, FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME);

            if (p_fsiCfg->dmaTrigger)
            {
                FSI_enableRxDMAEvent(p_fsiCfg->fsiBase);
            }

            // Interrupt configuration:
            const fsi_evt_cfg_t* evtCfg = p_fsiCfg->eventInterrupts;
            while (evtCfg->event != UINT16_MAX)
            {
                FSI_enableRxInterrupt(p_fsiCfg->fsiBase, evtCfg->intNum, evtCfg->event);
                evtCfg++;
            }
        }

        // Transmitter module:
        else
        {
            // Force a transmitter reset at the beginning:
            FSI_resetTxModule(p_fsiCfg->fsiBase, FSI_TX_MAIN_CORE_RESET);
            FSI_resetTxModule(p_fsiCfg->fsiBase, FSI_TX_CLOCK_RESET);
            FSI_resetTxModule(p_fsiCfg->fsiBase, FSI_TX_PING_TIMEOUT_CNT_RESET);

            DEVICE_DELAY_US(1);

            FSI_clearTxModuleReset(p_fsiCfg->fsiBase, FSI_TX_MAIN_CORE_RESET);
            FSI_clearTxModuleReset(p_fsiCfg->fsiBase, FSI_TX_CLOCK_RESET);
            FSI_clearTxModuleReset(p_fsiCfg->fsiBase, FSI_TX_PING_TIMEOUT_CNT_RESET);

            FSI_performTxInitialization(p_fsiCfg->fsiBase, p_fsiCfg->clkPrescaler);
            FSI_setTxDataWidth(p_fsiCfg->fsiBase, p_fsiCfg->dataWidth);

            if (p_fsiCfg->dmaTrigger)
            {
                FSI_enableTxDMAEvent(p_fsiCfg->fsiBase);
            }

            FSI_enableTxClock(p_fsiCfg->fsiBase);

            // Interrupt configuration:
            const fsi_evt_cfg_t* evtCfg = p_fsiCfg->eventInterrupts;
            while (evtCfg->event != UINT16_MAX)
            {
                FSI_enableTxInterrupt(p_fsiCfg->fsiBase, evtCfg->intNum, evtCfg->event);
                evtCfg++;
            }
        }

        p_fsiCfg++;
    }

    EDIS;
    return;
}

