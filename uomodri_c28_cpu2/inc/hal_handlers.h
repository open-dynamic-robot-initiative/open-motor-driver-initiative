/*
 * File name: hal_handlers.h
 * Description: Header file containing CPU2 HAL config structures.
 */

#ifndef __HAL_HANDLERS_CPU2_H_
#define __HAL_HANDLERS_CPU2_H_

#include "interrupts.h"

#include "hal.h"
#include "hal_defines.h"

/***********************************************************************
 * INTERRUPT HAL STRUCTURE
 ***********************************************************************/
static const int_cfg_t intCfgList[] =
{
#if DAISY_CHAIN_ENABLE
   INT_FSI_RX_DATA_FRAME_DEF,
   INT_FSI_RX_ERROR_DEF,
   INT_TIMER_DEF,
#endif

   { .interruptNumber = UINT32_MAX },
};

/***********************************************************************
 * TIMER HAL STRUCTURE
 ***********************************************************************/
static const tmr_cfg_t tmrCfgList[] =
{
#if DAISY_CHAIN_ENABLE
    TIMER_FSI_TX_DEF,
#endif

    { .timerBase = UINT32_MAX },
};

/***********************************************************************
 * DMA HAL STRUCTURE
 ***********************************************************************/
static const dma_cfg_t dmaCfgList[] =
{
#if DAISY_CHAIN_ENABLE
    DMA_FSI_TO_MEM_DEF,
    DMA_FSI_TO_TX_DEF,
#endif

    { .dmaBase = UINT32_MAX },
};

/***********************************************************************
 * FSI HAL STRUCTURE
 ***********************************************************************/
static const fsi_evt_cfg_t fsi_tx_evts[] = { {.event = UINT16_MAX}, };

static const fsi_evt_cfg_t fsi_rx_evts[] = {
    // Events that require a receiver reset:
    // NOTE: I've also tried polling:
    //       HWREGH(FSI_RX_CPU2_BASE + FSI_O_RX_VIS_1) & FSI_RX_VIS_1_RX_CORE_STS
    //       according to the TRM, but that triggered at weird times without any other errors.
    {.event = FSI_RX_EVT_TYPE_ERR,          .intNum = FSI_INT2},
    {.event = FSI_RX_EVT_EOF_ERR,           .intNum = FSI_INT2},
    {.event = FSI_RX_EVT_PING_WD_TIMEOUT,   .intNum = FSI_INT2},
    {.event = FSI_RX_EVT_FRAME_WD_TIMEOUT,  .intNum = FSI_INT2},

    // Events that mean something went wrong:
    {.event = FSI_RX_EVT_FRAME_OVERRUN,     .intNum = FSI_INT2},
    {.event = FSI_RX_EVT_OVERRUN,           .intNum = FSI_INT2},
    {.event = FSI_RX_EVT_ERR_FRAME,         .intNum = FSI_INT2},
    {.event = FSI_RX_EVT_CRC_ERR,           .intNum = FSI_INT2},

    // NOTE: The data frame event will be enabled at runtime.
    {.event = UINT16_MAX},
};

static const fsi_cfg_t fsiCfgList[] =
{
#if DAISY_CHAIN_ENABLE
    FSI_TX_CPU2_DEF(fsi_tx_evts),
    FSI_RX_CPU2_DEF(fsi_rx_evts),
#endif

    { .fsiBase = UINT32_MAX },           /* END OF ARRAY */
};

/***********************************************************************
 * GLOBAL HAL STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const hal_cfg_t  hal =
{
    .p_dmaHandle   = dmaCfgList,
    .p_fsiHandle   = fsiCfgList,
    .p_tmrHandle   = tmrCfgList,
    .p_intHandle   = intCfgList,
};

#endif /* HAL_HANDLERS_CPU2_H_ */
