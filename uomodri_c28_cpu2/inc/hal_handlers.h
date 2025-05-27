/*
 * File name: hal_handlers.h
 * Description: Header file containing CPU2 HAL config structures.
 */

#ifndef __HAL_HANDLERS_CPU2_H_
#define __HAL_HANDLERS_CPU2_H_

#include "hal.h"
#include "hal_defines.h"

/***********************************************************************
 * FSI STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const fsi_evt_cfg_t fsi_tx_evts[] =
{
 {.event = UINT16_MAX},
};

static const fsi_evt_cfg_t fsi_rx_evts[] =
{
 FSI_RX_CPU2_EVT_FRAME_DONE,
 {.event = UINT16_MAX},
};

static const fsi_cfg_t fsiCfgList[] =
{
 FSI_TX_CPU2_DEF(fsi_tx_evts),
 FSI_RX_CPU2_DEF(fsi_rx_evts),
 {.fsiBase = UINT32_MAX},           /* END OF ARRAY */
};

/***********************************************************************
 * GLOBAL HAL STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const hal_cfg_t  hal =
{
 .p_fsiHandle   = fsiCfgList,
};

#endif /* HAL_HANDLERS_CPU2_H_ */
